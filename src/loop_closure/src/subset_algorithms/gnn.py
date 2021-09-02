from __future__ import division

from copy import deepcopy

import math
import time

from subset_algorithms.util import constuct_pytorch_geometric_graph, run_model_on_list_of_graphs, choices, fast_deepcopy
import rospy
import numpy as np
from numpy.random import binomial
import random

from subset_algorithms.heuristic import MaximallySeperatedNodes, MaximumCovarianceNodes, Random
from subset_algorithms.base import GNNBasedLoopClosureBatchSelector



class BinomialCEM(GNNBasedLoopClosureBatchSelector):
    def __init__(self, r_lock, min_processing_seconds=30):
        super(BinomialCEM, self).__init__(r_lock, min_processing_seconds)
        self.initialized = False

    def make_histogram_string(self, probabilities, max_bins=10):
        hist, bin_edges = np.histogram(probabilities, bins=min(probabilities.shape[0], max_bins))
        s = ""
        for value, left_edge, right_edge in zip(hist, bin_edges, bin_edges[1:]):
            s += "%.2f : %d " % (((right_edge + left_edge) / 2.0 * 100), value)
        return s

    def set_params(self, parameters):
        self.samples = parameters["cem"]["samples"]
        self.alpha = parameters["cem"]["alpha"]
        self.ptile = parameters["cem"]["ptile"]

    def initialize_method(self):
        self.probabilities = np.ones(len(self.possible_new_edges)) * .5
        self.p_quantile_error = float("inf")
        self.running_iterations = 0
        self.initialized = True

    def step(self):
        # Sample graphs
        cur_samples = []
        for _ in range(self.samples):
            cur_samples.append(binomial(1, self.probabilities))
        # Make sure samples have exactly num_to_select entries which are 1
        for i in range(self.samples):
            flip_direction = 0
            if np.sum(cur_samples[i]) < self.number_of_loop_closures_to_select:
                flip_direction = 1
                num_samples_to_make_1 = self.number_of_loop_closures_to_select - np.sum(cur_samples[i])
                idxs_which_are_zero = [idx for idx in range(cur_samples[i].shape[0]) if cur_samples[i][idx] == 0]
                probs_for_idxs = [self.probabilities[idx] for idx in idxs_which_are_zero]
                idxs_to_flip_to_one = np.random.choice(idxs_which_are_zero, num_samples_to_make_1, replace=False,
                                                       p=probs_for_idxs / np.sum(probs_for_idxs))
                for idx in idxs_to_flip_to_one:
                    cur_samples[i][idx] = 1
            if np.sum(cur_samples[i]) > self.number_of_loop_closures_to_select:
                flip_direction = -1
                num_samples_to_make_0 = np.sum(cur_samples[i]) - self.number_of_loop_closures_to_select
                idxs_which_are_one = [idx for idx in range(cur_samples[i].shape[0]) if cur_samples[i][idx] == 1]
                probs_for_idxs = [1 - self.probabilities[idx] for idx in idxs_which_are_one]
                idxs_to_flip_to_zero = np.random.choice(idxs_which_are_one, num_samples_to_make_0, replace=False,
                                                        p=probs_for_idxs / np.sum(probs_for_idxs))
                for idx in idxs_to_flip_to_zero:
                    cur_samples[i][idx] = 0

            assert np.sum(
                    cur_samples[i]) == self.number_of_loop_closures_to_select, "Sum: %d Should be %d Flip Direction %d" % (np.sum(cur_samples[i]), self.number_of_loop_closures_to_select, flip_direction)
        # Compute fitness
        graphs_to_run = []
        for cur_sample in cur_samples:
            edges = []
            edge_attrs = []
            included = 0
            for edge_idx,include in enumerate(cur_sample):
                if include == 1:
                    try:
                        edges.append(self.possible_new_edges[edge_idx])
                        edge_attrs.append(self.possible_edge_attrs[edge_idx])
                        included += 1
                    except IndexError:
                        self.log_debug("IndexError")
            edges = np.concatenate((self.current_pose_graph_edges,np.array(edges)))
            edge_attrs = np.concatenate((self.current_pose_graph_edge_atrrs,np.array(edge_attrs)))
            assert included == self.number_of_loop_closures_to_select, "Included {}, Required {}".format(included,self.number_of_loop_closures_to_select)

            graphs_to_run.append(
                constuct_pytorch_geometric_graph(self.current_pose_graph_nodes, edges,
                                                 edge_attrs))
        errors = run_model_on_list_of_graphs(self.gnn_model, graphs_to_run)
        new_probabilities = np.zeros(len(self.possible_new_edges))

        self.p_quantile_error = min(np.percentile(errors, self.ptile), self.p_quantile_error)

        number_of_included_samples = 0
        for fitness, sample_idx in zip(errors, list(range(self.samples))):
            if fitness <= self.p_quantile_error:
                new_probabilities += np.array(cur_samples[sample_idx])
                number_of_included_samples += 1
        if number_of_included_samples != 0:
            new_probabilities /= number_of_included_samples
            new_probabilities = (1 - self.alpha) * self.probabilities + self.alpha * new_probabilities
            self.probabilities = new_probabilities
        hist_string = self.make_histogram_string(self.probabilities)
        self.log_debug(
            "CEM: {} bound {} effectivity {}, Errors N({:e},{:e}) Range: {:e}  - {}".format(
            self.running_iterations,self.p_quantile_error, number_of_included_samples / self.samples,np.mean(errors),np.std(errors, dtype=np.float64), np.max(errors)-np.min(errors),hist_string))

        self.running_iterations += 1

    def currently_computed_solution_(self):
        if not self.initialized and self.running_iterations == 0:
            return None
        else:
            # Sample final output (ML Sample)
            best_edges = sorted(zip(self.probabilities, list(range(len(self.possible_new_edges)))), reverse=True)[
                         :self.number_of_loop_closures_to_select]
            out_edges = []
            for p, new_edge_idx in best_edges:
                out_edges.append(new_edge_idx)
            return out_edges


class SimulatedAnnealing(GNNBasedLoopClosureBatchSelector):
    def __init__(self, r_lock, min_processing_seconds=30):
        super(SimulatedAnnealing, self).__init__(r_lock, min_processing_seconds)
        # This fallback is done because convex hulling can take a long time for a lot of closures
        self.initial_solution_finder = MaximallySeperatedNodes(r_lock, use_hull=True)
        self.fallback_initial_solution_finder = MaximallySeperatedNodes(r_lock, use_hull=False)
        self.max_number_of_loop_closures_to_use_initial_solution = 200
        self.initial_solution_finder_iterations = 30
        self.fallback_solution_finder_iterations = 100
        self.steps = 0
        self.min_temp = 1e-4

    def set_params(self, params):
        self.initial_solution_finder_iterations = params["simulated_annealing"]["initial_solution_finder_iterations"]
        self.fallback_solution_finder_iterations = params["simulated_annealing"]["fallback_solution_finder_iterations"]
        self.min_temp = params["simulated_annealing"]["min_temp"]
        self.Tmax = params["simulated_annealing"]["max_temp"]

        self.cooling = params["simulated_annealing"]["cooling"]

        self.max_number_of_loop_closures_to_use_initial_solution = params["simulated_annealing"]["max_number_of_loop_closures_to_use_initial_solution"]


    def initialize_method(self):
        self.initial_solution_finder.initialize(self.gnn_model, self.number_of_loop_closures_to_select, self.pose_graph,
                                                self.loop_candidates)
        self.fallback_initial_solution_finder.initialize(self.gnn_model, self.number_of_loop_closures_to_select,
                                                         self.pose_graph,
                                                         self.loop_candidates)
        self.found_initial_solution = False
        self.steps = 0

    def set_verbose(self, verbose):
        super(SimulatedAnnealing, self).set_verbose(verbose)
        self.initial_solution_finder.set_verbose(verbose)
        self.fallback_initial_solution_finder.set_verbose(verbose)

    def find_initial_solution(self):
        self.log_debug("Finding initial solution")
        if len(self.loop_candidates) <= self.max_number_of_loop_closures_to_use_initial_solution:
            for i in range(self.initial_solution_finder_iterations):
                self.initial_solution_finder.step()
            starting_solution = self.initial_solution_finder.currently_computed_solution_()
            method = "Base"
        else:
            for i in range(self.fallback_solution_finder_iterations):
                self.fallback_initial_solution_finder.step()
            starting_solution = self.fallback_initial_solution_finder.currently_computed_solution_()
            method = "Fallback"
        if starting_solution is None:
            self.log_warn("Couldn't find initial solution, falling back")
            starting_solution = np.random.randint(0, len(self.loop_candidates)-1, size=self.number_of_loop_closures_to_select).tolist()
            method = "Random"
        self.log_debug("%s : Starting solution length: %d, Supposed to be %d" % (method, len(starting_solution),self.number_of_loop_closures_to_select))
        self.all_points = []

        keys_to_nodes = dict()
        for node in self.pose_graph.nodes:
            keys_to_nodes[node.key] = node

        for idx, edge in enumerate(self.loop_candidates):
            to_node = keys_to_nodes[edge.key_to]
            from_node = keys_to_nodes[edge.key_from]
            average_x = (to_node.pose.position.x + from_node.pose.position.x) / 2
            average_y = (to_node.pose.position.y + from_node.pose.position.y) / 2
            average_z = (to_node.pose.position.z + from_node.pose.position.z) / 2
            self.all_points.append(np.array([average_x, average_y, average_z]))
        self.all_points = np.array(self.all_points)

        # self.distances = np.zeros((len(self.all_points), len(self.all_points)))
        # for i in range(len(self.all_points)):
        #     for j in range(len(self.all_points)):
        #         if i != j:
        #             self.distances[i, j] = 1 / (np.linalg.norm(self.all_points[i] - self.all_points[j]) + 1e-4)

        self.log_debug("Found initial solution")

        self.state = starting_solution
        self.T = self.Tmax
        self.E = self.energy()
        self.prevState = fast_deepcopy(self.state)
        self.prevEnergy = self.E
        self.best_state = fast_deepcopy(self.state)
        self.best_energy = self.E
        self.edge_idxs_not_in_solution = list(set(range(len(self.possible_new_edges))).difference(self.state))

        self.steps = 1
        self.accepts = 0
        self.improves = 0



    def step(self):
        if not self.found_initial_solution:
            self.find_initial_solution()

        self.take_annealing_step()
        self.found_initial_solution = True

    def take_annealing_step(self):
        self.steps += 1
        T = max(self.Tmax * ((1 - self.cooling) ** self.steps), self.min_temp)
        self.move()
        self.E = self.energy()
        dE = self.E - self.prevEnergy
        if dE > 0.0 and math.exp(-dE / T) < random.random():
            # Restore previous state
            self.state = fast_deepcopy(self.prevState)
            self.E = self.prevEnergy
        else:
            # Accept new state and compare to best state
            self.accepts += 1
            if dE < 0.0:
                self.improves += 1
            self.prevState = fast_deepcopy(self.state)
            self.prevEnergy = self.E
            if self.E < self.best_energy:
                self.best_state = fast_deepcopy(self.state)
                self.best_energy = self.E
        if (self.steps % 100) == 0:
            self.log_progress(
                self.steps, T, self.E, self.accepts / 100.0, self.improves / 100.0)
            self.accepts, self.improves = 0, 0

    def currently_computed_solution_(self):
        if self.found_initial_solution and self.steps >= 1:
            T = max(self.Tmax * ((1 - self.cooling) ** self.steps), self.min_temp)
            self.log_debug("Final Step: {}, E: {}, T:{}".format(self.steps,self.best_energy,T))
            return self.best_state
        else:
            return None



    def move(self, pow=1):
        swapped_out_idx = random.randint(0, len(self.state) - 1)
        old_edge = self.state[swapped_out_idx]

        # Inverse distance weighted
        if len(self.edge_idxs_not_in_solution) < 500:
            jump_idxs = self.edge_idxs_not_in_solution
        else:
            jump_idxs = random.sample(self.edge_idxs_not_in_solution,500)
        distance_weights = 1/ (np.linalg.norm(self.all_points[jump_idxs,:] - self.all_points[old_edge,:],ord=2,axis=1) + 1e-7)

        new_edge = choices(jump_idxs, distance_weights, k=1)[0]


        # Perform the swap
        self.state.pop(swapped_out_idx)
        self.edge_idxs_not_in_solution.remove(new_edge)

        self.state.append(new_edge)
        self.edge_idxs_not_in_solution.append(old_edge)
        #assert len(self.state) == self.num_to_select, f"State size didn't change, M  {len(self.state) }, # choose {self.num_to_select}"
        return (new_edge, old_edge)

    def energy(self):
        edges = []
        edge_attrs = []
        for edge_idx in self.state:
            try:
                edges.append(self.possible_new_edges[edge_idx])
                edge_attrs.append(self.possible_edge_attrs[edge_idx])
            except IndexError:
                self.log_debug("IndexError")
        edges = np.concatenate((self.current_pose_graph_edges,np.array(edges)))
        edge_attrs = np.concatenate((self.current_pose_graph_edge_atrrs,np.array(edge_attrs)))
        return run_model_on_list_of_graphs(self.gnn_model,
                                           [constuct_pytorch_geometric_graph(self.current_pose_graph_nodes, edges, edge_attrs)])[0]

    def log_progress(self, step, T, E, acceptance, improvement):
        self.log_debug(
            'Temp: {Temp:12.5f}  Energy: {Energy:12.2f}  Accept: {Accept:7.2%}  Improve: {Improve:7.2%} S: {Steps:f} '
                .format(Temp=T,
                        Energy=E,
                        Accept=acceptance,
                        Improve=improvement,
                        Steps=step,
                        ))


class DiscretePSO(GNNBasedLoopClosureBatchSelector):
    """
    Discrete Particle Swarm Optimization from "A New Discrete Particle Swarm Optimization Algorithm" by Strasser, Sheppard, and Butcher
    """
    def set_params(self, params):
        self.number_of_particles = params["pso"]["number_of_particles"]
        self.max_velocity = params["pso"]["max_velocity"]
        self.scaling_factor = params["pso"]["scaling_factor"]
        self.interia = params["pso"]["inertia"]
        self.cognitive = params["pso"]["cognitive"]
        self.social = params["pso"]["social"]
        self.topology = params["pso"]["topology"]

        assert self.topology in ["global","ring"]


    def initialize_method(self):
        self.initialize_particles_and_velocities_and_bests()
        self.i = 0


    def step(self):
        #Calculate new velocities and particles
        for p,particle in enumerate(self.particles):
            cognitive_mix = random.uniform(0,self.cognitive)
            social_mix = random.uniform(0,self.social)

            if self.topology == "global":
                topological_best = self.global_best_position
            elif self.topology == "ring":
                next_idx = (p + 1) % self.number_of_particles
                prev_idx = (p - 1) % self.number_of_particles
                if self.particle_best_fitnesses[next_idx] < self.particle_best_fitnesses[prev_idx]:
                    topological_best = self.particles[next_idx]
                else:
                    topological_best = self.particles[prev_idx]

            else:
                raise Exception()

            self.velocities[p] = np.clip(self.velocities[p] * self.interia + cognitive_mix * (self.particle_best_position[p] - particle) + social_mix * (topological_best - particle),-self.max_velocity,self.max_velocity)
            self.particles[p] = self.particles[p] + self.velocities[p]
        self.particles = self.normalize_particles(self.particles)

        #get samples
        samples = self.sample(self.particles)
        #calculate the fitnesses
        fitnesses = self.evaluate_fitness(samples)

        #Find new global best (if it exists)
        current_best = np.argmin(fitnesses)
        if fitnesses[current_best] < self.global_best_fitness:
            new_global_best_found = True
            self.global_best_position = self.particles[current_best]
            self.global_best_fitness = fitnesses[current_best]
            self.global_best_sample = samples[current_best]
        else:
            new_global_best_found = False

        personal_bests_found = 0
        #Find new personal bests (if they exist)
        for i, particle in enumerate(self.particles):
            if fitnesses[i] < self.particle_best_fitnesses[i]:
                self.particle_best_position[i] = self.calculate_new_best(self.particles[i], samples[i], scaling_factor=self.scaling_factor)
                personal_bests_found += 1

        self.i += 1
        self.log_debug("I: {} Global Best: {} New Global {} New Personals {}/{}, Fitnesses N({},{}), Velocity Mag: {}".format(self.i,self.global_best_fitness,new_global_best_found,personal_bests_found,self.number_of_particles,np.mean(fitnesses),np.std(fitnesses),np.linalg.norm(self.velocities)))



    def currently_computed_solution_(self):
        if self.global_best_sample is not None:
            return self.global_best_sample
        else:
            return None

    def initialize_particles_and_velocities_and_bests(self):
        self.global_best_sample = None
        #Particles dimensions are particles x output edge dimension x edge probability
        #for example, self.particles[0,1,2] is the probability of edge 2 being the first output edge for particle 0
        #Velocities are the same dimensions
        self.particles = np.random.rand(self.number_of_particles, self.number_of_loop_closures_to_select,len(self.possible_new_edges))
        self.velocities = np.clip(np.random.normal(0,scale=1.0,size=self.particles.shape),a_min=-self.max_velocity,a_max=self.max_velocity)
        self.particles = self.normalize_particles(self.particles)


        self.particle_best_position = self.particles.copy()
        initial_samples = self.sample(self.particle_best_position)
        self.particle_best_fitnesses = self.evaluate_fitness(initial_samples)


        best_particle_idx = np.argmin(self.particle_best_fitnesses)

        self.global_best_position = self.particle_best_position[best_particle_idx].copy()
        self.global_best_fitness = self.particle_best_fitnesses[best_particle_idx]
        self.global_best_sample = initial_samples[best_particle_idx]


    def normalize_particles(self, particles):
        for i,particle in enumerate(particles):
            for j in range(particle.shape[0]):
                #TODO may not need to clip
                particles[i,j] = np.clip(particles[i,j],0,1)
                particles[i,j] = particles[i,j] / np.sum(particles[i,j])
                #assert np.sum(particles[i,j]) == 1.0, "Particles should be normalized to 1, they are {}".format(np.sum(particles[i,j]))
        return particles

    def sample(self, particles):
        sampled_solutions = []
        for i, particle in enumerate(particles):
            solution = []
            for j in range(particle.shape[0]):
                chosen_edge = choices(range(particle.shape[1]),weights=particles[i,j],k=1)[0]
                solution.append(chosen_edge)
            assert len(solution) == self.number_of_loop_closures_to_select
            sampled_solutions.append(solution)
        return np.array(sampled_solutions)


    def evaluate_fitness(self, samples):
        assert len(samples.shape) == 2, "Samples should be 2d but it's {}, samples: {}".format(samples.shape,samples)
        graphs_to_run = []
        for cur_sample in samples:
            edges = []
            edge_attrs = []
            for edge_idx in cur_sample:
                try:
                    edges.append(self.possible_new_edges[edge_idx])
                    edge_attrs.append(self.possible_edge_attrs[edge_idx])
                except IndexError:
                    self.log_debug("IndexError")
            edges = np.concatenate((self.current_pose_graph_edges,np.array(edges)))
            edge_attrs = np.concatenate((self.current_pose_graph_edge_atrrs,np.array(edge_attrs)))

            graphs_to_run.append(
                constuct_pytorch_geometric_graph(self.current_pose_graph_nodes, edges,
                                                 edge_attrs))
        return run_model_on_list_of_graphs(self.gnn_model, graphs_to_run)

    def calculate_new_best(self,particle, sample, scaling_factor):
        out_best = np.zeros(shape=particle.shape)
        for d, dimension in enumerate(particle):
            dimension_sum = np.sum(dimension)
            for j,probability in enumerate(dimension):
                if sample[d] == j:
                    out_best[d,j] = scaling_factor * probability
                else:
                    out_best[d,j] = probability + (1-scaling_factor) * (dimension_sum - probability)
                assert 0 <= out_best[d,j] < 1
        return out_best




class DiscreteTabuSearch(SimulatedAnnealing):
    def set_params(self, params):
        self.initial_solution_finder_iterations = params["tabu"]["initial_solution_finder_iterations"]
        self.fallback_solution_finder_iterations = params["tabu"]["fallback_solution_finder_iterations"]
        self.max_number_of_loop_closures_to_use_initial_solution = params["tabu"]["max_number_of_loop_closures_to_use_initial_solution"]
        self.num_neighbors = params["tabu"]["num_neighbors"]
        self.max_tabu_length = params["tabu"]["max_tabu_length"]


    def initialize_method(self):
        super(DiscreteTabuSearch, self).initialize_method()
        self.tabu_list = dict()

        #Not used
        self.Tmax = 0

        self.find_initial_solution()
        self.found_initial_solution = True

    def undo_move(self, move):
        new_edge,old_edge = move
        self.state.remove(new_edge)
        self.state.append(old_edge)

        self.edge_idxs_not_in_solution.append(new_edge)
        self.edge_idxs_not_in_solution.remove(old_edge)

    def reapply_move(self,move):
        new_edge,old_edge = move
        self.state.remove(old_edge)
        self.state.append(new_edge)

        self.edge_idxs_not_in_solution.append(old_edge)
        self.edge_idxs_not_in_solution.remove(new_edge)



    def step(self):
        best_candidate_move = None
        best_candidate_fitness = float("inf")
        tabu_rejections = 0
        for i in range(self.num_neighbors):
            cur_move = self.move(pow=3)
            cur_energy = self.energy()
            if (cur_move not in self.tabu_list):
                if (cur_energy < best_candidate_fitness):
                    best_candidate_fitness = cur_energy
                    best_candidate_move = cur_move
            else:
                tabu_rejections += 1
            self.undo_move(cur_move)
        if best_candidate_move is None:
            self.log_warn("Tabu Search Couldn't Make Move")
            return None
        self.reapply_move(best_candidate_move)
        found_new_best = False
        if best_candidate_fitness <= self.best_energy:
            self.best_state = deepcopy(self.state)
            self.best_energy = best_candidate_fitness
            found_new_best = True
        self.tabu_list[best_candidate_move] = 0
        for move, count in self.tabu_list.items():
            self.tabu_list[move] += 1
            if self.tabu_list[move] > self.max_tabu_length:
                del self.tabu_list[move]
        self.steps += 1
        self.log_debug("S: {} Best: {} Bound: {} Rejected: {}".format(self.steps,found_new_best,self.best_energy,tabu_rejections))


