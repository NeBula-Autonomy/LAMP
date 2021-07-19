from __future__ import division


import random

import numpy as np
from scipy.spatial.qhull import ConvexHull
from scipy.spatial.distance import pdist
import rospy
from subset_algorithms.base import LoopClosureBatchSelector
import heapq
import random


class MaximallySeperatedNodes(LoopClosureBatchSelector):
    def __init__(self, r_lock,min_processing_seconds=30,iterations=100, use_hull=True):
        super(MaximallySeperatedNodes,self).__init__(r_lock,min_processing_seconds)
        self.iterations = iterations
        self.use_hull = use_hull

    def initialize_method(self):
        self.computed = False
        self.best_fitness = float("-inf")
        self.best_points = None

    def step(self):
        if not self.computed:
            self.log_debug("Creating Batch")
            self.points_to_indexes = dict()
            all_points = []
            accepted_points = []

            keys_to_nodes = dict()
            for node in self.pose_graph.nodes:
                keys_to_nodes[node.key] = node
            for idx, edge in enumerate(self.loop_candidates):
                to_node = keys_to_nodes[edge.key_to]
                from_node = keys_to_nodes[edge.key_from]
                average_x = (to_node.pose.position.x + from_node.pose.position.x) / 2.0
                average_y = (to_node.pose.position.y + from_node.pose.position.y) / 2.0
                average_z = (to_node.pose.position.z + from_node.pose.position.z) / 2.0
                all_points.append([average_x, average_y, average_z])
                self.points_to_indexes[(average_x, average_y, average_z)] = idx
            self.remaining_points = all_points

            if self.use_hull:
                self.log_debug("Convex Hulling points")
                try:
                    hull = ConvexHull(all_points)

                    for vertex in sorted(hull.vertices, reverse=True):
                        accepted_points.append(self.remaining_points[vertex])

                    # Have do this in two loops so we don't change the size of the list
                    for vertex in sorted(hull.vertices, reverse=True):
                        del self.remaining_points[vertex]

                    self.initially_accepted_points = accepted_points[:self.number_of_loop_closures_to_select]
                    self.best_points = self.initially_accepted_points
                    self.log_debug(
                        "Maximally Separate Heuristic points remaining after hull: %d" %  (self.number_of_loop_closures_to_select - len(accepted_points)))
                except Exception as e:
                    self.log_error("Couldn't make hull: " + str(e))
                    self.initially_accepted_points = []
            else:
                self.initially_accepted_points = []
            # Greedy solution
            # while len(accepted_points) < self.number_of_loop_closures_to_select:
            #     best_idx = None
            #     best_distance = float("-inf")
            #     for i, point in enumerate(remaining_points):
            #         dist = 0
            #         for point2 in accepted_points:
            #             dist += np.linalg.norm(np.array(point) - np.array(point2), ord=2)
            #         if dist > best_distance:
            #             best_idx = i
            #             best_distance = dist
            #     accepted_points.append(remaining_points[best_idx])
            #     del remaining_points[best_idx]
            #     self.log_debug(
            #         f"Maximally separate point found, {len(accepted_points)} / {self.number_of_loop_closures_to_select}")
            self.needed_extra_points = self.number_of_loop_closures_to_select - len(self.initially_accepted_points)
            # Randomized solution
            self.remaining_points = np.array(self.remaining_points)

        all_indexes = np.array(range(self.remaining_points.shape[0]))
        if self.needed_extra_points > 0:
            #indexes = np.array(random.sample(all_indexes, self.needed_extra_points))
            indexes = np.random.choice(all_indexes,size=self.needed_extra_points,replace=False)
            selected_remaining_points = self.remaining_points[indexes, :]
            if self.initially_accepted_points != []:
                selected_points = np.concatenate((selected_remaining_points, self.initially_accepted_points), axis=0)
            else:
                selected_points = selected_remaining_points
            #fitness = np.sum(pdist(selected_points))
            centroid = np.average(selected_points,axis=0)

            fitness = np.sum(np.linalg.norm(selected_points - centroid.reshape(3,1).T,ord=2,axis=1))
            if fitness > self.best_fitness:
                self.best_fitness = fitness
                self.best_points = selected_points
        self.computed = True

    def currently_computed_solution_(self):
        if self.computed:
            out_idxs = []
            for point in self.best_points:
                out_idxs.append(self.points_to_indexes[tuple(point)])
            return out_idxs
        else:
            return None


class MaximumCovarianceNodes(LoopClosureBatchSelector):
    def initialize_method(self):
        self.computed = False

    def step(self):
        if not self.computed:
            all_covariances = []
            for idx, edge in enumerate(self.loop_candidates):
                for node in self.pose_graph.nodes:
                    if node.key == edge.key_from:
                        from_node = node
                        break
                for node in self.pose_graph.nodes:
                    if node.key == edge.key_to:
                        to_node = node
                        break
                all_covariances.append((sum(to_node.covariance) + sum(from_node.covariance), idx))

            # all_covariances.sort(reverse=True)  # Want highest covariances first
            best_covariances = heapq.nlargest(self.number_of_loop_closures_to_select, all_covariances)
            self.out_idxs = []
            for cov, idx in best_covariances:
                self.out_idxs.append(idx)
            self.computed = True

    def currently_computed_solution_(self):
        if self.computed:
            return self.out_idxs


class Random(LoopClosureBatchSelector):
    def initialize_method(self):
        pass

    def step(self):
        pass

    def currently_computed_solution_(self):
        return np.random.randint(0, len(self.loop_candidates)-1, size=self.number_of_loop_closures_to_select).tolist()
