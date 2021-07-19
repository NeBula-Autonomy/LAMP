from gnn.gnn_model import LoopClosureGNN
import torch
from pose_graph_msgs.msg import PoseGraph, LoopCandidateArray, LoopComputationStatus
import rospy
from copy import deepcopy
import traceback
import os

from subset_algorithms.heuristic import MaximallySeperatedNodes, MaximumCovarianceNodes, Random
from subset_algorithms.gnn import BinomialCEM, SimulatedAnnealing, DiscretePSO, DiscreteTabuSearch
import threading
from concurrent.futures import ThreadPoolExecutor
import rospkg



def edge_selection_algorithm_dispatch(algorithm_name, lock):
    if algorithm_name == "cem":
        return BinomialCEM(lock)
    elif algorithm_name == "random":
        return Random(lock)
    elif algorithm_name == "heuristic_maximally_separated_nodes":
        return MaximallySeperatedNodes(lock)
    elif algorithm_name == "heuristic_maximum_covariance":
        return MaximumCovarianceNodes(lock)
    elif algorithm_name == "simulated_annealing":
        return SimulatedAnnealing(lock)
    elif algorithm_name == "pso":
        return DiscretePSO(lock)
    elif algorithm_name == "tabu":
        return DiscreteTabuSearch(lock)
    else:
        raise Exception("Edge subset selection algorithm unknown {algorithm_name}")


class LoopClosureBatcher:
    def __init__(self, params):
        rospack = rospkg.RosPack()
        cur_path = rospack.get_path("loop_closure")

        self.loop_closures_to_choose = params["number_of_loop_closures"]
        self.min_queue_size = params["min_queue_size"]
        self.min_processing_time_seconds = params["min_processing_time_seconds"]
        self.verbose = params["verbose"]
        try:
            with open(os.path.join(cur_path, "model", "current_gnn_model.pkl"), "rb") as f:
                mdict = torch.load(f)
        except Exception as e:
            self.log_warn("Couldn't load GPU model due to "  + str(e) + ", loading on CPU")
            with open(os.path.join(cur_path, "model", "current_gnn_model.pkl"), "rb") as f:
                mdict = torch.load(f, map_location=torch.device('cpu'))
        self.model = LoopClosureGNN(64, 7, 4, 1)
        self.model.load_state_dict(mdict)
        self.model.eval()

        self.pg_raw = None
        self.current_pose_graph_nodes = None
        self.current_pose_graph_edges = None
        self.current_pose_graph_edge_atrrs = None

        self.solution_lock = threading.RLock()
        self.queue_update_lock = threading.RLock()
        self.edge_subset_algorithm = edge_selection_algorithm_dispatch(params["algorithm"], self.solution_lock)

        self.edge_subset_algorithm.set_min_processing_seconds(self.min_processing_time_seconds)
        self.edge_subset_algorithm.set_verbose(self.verbose)
        self.edge_subset_algorithm.set_params(params)

        self.loop_candidates = []
        self.thread_pool = ThreadPoolExecutor(max_workers=2)
        self.current_edge_future = None
        self.previous_batch = None
        self.current_pose_graph = None

    def log_debug(self, s):
        if self.verbose >= 3:
            rospy.loginfo("Batcher Node: " + s)

    def log_info(self, s):
        if self.verbose >= 2:
            rospy.loginfo("Batcher Node: " + s)

    def log_warn(self, s):
        if self.verbose >= 2:
            rospy.logwarn("Batcher Node: " + s)

    def log_error(self, s):
        if self.verbose >= 1:
            rospy.logerr("Batcher Node: " + s)

    def create_publishers(self):
        self.loop_publisher = rospy.Publisher(rospy.resolve_name("~prioritized_loop_candidates"), LoopCandidateArray, queue_size=100)

    def register_callbacks(self):
        """
        Handle setting up ROS callbacks and services
        """
        rospy.Subscriber(rospy.resolve_name("~loop_candidates"), LoopCandidateArray,
                         self.add_loop_closure_to_queue)
        rospy.Subscriber(rospy.resolve_name("~pose_graph"), PoseGraph,
                         self.handle_pose_graph_message)
        rospy.Subscriber(rospy.resolve_name("~loop_computation_status"), LoopComputationStatus, self.handle_status)
        rospy.Subscriber(rospy.resolve_name("~output_loop_closures"), LoopCandidateArray, self.remove_loop_closures_from_queue)

    def remove_loop_closures_from_queue(self, loop_candidate_array):
        with self.queue_update_lock:
            size_before = len(self.loop_candidates)
            self.loop_candidates = [candidate for candidate in self.loop_candidates if candidate not in loop_candidate_array.candidates]
            size_after = len(self.loop_candidates)
            if size_before - size_after != 0:
                self.log_debug("Batcher Removed " + str(size_before - size_after) + " elements.")

    def add_loop_closure_to_queue(self, loop_candidate_array):
        with self.queue_update_lock:
            self.loop_candidates.extend(loop_candidate_array.candidates)
            self.log_info("Batcher received :" + str(len(loop_candidate_array.candidates)) + " Queue Size: " + str(
                len(self.loop_candidates)))

        if self.solution_lock.acquire():
            have_enough_candidates = len(self.loop_candidates) > self.min_queue_size
            # If we have enough loop closures then start processing
            if self.current_edge_future is None and have_enough_candidates:
                    self.start_finding_new_batch()
                    self.current_edge_future = self.thread_pool.submit(self.edge_subset_algorithm.run)

            # If there were previously not enough edges to do selection but now there are, restart processing
            elif not self.edge_subset_algorithm.is_processing and len(self.loop_candidates) > self.min_queue_size:
                self.start_finding_new_batch()
            elif have_enough_candidates and self.edge_subset_algorithm.stale_solution:
                self.start_finding_new_batch()
            self.solution_lock.release()


    def handle_pose_graph_message(self, pose_graph):
        """
        update internal pose graph representation when a new pose graph is published
        :return:
        """
        if self.current_pose_graph is None or  pose_graph.header.stamp >= self.current_pose_graph.header.stamp:
            self.current_pose_graph = pose_graph

    def start_finding_new_batch(self):
        working_loop_candidates = deepcopy(self.loop_candidates)
        if (self.loop_closures_to_choose < 1):
            number_of_loop_closures_to_choose = max(1, int(
                float(len(working_loop_candidates)) * self.loop_closures_to_choose))
        else:
            number_of_loop_closures_to_choose = min(len(working_loop_candidates), self.loop_closures_to_choose)
        if len(working_loop_candidates) < self.min_queue_size:
            self.edge_subset_algorithm.stop_processing()
        else:
            self.log_debug("Going to find  " + str(number_of_loop_closures_to_choose) + ", Queue Size: " + str(len(working_loop_candidates)) + ", # to choose " + str(self.loop_closures_to_choose))
            self.edge_subset_algorithm.initialize(self.model, number_of_loop_closures_to_choose,
                                                  self.current_pose_graph, working_loop_candidates)

    def handle_status(self, status):
        """
        Handle the loop closure computation status
        """
        if (status.type == status.COMPLETED_ALL):
            with self.solution_lock:
                if self.edge_subset_algorithm.stale_solution:
                    #self.log_debug("Stale Solution")
                    return None
                outbound_edge_indexes = self.edge_subset_algorithm.get_current_solution()
                if outbound_edge_indexes is None:
                    return None
                else:
                        self.previous_batch = outbound_edge_indexes
                        try:
                            self.edge_subset_algorithm.stop_processing()
                            out_edges = []
                            out_bundle = LoopCandidateArray()
                            out_bundle.originator = 1

                            for outbound_edge_index in outbound_edge_indexes:
                                try:
                                    edge = self.edge_subset_algorithm.loop_candidates[outbound_edge_index]
                                    out_edges.append(edge)
                                except IndexError:
                                    self.log_warn("Outbound edge index is not in loop candidates")

                            out_bundle.candidates = out_edges

                            for edge in out_edges:
                                self.loop_candidates = [cur_edge for cur_edge in self.loop_candidates if
                                                        cur_edge.key_from != edge.key_from or cur_edge.key_to != edge.key_to or cur_edge.type != edge.type]

                            self.log_info("Out " + str(len(out_edges)) + ", Queue Size: " + str(len(self.loop_candidates)))
                            self.start_finding_new_batch()

                            self.loop_publisher.publish(out_bundle)

                        except Exception as e:
                            self.log_error("Failed to make batch {traceback.format_exc()}")
        else:
            self.log_error("Unkown status {status}")
