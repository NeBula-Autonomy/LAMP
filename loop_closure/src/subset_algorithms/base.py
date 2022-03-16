import numpy as np
import rospy
from threading import Event, Lock
import traceback


class LoopClosureBatchSelector(object):
    def __init__(self,r_lock, min_processing_seconds=30):
        self.run_lock = r_lock
        self.is_processing = False
        self.processing_start_time = None
        self.min_processing_seconds = min_processing_seconds
        self.stale_solution = True

    def set_params(self,params):
        pass

    def set_verbose(self,verbose):
        # Verbosity goes from 0, (Nothing), 1 (Only errors), 2 errors and info,  3 (debug info)
        self.verbose = verbose

    def set_min_processing_seconds(self, min_processing_seconds):
        self.min_processing_seconds = min_processing_seconds

    def initialize(self, gnn_model, number_of_loop_closures_to_select, pose_graph, loop_candidates):
        self.stop_processing()
        self.base_initialize(gnn_model, number_of_loop_closures_to_select, pose_graph, loop_candidates)
        self.initialize_method()
        self.stale_solution = False
        self.start_processing()

    def base_initialize(self, gnn_model, number_of_loop_closures_to_select, pose_graph, loop_candidates):
        self.processing_start_time = rospy.get_rostime().secs
        self.gnn_model = gnn_model
        self.number_of_loop_closures_to_select = number_of_loop_closures_to_select
        self.pose_graph = pose_graph
        self.loop_candidates = loop_candidates

    def stop_processing(self):
        self.is_processing = False

    def start_processing(self):
        self.is_processing = True

    def initialize_method(self):
        raise NotImplementedError()

    def run(self):
        while True:
            try:
                if self.is_processing:
                    #Split these up to avoid locking
                    if self.run_lock.acquire(blocking=False):
                        self.step()
                        self.run_lock.release()
            except Exception as e:
                rospy.logerr("Error in Run: " +  str((traceback.format_exc())))
                try:
                    self.run_lock.release()
                except RuntimeError:
                    pass

    def step(self):
        raise NotImplementedError()

    def currently_computed_solution_(self):
        raise NotImplementedError()

    def get_current_solution(self):
        cur = rospy.get_rostime().secs
        solution = self.currently_computed_solution_()
        if self.processing_start_time is None or (cur - self.processing_start_time) <= self.min_processing_seconds:
            if self.processing_start_time is not None and self.min_processing_seconds is not None:
                self.log_debug("Time Lockout %d s remaining" % (self.min_processing_seconds - (cur - self.processing_start_time)))
            return None
        elif solution is None:
            self.log_debug("Solution not ready")
            return None
        else:
            self.stale_solution = True
            self.processing_start_time = rospy.get_rostime().secs
            return solution

    def log_debug(self, s):
        if self.verbose >= 3:
            rospy.loginfo("Batcher Alg: " + s)

    def log_info(self, s):
        if self.verbose >= 2:
            rospy.loginfo("Batcher Alg: " + s)

    def log_warn(self, s):
        if self.verbose >= 2:
            rospy.logwarn("Batcher Alg: " + s)

    def log_error(self, s):
        if self.verbose >= 1:
            rospy.logerr("Batcher Alg: " + s)


class GNNBasedLoopClosureBatchSelector(LoopClosureBatchSelector):
    def __init__(self, r_lock, min_processing_seconds=30):
        super(GNNBasedLoopClosureBatchSelector,self).__init__(r_lock, min_processing_seconds=min_processing_seconds)

    def initialize(self, gnn_model, number_of_loop_closures_to_select, pose_graph, loop_candidates):
        self.stop_processing()
        super(GNNBasedLoopClosureBatchSelector,self).base_initialize(gnn_model, number_of_loop_closures_to_select, pose_graph, loop_candidates)
        nodes = []
        self.node_ids_to_seq_id = dict()

        for i, node in enumerate(pose_graph.nodes):
                                                       x = node.pose.position.x
                                                       y = node.pose.position.y
                                                       z = node.pose.position.z

                                                       cx = node.covariance[0]
                                                       cy = node.covariance[7]
                                                       cz = node.covariance[14]
                                                       node_features = np.zeros(7)
                                                       node_features[0] = float(cx)
                                                       node_features[1] = float(cy)
                                                       node_features[2] = float(cz)
                                                       node_features[3] = float(x)
                                                       node_features[4] = float(y)
                                                       node_features[5] = float(z)
                                                       node_features[6] = 1.0
                                                       nodes.append(node_features)
                                                       self.node_ids_to_seq_id[node.key] = i

        edges = []
        edge_attrs = []
        for edge in pose_graph.edges:
            try:
                     edge_1 = self.node_ids_to_seq_id[edge.key_from]
                     edge_2 = self.node_ids_to_seq_id[edge.key_to]

                     edges.append((edge_1, edge_2))
                     edge_attr = np.zeros(4)
                     edge_attr[int(edge.type) - 1] = 1
                     edge_attrs.append(edge_attr)
            except KeyError:
                self.log_error("Error when handling loop closure bundle, {}, {} not found in pose graph".format(edge.key_from,edge.key_to))
            except IndexError as e:
                self.log_warn("Edge type not understood by batcher " + str(e))
        self.current_pose_graph_nodes = np.array(nodes)
        self.current_pose_graph_edges = np.array(edges)
        self.current_pose_graph_edge_atrrs = np.array(edge_attrs)
        self.possible_new_edges = []
        self.possible_edge_attrs = []
        for edge in loop_candidates:
            try:
                edge_1 = self.node_ids_to_seq_id[edge.key_from]
                edge_2 = self.node_ids_to_seq_id[edge.key_to]

                self.possible_new_edges.append((edge_1, edge_2))
                edge_attr = np.zeros(4)
                edge_attr[int(edge.type) - 1] = 1
                self.possible_edge_attrs.append(edge_attr)
            except KeyError as e:
                self.log_error("Error when handling loop closure bundle, can't find key " + str(e))
            except IndexError as e:
                self.log_warn("Edge type not understood by batcher " + str(e))

        self.initialize_method()
        self.stale_solution = False
        self.start_processing()
