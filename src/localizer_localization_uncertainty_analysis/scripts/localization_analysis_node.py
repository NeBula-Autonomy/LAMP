#!/usr/bin/env python
import rospy

from pose_graph_msgs.msg import *

import numpy as np
import math
import copy
import os
import pdb


class PoseGraphModifier(object):
    def __init__(self):
        # Parameters
        self.odom_att_sigma = 0.001
        self.odom_pos_sigma = 0.04

    def modify_pg(self, pg):
        for itr in range(len(pg.edges)):
            if pg.edges[itr].type == PoseGraphEdge.ODOM:
                pg.edges[itr] = self.modify_odom_edge(pg.edges[itr])

        return copy.deepcopy(pg)

    def modify_odom_edge(self, edge):
        modified_edge = edge
        modified_edge.covariance = self.generate_cov_matrix(self.odom_att_sigma,
                                                            self.odom_pos_sigma)
        return modified_edge

    def update_pg_values(self, pg_modified, pg_optimized):
        pg_updated = copy.deepcopy(pg_modified)
        for itr in range(len(pg_updated.nodes)):
            for node in pg_optimized.nodes:
                if pg_updated.nodes[itr].key == node.key:
                    pg_updated.nodes[itr].covariance = node.covariance
                    break

        # Temporary solution for zero valued covariance of artifact node
        artifact_key_dict = {}
        for edge in pg_updated.edges:
            if edge.type == PoseGraphEdge.ARTIFACT:
                artifact_key_dict[edge.key_to] = edge.key_from

        for itr in range(len(pg_updated.nodes)):
            if pg_updated.nodes[itr].key in artifact_key_dict.keys():
                if sum(pg_updated.nodes[itr].covariance) < 0.001:
                    rospy.logwarn("Artifact %s does not have covariance value", pg_updated.nodes[itr].ID)
                    # for node in pg_updated.nodes:
                    #     if node.key == artifact_key_dict[pg_updated.nodes[itr].key]:
                    #         pg_updated.nodes[itr].covariance = copy.deepcopy(node.covariance)
                    #         break

        return pg_updated

    def generate_cov_matrix(self, att_sigma, pos_sigma):
        cov_matrix = [0] * 36
        cov_matrix[0] = cov_matrix[7] = cov_matrix[14] = math.pow(att_sigma, 2)
        cov_matrix[21] = cov_matrix[28] = cov_matrix[35] = math.pow(pos_sigma, 2)
        return cov_matrix

class PoseGraphHandler(object):
    def __init__(self):
        # Objects
        self.pg_modififier = PoseGraphModifier()
        # Publisher and Subscriber
        sub_pose_graph = rospy.Subscriber('~pose_graph',
                                          PoseGraph,
                                          self.pg_callback)
        sub_opt_pg = rospy.Subscriber('~optimized_values',
                                      PoseGraph,
                                      self.optimized_pg_callback)
        self.pub_pg_tobe_optimized = rospy.Publisher('~pose_graph_to_optimize',
                                                     PoseGraph,
                                                     queue_size=10)

        # Class variable
        self.current_pg = None
        self.current_optimized_pg = None
        self.b_new_artifact = False
        self.artifact_number = 0
        self.pub_time = rospy.Time.now()

        rospy.loginfo("PoseGraphHandler is initialized")

    def get_pg(self):
        return copy.deepcopy(self.current_pg)

    def get_optimized_pg(self):
        return copy.deepcopy(self.current_optimized_pg)

    def is_new_artifact(self):
        if self.b_new_artifact:
            self.b_new_artifact = False
            return True
        else:
            return False

    def optimized_pg_callback(self, msg):
        rospy.loginfo("PoseGraphHandler: optimized_pose_graph is received from subscriber!")
        rospy.loginfo("PoseGraphHandler: Optimization time is %f seconds", (rospy.Time.now() - self.pub_time).to_sec())
        self.current_optimized_pg = msg

    def pg_callback(self, msg):
        rospy.loginfo_once("PoseGraphHandler: pose graph is received from subscriber!")
        self.current_pg = msg
        artifact_num = self.count_artifact_edge(msg)
        if artifact_num > self.artifact_number:
            self.b_new_artifact = True
        self.artifact_number = artifact_num

    def modify_pg(self):
        return self.pg_modififier.modify_pg(self.current_pg)

    def optimize(self, pg):
        self.pub_pg_tobe_optimized.publish(pg)
        self.pub_time = rospy.Time.now()
        rospy.loginfo("PoseGraphHandler: publishing current pg to optimizer")

    def update_pg_values(self, pg_mod, pg_opt):
        return self.pg_modififier.update_pg_values(pg_mod, pg_opt)

    def count_artifact_edge(self, pg):
        artifact_num = 0
        for edge in pg.edges:
            if edge.type == PoseGraphEdge.ARTIFACT:
                artifact_num += 1
        return artifact_num

class ConfidenceEstimator:
    def __init__(self):
        # parameters
        self.acceptance_radius = 5.0
        self.confidence_const = 1.0

        self.confidence_dict = {}  # key: percentage

        rospy.loginfo("ConfidenceEstimator is initialized")

    def estimate_artifact_confidence(self, pg):
        rospy.loginfo("ConfidenceEstimator: Estimating artifact confidence")
        rospy.loginfo("id x y z error radius scorability")
        for node in pg.nodes:
            if node.ID != "odom_node":  # is artifact?
                self.confidence_dict[node.ID] = self.get_confidence_from_cov(node.covariance)
                radius, err = self.calculate_eigen_value(node.covariance)
                rospy.loginfo("%s %.2f %.2f %.2f %.2f %.2f %.2f", node.ID, node.pose.position.x, node.pose.position.y, node.pose.position.z, err, radius, self.confidence_dict[node.ID])

    def get_confidence_from_cov(self, cov):
        # get volume
        radius, max_err = self.calculate_eigen_value(cov)

        k_score = 5.0 / max_err
        p_score = 1 - np.exp(-1 * k_score / 2.0)

        return p_score

    def get_confidence_dict(self):
        return self.confidence_dict

    def calculate_eigen_value(self, covariance):
        if len(covariance) == 36:
            cov_mat = np.array(covariance).reshape(6, 6)
            # if b_gtsam_cov:
            cov_mat = cov_mat[-3:,-3:]
        else:
            cov_mat = np.array(covariance).reshape(3, 3)
        s, v = np.linalg.eig(cov_mat)
        v[:, 0] /= np.linalg.norm(v[:, 0])
        v[:, 1] /= np.linalg.norm(v[:, 1])
        v[:, 2] /= np.linalg.norm(v[:, 2])

        if np.cross(v[:, 0], v[:, 1]).dot(v[:, 2]) < 0:
            # Make it right-handed
            v = v[:, [1, 0, 2]]
            s[0], s[1] = s[1], s[0]

        # Set ellipse scale
        k = 1.198  #2.296  # 68.26%
        # k = 11.82  # 99.74%
        x_rad = k * np.sqrt(s[0])
        y_rad = k * np.sqrt(s[1])
        z_rad = k * np.sqrt(s[2])

        radius = np.sqrt(x_rad * x_rad + y_rad * y_rad + z_rad * z_rad)
        max_err = np.sqrt(s.max())

        return radius, max_err

class LocalizationAnalysis:
    def __init__(self):
        # Read parameters
        self.data_rate = 0.1
        self.optimization_interval = 30.0  # second

        # Setup modules: PoseGraph Manager, Planning, ArtifactManager
        self.pg_handler = PoseGraphHandler()
        self.conf_estimator = ConfidenceEstimator()

        # Setup Publisher and Subscriber
        self.pub_mod_pg = rospy.Publisher('modified_pg',
                                          PoseGraph,
                                          queue_size=10)

        # Global variables
        self.last_opt_time = rospy.Time.now()


        rospy.loginfo("LocalizationAnalysis: is initialized!")

    def wait_for_pg(self):
        r = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            if self.pg_handler.get_pg():
                rospy.loginfo("LocalizationAnalysis: Received PG message")
                break
            r.sleep()

    def run(self):
        r = rospy.Rate(self.data_rate)
        self.wait_for_pg()
        while not rospy.is_shutdown():
            if self.is_optimize_pg():
                modified_pg = self.pg_handler.modify_pg()
                self.pg_handler.optimize(modified_pg)
                self.last_opt_time = rospy.Time.now()
                rospy.sleep(2.0)  # wait for the optimizer
                self.publish_modified_pg(self.pg_handler.get_optimized_pg(), modified_pg)
            r.sleep()

    def publish_modified_pg(self, pg_optimized, pg_modified):
        combined_pg = self.pg_handler.update_pg_values(pg_modified, pg_optimized)
        self.pub_mod_pg.publish(combined_pg)
        self.conf_estimator.estimate_artifact_confidence(self.pg_handler.get_pg())
        self.conf_estimator.estimate_artifact_confidence(combined_pg)

    def is_optimize_pg(self):
        if (rospy.Time.now() - self.last_opt_time).secs > self.optimization_interval:
            rospy.loginfo("LocalizationAnalysis: Optimizing after %.1f second!", self.optimization_interval)
            return True
        if self.pg_handler.is_new_artifact():
            rospy.loginfo("LocalizationAnalysis: Optimizing after new artifact")
            return True
        #  TODO: use this function to only optimize the pg when needed
        return False




def main():
    rospy.init_node('localization_analysis_node')

    # Instantiate server
    localization_analysis = LocalizationAnalysis()

    try:
        localization_analysis.run()
    except Exception as e:
        rospy.logerr("Exception received while running the LocalizationAnalysis: %s", e)

    rospy.spin()

if __name__ == '__main__':
    main()
