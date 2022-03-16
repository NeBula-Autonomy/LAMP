#!/usr/bin/env python
"""Test python bridge."""

import unittest

import rostest
import rospy

import networkx as nx

from geometry_msgs.msg import Pose
from pose_graph_msgs.msg import *
from pose_graph_msgs import bridge


class ROS2NXTest(object):

    def test_preserve_structure_on_nominal_input(self):
        msg = PoseGraph()
        msg.nodes.append(PoseGraphNode(key=0))
        msg.nodes.append(PoseGraphNode(key=1))
        msg.nodes.append(PoseGraphNode(key=2))
        msg.nodes.append(PoseGraphNode(key=3))
        msg.edges.append(PoseGraphEdge(key_from=0, key_to=1))
        msg.edges.append(PoseGraphEdge(key_from=1, key_to=2))

        g = bridge.to_networkx(msg)
        self.assertEqual(4, g.number_of_nodes())
        self.assertEqual(2, g.number_of_edges())

    def test_success_on_empty_graph(self):
        msg = PoseGraph()
        msg.nodes = []
        msg.edges = []

        g = bridge.to_networkx(msg)
        self.assertEqual(0, g.number_of_nodes())
        self.assertEqual(0, g.number_of_edges())

    def test_ignore_invalid_edge(self):
        msg = PoseGraph()
        msg.nodes = []
        msg.edges = [PoseGraphEdge(key_from=0, key_to=5)]

        # Do not throw exception for invalid edges
        g = bridge.to_networkx(msg)

    def test_preserve_vertex_property(self):
        msg = PoseGraph()
        msg.nodes = [PoseGraphNode()]
        msg.nodes[0].key = 12
        msg.nodes[0].ID = 'my-uuid'
        msg.nodes[0].pose.position.x = 39

        g = bridge.to_networkx(msg)
        self.assertEqual(g.nodes[12]['key'], 12)
        self.assertEqual(g.nodes[12]['ID'], 'my-uuid')
        self.assertEqual(g.nodes[12]['pose'].position.x, 39)

    def test_preserve_edge_property(self):
        msg = PoseGraph()
        msg.nodes.append(PoseGraphNode(key=0))
        msg.nodes.append(PoseGraphNode(key=1))
        msg.edges = [PoseGraphEdge()]
        msg.edges[0].key_from = 0
        msg.edges[0].key_to = 1
        msg.edges[0].type = PoseGraphEdge.LOOPCLOSE
        msg.edges[0].pose.position.x = 39

        g = bridge.to_networkx(msg)
        self.assertEqual(g.edges[0, 1]['key_from'], 0)
        self.assertEqual(g.edges[0, 1]['key_to'], 1)
        self.assertEqual(g.edges[0, 1]['type'], PoseGraphEdge.LOOPCLOSE)
        self.assertEqual(g.edges[0, 1]['pose'].position.x, 39)


class BridgeTest(ROS2NXTest,
                 unittest.TestCase):
    pass


if __name__ == '__main__':
    rospy.init_node('test_node')
    rostest.rosrun('pose_graph_msgs', 'test_bridge', BridgeTest)
