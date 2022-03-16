#!/usr/bin/env python
"""Test python bridge."""

import unittest

import rostest
import rospy

import networkx as nx

from geometry_msgs.msg import PoseWithCovariance
from roadmap_msgs.msg import *
from roadmap_msgs import bridge


class ROS2NXTest(object):

    def init_array(self, type, n=1):
        return [type() for i in range(n)]

    def test_preserve_structure_on_nominal_input(self):
        roadmap = Roadmap()
        roadmap.nodes.append(RoadmapNode(index=0))
        roadmap.nodes.append(RoadmapNode(index=1))
        roadmap.nodes.append(RoadmapNode(index=2))
        roadmap.nodes.append(RoadmapNode(index=3))
        roadmap.edges.append(RoadmapEdge(u=0, v=1))
        roadmap.edges.append(RoadmapEdge(u=1, v=2))

        g = bridge.to_networkx(roadmap)
        self.assertEqual(4, g.number_of_nodes())
        self.assertEqual(2, g.number_of_edges())

    def test_success_on_empty_graph(self):
        roadmap = Roadmap()
        roadmap.nodes = []
        roadmap.edges = []

        g = bridge.to_networkx(roadmap)
        self.assertEqual(0, g.number_of_nodes())
        self.assertEqual(0, g.number_of_edges())

    def test_fails_on_adding_invalid_edge(self):
        roadmap = Roadmap()
        roadmap.nodes = []
        roadmap.edges = [RoadmapEdge(u=0, v=5)]

        with self.assertRaises(ValueError):
            g = bridge.to_networkx(roadmap)

    def test_preserve_vertex_property(self):
        roadmap = Roadmap()
        roadmap.nodes = self.init_array(RoadmapNode, n=1)
        roadmap.nodes[0].index = 0
        roadmap.nodes[0].pose.pose.position.x = 39
        roadmap.nodes[0].properties = self.init_array(RoadmapProperty, n=3)
        roadmap.nodes[0].properties[0].key = 'data_str'
        roadmap.nodes[0].properties[0].value = 'this is a string'
        roadmap.nodes[0].properties[1].key = 'data_int'
        roadmap.nodes[0].properties[1].value = 9
        roadmap.nodes[0].properties[2].key = 'data_float'
        roadmap.nodes[0].properties[2].value = 0.54

        g = bridge.to_networkx(roadmap)
        self.assertEqual(g.nodes[0]['index'], 0)
        self.assertEqual(g.nodes[0]['pose'].pose.position.x, 39)
        self.assertEqual(g.nodes[0]['properties']['data_str'], 'this is a string')
        self.assertTrue(isinstance(g.nodes[0]['properties']['data_str'], basestring))
        self.assertEqual(g.nodes[0]['properties']['data_int'], 9)
        self.assertTrue(isinstance(g.nodes[0]['properties']['data_int'], int))
        self.assertEqual(g.nodes[0]['properties']['data_float'], 0.54)
        self.assertTrue(isinstance(g.nodes[0]['properties']['data_float'], float))

    def test_preserve_edge_property(self):
        roadmap = Roadmap()
        roadmap.nodes.append(RoadmapNode(index=0))
        roadmap.nodes.append(RoadmapNode(index=1))
        roadmap.edges = [RoadmapEdge()]
        roadmap.edges[0].u = 0
        roadmap.edges[0].v = 1
        roadmap.edges[0].properties = self.init_array(RoadmapProperty, n=3)
        roadmap.edges[0].properties[0].key = 'data_str'
        roadmap.edges[0].properties[0].value = 'this is a string'
        roadmap.edges[0].properties[1].key = 'data_int'
        roadmap.edges[0].properties[1].value = '9'
        roadmap.edges[0].properties[2].key = 'data_float'
        roadmap.edges[0].properties[2].value = '0.54'

        g = bridge.to_networkx(roadmap)
        self.assertEqual(g.edges[0, 1]['u'], 0)
        self.assertEqual(g.edges[0, 1]['v'], 1)
        self.assertEqual(g.edges[0, 1]['properties']['data_str'], 'this is a string')
        self.assertTrue(isinstance(g.edges[0, 1]['properties']['data_str'], basestring))
        self.assertEqual(g.edges[0, 1]['properties']['data_int'], '9')
        self.assertEqual(g.edges[0, 1]['properties']['data_float'], '0.54')


class NX2ROSTest(object):
    def test_preserve_structure_on_nominal_input_from_nx(self):
        g = nx.Graph()
        g.add_node(0)
        g.add_node(1)
        g.add_node(2)
        g.add_edge(0, 1)
        g.add_edge(1, 2)

        msg = bridge.from_networkx(g)
        self.assertEqual(len(msg.nodes), 3)
        self.assertEqual(len(msg.edges), 2)
        self.assertSetEqual(set([n.index for n in msg.nodes]),
                            set([0, 1, 2]))
        self.assertSetEqual(set([(e.u, e.v) for e in msg.edges]),
                            set([(0, 1), (1, 2)]))

    def test_preserve_vertex_property_from_nx(self):
        g = nx.Graph()
        g.add_node(0)
        g.nodes[0]['pose'] = PoseWithCovariance()
        g.nodes[0]['pose'].pose.position.x = 39
        g.nodes[0]['properties'] = {
            'data_str': 'this is a string',
            'data_float': 0.54,
            'data_int': 9,
        }

        msg = bridge.from_networkx(g)
        self.assertEqual(msg.nodes[0].index, 0)
        self.assertEqual(msg.nodes[0].pose.pose.position.x, 39)
        for prop in msg.nodes[0].properties:
            if prop.key == 'data_str':
                self.assertEqual(prop.value, 'this is a string')
            elif prop.key == 'data_int':
                self.assertEqual(prop.value, '9')
            elif prop.key == 'data_float':
                self.assertEqual(prop.value, '0.54')

    def test_preserve_edge_property_from_nx(self):
        g = nx.Graph()
        g.add_node(0)
        g.add_node(1)
        g.add_edge(0, 1)
        g.edges[0, 1]['properties'] = {
            'data_str': 'this is a string',
            'data_float': 0.54,
            'data_int': 9,
        }

        msg = bridge.from_networkx(g)
        self.assertEqual(msg.edges[0].u, 0)
        self.assertEqual(msg.edges[0].v, 1)
        for prop in msg.nodes[0].properties:
            if prop.key == 'data_str':
                self.assertEqual(prop.value, 'this is a string')
            elif prop.key == 'data_int':
                self.assertEqual(prop.value, '9')
            elif prop.key == 'data_float':
                self.assertEqual(prop.value, '0.54')


class BridgeTest(ROS2NXTest,
                 NX2ROSTest,
                 unittest.TestCase):
    pass


if __name__ == '__main__':
    rospy.init_node('test_node')
    rostest.rosrun('roadmap_msgs', 'test_bridge', BridgeTest)
