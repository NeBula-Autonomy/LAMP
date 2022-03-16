#!/usr/bin/env python
"""Bridge ROS message to graph structures."""

import rospy

import networkx as nx

from pose_graph_msgs.msg import *


def _check_networkx():
    """Check if networkx is installed."""
    try:
        nx
    except NameError:
        raise RuntimeError("Networkx library is not installed")


def to_networkx(msg):
    """Convert Roadmap message to nx.Graph."""
    _check_networkx()

    g = nx.Graph()

    # Copy graph property
    g.graph['header'] = msg.header

    # Copy nodes
    for node in msg.nodes:
        g.add_node(node.key, **{s: getattr(node, s) for s in node.__slots__})

    # Copy edges
    for edge in msg.edges:
        if not g.has_node(edge.key_from) or not g.has_node(edge.key_to):
            rospy.logwarn("Missing nodes used in the graph")
            continue
        g.add_edge(edge.key_from, edge.key_to,
                   **{s: getattr(edge, s) for s in edge.__slots__})

    return g


def from_networkx(g):
    """Convert nx.Graph to Roadmap message."""
    raise NotImplementedError
