#!/usr/bin/env python
"""Bridge ROS message to graph structures."""

import networkx as nx

from roadmap_msgs.msg import *


def _check_networkx():
    """Check if networkx is installed."""
    try:
        nx
    except NameError:
        raise RuntimeError("Networkx library is not installed")


def to_networkx(msg, check_error=True):
    """Convert Roadmap message to nx.Graph."""
    _check_networkx()

    g = nx.Graph()

    # Copy graph property
    g.graph['properties'] = {p.key: p.value for p in msg.properties}
    g.graph['header'] = msg.header

    # Copy nodes
    for node in msg.nodes:
        if check_error and g.has_node(node.index):
            raise ValueError("Duplicate node ID")
        g.add_node(node.index, **{s: getattr(node, s) for s in node.__slots__})
        g.nodes[node.index]['properties'] = {p.key: p.value for p in node.properties}

    # Copy edges
    for edge in msg.edges:
        if check_error and (not g.has_node(edge.u) or not g.has_node(edge.v)):
            raise ValueError("Missing nodes used in the graph")
        g.add_edge(edge.u, edge.v, **{s: getattr(edge, s) for s in edge.__slots__})
        g.edges[edge.u, edge.v]['properties'] = {p.key: p.value for p in edge.properties}

    return g


def from_networkx(g):
    """Convert nx.Graph to Roadmap message."""
    _check_networkx()

    msg = Roadmap()

    # Copy graph property
    if 'properties' in g.graph:
        for key, value in g.graph['properties'].iteritems():
            msg.properties.append(RoadmapProperty(key=key, value=str(value)))
    if 'header' in g.graph:
        msg.header = g.graph['header']

    # Copy nodes
    for n in g.nodes:
        # Copy values from all available fields
        node = RoadmapNode(**{s: g.nodes[n].get(s, None)
                              for s in RoadmapNode.__slots__})

        # Get index from descriptor
        node.index = n

        # Fill properties field
        node.properties = []
        if 'properties' in g.nodes[n]:
            for key, value in g.nodes[n]['properties'].iteritems():
                node.properties.append(RoadmapProperty(key=key, value=str(value)))
        msg.nodes.append(node)

    # Copy edges
    for e in g.edges:
        # Copy values from all available fields
        edge = RoadmapEdge(**{s: g.edges[e].get(s, None)
                              for s in RoadmapEdge.__slots__})

        # Get index from descriptor
        edge.u, edge.v = e

        # Fill properties field
        edge.properties = []
        if 'properties' in g.edges[e]:
            for key, value in g.edges[e]['properties'].iteritems():
                edge.properties.append(RoadmapProperty(key=key, value=str(value)))
        msg.edges.append(edge)

    return msg
