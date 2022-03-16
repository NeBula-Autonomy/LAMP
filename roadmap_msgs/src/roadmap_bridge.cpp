/*
 * @file  roadmap_bridge.cpp
 * @brief Bridge between ROS and BGL (Boost Graph Library)
 */

#include <exception>

#include "roadmap_msgs/roadmap_bridge.h"
#include "roadmap_msgs/roadmap_graph.h"

#include <stdio.h>

using namespace boost;
using namespace roadmap_msgs;

namespace roadmap {

RoadmapPtr toROSMsg(const RoadmapGraph &rg) {
  // Allocate memory
  auto g = rg.get_internal_graph();
  RoadmapPtr msg = RoadmapPtr(new Roadmap);
  msg->nodes.resize(num_vertices(g));
  msg->edges.resize(num_edges(g));

  // Copy vertex information
  std::pair<VertexIterator, VertexIterator> vp;
  int counter = 0;
  for (vp = vertices(g); vp.first != vp.second; ++vp.first) {
    Vertex v = *vp.first;
    msg->nodes[counter] = g[v];
  }

  // Copy edge information
  std::pair<EdgeIterator, EdgeIterator> ep;
  size_t edge_cnt = 0;
  for (ep = edges(g); ep.first != ep.second; ++ep.first, ++edge_cnt) {
    Edge e = *ep.first;
    msg->edges[edge_cnt] = g[e];

    // Assign BGL indices for source/target nodes
    msg->edges[edge_cnt].u = g[source(e, g)].index;
    msg->edges[edge_cnt].v = g[target(e, g)].index;
  }

  return msg;
}

RoadmapGraph fromROSMsg(const Roadmap &msg) {
  RoadmapGraph rg;
  printf("Invoking fromROSMsg\n");

  // Add vertices to graph
  for (auto &v : msg.nodes) {
    rg.add_node(v);
  }

  // Add edges to graph
  for (auto &e : msg.edges) {
    rg.add_edge(e);
  }
  return rg;
}

RoadmapGraph fromROSMsg(const RoadmapConstPtr &msg) { return fromROSMsg(*msg); }

} // namespace roadmap
