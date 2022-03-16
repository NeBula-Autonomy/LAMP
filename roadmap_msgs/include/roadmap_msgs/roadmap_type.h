/*
 * @file  roadmap_type.h
 * @brief Type definition for roadmap messages
 */

#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include "roadmap_msgs/Roadmap.h"

namespace roadmap {

typedef boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS,
                              roadmap_msgs::RoadmapNode,
                              roadmap_msgs::RoadmapEdge, roadmap_msgs::Roadmap>
    Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIterator;

typedef boost::shared_ptr<Graph> GraphPtr;
typedef boost::shared_ptr<Graph const> GraphConstPtr;

} // namespace roadmap
