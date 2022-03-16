#include "roadmap_msgs/roadmap_graph.h"
#include <boost/graph/copy.hpp>
#include <stdio.h>

namespace roadmap {

RoadmapGraph::RoadmapGraph() { m_temp_idx_counter = TEMP_INDEX_START; }

RoadmapGraph &RoadmapGraph::operator=(RoadmapGraph other) {
  printf("hello----\n");
  if (this != &other) {
    this->m_temp_idx_counter = other.m_temp_idx_counter;
    this->m_map_id_to_vert.clear();
    // boost::copy_graph(other.m_g, this->m_g);
    this->m_g = Graph();

    std::pair<VertexIterator, VertexIterator> vp;
    int counter = 0;
    for (vp = vertices(other.m_g); vp.first != vp.second; ++vp.first) {
      Vertex v = *vp.first;
      Vertex v1 = boost::add_vertex(other.m_g[v], this->m_g);
      this->m_map_id_to_vert[other.m_g[v].index] = v1;
    }

    // Copy edge information
    std::pair<EdgeIterator, EdgeIterator> ep;
    size_t edge_cnt = 0;
    for (ep = edges(other.m_g); ep.first != ep.second; ++ep.first, ++edge_cnt) {
      Edge e = *ep.first;
      roadmap_msgs::RoadmapEdge re;
      re.u = other.m_g[source(e, other.m_g)].index;
      re.v = other.m_g[target(e, other.m_g)].index;
      // TODO Do edge properties.
      Vertex v_u = this->m_map_id_to_vert[re.u];
      Vertex v_v = this->m_map_id_to_vert[re.v];
      boost::add_edge(v_u, v_v, re, this->m_g);
    }
  }
  return *this;
}

uint32_t RoadmapGraph::add_node(roadmap_msgs::RoadmapNode node) {
  Vertex v = boost::add_vertex(node, m_g);
  m_map_id_to_vert[node.index] = v;
  printf("Add node id %u vertex %x\n", node.index, v);

  return node.index;
}

uint32_t RoadmapGraph::add_node_idless(roadmap_msgs::RoadmapNode node) {
  roadmap_msgs::RoadmapNode node_copy = node;
  uint32_t node_prefix = node_copy.index >> 24;
  node_copy.index = m_temp_idx_counter;
  m_temp_idx_counter++;
  node_copy.index = node_copy.index | (node_prefix << 24);
  Vertex v = boost::add_vertex(node_copy, m_g);
  m_map_id_to_vert[node_copy.index] = v;
  printf("Add node idless node id %u vertex %x\n", node_copy.index, v);
  return node_copy.index;
}

void RoadmapGraph::add_edge(roadmap_msgs::RoadmapEdge edge) {
  Vertex v_u = m_map_id_to_vert.at(edge.u);
  Vertex v_v = m_map_id_to_vert.at(edge.v);
  boost::add_edge(v_u, v_v, edge, m_g);
}

Edge RoadmapGraph::get_edge(uint32_t u_idx, uint32_t v_idx) {
  Vertex v_u = m_map_id_to_vert.at(u_idx);
  Vertex v_v = m_map_id_to_vert.at(v_idx);
  return boost::edge(v_u, v_v, m_g).first;
}

bool RoadmapGraph::remove_node(uint32_t node_idx) {
  if (m_map_id_to_vert.find(node_idx) == m_map_id_to_vert.end()) {
    return false;
  }
  Vertex v = m_map_id_to_vert[node_idx];

  printf("del node id %u %x \n", node_idx, v);
  boost::remove_vertex(v, m_g);

  m_map_id_to_vert.erase(node_idx);
  return true;
}

roadmap_msgs::RoadmapNode RoadmapGraph::get_node(uint32_t node_idx) {

  printf(" -----  \n");
  for (auto &t : m_map_id_to_vert) {
    std::cout << t.first << " " << t.second << " " << m_g[t.second].index
              << "\n";
  }

  printf(" -----  \n");
  Vertex v = m_map_id_to_vert.at(node_idx);
  printf("get node node id %u %x\n", node_idx, v);
  return m_g[m_map_id_to_vert.at(node_idx)];
}

Graph RoadmapGraph::get_internal_graph() const { return m_g; }

void RoadmapGraph::set_vertex_property(uint32_t node_idx,
                                       const std::string &key,
                                       const std::string &value) {

  Vertex v = m_map_id_to_vert.at(node_idx);
  printf("set vert prop %u , %x\n", node_idx, v);
  for (auto &p : m_g[v].properties) {
    if (key == p.key) {
      p.value = value;
      return;
    }
  }

  // Otherwise, create a new property
  auto new_prop = roadmap_msgs::RoadmapProperty();
  new_prop.key = key;
  new_prop.value = value;
  m_g[v].properties.push_back(new_prop);
}

void RoadmapGraph::clear_vertex_properties(uint32_t node_idx) {
  Vertex v = m_map_id_to_vert.at(node_idx);
  m_g[v].properties.clear();
}

void RoadmapGraph::delete_vertex_property(uint32_t node_idx,
                                          const std::string &key) {
  Vertex v = m_map_id_to_vert.at(node_idx);
  printf("del vert prop %u, %x\n", node_idx, v);
  auto pend = std::remove_if(
      m_g[v].properties.begin(), m_g[v].properties.end(),
      [key](roadmap_msgs::RoadmapProperty &prop) { return (prop.key == key); });
  m_g[v].properties.erase(pend, m_g[v].properties.end());
}

void RoadmapGraph::set_vertex_type(uint32_t node_idx, const VertexType type) {
  Vertex v = m_map_id_to_vert.at(node_idx);
  printf("set vert type %u, %x \n", node_idx, v);
  m_g[v].type = type;
}

void RoadmapGraph::set_vertex_pose(uint32_t node_idx,
                                   const geometry_msgs::Pose pose) {
  Vertex v = m_map_id_to_vert.at(node_idx);
  printf("set vert pose %u, %x\n", node_idx, v);
  m_g[v].pose.pose = pose;
}

void RoadmapGraph::print_triad() {
  printf(" triad is -----  \n");
  for (auto &t : m_map_id_to_vert) {
    std::cout << t.first << " " << t.second << " " << m_g[t.second].index
              << "\n";
  }

  printf(" -----  \n");
}

} // namespace roadmap
