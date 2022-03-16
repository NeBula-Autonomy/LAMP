
#pragma once

#include "geometry_msgs/Pose.h"
#include "roadmap_msgs/roadmap_type.h"
#include <unordered_map>

#define TEMP_INDEX_START 20000

namespace roadmap {
class RoadmapGraph {

public:
  RoadmapGraph();
  RoadmapGraph &operator=(RoadmapGraph other); // copy assignment
  typedef roadmap_msgs::RoadmapNode::_type_type VertexType;

  enum DistanceMetric {
    EUCLIDEAN,
    EUCLIDEAN_2D,
  };

  uint32_t add_node_idless(roadmap_msgs::RoadmapNode node);
  uint32_t add_node(roadmap_msgs::RoadmapNode node);
  void add_edge(roadmap_msgs::RoadmapEdge edge);
  bool remove_node(uint32_t node_idx);
  roadmap_msgs::RoadmapNode get_node(uint32_t node_idx);
  Edge get_edge(uint32_t u_idx, uint32_t v_idx);
  Graph get_internal_graph() const;

  void set_node_pose(uint32_t node_idx, const geometry_msgs::Pose &pose);

  void set_vertex_property(uint32_t node_idx, const std::string &key,
                           const std::string &value);

  void clear_vertex_properties(uint32_t node_idx);

  void delete_vertex_property(uint32_t node_idx, const std::string &key);
  void set_vertex_type(uint32_t node_idx, const VertexType type);
  void set_vertex_pose(uint32_t node_idx, const geometry_msgs::Pose pose);
  void print_triad();

private:
  Graph m_g;
  std::unordered_map<uint32_t, Vertex> m_map_id_to_vert;
  uint32_t m_temp_idx_counter;
};

} // namespace roadmap
