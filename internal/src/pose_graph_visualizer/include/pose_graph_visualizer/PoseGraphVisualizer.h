#ifndef POSE_GRAPH_VISUALIZER_H
#define POSE_GRAPH_VISUALIZER_H

#include <ros/ros.h>

#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <pcl_ros/point_cloud.h>

#include <tf/transform_datatypes.h>

#include <map>
#include <vector>

class PoseGraphVisualizer {
 public:
  PoseGraphVisualizer() = default;
  ~PoseGraphVisualizer() = default;

  bool Initialize(const ros::NodeHandle& n);

  // Typedef for stored point clouds.
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  void MakeMenuMarker(const tf::Pose &position, const std::string &id_number);

  // Visualizes an edge between the two keys.
  bool HighlightEdge(unsigned int key1, unsigned int key2);
  // Removes the edge visualization between the two keys.
  // Removes all highlighting visualizations if both keys are zero.
  void UnhighlightEdge(unsigned int key1, unsigned int key2);

  // Highlights factor graph node associated with the given key.
  bool HighlightNode(unsigned int key);
  // Unhighlights factor graph node associated with the given key.
  // Removes all highlighting visualizations if the key is zero.
  void UnhighlightNode(unsigned int key);

  void VisualizePoseGraph();

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr &msg);
  void PoseGraphCallback(const pose_graph_msgs::PoseGraph::ConstPtr &msg);
  void PoseGraphNodeCallback(const pose_graph_msgs::PoseGraphNode::ConstPtr &msg);
  void PoseGraphEdgeCallback(const pose_graph_msgs::PoseGraphEdge::ConstPtr &msg);

  geometry_msgs::Point GetPositionMsg(unsigned int key) const;

  inline bool KeyExists(unsigned int key) const {
    return keyed_stamps_.find(key) != keyed_stamps_.end();
  }

  // Node name.
  std::string name_;

  // Keep a list of keyed laser scans, poses and timestamps.
  std::map<unsigned int, PointCloud::ConstPtr> keyed_scans_;
  std::map<unsigned int, tf::Pose> keyed_poses_;
  std::map<unsigned int, ros::Time> keyed_stamps_;
  std::map<double, unsigned int> stamps_keyed_;

  // Frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  // Visualization publishers.
  ros::Publisher odometry_edge_pub_;
  ros::Publisher loop_edge_pub_;
  ros::Publisher graph_node_pub_;
  ros::Publisher graph_node_id_pub_;
  ros::Publisher keyframe_node_pub_;
  ros::Publisher closure_area_pub_;
  ros::Publisher highlight_pub_;

  // Subscribers.
  ros::Subscriber keyed_scan_sub_;
  ros::Subscriber pose_graph_sub_;
  ros::Subscriber pose_graph_node_sub_;
  ros::Subscriber pose_graph_edge_sub_;

  typedef std::pair<unsigned int, unsigned int> Edge;
  std::vector<Edge> odometry_edges_;
  std::vector<Edge> loop_edges_;

  bool publish_interactive_markers_{true};

  // Proximity threshold used by LaserLoopClosureNode.
  double proximity_threshold_{1};
};

#endif
