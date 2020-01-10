#ifndef MERGER_H
#define MERGER_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>

#include <gtsam/inference/Symbol.h>

#include <utils/PrefixHandling.h>

#include <tf2/transform_datatypes.h>

#include <gtsam/inference/Symbol.h>

#include <utils/PrefixHandling.h>

#include <string>

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

typedef pose_graph_msgs::PoseGraphNode GraphNode;
typedef pose_graph_msgs::PoseGraphEdge GraphEdge;

class Merger {
public:
  Merger();

  void OnFastGraphMsg(const pose_graph_msgs::PoseGraphConstPtr& msg);

  void OnSlowGraphMsg(const pose_graph_msgs::PoseGraphConstPtr& msg);

  void OnFastPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void OnSlowPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);

  // Insertion, deletion and tracking
  void InsertNode(const pose_graph_msgs::PoseGraphNode& node);
  void ClearNodes();
  void InsertNewEdges(const pose_graph_msgs::PoseGraphConstPtr& msg);
  bool IsEdgeNew(const pose_graph_msgs::PoseGraphEdge& msg);


  std::set<char> GetNewRobots(const pose_graph_msgs::PoseGraphConstPtr& msg);

  // Utility functions
  void CleanUpMap(const ros::Time& stamp);
  pose_graph_msgs::PoseGraph GetCurrentGraph();
  void NormalizeNodeOrientation(pose_graph_msgs::PoseGraphNode & msg);

  geometry_utils::Transform3 GetPoseAtTime(const ros::Time& stamp);

private:
  pose_graph_msgs::PoseGraph current_graph_;
  pose_graph_msgs::PoseGraphConstPtr lastSlow;
  geometry_utils::Transform3 current_pose_est_;
  geometry_utils::Transform3 last_slow_pose_;
  geometry_utils::Transform3 current_fast_pose_;
  geometry_utils::Transform3 fast_pose_at_slow_;

  std::map<double, geometry_utils::Transform3> timestamped_poses_;

  bool b_received_first_fast_pose_;
  bool b_received_first_slow_pose_;

  bool b_block_slow_pose_update;

  ros::Subscriber fastGraphSub;
  ros::Subscriber slowGraphSub;
  ros::Subscriber fastPoseSub;
  ros::Subscriber slowPoseSub;

  ros::Publisher mergedGraphPub;
  ros::Publisher mergedPosePub;

  // unique edges stored in the graph, tracked by <key_from, key_to, type>
  std::set< std::tuple<gtsam::Key, gtsam::Key, int> > unique_edges_;

  // Robots included in the merged graph, specified by prefix char
  std::set<char> robots_;

  // Storing map from key to the index in the nodes vector
  std::map<long unsigned int, int> merged_graph_KeyToIndex_;

  pose_graph_msgs::PoseGraph merged_graph_;

  // Test class fixtures
  friend class TestMerger;
};

#endif