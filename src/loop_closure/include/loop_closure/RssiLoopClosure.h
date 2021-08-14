#pragma once
#include "loop_closure/LoopGeneration.h"
#include <core_msgs/CommNodeInfo.h>
#include <core_msgs/CommNodeStatus.h>
#include <gtsam/inference/Symbol.h>
#include <parameter_utils/ParameterUtils.h>
#include <silvus_msgs/SilvusStreamscape.h>
#include <std_msgs/Float64.h>
#include <utils/CommonFunctions.h>
#include <utils/PoseGraph.h>
#include <utils/PrefixHandling.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace lamp_loop_closure {

namespace pu = parameter_utils;

struct RssiRawInfo {
  bool b_dropped{false};
  ros::Time time_stamp_;
  core_msgs::CommNodeInfo node_info;
  pose_graph_msgs::PoseGraphNode graph_node;
};

struct LoopCandidateToPrepare {
  ros::Time time_stamp_;
  std::string robot_name_;
  pose_graph_msgs::PoseGraphNode pose_;
};

class RssiLoopClosure : public LoopGeneration {
public:
  RssiLoopClosure();

  ~RssiLoopClosure();

  bool Initialize(const ros::NodeHandle& n) override;

  bool LoadParameters(const ros::NodeHandle& n) override;

  bool CreatePublishers(const ros::NodeHandle& n) override;

  bool RegisterCallbacks(const ros::NodeHandle& n) override;

protected:
  void KeyedPoseCallback(
      const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) override;

  void GenerateLoops(const gtsam::Key& new_key);

private:
  bool LoadRssiParameters(const ros::NodeHandle& n);
  bool LoadRobotsList(const ros::NodeHandle& n);

  // ROS subscribers
  ros::Subscriber comm_node_raw_;
  ros::Subscriber comm_node_aggregated_status_sub_;
  ros::Subscriber pose_graph_sub_;

  // ROS publishers for timing
  ros::Publisher db_tracking;
  ros::Publisher visualize_rssi_placement;

  // callbacks
  void CommNodeAggregatedStatusCallback(
      const core_msgs::CommNodeStatus::ConstPtr& msg);
  void CommNodeRawCallback(const silvus_msgs::SilvusStreamscape::ConstPtr& msg);

  //  // ROS timer
  ros::Timer uwb_update_timer_;
  double update_rate_;
  void RssiTimerCallback(const ros::TimerEvent& event);

  // math calculation
  float CalculatePathLossForNeighbor(
      const silvus_msgs::SilvusStreamscapeNeighbor& neighbor);

  // visualization and printing
  void ShowRssiList() const;
  void ShowRobotList() const;
  void ShowDroppedRssiList() const;
  void VisualizeRssi();
  void VisualizeRobotNodesWithPotentialLoopClosure(
      const pose_graph_msgs::PoseGraphNode& node_pose,
      float red = 1.0,
      float green = 1.0,
      float blue = 1.0);

  // params
  float acceptable_shortest_rssi_distance_;

  // bool flags
  bool b_check_for_loop_closures_;

  std::map<std::string, int> rssi_id2key_;
  //  std::map<std::string, RssiInfo> rssi_list_;
  std::map<std::string, RssiRawInfo> rssi_node_dropped_list_;
  std::map<std::string, ros::Time> rssi_node_dropped_time_stamp_;

  //  std::map<std::string, silvus_msgs::SilvusStreamscapeNode>
  //  rssi_robot_list_;
  std::map<std::string, silvus_msgs::SilvusStreamscapeNode>
      rssi_scom_robot_list_;
  std::map<std::string, ros::Time> rssi_scom_robot_list_updated_time_stamp_;

  std::vector<LoopCandidateToPrepare> loop_candidates_to_prepare_;
  int idx2 = 0;
  std::vector<pose_graph_msgs::LoopCandidate> potential_candidates_;

  PoseGraph pose_graph_;

  const int ANTENAS_NUMBER_IN_ROBOT{2};

  mutable std::mutex raw_mutex_;

  // string parameters
  std::string node_name_;

  std::map<unsigned char, std::map<double, pose_graph_msgs::PoseGraphNode>>
      robots_trajectory_;
};
} // namespace lamp_loop_closure
