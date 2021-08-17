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

using PoseGraphNodeLoopClosureStatus =
    std::pair<bool, pose_graph_msgs::PoseGraphNode>;

struct RssiRawInfo {
  bool b_dropped{false};
  ros::Time time_stamp;
  core_msgs::CommNodeInfo comm_node_info;
  pose_graph_msgs::PoseGraphNode pose_graph_node;
  std::map<gtsam::Key, PoseGraphNodeLoopClosureStatus> nodes_around_comm;
};

struct LoopCandidateToPrepare {
  ros::Time time_stamp_;
  std::string robot_name_;
  pose_graph_msgs::PoseGraphNode pose_;
};

class RssiLoopClosure : public LoopGeneration {
  friend class TestRSSILoopGeneration;

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

private:
  //*************Parameters Loading*******************/
  bool LoadRssiParameters(const ros::NodeHandle& n);
  bool LoadRobotsList(const ros::NodeHandle& n);

  // params
  float measured_path_loss_dB_{55.0f};
  uint close_keys_threshold_{20};
  std::string radio_loop_closure_method_{"radio_to_nodes"};

  // variable of params
  std::string node_name_;

  // variables needed
  std::map<std::string, RssiRawInfo>
      rssi_scom_dropped_list_; //<scom2, scom3,...>
  std::map<std::string, ros::Time> rssi_scom_dropped_time_stamp_;
  std::map<std::string, silvus_msgs::SilvusStreamscapeNode>
      rssi_scom_robot_list_; // robot names <scom-husky4 , scom-spot2,...>
  std::map<std::string, ros::Time>
      rssi_scom_robot_list_updated_time_stamp_; // robot names <scom-husky4 ,
                                                // scom-spot2,...>
  std::map<unsigned char, std::map<double, pose_graph_msgs::PoseGraphNode>>
      robots_trajectory_;
  int idx2{0}; // todo it's not needed at some point

  // const
  const int ANTENAS_NUMBER_IN_ROBOT{2};

  // others
  mutable std::mutex raw_mutex_;

  // bool flags
  bool b_check_for_loop_closures_;

  // ROS subscribers
  ros::Subscriber comm_node_raw_;
  ros::Subscriber comm_node_aggregated_status_sub_;
  ros::Subscriber pose_graph_sub_;

  // ROS publishers for timing
  ros::Publisher db_tracking;
  ros::Publisher visualize_rssi_placement;
  ros::Publisher highlight_pub_;

  //*************Callbacks*******************/
  void CommNodeAggregatedStatusCallback(
      const core_msgs::CommNodeStatus::ConstPtr& msg);
  void CommNodeRawCallback(const silvus_msgs::SilvusStreamscape::ConstPtr& msg);

  //*************Timer*******************/
  ros::Timer uwb_update_timer_;
  double update_rate_;
  void RssiTimerCallback(const ros::TimerEvent& event);

  //*************HELPER FUNCTIONS*******************/
  void GenerateLoops();
  void RadioToNodesLoopClosure();
  pose_graph_msgs::PoseGraphNode GetClosestPoseAtTime(
      const std::map<double, pose_graph_msgs::PoseGraphNode>& robot_trajectory,
      const ros::Time& stamp,
      double time_threshold = 2.0,
      bool check_threshold = false);
  bool is_robot_radio(const std::string& hostname) const;

  //*************Math*******************/
  float CalculatePathLossForNeighbor(
      const silvus_msgs::SilvusStreamscapeNeighbor& neighbor);

  //*************Visualization and Printing*******************/
  void PrintDropStatus(const core_msgs::CommNodeInfo& msg) const;
  void ShowRssiList() const;
  void ShowRobotList() const;
  void ShowDroppedRssiList() const;
  void VisualizeRssi();
  void VisualizeRobotNodesWithPotentialLoopClosure(
      const pose_graph_msgs::PoseGraphNode& node_pose,
      float red = 1.0,
      float green = 1.0,
      float blue = 1.0,
      float scale = 0.5);
  bool VisualizeEdgesForPotentialLoopClosure(
      const pose_graph_msgs::PoseGraphNode& node1,
      const pose_graph_msgs::PoseGraphNode& node2);
};
} // namespace lamp_loop_closure
