#pragma once
#include "loop_closure/LoopGeneration.h"
#include "loop_closure/colors.h"
#include <core_msgs/CommNodeInfo.h>
#include <core_msgs/CommNodeStatus.h>
#include <gtsam/inference/Symbol.h>
#include <omp.h>
#include <parameter_utils/ParameterUtils.h>

#include <silvus_msgs/SilvusStreamscape.h>
#include <std_msgs/Float64.h>
#include <string>
#include <utils/CommonFunctions.h>
#include <utils/PoseGraph.h>
#include <utils/PrefixHandling.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
namespace lamp_loop_closure {

namespace pu = parameter_utils;

using PoseGraphNodeLoopClosureStatus =
    std::pair<bool, pose_graph_msgs::PoseGraphNode>;

struct PoseGraphNodeForLoopClosureStatus {
  bool was_sent{false};
  pose_graph_msgs::PoseGraphNode candidate_pose;
};

using NodesAroundCommOneFlyby = std::vector<PoseGraphNodeForLoopClosureStatus>;
using AllNodesAroundComm = std::vector<NodesAroundCommOneFlyby>;

struct RssiRawInfo {
  bool b_dropped{false};
  core_msgs::CommNodeInfo comm_node_info;
  pose_graph_msgs::PoseGraphNode pose_graph_node;
  AllNodesAroundComm all_nodes_around_comm;
  int flyby_number{0};
  const uint close_keys_threshold_{20};
  bool append_node(const pose_graph_msgs::PoseGraphNode& node);
  // if pose has key 0 it means that node haven't been found; if keys
  // are the same means that the robot doesn't move and is in the same
  // place what the comm node has a pose, and check if the same pose was
  // included before - > if 3x yes then accept the pose for the signal
  bool is_node_exist(const pose_graph_msgs::PoseGraphNode& node);
  bool is_node_same_as_comm(const pose_graph_msgs::PoseGraphNode& node);
  bool has_node_pose(const pose_graph_msgs::PoseGraphNode& node);
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

  std::map<std::string, silvus_msgs::SilvusStreamscapeNode>
      rssi_scom_robot_list_; // robot names <scom-husky4 , scom-spot2,...>
                             //  std::map<std::string, ros::Time>
  std::map<gtsam::Key, gtsam::Pose3> keyed_poses_;
  std::map<gtsam::Key, float> lowest_distance_;
  std::map<unsigned char, std::map<double, pose_graph_msgs::PoseGraphNode>>
      robots_trajectory_;

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
  ros::Publisher visualize_all_markers_;

  //*************Callbacks*******************/
  void CommNodeAggregatedStatusCallback(
      const core_msgs::CommNodeStatus::ConstPtr& msg);
  void CommNodeRawCallback(const silvus_msgs::SilvusStreamscape::ConstPtr& msg);

  //*************Timer*******************/
  ros::Timer uwb_update_timer_;
  double update_rate_;
  //  void RssiTimerCallback(const ros::TimerEvent& event);
  void Update(const ros::Time& time_stamp);

  //*************HELPER FUNCTIONS*******************/
  void GenerateLoops();
  void RadioToNodesLoopClosure();
  void NodesToNodesLoopClosures();
  pose_graph_msgs::PoseGraphNode GetClosestPoseAtTime(
      const std::map<double, pose_graph_msgs::PoseGraphNode>& robot_trajectory,
      const ros::Time& stamp,
      double time_threshold = 2.0,
      bool check_threshold = false) const;
  pose_graph_msgs::PoseGraphNode GetPoseGraphNodeFromKey(
      const std::map<double, pose_graph_msgs::PoseGraphNode>& robot_trajectory,
      const gtsam::Symbol& key) const;
  bool is_robot_radio(const std::string& hostname) const;

  //*************Math*******************/
  float CalculatePathLossForNeighbor(
      const silvus_msgs::SilvusStreamscapeNeighbor& neighbor);

  //*************Visualization and Printing*******************/
  visualization_msgs::MarkerArray all_markers_;
  void PrintDropStatus(const core_msgs::CommNodeInfo& msg) const;
  void ShowRssiList() const;
  void ShowRobotList() const;
  void ShowDroppedRssiList() const;
  void VisualizeRssi(const std::string& name, const RssiRawInfo& node);
  void VisualizeRobotNodesWithPotentialLoopClosure(
      const pose_graph_msgs::PoseGraphNode& node_pose,
      float red = 1.0,
      float green = 1.0,
      float blue = 1.0,
      float scale = 0.5,
      std::string name = "");
  bool VisualizeEdgesForPotentialLoopClosure(
      const pose_graph_msgs::PoseGraphNode& node1,
      const pose_graph_msgs::PoseGraphNode& node2);
};
} // namespace lamp_loop_closure
