/**
 *  @brief Testing the Loop Closure Generation Classes
 *  by Andrzej Reinke (andrzej.m.reinke@jpl.nasa.gov)
 */

#include <gtest/gtest.h>

#include "loop_closure/LoopGeneration.h"
#include "loop_closure/RssiLoopClosure.h"
#include <vector>
namespace lamp_loop_closure {
class TestRSSILoopGeneration : public ::testing::Test {
protected:
  TestRSSILoopGeneration() {
    // Load params
    system("rosparam load $(rospack find lamp)/config/lamp_settings.yaml ");
    system("rosparam load $(rospack find "
           "loop_closure)/config/rssi_parameters.yaml");
    std::vector<std::string> robot_list{"husky4", "spot1", "spot2"};
    ros::param::set("/robots_loop_closure", robot_list);
  }
  ~TestRSSILoopGeneration() {}

  //  void generateLoops(const gtsam::Key& new_key) {
  //    proximity_lc_.GenerateLoops(new_key);
  //  }

  void
  keyedPoseCallback(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
    rssi_lc_.KeyedPoseCallback(graph_msg);
  }

  void rssiTimerCallback() {
    rssi_lc_.RssiTimerCallback(ros::TimerEvent());
  }

  void commNodeAggregatedStatusCallback(
      const core_msgs::CommNodeStatus::ConstPtr& msg) {
    rssi_lc_.CommNodeAggregatedStatusCallback(msg);
  }

  pose_graph_msgs::PoseGraphNode getClosestPoseAtTime(
      const std::map<double, pose_graph_msgs::PoseGraphNode>& robot_trajectory,
      const ros::Time& stamp,
      double time_threshold = 2.0,
      bool check_threshold = false) {
    return rssi_lc_.GetClosestPoseAtTime(
        robot_trajectory, stamp, time_threshold, check_threshold);
  }

  float calculatePathLossForNeighbor(
      const silvus_msgs::SilvusStreamscapeNeighbor& neighbor) {
    return rssi_lc_.CalculatePathLossForNeighbor(neighbor);
  }

  void
  commNodeRawCallback(const silvus_msgs::SilvusStreamscape::ConstPtr& msg) {
    rssi_lc_.CommNodeRawCallback(msg);
  }
  bool isRobotRadio(const std::string hostname) {
    return rssi_lc_.is_robot_radio(hostname);
  }

  std::map<double, pose_graph_msgs::PoseGraphNode>
  getRobotTrajectory(char robot_prefix) {
    return rssi_lc_.robots_trajectory_[robot_prefix];
  }

  std::map<std::string, silvus_msgs::SilvusStreamscapeNode>
  getRssiScomRobotList() {
    return rssi_lc_.rssi_scom_robot_list_;
  }

  std::vector<pose_graph_msgs::LoopCandidate> getCandidates() {
    return rssi_lc_.candidates_;
  }

  std::map<std::string, RssiRawInfo> getScomDroppedList() {
    return rssi_lc_.rssi_scom_dropped_list_;
  }

  std::map<std::string, ros::Time> getScomDroppedListTimeStamps() {
    return rssi_lc_.rssi_scom_dropped_time_stamp_;
  }

  std::map<std::string, ros::Time> getRssiScomRobotListUpdatedTimeStamp() {
    return rssi_lc_.rssi_scom_robot_list_updated_time_stamp_;
  }

  //  std::vector<pose_graph_msgs::LoopCandidate> getCandidates() {
  //    return proximity_lc_.candidates_;
  //  }

  //  double distanceBetweenKeys(const gtsam::Symbol& key1,
  //                             const gtsam::Symbol& key2) {
  //    return proximity_lc_.DistanceBetweenKeys(key1, key2);
  //  }

  RssiLoopClosure rssi_lc_;
};

TEST_F(TestRSSILoopGeneration, TestInitialize) {
  ros::NodeHandle nh;
  bool init = rssi_lc_.Initialize(nh);
  ASSERT_TRUE(init);
}

TEST_F(TestRSSILoopGeneration, TestTrajectorySize) {
  ros::NodeHandle nh;
  bool init = rssi_lc_.Initialize(nh);
  pose_graph_msgs::PoseGraph::Ptr graph_msg(new pose_graph_msgs::PoseGraph);
  pose_graph_msgs::PoseGraphNode node1, node2, node3, node4, node5, node6;
  node1.key = gtsam::Symbol('a', 0);
  node1.header.stamp = ros::Time::now() + ros::Duration(24 * 60 * 60);
  node2.key = gtsam::Symbol('a', 1);
  node2.header.stamp = ros::Time::now() + ros::Duration(23 * 60 * 60);
  node3.key = gtsam::Symbol('b', 0);
  node3.header.stamp = ros::Time::now() + ros::Duration(22 * 60 * 60);
  node4.key = gtsam::Symbol('b', 1);
  node4.header.stamp = ros::Time::now() + ros::Duration(21 * 60 * 60);
  node5.key = gtsam::Symbol('b', 2);
  node5.header.stamp = ros::Time::now() + ros::Duration(20 * 60 * 60);
  node6.key = gtsam::Symbol('b', 3);
  node6.header.stamp = ros::Time::now() + ros::Duration(19 * 60 * 60);
  graph_msg->nodes.push_back(node1);
  graph_msg->nodes.push_back(node2);
  graph_msg->nodes.push_back(node3);
  graph_msg->nodes.push_back(node4);
  graph_msg->nodes.push_back(node5);
  graph_msg->nodes.push_back(node6);
  keyedPoseCallback(graph_msg);

  EXPECT_EQ(getRobotTrajectory('a').size(), 2);
  EXPECT_EQ(getRobotTrajectory('b').size(), 4);
}

// is robot radio todo i think it should be moved somewhere else
TEST_F(TestRSSILoopGeneration, TestRadioNameDetection) {
  ros::NodeHandle nh;
  bool init = rssi_lc_.Initialize(nh);
  EXPECT_EQ(isRobotRadio("husky"), false);
  EXPECT_EQ(isRobotRadio("scom-husky"), true);
  EXPECT_EQ(isRobotRadio("scom5"), false);
  EXPECT_EQ(isRobotRadio("scom1"), false);
  EXPECT_EQ(isRobotRadio("scom-spot"), true);
  EXPECT_EQ(isRobotRadio("hehe"), false);
}

TEST_F(TestRSSILoopGeneration, GetClosestPoseAtTime) {
  ros::NodeHandle nh;
  bool init = rssi_lc_.Initialize(nh);
  pose_graph_msgs::PoseGraph::Ptr graph_msg(new pose_graph_msgs::PoseGraph);
  pose_graph_msgs::PoseGraphNode node1, node2, node3, node4, node5, node6,
      node7, node8, node9, node10, node11;
  node1.key = gtsam::Symbol('a', 0);
  node1.header.stamp = ros::Time(0) + ros::Duration(14 * 60 * 60);
  node2.key = gtsam::Symbol('a', 1);
  node2.header.stamp = ros::Time(0) + ros::Duration(15 * 60 * 60);
  node3.key = gtsam::Symbol('b', 0);
  node3.header.stamp = ros::Time(0) + ros::Duration(16 * 60 * 60);
  node4.key = gtsam::Symbol('b', 1);
  node4.header.stamp = ros::Time(0) + ros::Duration(17 * 60 * 60);
  node5.key = gtsam::Symbol('b', 2);
  node5.header.stamp = ros::Time(0) + ros::Duration(18 * 60 * 60);
  node6.key = gtsam::Symbol('b', 3);
  node6.header.stamp = ros::Time(0) + ros::Duration(19 * 60 * 60);
  node7.key = gtsam::Symbol('b', 4);
  node7.header.stamp = ros::Time(0) + ros::Duration(20 * 60 * 60);
  node8.key = gtsam::Symbol('b', 5);
  node8.header.stamp = ros::Time(0) + ros::Duration(21 * 60 * 60);
  node9.key = gtsam::Symbol('b', 6);
  node9.header.stamp = ros::Time(0) + ros::Duration(22 * 60 * 60);
  node10.key = gtsam::Symbol('b', 7);
  node10.header.stamp = ros::Time(0) + ros::Duration(23 * 60 * 60);
  node11.key = gtsam::Symbol('b', 8);
  node11.header.stamp = ros::Time(0) + ros::Duration(24 * 60 * 60);
  graph_msg->nodes.push_back(node1);
  graph_msg->nodes.push_back(node2);
  graph_msg->nodes.push_back(node3);
  graph_msg->nodes.push_back(node4);
  graph_msg->nodes.push_back(node5);
  graph_msg->nodes.push_back(node6);
  graph_msg->nodes.push_back(node7);
  graph_msg->nodes.push_back(node8);
  graph_msg->nodes.push_back(node9);
  graph_msg->nodes.push_back(node10);
  graph_msg->nodes.push_back(node11);
  keyedPoseCallback(graph_msg);

  ros::Time stamp_b = ros::Time(0) + ros::Duration(20.8 * 60 * 60);

  EXPECT_EQ(getClosestPoseAtTime(getRobotTrajectory('b'), stamp_b).key,
            node8.key);

  EXPECT_EQ(getClosestPoseAtTime(getRobotTrajectory('a'), stamp_b).key,
            node2.key);

  EXPECT_EQ(getClosestPoseAtTime(getRobotTrajectory('a'), stamp_b, 1, true).key,
            0);
}

TEST_F(TestRSSILoopGeneration, CalculatePathLossForNeighbor) {
  silvus_msgs::SilvusStreamscapeNeighbor neighbor;
  // from Jeffrey example
  neighbor.received_signal_power_dBm.push_back(-25);
  neighbor.received_signal_power_dBm.push_back(-27);
  float actual_power = 684.0f;
  float actual_power_dBm = 10.0f * std::log10(actual_power);
  neighbor.my_txpw_actual_dBm = actual_power_dBm;
  EXPECT_NEAR(neighbor.my_txpw_actual_dBm, 28.35, 0.1);
  auto average_path_loss_dB = calculatePathLossForNeighbor(neighbor);
  EXPECT_NEAR(average_path_loss_dB, 54.35, 0.1);
}

TEST_F(TestRSSILoopGeneration, CommNodeRawCallback) {
  ros::NodeHandle nh;
  bool init = rssi_lc_.Initialize(nh);
  silvus_msgs::SilvusStreamscape::Ptr msg(new silvus_msgs::SilvusStreamscape);

  auto time_stamp1 = ros::Time::now();
  msg->header.stamp = time_stamp1;

  silvus_msgs::SilvusStreamscapeNode node1;
  node1.robot_name = "husky4";
  node1.node_label = "scom-husky4";
  node1.node_id = 1;
  node1.txpw_requested_dBm = 5.0;
  node1.txpw_actual_dBm = 2.0;
  silvus_msgs::SilvusStreamscapeNode node2;
  node2.robot_name = "";
  node2.node_id = 2;
  node2.node_label = "scom2";
  node2.txpw_requested_dBm = 5.0;
  node2.txpw_actual_dBm = 2.0;
  silvus_msgs::SilvusStreamscapeNode node3;
  node3.robot_name = "";
  node3.node_id = 3;
  node3.node_label = "scom3";
  node3.txpw_requested_dBm = 5.0;
  node3.txpw_actual_dBm = 2.0;
  silvus_msgs::SilvusStreamscapeNode node4;
  node4.robot_name = "";
  node4.node_id = 4;
  node4.node_label = "scom4";
  node4.txpw_requested_dBm = 15.0;
  node4.txpw_actual_dBm = 12.0;
  silvus_msgs::SilvusStreamscapeNode node5;
  node5.robot_name = "spot2";
  node5.node_label = "scom-spot2";
  node5.node_id = 5;
  node5.txpw_requested_dBm = 15.0;
  node5.txpw_actual_dBm = 12.0;

  msg->nodes.push_back(node1);
  msg->nodes.push_back(node2);
  msg->nodes.push_back(node3);
  msg->nodes.push_back(node4);
  msg->nodes.push_back(node5);

  auto rssi_scom_robot = getRssiScomRobotList();
  commNodeRawCallback(msg);
  EXPECT_EQ(rssi_scom_robot.size(), 8);
  rssi_scom_robot = getRssiScomRobotList();
  EXPECT_EQ(rssi_scom_robot["scom-spot2"].node_id, 5);
  EXPECT_NEAR(rssi_scom_robot["scom-spot2"].txpw_requested_dBm, 15.0, 0.1);
  EXPECT_NEAR(rssi_scom_robot["scom-spot2"].txpw_actual_dBm, 12.0, 0.1);
  EXPECT_EQ(rssi_scom_robot["scom-husky4"].node_id, 1);
  EXPECT_NEAR(rssi_scom_robot["scom-husky4"].txpw_requested_dBm, 5.0, 0.1);
  EXPECT_NEAR(rssi_scom_robot["scom-husky4"].txpw_actual_dBm, 2.0, 0.1);

  auto get_time_stamp_last_updates = getRssiScomRobotListUpdatedTimeStamp();
  EXPECT_NEAR(get_time_stamp_last_updates["scom-husky4"].toSec(),
              time_stamp1.toSec(),
              0.1);
  EXPECT_NEAR(get_time_stamp_last_updates["scom-spot2"].toSec(),
              time_stamp1.toSec(),
              0.1);

  silvus_msgs::SilvusStreamscape::Ptr msg2(new silvus_msgs::SilvusStreamscape);

  auto time_stamp2 = ros::Time::now() + ros::Duration(10 * 60 * 60);

  EXPECT_NE(time_stamp1.toSec(), time_stamp2.toSec());

  msg2->header.stamp = time_stamp2;

  silvus_msgs::SilvusStreamscapeNode node12;
  node12.robot_name = "husky4";
  node12.node_label = "scom-husky4";
  node12.node_id = 1;
  node12.txpw_requested_dBm = 52.0;
  node12.txpw_actual_dBm = 22.0;
  silvus_msgs::SilvusStreamscapeNode node22;
  node22.robot_name = "";
  node22.node_id = 2;
  node22.node_label = "scom2";
  node22.txpw_requested_dBm = 52.0;
  node22.txpw_actual_dBm = 22.0;
  silvus_msgs::SilvusStreamscapeNode node32;
  node32.robot_name = "";
  node32.node_id = 3;
  node32.node_label = "scom3";
  node32.txpw_requested_dBm = 52.0;
  node32.txpw_actual_dBm = 22.0;
  silvus_msgs::SilvusStreamscapeNode node42;
  node42.robot_name = "";
  node42.node_id = 4;
  node42.node_label = "scom4";
  node42.txpw_requested_dBm = 152.0;
  node42.txpw_actual_dBm = 122.0;
  silvus_msgs::SilvusStreamscapeNode node52;
  node52.robot_name = "spot2";
  node52.node_label = "scom-spot2";
  node52.node_id = 5;
  node52.txpw_requested_dBm = 152.0;
  node52.txpw_actual_dBm = 122.0;

  msg2->nodes.push_back(node12);
  msg2->nodes.push_back(node22);
  msg2->nodes.push_back(node32);
  msg2->nodes.push_back(node42);
  msg2->nodes.push_back(node52);

  commNodeRawCallback(msg2);
  EXPECT_EQ(rssi_scom_robot.size(), 8);
  rssi_scom_robot = getRssiScomRobotList();
  EXPECT_EQ(rssi_scom_robot["scom-spot2"].node_id, 5);
  EXPECT_NEAR(rssi_scom_robot["scom-spot2"].txpw_requested_dBm, 152.0, 0.1);
  EXPECT_NEAR(rssi_scom_robot["scom-spot2"].txpw_actual_dBm, 122.0, 0.1);
  EXPECT_EQ(rssi_scom_robot["scom-husky4"].node_id, 1);
  EXPECT_NEAR(rssi_scom_robot["scom-husky4"].txpw_requested_dBm, 52.0, 0.1);
  EXPECT_NEAR(rssi_scom_robot["scom-husky4"].txpw_actual_dBm, 22.0, 0.1);

  get_time_stamp_last_updates = getRssiScomRobotListUpdatedTimeStamp();
  EXPECT_NEAR(get_time_stamp_last_updates["scom-husky4"].toSec(),
              time_stamp2.toSec(),
              0.1);
  EXPECT_NEAR(get_time_stamp_last_updates["scom-spot2"].toSec(),
              time_stamp2.toSec(),
              0.1);
}

TEST_F(TestRSSILoopGeneration, CommNodeAggregatedStatusCallback) {
  ros::NodeHandle nh;
  bool init = rssi_lc_.Initialize(nh);
  pose_graph_msgs::PoseGraph::Ptr graph_msg(new pose_graph_msgs::PoseGraph);
  pose_graph_msgs::PoseGraphNode node1, node2, node3, node4, node5, node6,
      node12, node22, node32, node42, node52, node62;
  node1.key = gtsam::Symbol('f', 0);
  node1.header.stamp = ros::Time::now() + ros::Duration(1 * 60 * 60);
  node2.key = gtsam::Symbol('f', 1);
  node2.header.stamp = ros::Time::now() + ros::Duration(2 * 60 * 60);
  node3.key = gtsam::Symbol('f', 0);
  node3.header.stamp = ros::Time::now() + ros::Duration(3 * 60 * 60);
  node4.key = gtsam::Symbol('f', 1);
  node4.header.stamp = ros::Time::now() + ros::Duration(4 * 60 * 60);
  node5.key = gtsam::Symbol('f', 2);
  node5.header.stamp = ros::Time::now() + ros::Duration(5 * 60 * 60);
  node6.key = gtsam::Symbol('f', 3);
  node6.header.stamp = ros::Time::now() + ros::Duration(6 * 60 * 60);
  node12.key = gtsam::Symbol('d', 0);
  node12.header.stamp = ros::Time::now() + ros::Duration(1 * 60 * 60);
  node22.key = gtsam::Symbol('d', 1);
  node22.header.stamp = ros::Time::now() + ros::Duration(2 * 60 * 60);
  node32.key = gtsam::Symbol('d', 0);
  node32.header.stamp = ros::Time::now() + ros::Duration(3 * 60 * 60);
  node42.key = gtsam::Symbol('d', 1);
  node42.header.stamp = ros::Time::now() + ros::Duration(4 * 60 * 60);
  node52.key = gtsam::Symbol('d', 2);
  node52.header.stamp = ros::Time::now() + ros::Duration(5 * 60 * 60);
  node62.key = gtsam::Symbol('d', 3);
  node62.header.stamp = ros::Time::now() + ros::Duration(6 * 60 * 60);
  graph_msg->nodes.push_back(node1);
  graph_msg->nodes.push_back(node2);
  graph_msg->nodes.push_back(node3);
  graph_msg->nodes.push_back(node4);
  graph_msg->nodes.push_back(node5);
  graph_msg->nodes.push_back(node6);
  graph_msg->nodes.push_back(node12);
  graph_msg->nodes.push_back(node22);
  graph_msg->nodes.push_back(node32);
  graph_msg->nodes.push_back(node42);
  graph_msg->nodes.push_back(node52);
  graph_msg->nodes.push_back(node62);
  keyedPoseCallback(graph_msg);

  core_msgs::CommNodeStatus::Ptr node_status(new core_msgs::CommNodeStatus());

  core_msgs::CommNodeInfo remain;
  remain.hostname = "scom1";
  remain.uwb_id = "A32";
  remain.robot_name = "husky4";
  remain.pose_graph_key = 151241241241;
  remain.relative_pose = geometry_msgs::Pose();

  core_msgs::CommNodeInfo remain2;
  remain2.hostname = "scom2";
  remain2.uwb_id = "A322";
  remain2.robot_name = "husky4";
  remain2.pose_graph_key = 151241241242;
  remain2.relative_pose = geometry_msgs::Pose();

  core_msgs::CommNodeInfo remain3;
  remain3.hostname = "scom3";
  remain3.uwb_id = "A321111";
  remain3.robot_name = "husky4";
  remain3.pose_graph_key = 151241241243;
  remain3.relative_pose = geometry_msgs::Pose();

  core_msgs::CommNodeInfo remain4;
  remain4.hostname = "scom4";
  remain4.uwb_id = "A321111";
  remain4.robot_name = "husky4";
  remain4.pose_graph_key = 151241241244;
  remain4.relative_pose = geometry_msgs::Pose();

  core_msgs::CommNodeInfo remain5;
  remain5.hostname = "scom-husky4";
  remain5.uwb_id = "A321111";
  remain5.robot_name = "husky4";
  remain5.pose_graph_key = 151241241245;
  remain5.relative_pose = geometry_msgs::Pose();

  EXPECT_EQ(getScomDroppedList().size(), 0);
  EXPECT_EQ(getScomDroppedListTimeStamps().size(), 0);

  commNodeAggregatedStatusCallback(node_status);
  EXPECT_EQ(getScomDroppedList().size(), 0);
  EXPECT_EQ(getScomDroppedListTimeStamps().size(), 0);

  auto time_stamp1 = node_status->header.stamp =
      ros::Time(0) + ros::Duration(1 * 60 * 60);
  node_status->dropped.push_back(remain);
  commNodeAggregatedStatusCallback(node_status);
  EXPECT_EQ(getScomDroppedList().size(), 1);
  EXPECT_EQ(getScomDroppedListTimeStamps().size(), 1);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain.hostname].toSec(),
              time_stamp1.toSec(),
              0.01);

  auto time_stamp2 = node_status->header.stamp =
      ros::Time(0) + ros::Duration(2 * 60 * 60);
  node_status->dropped.push_back(remain2);
  commNodeAggregatedStatusCallback(node_status);
  EXPECT_EQ(getScomDroppedList().size(), 2);
  EXPECT_EQ(getScomDroppedListTimeStamps().size(), 2);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain.hostname].toSec(),
              time_stamp1.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain2.hostname].toSec(),
              time_stamp2.toSec(),
              0.01);

  auto time_stamp3 = node_status->header.stamp =
      ros::Time(0) + ros::Duration(3 * 60 * 60);
  node_status->dropped.push_back(remain3);
  commNodeAggregatedStatusCallback(node_status);
  EXPECT_EQ(getScomDroppedList().size(), 3);
  EXPECT_EQ(getScomDroppedListTimeStamps().size(), 3);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain.hostname].toSec(),
              time_stamp1.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain2.hostname].toSec(),
              time_stamp2.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain3.hostname].toSec(),
              time_stamp3.toSec(),
              0.01);

  auto time_stamp4 = node_status->header.stamp =
      ros::Time(0) + ros::Duration(4 * 60 * 60);
  node_status->dropped.push_back(remain4);
  commNodeAggregatedStatusCallback(node_status);
  EXPECT_EQ(getScomDroppedList().size(), 4);
  EXPECT_EQ(getScomDroppedListTimeStamps().size(), 4);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain.hostname].toSec(),
              time_stamp1.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain2.hostname].toSec(),
              time_stamp2.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain3.hostname].toSec(),
              time_stamp3.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain4.hostname].toSec(),
              time_stamp4.toSec(),
              0.01);

  commNodeAggregatedStatusCallback(node_status);
  EXPECT_EQ(getScomDroppedList().size(), 4);
  EXPECT_EQ(getScomDroppedListTimeStamps().size(), 4);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain.hostname].toSec(),
              time_stamp1.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain2.hostname].toSec(),
              time_stamp2.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain3.hostname].toSec(),
              time_stamp3.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain4.hostname].toSec(),
              time_stamp4.toSec(),
              0.01);
  commNodeAggregatedStatusCallback(node_status);

  auto time_stamp5 = node_status->header.stamp =
      ros::Time(0) + ros::Duration(5 * 60 * 60);
  node_status->dropped.push_back(remain5);
  EXPECT_EQ(getScomDroppedList().size(), 4);
  EXPECT_EQ(getScomDroppedListTimeStamps().size(), 4);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain.hostname].toSec(),
              time_stamp1.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain2.hostname].toSec(),
              time_stamp2.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain3.hostname].toSec(),
              time_stamp3.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain4.hostname].toSec(),
              time_stamp4.toSec(),
              0.01);

  commNodeAggregatedStatusCallback(node_status);
  EXPECT_EQ(getScomDroppedList().size(), 4);
  EXPECT_EQ(getScomDroppedListTimeStamps().size(), 4);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain.hostname].toSec(),
              time_stamp1.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain2.hostname].toSec(),
              time_stamp2.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain3.hostname].toSec(),
              time_stamp3.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain4.hostname].toSec(),
              time_stamp4.toSec(),
              0.01);

  commNodeAggregatedStatusCallback(node_status);
  EXPECT_EQ(getScomDroppedList().size(), 4);
  EXPECT_EQ(getScomDroppedListTimeStamps().size(), 4);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain.hostname].toSec(),
              time_stamp1.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain2.hostname].toSec(),
              time_stamp2.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain3.hostname].toSec(),
              time_stamp3.toSec(),
              0.01);
  EXPECT_NEAR(getScomDroppedListTimeStamps()[remain4.hostname].toSec(),
              time_stamp4.toSec(),
              0.01);
}

TEST_F(TestRSSILoopGeneration, RadioToNodesLoopClosure) {
  ros::NodeHandle nh;
  bool init = rssi_lc_.Initialize(nh);

  pose_graph_msgs::PoseGraph::Ptr graph_msg(new pose_graph_msgs::PoseGraph);
  pose_graph_msgs::PoseGraphNode node1, node2, node3, node4, node5, node6,
      node12, node22, node32, node42, node52, node62;
  node1.key = gtsam::Symbol('f', 0);
  node1.header.stamp = ros::Time::now() + ros::Duration(1 * 60 * 60);
  node2.key = gtsam::Symbol('f', 1);
  node2.header.stamp = ros::Time::now() + ros::Duration(2 * 60 * 60);
  node12.key = gtsam::Symbol('d', 0);
  node12.header.stamp = ros::Time::now() + ros::Duration(1 * 60 * 60);
  node22.key = gtsam::Symbol('d', 1);
  node22.header.stamp = ros::Time::now() + ros::Duration(2 * 60 * 60);
  graph_msg->nodes.push_back(node1);
  graph_msg->nodes.push_back(node2);
  graph_msg->nodes.push_back(node12);
  graph_msg->nodes.push_back(node22);
  keyedPoseCallback(graph_msg);

  silvus_msgs::SilvusStreamscape::Ptr msg(new silvus_msgs::SilvusStreamscape);
  core_msgs::CommNodeStatus::Ptr node_status(new core_msgs::CommNodeStatus());
  core_msgs::CommNodeInfo remain;
  remain.hostname = "scom1";
  remain.uwb_id = "A32";
  remain.robot_name = "husky4";
  remain.pose_graph_key = 151241241241;
  remain.relative_pose = geometry_msgs::Pose();

  core_msgs::CommNodeInfo remain2;
  remain2.hostname = "scom2";
  remain2.uwb_id = "A322";
  remain2.robot_name = "husky4";
  remain2.pose_graph_key = 151241241242;
  remain2.relative_pose = geometry_msgs::Pose();

  auto time_stamp1 = node_status->header.stamp =
      ros::Time(0) + ros::Duration(1 * 60 * 60);
  node_status->dropped.push_back(remain);
  auto time_stamp2 = node_status->header.stamp =
      ros::Time(0) + ros::Duration(2 * 60 * 60);
  node_status->dropped.push_back(remain2);
  commNodeAggregatedStatusCallback(node_status);

  silvus_msgs::SilvusStreamscapeNode comm_node1;
  comm_node1.robot_name = "husky4";
  comm_node1.node_label = "scom-husky4";
  comm_node1.node_id = 1;
  comm_node1.txpw_requested_dBm = 5.0;
  comm_node1.txpw_actual_dBm = 2.0;
  silvus_msgs::SilvusStreamscapeNeighbor neighbour1;
  neighbour1.neighbor_node_label = "scom1";
  neighbour1.my_txpw_actual_dBm = 120000;
  neighbour1.received_signal_power_dBm.push_back(-1);
  neighbour1.received_signal_power_dBm.push_back(-2);
  silvus_msgs::SilvusStreamscapeNeighbor neighbour2;
  neighbour2.neighbor_node_label = "scom2";
  neighbour2.my_txpw_actual_dBm = 1;
  neighbour2.received_signal_power_dBm.push_back(-3);
  neighbour2.received_signal_power_dBm.push_back(-3);
  silvus_msgs::SilvusStreamscapeNeighbor neighbour3;
  neighbour3.neighbor_node_label = "scom3";
  neighbour3.my_txpw_actual_dBm = 1;
  neighbour3.received_signal_power_dBm.push_back(-2);
  neighbour3.received_signal_power_dBm.push_back(-2);
  silvus_msgs::SilvusStreamscapeNeighbor neighbour4;
  neighbour4.neighbor_node_label = "scom4";
  neighbour4.my_txpw_actual_dBm = 1;
  neighbour4.received_signal_power_dBm.push_back(-1);
  neighbour4.received_signal_power_dBm.push_back(-1);
  silvus_msgs::SilvusStreamscapeNeighbor neighbour5;
  neighbour5.neighbor_node_label = "scom5";
  neighbour5.my_txpw_actual_dBm = 1;
  neighbour5.received_signal_power_dBm.push_back(-51);
  neighbour5.received_signal_power_dBm.push_back(-12);
  comm_node1.neighbors.push_back(neighbour1);
  comm_node1.neighbors.push_back(neighbour2);
  comm_node1.neighbors.push_back(neighbour3);
  comm_node1.neighbors.push_back(neighbour4);
  comm_node1.neighbors.push_back(neighbour5);
  msg->nodes.push_back(comm_node1);
  commNodeRawCallback(msg);
  rssiTimerCallback();

  EXPECT_EQ(getScomDroppedList()["scom1"].nodes_around_comm.size(), 1);
  // TODO: work it out
  //  std::vector<pose_graph_msgs::LoopCandidate> candidates = getCandidates();
  //  EXPECT_EQ(1, candidates.size());
  //  std::cout << "candidateS: " << candidates.size() << std::endl;
}
} // namespace lamp_loop_closure

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_loop_generation");
  return RUN_ALL_TESTS();
}
