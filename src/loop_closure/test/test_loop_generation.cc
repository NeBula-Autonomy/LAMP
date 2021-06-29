/**
 *  @brief Testing the Loop Closure Generation Classes
 *
 */

#include <gtest/gtest.h>

#include "loop_closure/LoopGeneration.h"
#include "loop_closure/ProximityLoopGeneration.h"

namespace lamp_loop_closure {
class TestLoopGeneration : public ::testing::Test {
 protected:
  TestLoopGeneration() {
    // Load params
    system("rosparam load $(rospack find lamp)/config/lamp_settings.yaml");
    system(
        "rosparam load $(rospack find "
        "loop_closure)/config/laser_parameters.yaml");
  }
  ~TestLoopGeneration() {}

  void generateLoops(const gtsam::Key& new_key) {
    proximity_lc_.GenerateLoops(new_key);
  }

  void keyedPoseCallback(
      const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
    proximity_lc_.KeyedPoseCallback(graph_msg);
  }

  std::vector<pose_graph_msgs::LoopCandidate> getCandidates() {
    return proximity_lc_.candidates_;
  }

  double distanceBetweenKeys(const gtsam::Symbol& key1,
                             const gtsam::Symbol& key2) {
    return proximity_lc_.DistanceBetweenKeys(key1, key2);
  }

  ProximityLoopGeneration proximity_lc_;
};

TEST_F(TestLoopGeneration, TestInitialize) {
  ros::NodeHandle nh;
  bool init = proximity_lc_.Initialize(nh);
  ASSERT_TRUE(init);
}

TEST_F(TestLoopGeneration, TestBetweenDistance) {
  ros::NodeHandle nh;
  bool init = proximity_lc_.Initialize(nh);
  pose_graph_msgs::PoseGraph::Ptr graph_msg(new pose_graph_msgs::PoseGraph);
  pose_graph_msgs::PoseGraphNode node1, node2;
  node1.key = gtsam::Symbol('a', 0);
  node2.key = gtsam::Symbol('b', 0);
  node2.pose.position.x = 3;
  graph_msg->nodes.push_back(node1);
  graph_msg->nodes.push_back(node2);

  keyedPoseCallback(graph_msg);

  double dist =
      distanceBetweenKeys(gtsam::Symbol('a', 0), gtsam::Symbol('b', 0));
  EXPECT_EQ(3, dist);
}

TEST_F(TestLoopGeneration, TestGenerateLoops) {
  ros::NodeHandle nh;
  bool init = proximity_lc_.Initialize(nh);
  pose_graph_msgs::PoseGraph::Ptr graph_msg(new pose_graph_msgs::PoseGraph);
  pose_graph_msgs::PoseGraphNode node1, node2, node3, node4, node5;
  node1.key = gtsam::Symbol('a', 0);
  node2.key = gtsam::Symbol('b', 0);
  node3.key = gtsam::Symbol('c', 0);
  node4.key = gtsam::Symbol('a', 1);
  node5.key = gtsam::Symbol('a', 100);
  node2.pose.position.x = 3;
  node3.pose.position.x = 1000;
  node4.pose.position.x = 2;
  node5.pose.position.x = 2;
  graph_msg->nodes.push_back(node1);
  graph_msg->nodes.push_back(node2);
  graph_msg->nodes.push_back(node3);
  graph_msg->nodes.push_back(node4);
  graph_msg->nodes.push_back(node5);

  keyedPoseCallback(graph_msg);

  std::vector<pose_graph_msgs::LoopCandidate> candidates = getCandidates();
  EXPECT_EQ(5, candidates.size());
  auto candidate0 = candidates[0];
  EXPECT_EQ(gtsam::Symbol('b', 0), candidate0.key_from);
  EXPECT_EQ(gtsam::Symbol('a', 0), candidate0.key_to);
  auto candidate2 = candidates[2];
  EXPECT_EQ(gtsam::Symbol('a', 100), candidate2.key_from);
  EXPECT_EQ(gtsam::Symbol('a', 0), candidate2.key_to);
}

}  // namespace lamp_loop_closure

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_loop_generation");
  return RUN_ALL_TESTS();
}
