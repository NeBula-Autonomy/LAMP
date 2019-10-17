/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include <math.h>
#include <ros/ros.h>

#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

#include <utils/CommonFunctions.h>
#include <utils/CommonStructs.h>

class TestUtils : public ::testing::Test {
public:
  TestUtils() {
    // Set params
  }
  ~TestUtils() {}

protected:
private:
};

TEST_F(TestUtils, PoseGraphMsgToGtsam) {
  ros::NodeHandle nh, pnh("~");

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values values;
  pose_graph_msgs::PoseGraph g;

  // Make the graph
  pose_graph_msgs::PoseGraphNode n0, n1, n2, a0;
  pose_graph_msgs::PoseGraphEdge e0, e1, e2;

  n0.key = gtsam::Symbol('a', 0);
  n0.pose.position.x = 0.0;
  n0.pose.position.y = 0.0;
  n0.pose.position.z = 0.0;

  n1.key = gtsam::Symbol('a', 1);
  n1.pose.position.x = 1.0;
  n1.pose.position.y = 0.0;
  n1.pose.position.z = 0.0;

  n2.key = gtsam::Symbol('a', 2);
  n2.pose.position.x = 2.0;
  n2.pose.position.y = 0.0;
  n2.pose.position.z = 0.0;

  a0.key = gtsam::Symbol('m', 0);
  a0.pose.position.x = 1.0;
  a0.pose.position.y = 1.0;
  a0.pose.position.z = 0.0;

  e0.key_from = n0.key;
  e0.key_to = n1.key;
  e0.pose.position.x = 1.0;
  e0.pose.position.y = 0.0;
  e0.pose.position.z = 0.0;

  e1.key_from = n1.key;
  e1.key_to = n2.key;
  e1.pose.position.x = 1.0;
  e1.pose.position.y = 0.0;
  e1.pose.position.z = 0.0;

  e2.key_from = n1.key;
  e2.key_to = a0.key;
  e2.pose.position.x = 0.0;
  e2.pose.position.y = 1.0;
  e2.pose.position.z = 0.0;

  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  g.nodes.push_back(n2);
  g.nodes.push_back(a0);
  g.edges.push_back(e0);
  g.edges.push_back(e1);
  g.edges.push_back(e2);

  pose_graph_msgs::PoseGraph::ConstPtr graph_msg(
      new pose_graph_msgs::PoseGraph(g));

  // Convert to gtsam type
  utils::PoseGraphMsgToGtsam(graph_msg, &nfg, &values);

  // Check the size of the GTSAM data structures
  EXPECT_EQ(3, nfg.size());
  EXPECT_EQ(4, values.size());

  // TODO test the nfg & values explicitly
}

TEST(TestCommonFunctions, TestGtsamToRosPose) {
  // Create gtsam pose3
  double x = 1.0;
  double y = 2.0;
  double z = 3.0;
  gtsam::Pose3 gtsam_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, y, z));

  // Test the function
  geometry_msgs::Pose ros_pose = utils::GtsamToRosMsg(gtsam_pose);

  // Test
  EXPECT_NEAR(ros_pose.position.x, x, 1e-7);
  EXPECT_NEAR(ros_pose.position.y, y, 1e-7);
  EXPECT_NEAR(ros_pose.position.z, z, 1e-7);

  // No rotation
  EXPECT_NEAR(ros_pose.orientation.w, 1.0, 1e-7);
}

TEST(TestCommonFunctions, TestGtsamToRosPoseCovar) {
  // Create gtsam pose3
  double x = 1.0;
  double y = 2.0;
  double z = 3.0;
  gtsam::Pose3 gtsam_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, y, z));

  gtsam::Matrix66 covariance;
  covariance(0, 0) = 1.0;

  // Test the function
  geometry_msgs::PoseWithCovariance ros_pose =
      utils::GtsamToRosMsg(gtsam_pose, covariance);

  // Test
  EXPECT_NEAR(ros_pose.pose.position.x, x, 1e-7);
  EXPECT_NEAR(ros_pose.pose.position.y, y, 1e-7);
  EXPECT_NEAR(ros_pose.pose.position.z, z, 1e-7);

  // No rotation
  EXPECT_NEAR(ros_pose.pose.orientation.w, 1.0, 1e-7);

  // Covariance
  EXPECT_NEAR(ros_pose.covariance[0], 1.0, 1e-7);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_utils");
  return RUN_ALL_TESTS();
}
