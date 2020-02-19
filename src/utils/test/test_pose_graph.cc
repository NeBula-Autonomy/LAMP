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
#include <gtsam/linear/NoiseModel.h>

#include <utils/CommonFunctions.h>
#include <utils/CommonStructs.h>

class TestPoseGraphClass : public ::testing::Test {
  public:

    PoseGraph pose_graph_;
    pose_graph_msgs::PoseGraphNode n0, n1;
    pose_graph_msgs::PoseGraphEdge e0;
    gtsam::Symbol initial_key_;
    gtsam::Vector6 initial_noise_;


    TestPoseGraphClass() {
      // Set params

      n0.key = gtsam::Symbol('a', 1);
      n0.pose.position.x = 0.0;
      n0.pose.position.y = 0.0;
      n0.pose.position.z = 0.0;

      n1.key = gtsam::Symbol('a', 2);
      n1.pose.position.x = 1.0;
      n1.pose.position.y = 0.0;
      n1.pose.position.z = 0.0;

      e0.key_from = n0.key;
      e0.key_to = n1.key;
      e0.pose.position.x = 1.0;
      e0.pose.position.y = 0.0;
      e0.pose.position.z = 0.0;

      initial_noise_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    }
    ~TestPoseGraphClass() {}

  protected:

    double tolerance_ = 1e-5;
  private:
};

TEST_F(TestPoseGraphClass, TestInitialize) {
  ros::Time::init();

  gtsam::noiseModel::Diagonal::shared_ptr covariance(
      gtsam::noiseModel::Diagonal::Sigmas(initial_noise_));

  pose_graph_.Initialize(initial_key_, gtsam::Pose3(), covariance);

  EXPECT_EQ(pose_graph_.GetValues().size(), 1);
  EXPECT_EQ(pose_graph_.GetNfg().size(), 1);

}

TEST_F(TestPoseGraphClass, TestTrackNode) {
  ros::Time::init();
  gtsam::noiseModel::Diagonal::shared_ptr covariance(
    gtsam::noiseModel::Diagonal::Sigmas(initial_noise_));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  pose_graph_.Initialize(initial_key_, gtsam::Pose3(), covariance);
  
  pose_graph_.TrackNode(ros::Time(10.0), gtsam::Symbol('a', 1), gtsam::Pose3(), noise);
  pose_graph_.TrackNode(ros::Time(30.0), gtsam::Symbol('a', 2), gtsam::Pose3(), noise);

  EXPECT_EQ(pose_graph_.GetValues().size(), 3);
  EXPECT_EQ(pose_graph_.GetNfg().size(), 1);

  // Something to track the addition of the node messages

}

TEST_F(TestPoseGraphClass, TestTrackNodeWithNode) {
  ros::Time::init();
  gtsam::noiseModel::Diagonal::shared_ptr covariance(
    gtsam::noiseModel::Diagonal::Sigmas(initial_noise_));

  pose_graph_.Initialize(initial_key_, gtsam::Pose3(), covariance);
  
  pose_graph_.TrackNode(n0);
  pose_graph_.TrackNode(n1);

  EXPECT_EQ(pose_graph_.GetValues().size(), 3);
  EXPECT_EQ(pose_graph_.GetNfg().size(), 1);

  // Something to track the addition of the node messages

}

TEST_F(TestPoseGraphClass, TestTrackEdges) {
  ros::Time::init();
  gtsam::noiseModel::Diagonal::shared_ptr covariance(
    gtsam::noiseModel::Diagonal::Sigmas(initial_noise_));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  pose_graph_.Initialize(initial_key_, gtsam::Pose3(), covariance);
  
  pose_graph_.TrackFactor(gtsam::Symbol('a', 0), gtsam::Symbol('a', 1), pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(), noise);
  pose_graph_.TrackFactor(gtsam::Symbol('a', 1), gtsam::Symbol('a', 2), pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(1.0, 0.0, 0.0) ), noise);

  EXPECT_EQ(pose_graph_.GetValues().size(), 1);
  EXPECT_EQ(pose_graph_.GetNfg().size(), 3);
}


TEST_F(TestPoseGraphClass, TestTrackEdgesAndNodes) {
  ros::Time::init();
  gtsam::noiseModel::Diagonal::shared_ptr covariance(
    gtsam::noiseModel::Diagonal::Sigmas(initial_noise_));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  pose_graph_.Initialize(initial_key_, gtsam::Pose3(), covariance);
  
  pose_graph_.TrackNode(n0);
  pose_graph_.TrackNode(n1);

  pose_graph_.TrackFactor(gtsam::Symbol('a', 0), gtsam::Symbol('a', 1), pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(1.0, 0.0, 0.0) ), noise);
  pose_graph_.TrackFactor(e0);

  EXPECT_EQ(pose_graph_.GetValues().size(), 3);
  EXPECT_EQ(pose_graph_.GetNfg().size(), 3);
}

TEST_F(TestPoseGraphClass, Multi_robot_add){
  ros::Time::init();
  gtsam::noiseModel::Diagonal::shared_ptr covariance(
    gtsam::noiseModel::Diagonal::Sigmas(initial_noise_));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  pose_graph_.Initialize(initial_key_, gtsam::Pose3(), covariance);

  // Robot a node
  pose_graph_.TrackNode(n0);
  pose_graph_.TrackNode(n1);
  // Robot b node
  pose_graph_.TrackNode(ros::Time(1.0), gtsam::Symbol('b', 0), gtsam::Pose3(), noise);
  pose_graph_.TrackNode(ros::Time(3.0), gtsam::Symbol('b', 1), gtsam::Pose3(), noise);

  // Robot a factors
  pose_graph_.TrackFactor(gtsam::Symbol('a', 0), gtsam::Symbol('a', 1), pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(1.0, 0.0, 0.0) ), noise);
  pose_graph_.TrackFactor(e0);

  // Robot b factors 
  pose_graph_.TrackPrior(gtsam::Symbol('b', 0), gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(0.0, 0.0, 0.0) ), noise);
  pose_graph_.TrackFactor(gtsam::Symbol('b', 0), gtsam::Symbol('b', 1), pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(2.0, 0.0, 0.0) ), noise);

  EXPECT_EQ(pose_graph_.GetValues().size(), 5);
  EXPECT_EQ(pose_graph_.GetNfg().size(), 5);
  
}


// TEST_F(TestPoseGraphClass, RemoveRobotFromGraph){
//   // Add to graph
//   static const gtsam::SharedNoiseModel& noise =
//       gtsam::noiseModel::Isotropic::Variance(6, 0.1);

//   pose_graph_.Initialize(initial_key_, gtsam::Pose3(), noise);

//   // Robot a node
//   pose_graph_.TrackNode(n0);
//   pose_graph_.TrackNode(n1);
//   // Robot b node
//   pose_graph_.TrackNode(ros::Time(1.0), gtsam::Symbol('b', 0), gtsam::Pose3(), noise);
//   pose_graph_.TrackNode(ros::Time(3.0), gtsam::Symbol('b', 1), gtsam::Pose3(), noise);

//   // Robot a factors
//   pose_graph_.TrackFactor(gtsam::Symbol('a', 0), gtsam::Symbol('a', 1), pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(1.0, 0.0, 0.0) ), noise);
//   pose_graph_.TrackFactor(e0);

//   // Robot b factors 
//   pose_graph_.TrackFactor(gtsam::Symbol('b', 0), gtsam::Symbol('b', 0), pose_graph_msgs::PoseGraphEdge::PRIOR, gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(0.0, 0.0, 0.0) ), noise);
//   pose_graph_.TrackFactor(gtsam::Symbol('b', 0), gtsam::Symbol('b', 1), pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(2.0, 0.0, 0.0) ), noise);

//   // Make a copy 
//   pose_graph_back = pose_graph_;

//   // Remove robot a
//   std::string robot_name = "husky1";
//   pose_graph_.RemoveRobotFromGraph(robot_name);

//   EXPECT_EQ(pose_graph_.GetValues().size(), 2);
//   EXPECT_EQ(pose_graph_.GetNfg().size(), 2);

//   // Remove robot b 
//   robot_name = "husky2";
//   pose_graph_back.RemoveRobotFromGraph(robot_name);

//   EXPECT_EQ(pose_graph_back.GetValues().size(), 3);
//   EXPECT_EQ(pose_graph_back.GetNfg().size(), 3);
// }


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_utils");
  return RUN_ALL_TESTS();
}
