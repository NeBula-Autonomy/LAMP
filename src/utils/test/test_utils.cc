/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include "utils/CommonFunctions.h"

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
  return RUN_ALL_TESTS();
}
