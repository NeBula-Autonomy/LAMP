/**
 *  @brief Test cases for talker class
 *
 *  This shows an example usage of gtest.
 *  The fixture pattern is used to execute the common operations
 *  before and after each test case.
 */

#include <gtest/gtest.h>

#include "point_cloud_odometry/PointCloudOdometry.h"

class PointCloudOdometryTest : public ::testing::Test {
protected:
  virtual void SetUp() {
  }

  virtual void TearDown() {
  }
};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PointCloudOdometryTest");
  return RUN_ALL_TESTS();
}
