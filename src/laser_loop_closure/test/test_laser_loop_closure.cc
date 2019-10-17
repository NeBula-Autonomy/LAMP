/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for laser_loop_closure class
 *
 */

#include <gtest/gtest.h>

#include "laser_loop_closure/LaserLoopClosure.h"

class LaserLoopClosureTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    test_variable = new LaserLoopClosure();
    test_point_cloud = nullptr;
  }

  virtual void TearDown() {}
  LaserLoopClosure* test_variable;
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>* test_point_cloud;
};

/**
 * Checks for null pointer (no pointcloud) for maximum likelihood point
 * estimation
 */
TEST_F(LaserLoopClosureTest, GetMaximumLikelihoodPoints) {
  bool result = test_variable->GetMaximumLikelihoodPoints(test_point_cloud);
  EXPECT_EQ(result, false);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "LaserLoopClosureTest");
  return RUN_ALL_TESTS();
}
