/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for point_cloud_visualizer
 */

#include <gtest/gtest.h>

#include "point_cloud_visualizer/PointCloudVisualizer.h"

class PointCloudVisualizerTest : public ::testing::Test {
protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PointCloudVisualizerTest");
  return RUN_ALL_TESTS();
}
