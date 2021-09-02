/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for point_cloud_visualizer
 */

#include <gtest/gtest.h>

#include "point_cloud_visualizer/PointCloudVisualizer.h"

class TestPointCloudVisualizer : public ::testing::Test {
protected:
  TestPointCloudVisualizer(){};
  ~TestPointCloudVisualizer(){};
  virtual void SetUp() {}

  virtual void TearDown() {}

  PointCloudVisualizer pc_vis_;

  bool isPointInsideTheCone(const gu::Transform3& current_pose,
                            const gu::Transform3 pose) {
    return pc_vis_.IsPointInsideTheCone(current_pose, pose);
  }
  bool isPointInsideTheNegativeCone(const gu::Transform3& current_pose,
                                    const gu::Transform3 pose) {
    return pc_vis_.IsPointInsideTheNegativeCone(current_pose, pose);
  }
};

// TEST_F(TestPointCloudVisualizer, TestSetInitialPositionNoParam) {
//  gu::Transform3 current_pose = gu::Transform3::Identity();
//  ROS_INFO_STREAM("RUNNNING OVER");
//  for (int i = -10; i < 10; ++i) {
//    for (int j = -10; j < 10; ++j) {
//      for (int k = -10; k < 10; ++k) {
//        gu::Transform3 pose = gu::Transform3::Identity();
//        pose.translation = gu::Vector3Base<double>(i, j, k);
//        if (isPointInsideTheCone(current_pose, pose)) {
//          ROS_INFO_STREAM(pose.translation
//                          << " Inside: "
//                          << isPointInsideTheCone(current_pose, pose)
//                          << " radius: "
//                          << std::sqrt(std::pow(pose.translation.X(), 2) +
//                                       std::pow(pose.translation.Y(), 2)));
//        }
//      }
//    }
//  }
//}

TEST_F(TestPointCloudVisualizer, TestNegative) {
  gu::Transform3 current_pose = gu::Transform3::Identity();
  ROS_INFO_STREAM("RUNNNING OVER32");
  for (int i = -10; i < 10; ++i) {
    for (int j = -10; j < 10; ++j) {
      for (int k = -10; k < 10; ++k) {
        gu::Transform3 pose = gu::Transform3::Identity();
        pose.translation = gu::Vector3Base<double>(i, j, k);
        //  ROS_INFO_STREAM("i: " << i << " j: " << j << "k: " << k);
        if (isPointInsideTheNegativeCone(current_pose, pose)) {
          ROS_INFO_STREAM(pose.translation
                          << " Inside: "
                          << " radius: "
                          << std::sqrt(std::pow(pose.translation.X(), 2) +
                                       std::pow(pose.translation.Y(), 2)));
        }
      }
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PointCloudVisualizerTest");
  return RUN_ALL_TESTS();
}
