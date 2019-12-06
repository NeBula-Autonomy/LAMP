/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include <math.h>
#include <ros/ros.h>

#include <pose_graph_merger/TwoPoseGraphMerge.h>

class TestTwoPoseGraphMerge : public ::testing::Test {
public: 

  TestTwoPoseGraphMerge() {
    // Set params
  }
  ~TestTwoPoseGraphMerge() {}

  TwoPoseGraphMerge merger_;

protected:
  // Tolerance on EXPECT_NEAR assertions
  double tolerance_ = 1e-5;

private:
};

TEST_F(TestTwoPoseGraphMerge, Initialization){
  // ros::init(argc, argv, "two_pg_merge");
  ros::NodeHandle n("~");

  bool result = merger_.Initialize(n);

  EXPECT_TRUE(result);
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_pose_graph_merger");
  return RUN_ALL_TESTS();
}