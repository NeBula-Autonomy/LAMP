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

#include <loop_closure/LaserLoopClosure.h>

class TestLoopClosure : public ::testing::Test {
  public:
    TestLoopClosure() :
      lc(ros::NodeHandle()) {
      // Set params
    }
    ~TestLoopClosure() {}

    LaserLoopClosure lc;

  protected:
    // Tolerance on EXPECT_NEAR assertions
    double tolerance_ = 1e-5;

  private:
};

TEST_F(TestLoopClosure, LastClosureKeyMatrix) {

  EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_loop_closure");
  return RUN_ALL_TESTS();
}