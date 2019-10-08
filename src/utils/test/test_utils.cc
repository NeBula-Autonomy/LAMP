/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <math.h>

#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>

#include <utils/CommonFunctions.h>

class TestUtils : public ::testing::Test {

  public: 
    TestUtils(){
      // Set params

    }
    ~TestUtils(){}

  protected:

  private:

};


TEST_F(TestUtils, BasicMerge) {
  ros::NodeHandle nh, pnh("~");


    ASSERT_TRUE(true);
}



int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_utils");
  return RUN_ALL_TESTS();
}