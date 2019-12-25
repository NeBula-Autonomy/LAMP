/**
 *  @brief Testing the LaserLoopClosure class
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

class TestLaserLoopClosure : public ::testing::Test {
  public:
    TestLaserLoopClosure() :
      lc(ros::NodeHandle()) {
      // Set params
    }
    ~TestLaserLoopClosure() {}

    // Pass-through and getter/setter methods
    pose_graph_msgs::PoseGraphEdge AddLoopClosure(
            gtsam::Symbol key1, 
            gtsam::Symbol key2,
            geometry_utils::Transform3& delta,
            gtsam::Matrix66& covariance) {
      lc.CreateLoopClosureEdge(key1, key2, delta, covariance);
    }
    pose_graph_msgs::PoseGraphEdge AddLoopClosure(
            gtsam::Symbol key1, 
            gtsam::Symbol key2) {
      geometry_utils::Transform3 delta;
      gtsam::Matrix66 covariance;
      return AddLoopClosure(key1, key2, delta, covariance);
    }    
    pose_graph_msgs::PoseGraphEdge AddLoopClosure(
            char c1, 
            int i1, 
            char c2, 
            int i2) {
      return AddLoopClosure(gtsam::Symbol(c1, i1), gtsam::Symbol(c2, i2));
    }

    gtsam::Key GetLastClosureKey(
            gtsam::Symbol key1, 
            gtsam::Symbol key2) { 
      return GetLastClosureKey(key1.chr(), key2.chr()); 
    }
    gtsam::Key GetLastClosureKey(
            char c1, 
            char c2) { 
      return lc.last_closure_key_[{c1, c2}]; 
    }
    LaserLoopClosure lc;

  protected:
    // Tolerance on EXPECT_NEAR assertions
    double tolerance_ = 1e-5;

  private:
};

TEST_F(TestLaserLoopClosure, LastClosureKeySingleRobot) {

  // chronological order test
  AddLoopClosure('a', 10, 'a', 20);
  EXPECT_EQ(GetLastClosureKey('a', 'a'), gtsam::Symbol('a', 20));

  // reverse order test
  AddLoopClosure('b', 100, 'b', 30);
  EXPECT_EQ(GetLastClosureKey('b', 'b'), gtsam::Symbol('b', 100));
}


TEST_F(TestLaserLoopClosure, LastClosureKeyMultiRobot) {

  gtsam::Symbol key1, key2;
  geometry_utils::Transform3 delta;
  gtsam::Matrix66 covariance;

  // loop closures between every combination
  AddLoopClosure('a', 99, 'a', 1);
  AddLoopClosure('b', 31, 'b', 66);
  AddLoopClosure('c', 70, 'c', 5);
  AddLoopClosure('a', 10, 'b', 20);
  AddLoopClosure('c', 15, 'a', 30);
  AddLoopClosure('b', 24, 'c', 17);

  // test the entire matrix
  EXPECT_EQ(GetLastClosureKey('a', 'a'), gtsam::Symbol('a', 99));
  EXPECT_EQ(GetLastClosureKey('b', 'b'), gtsam::Symbol('b', 66));
  EXPECT_EQ(GetLastClosureKey('c', 'c'), gtsam::Symbol('c', 70));
  EXPECT_EQ(GetLastClosureKey('a', 'b'), gtsam::Symbol('a', 10));
  EXPECT_EQ(GetLastClosureKey('a', 'c'), gtsam::Symbol('a', 30));
  EXPECT_EQ(GetLastClosureKey('b', 'a'), gtsam::Symbol('b', 20));
  EXPECT_EQ(GetLastClosureKey('b', 'c'), gtsam::Symbol('b', 24));
  EXPECT_EQ(GetLastClosureKey('c', 'a'), gtsam::Symbol('c', 15));
  EXPECT_EQ(GetLastClosureKey('c', 'b'), gtsam::Symbol('c', 17));

  // updates to every combination
  AddLoopClosure('a', 150, 'a', 23);
  AddLoopClosure('b', 75, 'b', 61);
  AddLoopClosure('c', 99, 'c', 0);
  AddLoopClosure('a', 35, 'b', 12);
  AddLoopClosure('c', 1, 'a', 3);
  AddLoopClosure('b', 999, 'c', 5);

  // test the entire matrix for update
  EXPECT_EQ(GetLastClosureKey('a', 'a'), gtsam::Symbol('a', 150));
  EXPECT_EQ(GetLastClosureKey('b', 'b'), gtsam::Symbol('b', 75));
  EXPECT_EQ(GetLastClosureKey('c', 'c'), gtsam::Symbol('c', 99));
  EXPECT_EQ(GetLastClosureKey('a', 'b'), gtsam::Symbol('a', 35));
  EXPECT_EQ(GetLastClosureKey('a', 'c'), gtsam::Symbol('a', 30));
  EXPECT_EQ(GetLastClosureKey('b', 'a'), gtsam::Symbol('b', 20));
  EXPECT_EQ(GetLastClosureKey('b', 'c'), gtsam::Symbol('b', 999));
  EXPECT_EQ(GetLastClosureKey('c', 'a'), gtsam::Symbol('c', 15));
  EXPECT_EQ(GetLastClosureKey('c', 'b'), gtsam::Symbol('c', 17));
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_loop_closure");
  return RUN_ALL_TESTS();
}