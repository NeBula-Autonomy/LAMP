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

#include <pose_graph_merger/merger.h>

class TestMerger : public ::testing::Test {

  public: 
    TestMerger(){
      // Set params

    }
    ~TestMerger(){}

    Merger merger;

  private:
};


TEST_F(TestMerger, BasicMerge) {
  ros::NodeHandle nh, pnh("~");

  pose_graph_msgs::PoseGraph g;

  // Make the fast graph
  pose_graph_msgs::PoseGraphNode n0, n1, n2; 
  pose_graph_msgs::PoseGraphEdge e0, e1;

  n0.ID = "a0";
  n0.key = gtsam::Symbol('a', 0);
  n0.pose.position.x = 0.0;
  n0.pose.position.y = 0.0;
  n0.pose.position.z = 0.0;
  
  n1.ID = "a1";
  n1.key = gtsam::Symbol('a', 1);
  n1.pose.position.x = 1.0;
  n1.pose.position.y = 0.0;
  n1.pose.position.z = 0.0;

  n2.ID = "a2";
  n2.key = gtsam::Symbol('a', 2);
  n2.pose.position.x = 3.0;
  n2.pose.position.y = 0.0;
  n2.pose.position.z = 0.0;

  e0.key_from = n0.key;
  e0.key_to = n1.key;
  e0.pose.position.x = 1.0;
  e0.pose.position.y = 0.0;
  e0.pose.position.z = 0.0;

  e1.key_from = n1.key;
  e1.key_to = n2.key;
  e1.pose.position.x = 2.0;
  e1.pose.position.y = 0.0;
  e1.pose.position.z = 0.0;

  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  g.nodes.push_back(n2);
  g.edges.push_back(e0);
  g.edges.push_back(e1);

  pose_graph_msgs::PoseGraphConstPtr fast_graph(new pose_graph_msgs::PoseGraph(g));

  // Make the slow graph
  g = pose_graph_msgs::PoseGraph();

  n0.ID = "a0";
  n0.key = gtsam::Symbol('a', 0);
  n0.pose.position.x = 0.0;
  n0.pose.position.y = 0.0;
  n0.pose.position.z = 0.0;
  
  n1.ID = "a1";
  n1.key = gtsam::Symbol('a', 1);
  n1.pose.position.x = 0.0;
  n1.pose.position.y = 1.0;
  n1.pose.position.z = 0.0;
  n1.pose.orientation.z = sqrt(0.5);
  n1.pose.orientation.w = sqrt(0.5);

  e0.key_from = n0.key;
  e0.key_to = n1.key;
  e0.pose.position.x = 0.0;
  e0.pose.position.y = 1.0;
  e0.pose.position.z = 0.0;
  e0.pose.orientation.z = sqrt(0.5);
  e0.pose.orientation.w = sqrt(0.5);

  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  g.edges.push_back(e0);

  pose_graph_msgs::PoseGraphConstPtr slow_graph(new pose_graph_msgs::PoseGraph(g));

  // Give graphs to merger
  merger.OnSlowGraphMsg(slow_graph);
  merger.OnFastGraphMsg(fast_graph);

  pose_graph_msgs::PoseGraph current_graph = merger.GetCurrentGraph();

  // Check for correct number of nodes and edges
  EXPECT_EQ(3, current_graph.nodes.size());
  EXPECT_EQ(2, current_graph.edges.size());

  ROS_INFO_STREAM("Size of final graph: " << current_graph.nodes.size());

  // Get node a2
  float x, y, z;
  bool found = false;
  for (const GraphNode& node : current_graph.nodes) {
    if (node.key != gtsam::Symbol('a', 2)) {
      found = true;
      continue;
    }
    x = node.pose.position.x;
    y = node.pose.position.y;
    z = node.pose.position.z;
    break;
  }

  // Check that the node exists
  EXPECT_TRUE(found);

  // Check that final pose is correct
  EXPECT_NEAR(0.0, x, 1e-5);
  EXPECT_NEAR(3.0, y, 1e-5);
  EXPECT_NEAR(0.0, z, 1e-5);
}


TEST_F(TestMerger, MergeWithValuesOnlyInSlowGraph) {
  ros::NodeHandle nh, pnh("~");

  pose_graph_msgs::PoseGraph g;

  // Make the fast graph
  pose_graph_msgs::PoseGraphNode n0, n1, n2; 
  pose_graph_msgs::PoseGraphEdge e0, e1;

  n0.ID = "a0";
  n0.key = gtsam::Symbol('a', 0);
  n0.pose.position.x = 0.0;
  n0.pose.position.y = 0.0;
  n0.pose.position.z = 0.0;
  
  n1.ID = "a1";
  n1.key = gtsam::Symbol('a', 1);
  n1.pose.position.x = 5.0;
  n1.pose.position.y = 0.0;
  n1.pose.position.z = 0.0;

  n2.ID = "a2";
  n2.key = gtsam::Symbol('a', 2);
  n2.pose.position.x = 8.0;
  n2.pose.position.y = 0.0;
  n2.pose.position.z = 0.0;

  e0.key_from = n0.key;
  e0.key_to = n1.key;
  e0.pose.position.x = 5.0;
  e0.pose.position.y = 0.0;
  e0.pose.position.z = 0.0;

  e1.key_from = n1.key;
  e1.key_to = n2.key;
  e1.pose.position.x = 3.0;
  e1.pose.position.y = 0.0;
  e1.pose.position.z = 0.0;

  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  g.nodes.push_back(n2);
  g.edges.push_back(e0);
  g.edges.push_back(e1);

  pose_graph_msgs::PoseGraphConstPtr fast_graph(new pose_graph_msgs::PoseGraph(g));

  // Make the slow graph
  g = pose_graph_msgs::PoseGraph();

  n0.ID = "a0";
  n0.key = gtsam::Symbol('a', 0);
  n0.pose.position.x = 0.0;
  n0.pose.position.y = 0.0;
  n0.pose.position.z = 0.0;
  
  n1.ID = "a1";
  n1.key = gtsam::Symbol('a', 1);
  n1.pose.position.x = 0.0;
  n1.pose.position.y = 5.0;
  n1.pose.position.z = 0.0;
  n1.pose.orientation.z = sqrt(0.5);
  n1.pose.orientation.w = sqrt(0.5);

  g.nodes.push_back(n0);
  g.nodes.push_back(n1);

  pose_graph_msgs::PoseGraphConstPtr slow_graph(new pose_graph_msgs::PoseGraph(g));

  // Give graphs to merger
  merger.OnSlowGraphMsg(slow_graph);
  merger.OnFastGraphMsg(fast_graph);

  pose_graph_msgs::PoseGraph current_graph = merger.GetCurrentGraph();

  // Check for correct number of nodes and edges
  EXPECT_EQ(3, current_graph.nodes.size());
  EXPECT_EQ(2, current_graph.edges.size());

  ROS_INFO_STREAM("Size of final graph: " << current_graph.nodes.size());

  // Get node a2
  float x, y, z;
  bool found = false;
  for (const GraphNode& node : current_graph.nodes) {
    if (node.key != gtsam::Symbol('a', 2)) {
      found = true;
      continue;
    }
    x = node.pose.position.x;
    y = node.pose.position.y;
    z = node.pose.position.z;
    break;
  }

  // Check that the node exists
  EXPECT_TRUE(found);

  // Check that final pose is correct
  EXPECT_NEAR(0.0, x, 1e-5);
  EXPECT_NEAR(8.0, y, 1e-5);
  EXPECT_NEAR(0.0, z, 1e-5);
}


TEST_F(TestMerger, MergeWithArtifactInFastGraph) {
  ros::NodeHandle nh, pnh("~");

  pose_graph_msgs::PoseGraph g;

  // Make the fast graph
  pose_graph_msgs::PoseGraphNode n0, n1, n2, n3, a0; 
  pose_graph_msgs::PoseGraphEdge e0, e1, e2, e3;

  n0.ID = "a0";
  n0.key = gtsam::Symbol('a', 0);
  n0.pose.position.x = 0.0;
  n0.pose.position.y = 0.0;
  n0.pose.position.z = 0.0;
  
  n1.ID = "a1";
  n1.key = gtsam::Symbol('a', 1);
  n1.pose.position.x = 1.0;
  n1.pose.position.y = 0.0;
  n1.pose.position.z = 0.0;

  n2.ID = "a2";
  n2.key = gtsam::Symbol('a', 2);
  n2.pose.position.x = 2.0;
  n2.pose.position.y = 0.0;
  n2.pose.position.z = 0.0;

  n3.ID = "a3";
  n3.key = gtsam::Symbol('a', 3);
  n3.pose.position.x = 3.0;
  n3.pose.position.y = 0.0;
  n3.pose.position.z = 0.0;

  a0.ID = "m0";
  a0.key = gtsam::Symbol('m', 0);
  a0.pose.position.x = 2.0;
  a0.pose.position.y = 1.0;
  a0.pose.position.z = 0.0;

  e0.key_from = n0.key;
  e0.key_to = n1.key;
  e0.pose.position.x = 1.0;
  e0.pose.position.y = 0.0;
  e0.pose.position.z = 0.0;

  e1.key_from = n1.key;
  e1.key_to = n2.key;
  e1.pose.position.x = 1.0;
  e1.pose.position.y = 0.0;
  e1.pose.position.z = 0.0;

  e2.key_from = n2.key;
  e2.key_to = n3.key;
  e2.pose.position.x = 1.0;
  e2.pose.position.y = 0.0;
  e2.pose.position.z = 0.0;

  e3.key_from = n2.key;
  e3.key_to = a0.key;
  e3.pose.position.x = 0.0;
  e3.pose.position.y = 1.0;
  e3.pose.position.z = 0.0;

  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  g.nodes.push_back(n2);
  g.nodes.push_back(n3);
  g.nodes.push_back(a0);
  g.edges.push_back(e0);
  g.edges.push_back(e1);
  g.edges.push_back(e2);
  g.edges.push_back(e3);

  pose_graph_msgs::PoseGraphConstPtr fast_graph(new pose_graph_msgs::PoseGraph(g));

  // Make the slow graph
  g = pose_graph_msgs::PoseGraph();

  n0.ID = "a0";
  n0.key = gtsam::Symbol('a', 0);
  n0.pose.position.x = 0.0;
  n0.pose.position.y = 0.0;
  n0.pose.position.z = 0.0;
  
  n1.ID = "a1";
  n1.key = gtsam::Symbol('a', 1);
  n1.pose.position.x = 0.0;
  n1.pose.position.y = 1.0;
  n1.pose.position.z = 0.0;
  n1.pose.orientation.z = sqrt(0.5);
  n1.pose.orientation.w = sqrt(0.5);

  n2.ID = "a2";
  n2.key = gtsam::Symbol('a', 2);
  n2.pose.position.x = 0.0;
  n2.pose.position.y = 1.0;
  n2.pose.position.z = 0.0;
  n2.pose.orientation.z = sqrt(0.5);
  n2.pose.orientation.w = sqrt(0.5);

  g.nodes.push_back(n0);
  g.nodes.push_back(n1);

  pose_graph_msgs::PoseGraphConstPtr slow_graph(new pose_graph_msgs::PoseGraph(g));

  // Give graphs to merger
  merger.OnSlowGraphMsg(slow_graph);
  merger.OnFastGraphMsg(fast_graph);

  pose_graph_msgs::PoseGraph current_graph = merger.GetCurrentGraph();

  // Check for correct number of nodes and edges
  EXPECT_EQ(5, current_graph.nodes.size());
  EXPECT_EQ(4, current_graph.edges.size());

  ROS_INFO_STREAM("Size of final graph: " << current_graph.nodes.size());

  // Get node m0
  float x, y, z;
  bool found = false;
  for (const GraphNode& node : current_graph.nodes) {
    if (node.key != gtsam::Symbol('m', 0)) {
      found = true;
      continue;
    }
    x = node.pose.position.x;
    y = node.pose.position.y;
    z = node.pose.position.z;
    break;
  }

  // Check that the node exists
  EXPECT_TRUE(found);

  // Check that final pose is correct
  EXPECT_NEAR(-1.0, x, 1e-5);
  EXPECT_NEAR(2.0, y, 1e-5);
  EXPECT_NEAR(0.0, z, 1e-5);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_pose_graph_merger");
  return RUN_ALL_TESTS();
}