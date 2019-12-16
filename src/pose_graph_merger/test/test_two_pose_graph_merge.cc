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

#include <pose_graph_merger/TwoPoseGraphMerge.h>

class TestTwoPoseGraphMerge : public ::testing::Test {
public: 

  TestTwoPoseGraphMerge() {
    // Set params
  }
  ~TestTwoPoseGraphMerge() {}

  TwoPoseGraphMerge merge_;

protected:
  // Tolerance on EXPECT_NEAR assertions
  double tolerance_ = 1e-5;

  void ProcessBaseGraph(const pose_graph_msgs::PoseGraphConstPtr& msg){
    merge_.ProcessBaseGraph(msg);
  }

  void ProcessRobotGraph(const pose_graph_msgs::PoseGraphConstPtr& msg){
    merge_.ProcessRobotGraph(msg);
  }

  pose_graph_msgs::PoseGraph GetMergedGraph(){
    return merge_.GetMergedGraph();
  }

  pose_graph_msgs::PoseGraphNode CreateNode(char prefix, int index, double x, double y, double z){
    pose_graph_msgs::PoseGraphNode node;
    node.key = gtsam::Symbol(prefix, index);
    node.pose.position.x = x;
    node.pose.position.y = y;
    node.pose.position.z = z;
    return node;
  }

  pose_graph_msgs::PoseGraphEdge CreateEdge(gtsam::Symbol key_from,
                                            gtsam::Symbol key_to, 
                                            double x, double y, double z){
    pose_graph_msgs::PoseGraphEdge edge;
    edge.key_from = key_from;
    edge.key_to = key_to;
    edge.pose.position.x = x;
    edge.pose.position.y = y;
    edge.pose.position.z = z;
    edge.type = pose_graph_msgs::PoseGraphEdge::ODOM;
    return edge;
  }

  geometry_msgs::PoseStamped GetRobotPose(){return merge_.robot_pose_;}
  geometry_msgs::PoseStamped GetMergedPose(){return merge_.merged_pose_;}

private:
};

TEST_F(TestTwoPoseGraphMerge, Initialization){
  // ros::init(argc, argv, "two_pg_merge");
  ros::NodeHandle n("~");

  bool result = merge_.Initialize(n);

  EXPECT_TRUE(result);
}

TEST_F(TestTwoPoseGraphMerge, BasicMerge) {
  ros::NodeHandle nh, pnh("~");

  merge_.Initialize(pnh);

  pose_graph_msgs::PoseGraph g;

  // Make the fast graph
  pose_graph_msgs::PoseGraphNode n0, n1, n2;
  pose_graph_msgs::PoseGraphEdge e0, e1;

  n0 = CreateNode('a',0, 0.0, 0.0, 0.0);
  n1 = CreateNode('a',1, 1.0, 0.0, 0.0);
  n2 = CreateNode('a',2, 3.0, 0.0, 0.0);

  e0 = CreateEdge(n0.key, n1.key, 1.0, 0.0, 0.0);
  e1 = CreateEdge(n1.key, n2.key, 2.0, 0.0, 0.0);
  
  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  g.nodes.push_back(n2);
  g.edges.push_back(e0);
  g.edges.push_back(e1);

  pose_graph_msgs::PoseGraphConstPtr fast_graph(
      new pose_graph_msgs::PoseGraph(g));

  // Make the slow graph
  g = pose_graph_msgs::PoseGraph();

  n0 = CreateNode('a',0, 0.0, 0.0, 0.0);
  n1 = CreateNode('a',1, 0.0, 1.0, 0.0);
  n1.pose.orientation.z = sqrt(0.5);
  n1.pose.orientation.w = sqrt(0.5);

  e0 = CreateEdge(n0.key, n1.key, 0.0, 1.0, 0.0);
  e0.pose.orientation.z = sqrt(0.5);
  e0.pose.orientation.w = sqrt(0.5);
  
  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  g.edges.push_back(e0);

  pose_graph_msgs::PoseGraphConstPtr slow_graph(
      new pose_graph_msgs::PoseGraph(g));

  // Give graphs to merger
  ProcessBaseGraph(slow_graph);
  ProcessRobotGraph(fast_graph);

  // Get the result
  pose_graph_msgs::PoseGraph current_graph = GetMergedGraph();

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
  EXPECT_NEAR(0.0, x, tolerance_);
  EXPECT_NEAR(3.0, y, tolerance_);
  EXPECT_NEAR(0.0, z, tolerance_);
}

TEST_F(TestTwoPoseGraphMerge, StandardCase) {
  // Also with the latest node in the merged from the other robot
  ros::NodeHandle nh, pnh("~");

  merge_.Initialize(pnh);

  pose_graph_msgs::PoseGraph g;

  // Make the base station graph
  pose_graph_msgs::PoseGraphNode n0, n1, n2, n3, n4, n5;
  pose_graph_msgs::PoseGraphEdge e0, e1, e2, e3;

  n0 = CreateNode('a',0, 0.0, 0.0, 0.0);
  n1 = CreateNode('a',1, 1.0, 0.0, 0.0);
  n3 = CreateNode('b',0, 0.0, 1.0, 0.0);
  n4 = CreateNode('b',1, 2.0, 1.0, 0.0);
  n5 = CreateNode('b',2, 4.0, 1.0, 0.0);

  e0 = CreateEdge(n0.key, n1.key, 1.0, 0.0, 0.0);
  e1 = CreateEdge(n1.key, n2.key, 2.0, 0.0, 0.0);
  e2 = CreateEdge(n3.key, n4.key, 2.0, 0.0, 0.0);
  e3 = CreateEdge(n4.key, n5.key, 2.0, 0.0, 0.0);
  
  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  
  g.nodes.push_back(n3);
  g.nodes.push_back(n4);
  g.nodes.push_back(n5);

  g.edges.push_back(e0);

  g.edges.push_back(e2);
  g.edges.push_back(e3);

  pose_graph_msgs::PoseGraphConstPtr base_graph(
      new pose_graph_msgs::PoseGraph(g));

  // Make the robot graph
  g = pose_graph_msgs::PoseGraph();

  n0 = CreateNode('a',0, 0.0, 0.0, 0.0);
  n1 = CreateNode('a',1, 2.0, 0.0, 0.0);
  n2 = CreateNode('a',2, 4.0, 0.0, 0.0);

  e0 = CreateEdge(n0.key, n1.key, 2.0, 0.0, 0.0);
  e1 = CreateEdge(n1.key, n2.key, 2.0, 0.0, 0.0);
  
  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  g.nodes.push_back(n2);
  g.edges.push_back(e0);
  g.edges.push_back(e1);

  pose_graph_msgs::PoseGraphConstPtr robot_graph(
      new pose_graph_msgs::PoseGraph(g));

  // Give graphs to merger
  ProcessBaseGraph(base_graph);
  ProcessRobotGraph(robot_graph);

  // Get the result
  pose_graph_msgs::PoseGraph current_graph = GetMergedGraph();

  // Check for correct number of nodes and edges
  EXPECT_EQ(6, current_graph.nodes.size());
  EXPECT_EQ(4, current_graph.edges.size());

  // Check output poses
  geometry_msgs::PoseStamped robot_pose = GetRobotPose();
  geometry_msgs::PoseStamped merged_pose = GetMergedPose();

  // Check that final pose is correct for the robot (what is in the robot graph)
  EXPECT_NEAR(4.0, robot_pose.pose.position.x, tolerance_);
  EXPECT_NEAR(0.0, robot_pose.pose.position.y, tolerance_);
  EXPECT_NEAR(0.0, robot_pose.pose.position.z, tolerance_);

  // Check that merged pose is correct (what is in the merged graph, taking new delta from the robot)
  EXPECT_NEAR(3.0, merged_pose.pose.position.x, tolerance_);
  EXPECT_NEAR(0.0, merged_pose.pose.position.y, tolerance_);
  EXPECT_NEAR(0.0, merged_pose.pose.position.z, tolerance_);
}

TEST_F(TestTwoPoseGraphMerge, NoNewOnFast) {
  ros::NodeHandle nh, pnh("~");

  merge_.Initialize(pnh);

  pose_graph_msgs::PoseGraph g;

  // Make the base station graph
  pose_graph_msgs::PoseGraphNode n0, n1, n2, n3, n4, n5;
  pose_graph_msgs::PoseGraphEdge e0, e1, e2, e3;

  n0 = CreateNode('a',0, 0.0, 0.0, 0.0);
  n1 = CreateNode('a',1, 1.0, 0.0, 0.0);
  n2 = CreateNode('a',2, 3.0, 0.0, 0.0);
  n3 = CreateNode('b',0, 0.0, 1.0, 0.0);
  n4 = CreateNode('b',1, 2.0, 1.0, 0.0);
  n5 = CreateNode('b',2, 4.0, 1.0, 0.0);

  e0 = CreateEdge(n0.key, n1.key, 1.0, 0.0, 0.0);
  e1 = CreateEdge(n1.key, n2.key, 2.0, 0.0, 0.0);
  e2 = CreateEdge(n3.key, n4.key, 2.0, 0.0, 0.0);
  e3 = CreateEdge(n4.key, n5.key, 2.0, 0.0, 0.0);
  
  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  g.nodes.push_back(n2);
  g.nodes.push_back(n3);
  g.nodes.push_back(n4);
  g.nodes.push_back(n5);

  g.edges.push_back(e0);
  g.edges.push_back(e1);
  g.edges.push_back(e2);
  g.edges.push_back(e3);

  pose_graph_msgs::PoseGraphConstPtr base_graph(
      new pose_graph_msgs::PoseGraph(g));

  // Make the robot graph
  g = pose_graph_msgs::PoseGraph();

  n0 = CreateNode('a',0, 0.0, 0.0, 0.0);
  n1 = CreateNode('a',1, 1.0, 0.0, 0.0);
  n2 = CreateNode('a',2, 2.0, 0.0, 0.0);

  e0 = CreateEdge(n0.key, n1.key, 1.0, 0.0, 0.0);
  e1 = CreateEdge(n1.key, n2.key, 1.0, 0.0, 0.0);
  
  g.nodes.push_back(n0);
  g.nodes.push_back(n1);
  g.nodes.push_back(n2);
  g.edges.push_back(e0);
  g.edges.push_back(e1);

  pose_graph_msgs::PoseGraphConstPtr robot_graph(
      new pose_graph_msgs::PoseGraph(g));

  // Give graphs to merger
  ProcessBaseGraph(base_graph);
  ProcessRobotGraph(robot_graph);

  // Get the result
  pose_graph_msgs::PoseGraph current_graph = GetMergedGraph();

  // Check for correct number of nodes and edges
  EXPECT_EQ(6, current_graph.nodes.size());
  EXPECT_EQ(4, current_graph.edges.size());

  // Check output poses
  geometry_msgs::PoseStamped robot_pose = GetRobotPose();
  geometry_msgs::PoseStamped merged_pose = GetMergedPose();

  // Check that final pose is correct for the robot (what is in the robot graph)
  EXPECT_NEAR(2.0, robot_pose.pose.position.x, tolerance_);
  EXPECT_NEAR(0.0, robot_pose.pose.position.y, tolerance_);
  EXPECT_NEAR(0.0, robot_pose.pose.position.z, tolerance_);

  // Check that merged pose is correct (what is in the merged graph, taking new delta from the robot)
  EXPECT_NEAR(3.0, merged_pose.pose.position.x, tolerance_);
  EXPECT_NEAR(0.0, merged_pose.pose.position.y, tolerance_);
  EXPECT_NEAR(0.0, merged_pose.pose.position.z, tolerance_);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_pose_graph_merger");
  return RUN_ALL_TESTS();
}
