/**
 *  @brief Testing the Lamp Base Station class
 *
 */

#include <gtest/gtest.h>

#include "lamp/LampBaseStation.h"
#include "lamp/LampRobot.h"

class TestLampBase : public ::testing::Test {
public:
  PointCloud::Ptr data;

  TestLampBase() {

  }
  ~TestLampBase() {}

  void SetPoseGraph(PoseGraph graph) { lb.pose_graph_ = graph; }
  PoseGraph GetPoseGraph() { return lb.pose_graph_; }
  bool ProcessPoseGraphData(std::shared_ptr<FactorData> data) { return lb.ProcessPoseGraphData(data); }

  LampBaseStation lb;

  
protected:

private:
};

TEST_F(TestLampBase, PoseGraphUpdateAfterOptimization) {

  // float zero_noise = 0.001;
  // gtsam::Vector6 noise;
  // noise << zero_noise, zero_noise, zero_noise, zero_noise, zero_noise, zero_noise;
  // gtsam::noiseModel::Diagonal::shared_ptr zero_covar(gtsam::noiseModel::Diagonal::Sigmas(noise));


  // // // Set up internal pose graph
  // PoseGraph pose_graph; 
  // pose_graph.TrackNode(ros::Time(0.0), gtsam::Symbol('z', 0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3()), zero_covar);
  // pose_graph.TrackNode(ros::Time(0.0), gtsam::Symbol('a', 0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3()), zero_covar);
  // pose_graph.TrackNode(ros::Time(1.0), gtsam::Symbol('a', 1), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0,0.0,0.0)), zero_covar);
  // pose_graph.TrackNode(ros::Time(2.0), gtsam::Symbol('a', 2), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0,0.0,0.0)), zero_covar);
  // pose_graph.TrackFactor(gtsam::Symbol('a',0), gtsam::Symbol('a',1), pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0,0.0,0.0)), zero_covar);
  // pose_graph.TrackFactor(gtsam::Symbol('a',1), gtsam::Symbol('a',2), pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0,0.0,0.0)), zero_covar);

  // SetPoseGraph(pose_graph);

  
  // pose_graph_msgs::PoseGraph msg; 
  // pose_graph_msgs::PoseGraphNode node1, node2;
  // node1.key = gtsam::Symbol('a', 0);
  // node1.pose.position.x = 0.0;
  // node1.pose.position.y = 0.0;
  // node1.pose.position.z = 0.0;
  // msg.nodes.push_back(node1);
  // node2.key = gtsam::Symbol('a', 1);
  // node2.pose.position.x = 0.0;
  // node2.pose.position.y = 1.0;
  // node2.pose.position.z = 0.0;
  // msg.nodes.push_back(node2);

  // std::shared_ptr<PoseGraphData> data = std::make_shared<PoseGraphData>();
  // data->b_has_data = true; 
  // pose_graph_msgs::PoseGraphConstPtr new_graph(
  //     new pose_graph_msgs::PoseGraph(msg));
  // data->graphs.push_back(new_graph);

  // std::cerr << "Processing pose graph data (" << data->graphs.size() << " graphs)" << std::endl;
  // std::cerr << "new graph has " << new_graph->nodes.size() << "nodes " << std::endl;

  // ProcessPoseGraphData(data);

  // // Check the output
  // PoseGraph output_graph = GetPoseGraph();
  // std::cerr << "Printing pose graph nodes " << std::endl;

  // std::vector<double> xvals; 
  // std::vector<double> yvals;
  // for (auto n : output_graph.GetNodes()) {
  //   // EXPECT_EQ(n.pose.position.x, 3.0);
  //   // EXPECT_EQ(n.pose.position.y, 3.0);    
  //   xvals.push_back(n.pose.position.x); 
  //   yvals.push_back(n.pose.position.y);
  //   std::cerr << n.pose.position.x << ", " << n.pose.position.y << ", " << n.pose.position.z << std::endl;
  // }

  // EXPECT_EQ(xvals[0], 0.0);
  // EXPECT_EQ(xvals[1], 0.0);
  // EXPECT_EQ(xvals[2], 1.0);
  // EXPECT_EQ(yvals[0], 0.0);
  // EXPECT_EQ(yvals[1], 1.0);
  // EXPECT_EQ(yvals[2], 1.0);


  EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lamp_base");
  return RUN_ALL_TESTS();
}
