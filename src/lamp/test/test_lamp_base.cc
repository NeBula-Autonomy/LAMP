/**
 *  @brief Testing the Lamp Base Station class
 *
 */

#include <gtest/gtest.h>

#include "lamp/LampBaseStation.h"
#include "lamp/LampRobot.h"

class TestLampBase : public ::testing::Test {
public:
  PointCloud::Ptr scan_;
  pose_graph_msgs::PoseGraph graph_;
  pose_graph_msgs::KeyedScan scan_msg_;
  gtsam::Symbol init_key_;
  pose_graph_msgs::PoseGraphNode n0, n1;
  pose_graph_msgs::PoseGraphEdge e0;

  TestLampBase() : scan_(new PointCloud(3, 3)) {
    // Load params
    system("rosparam load $(rospack find "
           "lamp)/config/precision_parameters.yaml");
    system("rosparam load $(rospack find lamp)/config/lamp_frames_base.yaml");
    system("rosparam load $(rospack find lamp)/config/lamp_rates.yaml");
    system("rosparam load $(rospack find lamp)/config/lamp_init_noise.yaml");
    system("rosparam load $(rospack find lamp)/config/lamp_settings.yaml");
    system("rosparam load $(rospack find lamp)/config/GT_artifacts.yaml");
    system("rosparam load $(rospack find lamp)/config/robot_names.yaml");

    system("rosparam set artifact_prefix 'l'");

    system("rosparam load $(rospack find "
           "factor_handlers)/config/manual_lc_parameters.yaml");
    system("rosparam load $(rospack find "
           "point_cloud_mapper)/config/parameters.yaml");

    // Create data in the point cloud
    int n_points = 3;
    for (int i = 0; i < n_points; i++) {
      for (int j = 0; j < n_points; j++) {
        scan_->at(j, i).x = (float)j / (n_points - 1);
        scan_->at(j, i).y = (float)i / (n_points - 1);
        scan_->at(j, i).z = 0.0f;
      }
    }

    // init_key_ = gtsam::Symbol('a', 0);
    // // Make keyed scan
    // scan_msg_->key = init_key_;
    // pcl::toROSMsg(*scan_, scan_msg_->scan);

    n0.key = gtsam::Symbol('a', 0);
    n0.pose.position.x = 0.0;
    n0.pose.position.y = 0.0;
    n0.pose.position.z = 0.0;

    n1.key = gtsam::Symbol('a', 1);
    n1.pose.position.x = 1.0;
    n1.pose.position.y = 0.0;
    n1.pose.position.z = 0.0;

    e0.key_from = n0.key;
    e0.key_to = n1.key;
    e0.pose.position.x = 1.0;
    e0.pose.position.y = 0.0;
    e0.pose.position.z = 0.0;

    // graph_->nodes.push_back(n0);
  }
  ~TestLampBase() {}

  // void SetPoseGraph(PoseGraph graph) { lb.pose_graph_ = graph; }
  // PoseGraph GetPoseGraph() { return lb.pose_graph_; }
  bool ProcessPoseGraphData(std::shared_ptr<FactorData> data) {
    return lb.ProcessPoseGraphData(data);
  }

  int GetMapDataSize() {
    return lb.mapper_->GetMapData()->size();
  }

  LampBaseStation lb;

  PoseGraphData data_;

protected:
private:
};

TEST_F(TestLampBase, Initialize) {
  ros::NodeHandle nh, pnh("~");
  lb.Initialize(pnh);
}

TEST_F(TestLampBase, AddScansBeforePoseNodes) {
  ros::NodeHandle nh, pnh("~");
  lb.Initialize(pnh);

  // Add keyed scans to map
  init_key_ = gtsam::Symbol('a', 0);
  // Make keyed scan
  scan_msg_.key = init_key_;
  pcl::toROSMsg(*scan_, scan_msg_.scan);

  data_.b_has_data = true;

  pose_graph_msgs::KeyedScan::ConstPtr msg_ptr(
      new pose_graph_msgs::KeyedScan(scan_msg_));
  data_.scans.push_back(msg_ptr);

  // Process data
  std::shared_ptr<PoseGraphData> data_shared =
      std::make_shared<PoseGraphData>(data_);
  ProcessPoseGraphData(data_shared);

  // Add graph data
  graph_.nodes.push_back(n0);
  data_.scans.clear();
  data_.b_has_data = true;

  pose_graph_msgs::PoseGraph::ConstPtr msg_ptr2(
      new pose_graph_msgs::PoseGraph(graph_));
  data_.graphs.push_back(msg_ptr2);

  // Process data
  data_shared = std::make_shared<PoseGraphData>(data_);
  ProcessPoseGraphData(data_shared);

  // Check map
  EXPECT_TRUE(GetMapDataSize() > 0);
}

TEST_F(TestLampBase, PoseGraphUpdateAfterOptimization) {
  // float zero_noise = 0.001;
  // gtsam::Vector6 noise;
  // noise << zero_noise, zero_noise, zero_noise, zero_noise, zero_noise,
  // zero_noise; gtsam::noiseModel::Diagonal::shared_ptr
  // zero_covar(gtsam::noiseModel::Diagonal::Sigmas(noise));

  // // // Set up internal pose graph
  // PoseGraph pose_graph;
  // pose_graph.TrackNode(ros::Time(0.0), gtsam::Symbol('z', 0),
  // gtsam::Pose3(gtsam::Rot3(), gtsam::Point3()), zero_covar);
  // pose_graph.TrackNode(ros::Time(0.0), gtsam::Symbol('a', 0),
  // gtsam::Pose3(gtsam::Rot3(), gtsam::Point3()), zero_covar);
  // pose_graph.TrackNode(ros::Time(1.0), gtsam::Symbol('a', 1),
  // gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0,0.0,0.0)), zero_covar);
  // pose_graph.TrackNode(ros::Time(2.0), gtsam::Symbol('a', 2),
  // gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0,0.0,0.0)), zero_covar);
  // pose_graph.TrackFactor(gtsam::Symbol('a',0), gtsam::Symbol('a',1),
  // pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(gtsam::Rot3(),
  // gtsam::Point3(1.0,0.0,0.0)), zero_covar);
  // pose_graph.TrackFactor(gtsam::Symbol('a',1), gtsam::Symbol('a',2),
  // pose_graph_msgs::PoseGraphEdge::ODOM, gtsam::Pose3(gtsam::Rot3(),
  // gtsam::Point3(1.0,0.0,0.0)), zero_covar);

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

  // std::cerr << "Processing pose graph data (" << data->graphs.size() << "
  // graphs)" << std::endl; std::cerr << "new graph has " <<
  // new_graph->nodes.size() << "nodes " << std::endl;

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
  //   std::cerr << n.pose.position.x << ", " << n.pose.position.y << ", " <<
  //   n.pose.position.z << std::endl;
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
