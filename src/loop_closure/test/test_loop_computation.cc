/**
 *  @brief Testing the Loop Closure Computation classes
 *
 */

#include <gtest/gtest.h>

#include "loop_closure/IcpLoopComputation.h"
#include "loop_closure/LoopComputation.h"

namespace lamp_loop_closure {
class TestLoopComputation : public ::testing::Test {
 protected:
  TestLoopComputation() {
    // Load params
    system("rosparam load $(rospack find lamp)/config/lamp_settings.yaml");
    system(
        "rosparam load $(rospack find "
        "lamp)/config/precision_parameters.yaml");
    system(
        "rosparam load $(rospack find "
        "loop_closure)/config/laser_parameters.yaml");
  }
  ~TestLoopComputation() {}

  void computeTransforms() { icp_compute_.ComputeTransforms(); }

  void keyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
    icp_compute_.KeyedScanCallback(scan_msg);
  }

  void keyedPoseCallback(
      const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
    icp_compute_.KeyedPoseCallback(graph_msg);
  }

  bool performAlignment(const gtsam::Symbol& key1,
                        const gtsam::Symbol& key2,
                        const gtsam::Pose3& pose1,
                        const gtsam::Pose3& pose2,
                        geometry_utils::Transform3* delta,
                        gtsam::Matrix66* covariance) {
    return icp_compute_.PerformAlignment(
        key1, key2, pose1, pose2, delta, covariance);
  }

  void getSacInitialAlignment(PointCloud::ConstPtr source,
                              PointCloud::ConstPtr target,
                              Eigen::Matrix4f* tf_out,
                              double& sac_fitness_score) {
    icp_compute_.GetSacInitialAlignment(
        source, target, tf_out, sac_fitness_score);
  }

  void getTeaserInitialAlignment(PointCloud::ConstPtr source,
                                 PointCloud::ConstPtr target,
                                 Eigen::Matrix4f* tf_out,
                                 int& n_inliers) {
    icp_compute_.GetTeaserInitialAlignment(source, target, tf_out, n_inliers);
  }

  IcpLoopComputation icp_compute_;
};

TEST_F(TestLoopComputation, TestInitialize) {
  ros::NodeHandle nh;
  bool init = icp_compute_.Initialize(nh);
  ASSERT_TRUE(init);
}

}  // namespace lamp_loop_closure

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_loop_computation");
  return RUN_ALL_TESTS();
}
