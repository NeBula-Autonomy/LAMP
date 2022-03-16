/**
 *  @brief Testing the Loop Closure Prioritization classes
 *
 */

#include <gtest/gtest.h>

#include "loop_closure/GenericLoopPrioritization.h"
#include "loop_closure/LoopPrioritization.h"
#include "loop_closure/ObservabilityLoopPrioritization.h"

#include "test_artifacts.h"

namespace lamp_loop_closure {
class TestLoopPrioritization : public ::testing::Test {
 protected:
  TestLoopPrioritization() {
    // Load params
    system("rosparam load $(rospack find lamp)/config/lamp_settings.yaml");
    system(
        "rosparam load $(rospack find lamp)/config/precision_parameters.yaml");
    system(
        "rosparam load $(rospack find "
        "loop_closure)/config/laser_parameters.yaml");
  }
  ~TestLoopPrioritization() {}

  void genericPopulatePriorityQueue() { generic_.PopulatePriorityQueue(); }

  void genericKeyedScanCallback(
      const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
    generic_.KeyedScanCallback(scan_msg);
  }

  void genericCandidateCallback(
      const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates) {
    generic_.InputCallback(input_candidates);
  }

  void genericProcessTimerCallback(const ros::TimerEvent& ev) {
    generic_.ProcessTimerCallback(ev);
  }

  pose_graph_msgs::LoopCandidateArray genericGetBestCandidates() {
    return generic_.GetBestCandidates();
  }

  void observPopulatePriorityQueue() { observ_.PopulatePriorityQueue(); }

  void observKeyedScanCallback(
      const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
    observ_.KeyedScanCallback(scan_msg);
  }

  void observCandidateCallback(
      const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates) {
    observ_.InputCallback(input_candidates);
  }

  void observProcessTimerCallback(const ros::TimerEvent& ev) {
    observ_.ProcessTimerCallback(ev);
  }

  pose_graph_msgs::LoopCandidateArray observGetBestCandidates() {
    return observ_.GetBestCandidates();
  }

  GenericLoopPrioritization generic_;
  ObservabilityLoopPrioritization observ_;
};

TEST_F(TestLoopPrioritization, TestInitialize) {
  ros::NodeHandle nh;
  bool init = generic_.Initialize(nh);
  ASSERT_TRUE(init);

  init = observ_.Initialize(nh);
  ASSERT_TRUE(init);
}

TEST_F(TestLoopPrioritization, TestObservabilityThreshold) {
  ros::NodeHandle nh;
  generic_.Initialize(nh);
  observ_.Initialize(nh);

  // Add some keyed scans
  PointCloud::Ptr plane(new PointCloud);
  PointCloud::Ptr corner(new PointCloud);
  plane = GeneratePlane();
  corner = GenerateCorner();

  pose_graph_msgs::KeyedScan::Ptr ks0(new pose_graph_msgs::KeyedScan);
  *ks0 = PointCloudToKeyedScan(plane, gtsam::Symbol('a', 0));
  pose_graph_msgs::KeyedScan::Ptr ks1(new pose_graph_msgs::KeyedScan);
  *ks1 = PointCloudToKeyedScan(corner, gtsam::Symbol('a', 1));
  pose_graph_msgs::KeyedScan::Ptr ks2(new pose_graph_msgs::KeyedScan);
  *ks2 = PointCloudToKeyedScan(corner, gtsam::Symbol('a', 2));

  genericKeyedScanCallback(ks0);
  genericKeyedScanCallback(ks1);
  genericKeyedScanCallback(ks2);
  observKeyedScanCallback(ks0);
  observKeyedScanCallback(ks1);
  observKeyedScanCallback(ks2);

  // Create loop candidates
  pose_graph_msgs::LoopCandidateArray::Ptr candidates(
      new pose_graph_msgs::LoopCandidateArray);
  pose_graph_msgs::LoopCandidate c0, c1;
  c0.key_from = gtsam::Symbol('a', 2);
  c0.key_to = gtsam::Symbol('a', 1);
  c1.key_from = gtsam::Symbol('a', 2);
  c1.key_to = gtsam::Symbol('a', 0);
  candidates->candidates.push_back(c0);
  candidates->candidates.push_back(c1);

  genericCandidateCallback(candidates);
  observCandidateCallback(candidates);

  // Trigger timer event
  ros::TimerEvent timer_event;
  timer_event.current_real = ros::Time::now();
  timer_event.current_expected = ros::Time::now();
  genericProcessTimerCallback(timer_event);
  observProcessTimerCallback(timer_event);

  // Check candidates
  pose_graph_msgs::LoopCandidateArray gen_candidates =
      genericGetBestCandidates();
  pose_graph_msgs::LoopCandidateArray observ_candidates =
      observGetBestCandidates();

  EXPECT_EQ(0, gen_candidates.candidates.size());
  EXPECT_EQ(0, observ_candidates.candidates.size());

  // EXPECT_EQ(gtsam::Symbol('a', 1), gen_candidates.candidates[0].key_to);
  // EXPECT_EQ(gtsam::Symbol('a', 1), observ_candidates.candidates[0].key_to);
}

TEST_F(TestLoopPrioritization, TestPrioritization) {
  ros::NodeHandle nh;
  observ_.Initialize(nh);

  // Add some keyed scans
  PointCloud::Ptr corner(new PointCloud);
  PointCloud::Ptr box(new PointCloud);
  corner = GenerateCorner();
  box = GenerateBox();

  pose_graph_msgs::KeyedScan::Ptr ks0(new pose_graph_msgs::KeyedScan);
  *ks0 = PointCloudToKeyedScan(box, gtsam::Symbol('a', 0));
  pose_graph_msgs::KeyedScan::Ptr ks1(new pose_graph_msgs::KeyedScan);
  *ks1 = PointCloudToKeyedScan(corner, gtsam::Symbol('a', 1));
  pose_graph_msgs::KeyedScan::Ptr ks2(new pose_graph_msgs::KeyedScan);
  *ks2 = PointCloudToKeyedScan(corner, gtsam::Symbol('a', 2));

  observKeyedScanCallback(ks0);
  observKeyedScanCallback(ks1);
  observKeyedScanCallback(ks2);

  // Create loop candidates
  pose_graph_msgs::LoopCandidateArray::Ptr candidates(
      new pose_graph_msgs::LoopCandidateArray);
  pose_graph_msgs::LoopCandidate c0, c1;
  c0.key_from = gtsam::Symbol('a', 2);
  c0.key_to = gtsam::Symbol('a', 0);
  c1.key_from = gtsam::Symbol('a', 2);
  c1.key_to = gtsam::Symbol('a', 1);
  candidates->candidates.push_back(c0);
  candidates->candidates.push_back(c1);

  observCandidateCallback(candidates);

  // Trigger timer event
  ros::TimerEvent timer_event;
  timer_event.current_real = ros::Time::now();
  timer_event.current_expected = ros::Time::now();
  observProcessTimerCallback(timer_event);

  // Check the candidates
  pose_graph_msgs::LoopCandidateArray observ_candidates =
      observGetBestCandidates();

  EXPECT_EQ(0, observ_candidates.candidates.size());

  //   EXPECT_EQ(gtsam::Symbol('a', 0), observ_candidates.candidates[0].key_to);
  //   EXPECT_EQ(gtsam::Symbol('a', 1), observ_candidates.candidates[1].key_to);
}

}  // namespace lamp_loop_closure

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_loop_prioritization");
  return RUN_ALL_TESTS();
}
