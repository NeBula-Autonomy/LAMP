/**
 *  @brief Testing the Loop Closure Prioritization classes
 *
 */

#include <gtest/gtest.h>

#include "loop_closure/GenericLoopPrioritization.h"
#include "loop_closure/LoopPrioritization.h"
#include "loop_closure/ObservabilityLoopPrioritization.h"

namespace lamp_loop_closure {
class TestLoopPrioritization : public ::testing::Test {
 protected:
  TestLoopPrioritization() {
    // Load params
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
  GenericLoopPrioritization generic_;
  // TODO(Yun) add tests for observability prioritization
};

TEST_F(TestLoopPrioritization, TestInitialize) {
  ros::NodeHandle nh;
  bool init = generic_.Initialize(nh);
  ASSERT_TRUE(init);
}

}  // namespace lamp_loop_closure

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_loop_prioritization");
  return RUN_ALL_TESTS();
}
