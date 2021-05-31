/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include "loop_closure/LoopGeneration.h"
#include "loop_closure/ProximityLoopGeneration.h"

namespace lamp_loop_closure {
class TestLoopGeneration : public ::testing::Test {
 protected:
  TestLoopGeneration() {
    // Load params
    system("rosparam load $(rospack find lamp)/config/lamp_settings.yaml");
    system(
        "rosparam load $(rospack find "
        "loop_closure)/config/laser_parameters.yaml");
  }
  ~TestLoopGeneration() {}

  void generateLoops(const gtsam::Key& new_key) {
    proximity_lc_.GenerateLoops(new_key);
  }

  void keyedPoseCallback(
      const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
    proximity_lc_.KeyedPoseCallback(graph_msg);
  }

  double distanceBetweenKeys(const gtsam::Symbol& key1,
                             const gtsam::Symbol& key2) {
    proximity_lc_.DistanceBetweenKeys(key1, key2);
  }

  ProximityLoopGeneration proximity_lc_;
};

TEST_F(TestLoopGeneration, TestInitialize) {
  ros::NodeHandle nh;
  bool init = proximity_lc_.Initialize(nh);
  ASSERT_TRUE(init);
}

}  // namespace lamp_loop_closure

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_loop_generation");
  return RUN_ALL_TESTS();
}
