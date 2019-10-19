/**
 *  @brief Testing the AprilTagHandler class
 *
 */

#include <gtest/gtest.h>

#include "factor_handlers/AprilTagHandler.h"

class TestAprilTagHandler : public ::testing::Test{
  public: 
    ros::NodeHandle nh;
    AprilTagHandler apt_handle;
    TestAprilTagHandler(){
      apt_handle.use_artifact_loop_closure_ = true;
    }
    ~TestAprilTagHandler(){}
};

// Check if callback working correctly.
TEST_F(TestAprilTagHandler, AprilTagCallback) {
}

// Check if April tag message is being converted into artifacts correctly
TEST_F(TestAprilTagHandler, ConvertAprilTagMsgToArtifactMsg) {
}

// Check if ground truth is being extracted.
TEST_F(TestAprilTagHandler, GetGroundTruthData) {
}

// Check if parameters are being loaded properly. 
TEST_F(TestAprilTagHandler, LoadParameters) {
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_april_tag_handler");
  return RUN_ALL_TESTS();
}