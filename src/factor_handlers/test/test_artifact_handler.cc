/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include "factor_handlers/ArtifactHandler.h"

class TestArtifactHandler : public ::testing::Test{
  public: 
    ros::NodeHandle nh;
    ArtifactHandler art_handle;
    TestArtifactHandler(){}
    ~TestArtifactHandler(){}
    bool LoadParameters() {return art_handle.LoadParameters(nh);}
};

TEST_F(TestArtifactHandler, ArtifactInfoInitialize)
{
  // Constructor
  ArtifactInfo art_;

  // Check if ArtifactInfo is initialised correctly
  EXPECT_EQ(art_.num_updates, 0);
  EXPECT_EQ(art_.id, "");
  EXPECT_EQ(art_.global_pose.translation().vector(), Eigen::Vector3d(0.0,0.0,0.0));
}

TEST_F(TestArtifactHandler, LoadParameters)
{
  ros::param::set("frame_id/artifacts_in_global", false);
  ros::param::set("use_artifact_loop_closure", true);
  ros::param::set("artifact_prefix", "Artifact");
  ASSERT_TRUE(LoadParameters());
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_artifact_handler");
  return RUN_ALL_TESTS();
}
