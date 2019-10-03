/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include "factor_handlers/ArtifactHandler.h"

class TestArtifactHandler : public ::testing::Test{
  private:

  public: 
    TestArtifactHandler(){}
    ~TestArtifactHandler(){}

};

TEST(TestArtifactHandler, Initialize)
{
  // Constructor
  ArtifactInfo art_;

  // Check if ArtifactInfo is initialised
  EXPECT_EQ(art_.num_updates, 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_artifact_handler");
  return RUN_ALL_TESTS();
}
