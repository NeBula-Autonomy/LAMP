/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include "lamp/LampRobot.h"

class TestLampRobot : public ::testing::Test{
  private:

  public: 
    TestLampRobot(){}
    ~TestLampRobot(){}

}

TEST(TestLampRobot, Initialization)
{
  ros::NodeHandle nh, pnh("~");

  // Constructor
  LampRobot lr(nh, pnh);

  lr.Initialize();
  
  EXPECT_EQ(sampled_points_gt[0], sampled_points[0]);
}

TEST(TestLampRobot, highestConfidenceIndex)
{
  
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lamp_robot");
  return RUN_ALL_TESTS();
}
