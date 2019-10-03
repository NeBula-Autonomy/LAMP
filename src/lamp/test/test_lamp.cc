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

};

TEST(TestLampRobot, Initialization)
{
  ros::NodeHandle nh, pnh("~");

  // Constructor
  LampRobot lr;

  bool result = lr.Initialize(nh);
  
  ASSERT_TRUE(result);
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
