/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include "lamp/LampRobot.h"

class TestLampRobot : public ::testing::Test {
  private:

  public: 
    TestLampRobot(){
      // Set params

    }
    ~TestLampRobot(){}

    LampRobot lr;

    bool SetInitialKey() {return lr.SetInitialKey();}
    bool SetFactorPrecisions() {return lr.SetFactorPrecisions();}
    bool SetInitialPosition() {return lr.SetInitialPosition();}

    int GetValuesSize() {return lr.values_.size();}


};

TEST_F(TestLampRobot, SetInitialKey) {

  // Set string
  std::string prefix = "a";
  ros::param::set("robot_prefix", prefix);

  // Set key (with Friend Class)
  SetInitialKey();
  
  // Retrieve result
  gtsam::Symbol key_gtsam = lr.GetInitialKey();
  std::string key_string = std::string(key_gtsam);
  // std::string key_string = gtsam::DefaultKeyFormatter(key_gtsam);

  ROS_INFO_STREAM("Initial key is" << key_string);

  ASSERT_EQ(std::string("a0"), key_string);
}

TEST_F(TestLampRobot, TestSetInitialPosition) {
  // Set params
  ros::param::set("fiducial_calibration/position/x", 1.0);
  ros::param::set("fiducial_calibration/position/y", 1.0);
  ros::param::set("fiducial_calibration/position/z", 1.0);
  ros::param::set("fiducial_calibration/orientation/x", 0.0);
  ros::param::set("fiducial_calibration/orientation/y", 0.0);
  ros::param::set("fiducial_calibration/orientation/z", 0.0);
  ros::param::set("fiducial_calibration/orientation/w", 1.0);

  ros::param::set("init/position_sigma/x", 1.0);
  ros::param::set("init/position_sigma/y", 1.0);
  ros::param::set("init/position_sigma/z", 1.0);
  ros::param::set("init/orientation_sigma/roll", 1.0);
  ros::param::set("init/orientation_sigma/pitch", 1.0);
  ros::param::set("init/orientation_sigma/yaw", 1.0);

  EXPECT_TRUE(SetInitialPosition());

  EXPECT_EQ(GetValuesSize(),1);
  
}

TEST_F(TestLampRobot, TestSetInitialPositionNoParam) {
  // del params
  ros::param::del("fiducial_calibration/position/y");
  ros::param::del("fiducial_calibration/position/x");
  ros::param::del("fiducial_calibration/position/z");
  ros::param::del("fiducial_calibration/orientation/x");
  ros::param::del("fiducial_calibration/orientation/y");
  ros::param::del("fiducial_calibration/orientation/z");
  ros::param::del("fiducial_calibration/orientation/w");
  ros::param::del("init/position_sigma/x");
  ros::param::del("init/position_sigma/y");
  ros::param::del("init/position_sigma/z");
  ros::param::del("init/orientation_sigma/roll");
  ros::param::del("init/orientation_sigma/pitch");
  ros::param::del("init/orientation_sigma/yaw");

  EXPECT_FALSE(SetInitialPosition());

  EXPECT_EQ(GetValuesSize(),0);
  
}

TEST_F(TestLampRobot, SetFactorPrecisions) {

  // Set all parameter values  
  ros::param::set("manual_lc_rot_precision", 1.0);
  ros::param::set("manual_lc_trans_precision", 1.0);
  ros::param::set("laser_lc_rot_sigma", 1.0);
  ros::param::set("laser_lc_trans_sigma", 1.0);
  ros::param::set("artifact_rot_precision", 1.0);
  ros::param::set("artifact_trans_precision", 1.0);
  ros::param::set("point_estimate_precision", 1.0);
  ros::param::set("fiducial_trans_precision", 1.0);
  ros::param::set("fiducial_rot_precision", 1.0);

  ASSERT_TRUE(SetFactorPrecisions());
}
  

TEST_F(TestLampRobot, Initialization) {
  ros::NodeHandle nh, pnh("~");

  bool result = lr.Initialize(nh);
  
  ASSERT_TRUE(result);
}



// TEST_F(TestLampRobot, NoSetInitialKey) {

//   // Delete param
//   ros::param::del("robot_prefix");

//   // Set key (with Friend Class)
//   SetInitialKey();
  
//   // Retrieve result
//   gtsam::Symbol key_gtsam = lr.GetInitialKey();
//   std::string key_string = std::string(key_gtsam);
//   // std::string key_string = gtsam::DefaultKeyFormatter(key_gtsam);

//   ROS_INFO_STREAM("Initial key is" << key_string);
  
//   ASSERT_EQ(std::string("0"), key_string);
// }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lamp_robot");
  return RUN_ALL_TESTS();
}
