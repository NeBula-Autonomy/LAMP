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

    // Pass-through functions
    bool SetInitialKey() {return lr.SetInitialKey();}
    bool SetFactorPrecisions() {return lr.SetFactorPrecisions();}
    bool SetInitialPosition() {return lr.SetInitialPosition();}
    int GetValuesSize() {return lr.values_.size();}
    gtsam::Symbol GetKeyAtTime(const ros::Time& stamp) {return lr.GetKeyAtTime(stamp);}

    // Access functions
    void AddStampToOdomKey(ros::Time stamp, gtsam::Symbol key) {
      lr.stamp_to_odom_key_[stamp.toSec()] = key;
    }

    // Other utilities

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

  EXPECT_EQ(std::string("a0"), key_string);
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

  EXPECT_TRUE(SetFactorPrecisions());
}
  

TEST_F(TestLampRobot, GetKeyAtTime) {
  ros::Time::init();

  // Build map
  AddStampToOdomKey(ros::Time(0.0), gtsam::Symbol('a', 0));
  AddStampToOdomKey(ros::Time(0.5), gtsam::Symbol('a', 1));
  AddStampToOdomKey(ros::Time(1.0), gtsam::Symbol('a', 2));
  AddStampToOdomKey(ros::Time(1.5), gtsam::Symbol('a', 3));
  AddStampToOdomKey(ros::Time(2.0), gtsam::Symbol('a', 4));

  // Exact matches
  EXPECT_EQ(gtsam::Symbol('a', 0), GetKeyAtTime(ros::Time(0.0)));
  EXPECT_EQ(gtsam::Symbol('a', 1), GetKeyAtTime(ros::Time(0.5)));
  EXPECT_EQ(gtsam::Symbol('a', 2), GetKeyAtTime(ros::Time(1.0)));
  EXPECT_EQ(gtsam::Symbol('a', 3), GetKeyAtTime(ros::Time(1.5)));
  EXPECT_EQ(gtsam::Symbol('a', 4), GetKeyAtTime(ros::Time(2.0)));

  // Rounding to nearest
  EXPECT_EQ(gtsam::Symbol('a', 0), GetKeyAtTime(ros::Time(0.1)));
  EXPECT_EQ(gtsam::Symbol('a', 1), GetKeyAtTime(ros::Time(0.4)));
  EXPECT_EQ(gtsam::Symbol('a', 2), GetKeyAtTime(ros::Time(0.76)));
  EXPECT_EQ(gtsam::Symbol('a', 3), GetKeyAtTime(ros::Time(1.49999)));
  EXPECT_EQ(gtsam::Symbol('a', 4), GetKeyAtTime(ros::Time(2.1)));
}



TEST_F(TestLampRobot, Initialization) {
  ros::NodeHandle nh, pnh("~");

  bool result = lr.Initialize(nh);
  
  EXPECT_TRUE(result);
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lamp_robot");
  return RUN_ALL_TESTS();
}
