/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include "lamp/LampRobot.h"

class TestLampRobot : public ::testing::Test {
  public: 
    TestLampRobot(){
      // Load params
      system("rosparam load $(rospack find "
             "lamp)/config/precision_parameters.yaml");
      system("rosparam load $(rospack find lamp)/config/lamp_frames.yaml");
      system("rosparam load $(rospack find lamp)/config/lamp_rates.yaml");
      system("rosparam load $(rospack find lamp)/config/lamp_init_noise.yaml");
      system("rosparam load $(rospack find lamp)/config/lamp_settings.yaml");

      system("rosparam load $(rospack find "
             "point_cloud_filter)/config/parameters.yaml");
      system("rosparam load $(rospack find "
             "point_cloud_mapper)/config/parameters.yaml");
      system("rosparam load $(rospack find "
             "factor_handlers)/config/odom_parameters.yaml");
    }
    ~TestLampRobot(){}

    LampRobot lr;

    // Pass-through functions
    bool SetInitialKey() {return lr.SetInitialKey();}
    bool SetFactorPrecisions() {return lr.SetFactorPrecisions();}
    bool SetInitialPosition() {return lr.SetInitialPosition();}
    int GetValuesSize() {return lr.values_.size();}
    gtsam::Symbol GetKeyAtTime(const ros::Time& stamp) {return lr.GetKeyAtTime(stamp);}
    pose_graph_msgs::PoseGraphConstPtr ConvertPoseGraphToMsg() {return lr.ConvertPoseGraphToMsg();}
    gtsam::SharedNoiseModel SetFixedNoiseModels(std::string type) {
      return lr.SetFixedNoiseModels(type);
    }
    void TrackEdges(gtsam::Symbol key_from, gtsam::Symbol key_to, int type, gtsam::Pose3 pose, gtsam::SharedNoiseModel covariance) {
      lr.TrackEdges(key_from, key_to, type, pose, covariance);
    }
    void TrackPriors(ros::Time stamp, gtsam::Symbol key, gtsam::Pose3 pose, gtsam::SharedNoiseModel covariance) {
      lr.TrackPriors(stamp, key, pose, covariance);
    }

    // Access functions
    void AddStampToOdomKey(ros::Time stamp, gtsam::Symbol key) {
      lr.stamp_to_odom_key_[stamp.toSec()] = key;
    }
    void AddKeyedStamp(gtsam::Symbol key, ros::Time stamp) {
      lr.keyed_stamps_[key] = stamp;
    }
    void SetTimeThreshold(double threshold) {lr.time_threshold_ = threshold;}
    void SetPrefix(char c) {lr.prefix_ = c;}
    void InsertValues(gtsam::Symbol key, gtsam::Pose3 pose) { lr.values_.insert(key, pose); }

    // Other utilities

  protected: 
    
    // Tolerance on EXPECT_NEAR assertions
    double tolerance_ = 1e-5;

  private:



};

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

  EXPECT_EQ(GetValuesSize(), 0);
}

TEST_F(TestLampRobot, TestSetInitialPosition) {
  ros::Time::init();

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

TEST_F(TestLampRobot, Initialization) {
  ros::NodeHandle nh, pnh("~");

  bool result = lr.Initialize(nh);

  EXPECT_TRUE(result);
}

TEST_F(TestLampRobot, GetKeyAtTime) {
  ros::Time::init();

  SetTimeThreshold(0.001);

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

  // Rounding to nearest (within threshold)
  EXPECT_EQ(gtsam::Symbol('a', 0), GetKeyAtTime(ros::Time(0.0000001)));
  EXPECT_EQ(gtsam::Symbol('a', 1), GetKeyAtTime(ros::Time(0.4990001)));
  EXPECT_EQ(gtsam::Symbol('a', 2), GetKeyAtTime(ros::Time(0.9999999)));
  EXPECT_EQ(gtsam::Symbol('a', 3), GetKeyAtTime(ros::Time(1.5009999)));
  EXPECT_EQ(gtsam::Symbol('a', 4), GetKeyAtTime(ros::Time(1.9999000)));
}

TEST_F(TestLampRobot, GetKeyAtTimeOutsideThreshold) {
  ros::Time::init();
  SetTimeThreshold(0.001);

  // Build map
  AddStampToOdomKey(ros::Time(0.0), gtsam::Symbol('a', 0));
  AddStampToOdomKey(ros::Time(0.5), gtsam::Symbol('a', 1));
  AddStampToOdomKey(ros::Time(1.0), gtsam::Symbol('a', 2));
  AddStampToOdomKey(ros::Time(1.5), gtsam::Symbol('a', 3));
  AddStampToOdomKey(ros::Time(2.0), gtsam::Symbol('a', 4));

  // Expect empty keys from invalid inputs
  EXPECT_EQ(gtsam::Symbol(), GetKeyAtTime(ros::Time(0.002)));
  EXPECT_EQ(gtsam::Symbol(), GetKeyAtTime(ros::Time(0.25)));
  EXPECT_EQ(gtsam::Symbol(), GetKeyAtTime(ros::Time(2.2)));
  EXPECT_EQ(gtsam::Symbol(), GetKeyAtTime(ros::Time(100000.0)));
}

TEST_F(TestLampRobot, GetKeyAtTimeEmpty) {
  ros::Time::init();
  SetTimeThreshold(0.001);

  // Expect empty keys from invalid inputs
  EXPECT_EQ(gtsam::Symbol(), GetKeyAtTime(ros::Time(0.0)));
  EXPECT_EQ(gtsam::Symbol(), GetKeyAtTime(ros::Time(1.0)));
}

TEST_F(TestLampRobot, ConvertPoseGraphToMsg) {
  ros::Time::init();

  // NOTE: this test does not use a full valid graph

  // Set up the robot prefix for odom nodes
  SetPrefix('a');

  static const gtsam::SharedNoiseModel& noise =
    gtsam::noiseModel::Isotropic::Variance(6, 0.1);


  // Test values 
  InsertValues(gtsam::Symbol('a', 100), gtsam::Pose3(gtsam::Rot3(sqrt(0.5),0,0,sqrt(0.5)), gtsam::Point3(420.0, 69.0, 0.0)));
  InsertValues(gtsam::Symbol('a', 101), gtsam::Pose3(gtsam::Rot3(sqrt(0.3),sqrt(0.3),sqrt(0.4),0.0), gtsam::Point3(10.0, -1.0, 1000.0)));
  InsertValues(gtsam::Symbol('m', 0), gtsam::Pose3(gtsam::Rot3(1,0,0,0), gtsam::Point3(500.0, 433.5, -2.5)));

  // Test edges 
  TrackEdges(gtsam::Symbol('a', 55), 
             gtsam::Symbol('a', 56), 
             pose_graph_msgs::PoseGraphEdge::ODOM, 
             gtsam::Pose3(gtsam::Rot3(1,0,0,0), 
             gtsam::Point3(1.0, 0, 0.1)), noise); 
  TrackEdges(gtsam::Symbol('a', 32), 
             gtsam::Symbol('m', 0), 
             pose_graph_msgs::PoseGraphEdge::ARTIFACT, 
             gtsam::Pose3(gtsam::Rot3(0,0,1,0), 
             gtsam::Point3(0, 0.9, 21.1)), noise); 
  
  // Test priors
  AddKeyedStamp(gtsam::Symbol('a', 50), ros::Time(67589467.0));
  TrackPriors(ros::Time(67589467.0), 
            gtsam::Symbol('a', 50), 
            gtsam::Pose3(gtsam::Rot3(1,0,0,0), gtsam::Point3(1.0, -2.2, 0.03)),
            noise); 


  // Convert pose-graph to message
  pose_graph_msgs::PoseGraphConstPtr g = ConvertPoseGraphToMsg();

  float x,y,z;
  for (auto n : g->nodes) {
    x = n.pose.position.x;
    y = n.pose.position.y;
    z = n.pose.position.z;
    ROS_INFO_STREAM("Node: (" << x << ", " << y << ", " << z << ")");
  }

  // Node a100 - check all information
  pose_graph_msgs::PoseGraphNode n = g->nodes[0];
  EXPECT_EQ("odom", n.ID);
  EXPECT_EQ(gtsam::Symbol('a', 100), n.key);
  EXPECT_NEAR(420.0, n.pose.position.x, tolerance_);
  EXPECT_NEAR(69.0, n.pose.position.y, tolerance_);
  EXPECT_NEAR(0.0, n.pose.position.z, tolerance_);
  EXPECT_NEAR(sqrt(0.5), n.pose.orientation.w, tolerance_);
  EXPECT_NEAR(0.0, n.pose.orientation.x, tolerance_);
  EXPECT_NEAR(0.0, n.pose.orientation.y, tolerance_);
  EXPECT_NEAR(sqrt(0.5), n.pose.orientation.z, tolerance_);

  // Node a101 - check all information
  n = g->nodes[1];
  EXPECT_EQ("odom", n.ID);
  EXPECT_EQ(gtsam::Symbol('a', 101), n.key);
  EXPECT_NEAR(10.0, n.pose.position.x, tolerance_);
  EXPECT_NEAR(-1.0, n.pose.position.y, tolerance_);
  EXPECT_NEAR(1000.0, n.pose.position.z, tolerance_);
  EXPECT_NEAR(sqrt(0.3), n.pose.orientation.w, tolerance_);
  EXPECT_NEAR(sqrt(0.3), n.pose.orientation.x, tolerance_);
  EXPECT_NEAR(sqrt(0.4), n.pose.orientation.y, tolerance_);
  EXPECT_NEAR(0.0, n.pose.orientation.z, tolerance_);

  // Node m0 - check all information
  n = g->nodes[2];
  EXPECT_EQ("Artifact", n.ID);
  EXPECT_EQ(gtsam::Symbol('m', 0), n.key);
  EXPECT_NEAR(500.0, n.pose.position.x, tolerance_);
  EXPECT_NEAR(433.5, n.pose.position.y, tolerance_);
  EXPECT_NEAR(-2.5, n.pose.position.z, tolerance_);
  EXPECT_NEAR(1.0, n.pose.orientation.w, tolerance_);
  EXPECT_NEAR(0.0, n.pose.orientation.x, tolerance_);
  EXPECT_NEAR(0.0, n.pose.orientation.y, tolerance_);
  EXPECT_NEAR(0.0, n.pose.orientation.z, tolerance_);

  // Odom edge (TODO: test the covariance)
  pose_graph_msgs::PoseGraphEdge e = g->edges[0];
  EXPECT_EQ(e.type, pose_graph_msgs::PoseGraphEdge::ODOM);
  EXPECT_EQ(e.key_from, gtsam::Symbol('a', 55));
  EXPECT_EQ(e.key_to, gtsam::Symbol('a', 56));
  EXPECT_NEAR(1.0, e.pose.position.x, tolerance_);
  EXPECT_NEAR(0.0, e.pose.position.y, tolerance_);
  EXPECT_NEAR(0.1, e.pose.position.z, tolerance_);
  EXPECT_NEAR(1.0, e.pose.orientation.w, tolerance_);
  EXPECT_NEAR(0.0, e.pose.orientation.x, tolerance_);
  EXPECT_NEAR(0.0, e.pose.orientation.y, tolerance_);
  EXPECT_NEAR(0.0, e.pose.orientation.z, tolerance_);
  
  // Artifact edge (TODO: test the covariance)
  e = g->edges[1];
  EXPECT_EQ(e.type, pose_graph_msgs::PoseGraphEdge::ARTIFACT);
  EXPECT_EQ(e.key_from, gtsam::Symbol('a', 32));
  EXPECT_EQ(e.key_to, gtsam::Symbol('m', 0));
  EXPECT_NEAR(0.0, e.pose.position.x, tolerance_);
  EXPECT_NEAR(0.9, e.pose.position.y, tolerance_);
  EXPECT_NEAR(21.1, e.pose.position.z, tolerance_);
  EXPECT_NEAR(0.0, e.pose.orientation.w, tolerance_);
  EXPECT_NEAR(0.0, e.pose.orientation.x, tolerance_);
  EXPECT_NEAR(1.0, e.pose.orientation.y, tolerance_);
  EXPECT_NEAR(0.0, e.pose.orientation.z, tolerance_);

  // Prior factor (TODO: test the covariance)
  n = g->priors[0];
  EXPECT_EQ(n.key, gtsam::Symbol('a', 50));
  EXPECT_NEAR(67589467, n.header.stamp.toSec(), tolerance_);
  EXPECT_NEAR(1.0, n.pose.position.x, tolerance_);
  EXPECT_NEAR(-2.2, n.pose.position.y, tolerance_);
  EXPECT_NEAR(0.03, n.pose.position.z, tolerance_);
  EXPECT_NEAR(1.0, n.pose.orientation.w, tolerance_);
  EXPECT_NEAR(0.0, n.pose.orientation.x, tolerance_);
  EXPECT_NEAR(0.0, n.pose.orientation.y, tolerance_);
  EXPECT_NEAR(0.0, n.pose.orientation.z, tolerance_);
}

// TODO - work out how to pass around SharedNoiseModels
// TEST_F(TestLampRobot, TestSetFixedCovariancesOdom){

//   double attitude_sigma = 0.1;
//   double position_sigma = 0.2;

//   ros::param::get("attitude_sigma",attitude_sigma);
//   ros::param::get("position_sigma",position_sigma);
//   // lr.attitude_sigma_ = attitude_sigma;
//   // lr.position_sigma_ = position_sigma;

//   // Set the paramters
//   gtsam::SharedNoiseModel noise = SetFixedNoiseModels("odom");

//   EXPECT_NEAR(noise.sigmas()[0], attitude_sigma, tolerance_)
//   EXPECT_NEAR(noise.sigmas()[3], position_sigma, tolerance_)

// }

// TEST_F(TestLampRobot, TestSetFixedCovariancesLaserLoopClosure){

//   double laser_lc_rot_sigma_ = 0.1;
//   double laser_lc_trans_sigma_ = 0.2;

//   lr.laser_lc_rot_sigma_ = laser_lc_rot_sigma_;
//   lr.laser_lc_trans_sigma_ = laser_lc_trans_sigma_;

//   // Set the paramters
//   gtsam::SharedNoiseModel noise = SetFixedNoiseModels("laser_loop_closure");

//   EXPECT_NEAR(noise.sigmas()[0], laser_lc_rot_sigma_, tolerance_)
//   EXPECT_NEAR(noise.sigmas()[3], laser_lc_trans_sigma_, tolerance_)

// }

// TEST_F(TestLampRobot, TestSetFixedCovariancesManualLoopClosure){

//   double manual_lc_rot_precision_ = 0.1;
//   double manual_lc_trans_precision_ = 0.2;

//   lr.laser_lc_rot_sigma_ = manual_lc_rot_precision_;
//   lr.laser_lc_trans_sigma_ = manual_lc_trans_precision_;

//   // Set the paramters
//   gtsam::SharedNoiseModel noise = SetFixedNoiseModels("manual_loop_closure");

//   EXPECT_NEAR(noise.sigmas()[0], manual_lc_rot_precision_, tolerance_)
//   EXPECT_NEAR(noise.sigmas()[3], manual_lc_trans_precision_, tolerance_)

// }

// TEST_F(TestLampRobot, TestSetFixedCovariancesError){

//   // Set the paramters
//   gtsam::SharedNoiseModel noise = SetFixedNoiseModels("something_wrong");

//   // TODO - look at throwing error

// }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lamp_robot");
  return RUN_ALL_TESTS();
}
