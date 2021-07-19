/**
 *  @brief Test cases for talker class
 *
 *  This file shows an example usage of gtest.
 */

#include <factor_handlers/OdometryHandler.h>
#include <gtest/gtest.h>

class OdometryHandlerTest : public ::testing::Test {
public:
  OdometryHandlerTest() {
    // Load Params
    system("rosparam load $(rospack find "
           "factor_handlers)/config/odom_parameters.yaml");

    tolerance_ = 1e-5;

    t1 = 1.00;
    t2 = 1.05;
    t3 = 1.10;
    t4 = 1.15;
    t5 = 1.20; 

    t1_ros.fromSec(t1);
    t2_ros.fromSec(t2);
    t3_ros.fromSec(t3);
    t4_ros.fromSec(t4);
    t5_ros.fromSec(t5);

    // Fill the three messages
    msg_first.header.stamp = t1_ros;
    msg_first.pose.pose.position.x = 1;
    msg_first.pose.pose.position.y = 0;
    msg_first.pose.pose.position.z = 0;
    msg_first.pose.pose.orientation.x = 0;
    msg_first.pose.pose.orientation.y = 0;
    msg_first.pose.pose.orientation.z = 0;
    msg_first.pose.pose.orientation.w = 1;

    msg_second.header.stamp = t2_ros;
    msg_second.pose.pose.position.x = 2;
    msg_second.pose.pose.position.y = 0;
    msg_second.pose.pose.position.z = 0;
    msg_second.pose.pose.orientation.x = 0;
    msg_second.pose.pose.orientation.y = 0;
    msg_second.pose.pose.orientation.z = 0;
    msg_second.pose.pose.orientation.w = 1;

    msg_third.header.stamp = t3_ros;
    msg_third.pose.pose.position.x = 3;
    msg_third.pose.pose.position.y = 0;
    msg_third.pose.pose.position.z = 0;
    msg_third.pose.pose.orientation.x = 0;
    msg_third.pose.pose.orientation.y = 0;
    msg_third.pose.pose.orientation.z = 0;
    msg_third.pose.pose.orientation.w = 1;

    msg_fourth.header.stamp = t4_ros;
    msg_fourth.pose.pose.position.x = 3;
    msg_fourth.pose.pose.position.y = 1;
    msg_fourth.pose.pose.position.z = 0;
    msg_fourth.pose.pose.orientation.x = 0;
    msg_fourth.pose.pose.orientation.y = 0;
    msg_fourth.pose.pose.orientation.z = sqrt(2);
    msg_fourth.pose.pose.orientation.w = sqrt(2);

    msg_fifth.header.stamp = t5_ros;
    msg_fifth.pose.pose.position.x = 2;
    msg_fifth.pose.pose.position.y = 1;
    msg_fifth.pose.pose.position.z = 0;
    msg_fifth.pose.pose.orientation.x = 0;
    msg_fifth.pose.pose.orientation.y = 0;
    msg_fifth.pose.pose.orientation.z = 1;
    msg_fifth.pose.pose.orientation.w = 0;
  }

  OdometryHandler oh;

  double tolerance_;

protected:
  // Odometry Callbacks ------------------------------------------------
  void LidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    oh.LidarOdometryCallback(msg);
  }
  void VisualOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    oh.VisualOdometryCallback(msg);
  }
  void WheelOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    oh.WheelOdometryCallback(msg);
  }
  bool GetPoseAtTime(const ros::Time stamp,
                     const OdomPoseBuffer& odom_buffer,
                     PoseCovStamped& output) {
    return oh.GetPoseAtTime(stamp, odom_buffer, output);
  }
  /*
  bool GetPosesAtTimes(const ros::Time t1,
                       const ros::Time t2,
                       const OdomPoseBuffer& odom_buffer,
                       PoseCovStampedPair& output_poses,
                       const int odom_buffer_id) {
    return oh.GetPosesAtTimes(t1, t2, odom_buffer, output_poses, odom_buffer_id);
  }
  */
  bool GetOdomDelta(const ros::Time t_now, GtsamPosCov& delta_pose) {
    return oh.GetOdomDelta(t_now, delta_pose);
  }
  bool GetOdomDeltaLatestTime(ros::Time& t_latest, GtsamPosCov& delta_pose) {
    return oh.GetOdomDeltaLatestTime(t_latest, delta_pose);
  }
    
  virtual std::shared_ptr<FactorData> GetData() {
    return oh.GetData();
  }
  void FillGtsamPosCovOdom(const OdomPoseBuffer& odom_buffer,
                           GtsamPosCov& measurement,
                           const ros::Time t1,
                           const ros::Time t2, const int odom_buffer_id) {
    oh.FillGtsamPosCovOdom(odom_buffer, measurement, t1, t2, odom_buffer_id);
  }
  GtsamPosCov GetFusedOdomDeltaBetweenTimes(const ros::Time t1,
                                            const ros::Time t2) {
    return oh.GetFusedOdomDeltaBetweenTimes(t1, t2);
  }

  gtsam::Pose3 GetTransform(PoseCovStampedPair pose_cov_stamped_pair) {
    return oh.GetTransform(pose_cov_stamped_pair);
  }
  gtsam::SharedNoiseModel
  GetCovariance(PoseCovStampedPair pose_cov_stamped_pair) {
    return oh.GetCovariance(pose_cov_stamped_pair);
  }
  gtsam::Pose3 ToGtsam(const gu::Transform3& pose) const {
    return oh.ToGtsam(pose);
  }
  void PointCloudCallback(const PointCloudConstPtr& msg) {
    return oh.PointCloudCallback(msg);
  }
  bool InsertMsgInBuffer(const Odometry::ConstPtr& odom_msg,
                         OdomPoseBuffer& buffer) {
    return oh.InsertMsgInBuffer(odom_msg, buffer);
  }
  double CalculatePoseDelta(const GtsamPosCov gtsam_pos_cov) {
    return oh.CalculatePoseDelta(gtsam_pos_cov);
  }
  bool GetKeyedScanAtTime(const ros::Time& stamp, PointCloud::Ptr& msg) {
    return oh.GetKeyedScanAtTime(stamp, msg);
  }
  void ClearPreviousPointCloudScans(const PointCloudBuffer::iterator& itrTime) {
    return oh.ClearPreviousPointCloudScans(itrTime);
  }

  PointCloudBuffer *GetPointCloudBuffer() {
    PointCloudBuffer *ptr = &oh.point_cloud_buffer_;
    return ptr;
  }

  // Create three messages
  PoseCovStamped msg_first;
  PoseCovStamped msg_second;
  PoseCovStamped msg_third;
  PoseCovStamped msg_fourth;
  PoseCovStamped msg_fifth;

  ros::Time t1_ros;
  ros::Time t2_ros;
  ros::Time t3_ros;
  ros::Time t4_ros;
  ros::Time t5_ros;

  double t1; 
  double t2;
  double t3;
  double t4; 
  double t5;

private:
};

/* TEST Initialize */
TEST_F(OdometryHandlerTest, Initialization) {
  ros::NodeHandle nh;
  bool result = oh.Initialize(nh);
  ASSERT_TRUE(result);
}

// Getters ------------------------------------------------------------

/* TEST GetTransform */
TEST_F(OdometryHandlerTest, TestGetTransform) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  PoseCovStampedPair pose_cov_stamped_pair;
  geometry_msgs::Pose pose1;
  pose1.position.x = 0;
  pose1.position.y = 0;
  pose1.position.z = 0;
  pose1.orientation.x = 0;
  pose1.orientation.y = 0;
  pose1.orientation.z = 0;
  pose1.orientation.w = 1;
  geometry_msgs::Pose pose2;
  pose2.position.x = 1;
  pose2.position.y = 0;
  pose2.position.z = 0;
  pose2.orientation.x = 0;
  pose2.orientation.y = 0;
  pose2.orientation.z = 0;
  pose2.orientation.w = 1;
  pose_cov_stamped_pair.first.pose.pose = pose1;
  pose_cov_stamped_pair.second.pose.pose = pose2;
  gtsam::Pose3 transform_actual = GetTransform(pose_cov_stamped_pair);
  gtsam::Point3 position = gtsam::Point3(1, 0, 0);
  gtsam::Rot3 rotation = gtsam::Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1);
  gtsam::Pose3 transform_expected = gtsam::Pose3(rotation, position);
  ASSERT_TRUE(transform_actual.equals(transform_expected));
}

TEST_F(OdometryHandlerTest, TestGetTransformRotation) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  PoseCovStampedPair pose_cov_stamped_pair;
  geometry_msgs::Pose pose1;
  pose1.position.x = 0;
  pose1.position.y = 0;
  pose1.position.z = 0;
  pose1.orientation.x = 0;
  pose1.orientation.y = 0;
  pose1.orientation.z = 0;
  pose1.orientation.w = 1;
  geometry_msgs::Pose pose2;
  pose2.position.x = 1;
  pose2.position.y = 0;
  pose2.position.z = 0;
  pose2.orientation.x = 0;
  pose2.orientation.y = 0;
  pose2.orientation.z = sqrt(0.5);
  pose2.orientation.w = sqrt(0.5);
  pose_cov_stamped_pair.first.pose.pose = pose1;
  pose_cov_stamped_pair.second.pose.pose = pose2;
  gtsam::Pose3 transform_actual = GetTransform(pose_cov_stamped_pair);
  gtsam::Point3 position = gtsam::Point3(1, 0, 0);
  gtsam::Rot3 rotation = gtsam::Rot3(sqrt(0.5), 0, 0, sqrt(0.5));
  gtsam::Pose3 transform_expected = gtsam::Pose3(rotation, position);
  ASSERT_TRUE(transform_actual.equals(transform_expected));
}

TEST_F(OdometryHandlerTest, TestGetTransformRotation2) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  PoseCovStampedPair pose_cov_stamped_pair;
  geometry_msgs::Pose pose1;
  pose1.position.x = 0;
  pose1.position.y = 0;
  pose1.position.z = 0;
  pose1.orientation.x = 0;
  pose1.orientation.y = 0;
  pose1.orientation.z = sqrt(0.5);
  pose1.orientation.w = -sqrt(0.5);
  geometry_msgs::Pose pose2;
  pose2.position.x = 0;
  pose2.position.y = 0;
  pose2.position.z = 0;
  pose2.orientation.x = 0;
  pose2.orientation.y = 0;
  pose2.orientation.z = sqrt(0.5);
  pose2.orientation.w = sqrt(0.5);
  pose_cov_stamped_pair.first.pose.pose = pose1;
  pose_cov_stamped_pair.second.pose.pose = pose2;
  gtsam::Pose3 transform_actual = GetTransform(pose_cov_stamped_pair);
  gtsam::Point3 position = gtsam::Point3(0, 0, 0);
  gtsam::Rot3 rotation = gtsam::Rot3(0, 0, 0, 1);
  gtsam::Pose3 transform_expected = gtsam::Pose3(rotation, position);
  ASSERT_TRUE(transform_actual.equals(transform_expected));
}

TEST_F(OdometryHandlerTest, TestGetPoseAtTime) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  PoseCovStamped myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer;
  myBuffer[t1_ros.toSec()] = msg_first;
  myBuffer[t2_ros.toSec()] = msg_second;
  myBuffer[t3_ros.toSec()] = msg_third;
  bool result = GetPoseAtTime(t3_ros, myBuffer, myOutput);
  EXPECT_NEAR(
      msg_third.pose.pose.position.x, myOutput.pose.pose.position.x, 1e-5);
  ASSERT_TRUE(result);
}

TEST_F(OdometryHandlerTest, TestGetPoseAtTimeFuture) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.5");
  oh.Initialize(nh);
  // Create an output
  PoseCovStamped myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer;
  myBuffer[t1_ros.toSec()] = msg_first;
  myBuffer[t2_ros.toSec()] = msg_second;
  myBuffer[t3_ros.toSec()] = msg_third;
  ros::Time query;
  query.fromSec(1.5);
  bool result = GetPoseAtTime(query, myBuffer, myOutput);
  EXPECT_NEAR(
      msg_third.pose.pose.position.x, myOutput.pose.pose.position.x, 1e-5);
  ASSERT_TRUE(result);
}

TEST_F(OdometryHandlerTest, TestGetPoseAtTimePast) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.5");
  oh.Initialize(nh);
  // Create an output
  PoseCovStamped myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer;
  myBuffer[t1_ros.toSec()] = msg_first;
  myBuffer[t2_ros.toSec()] = msg_second;
  myBuffer[t3_ros.toSec()] = msg_third;
  ros::Time query;
  query.fromSec(0.6);
  bool result = GetPoseAtTime(query, myBuffer, myOutput);
  EXPECT_NEAR(
      msg_first.pose.pose.position.x, myOutput.pose.pose.position.x, 1e-5);
  ASSERT_TRUE(result);
}

TEST_F(OdometryHandlerTest, TestGetPoseAtTimeOutOfRange) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.5");
  oh.Initialize(nh);
  // Create an output
  PoseCovStamped myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer;

  myBuffer[t1_ros.toSec()] = msg_first;
  myBuffer[t2_ros.toSec()] = msg_second;
  myBuffer[t3_ros.toSec()] = msg_third;
  ros::Time query;
  query.fromSec(5000);
  bool result = GetPoseAtTime(query, myBuffer, myOutput);
  EXPECT_NEAR(
      msg_third.pose.pose.position.x, myOutput.pose.pose.position.x, 1e-5);
  EXPECT_FALSE(result);
}

TEST_F(OdometryHandlerTest, TestGetPoseBetweenTimesExact) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer;
  myBuffer[t1_ros.toSec()] = msg_first;
  myBuffer[t2_ros.toSec()] = msg_second;
  myBuffer[t3_ros.toSec()] = msg_third;
  FillGtsamPosCovOdom(myBuffer, myOutput, t1_ros, t2_ros, LIDAR_ODOM_BUFFER_ID);
  EXPECT_NEAR(1, myOutput.pose.x(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  FillGtsamPosCovOdom(myBuffer, myOutput, t1_ros, t3_ros, LIDAR_ODOM_BUFFER_ID);
  EXPECT_NEAR(2, myOutput.pose.x(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  FillGtsamPosCovOdom(myBuffer, myOutput, t2_ros, t3_ros, LIDAR_ODOM_BUFFER_ID);
  EXPECT_NEAR(1, myOutput.pose.x(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
}

TEST_F(OdometryHandlerTest, TestGetPoseBetweenTimesOff) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer;
  myBuffer[t1_ros.toSec()] = msg_first;
  myBuffer[t2_ros.toSec()] = msg_second;
  myBuffer[t3_ros.toSec()] = msg_third;
  ros::Time query1, query2, query3;
  query1.fromSec(1.01);
  query2.fromSec(1.04);
  query3.fromSec(1.08);
  FillGtsamPosCovOdom(myBuffer, myOutput, query1, query2, LIDAR_ODOM_BUFFER_ID);
  EXPECT_NEAR(1, myOutput.pose.x(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  FillGtsamPosCovOdom(myBuffer, myOutput, query1, query3, LIDAR_ODOM_BUFFER_ID);
  EXPECT_NEAR(2, myOutput.pose.x(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  FillGtsamPosCovOdom(myBuffer, myOutput, query2, query3, LIDAR_ODOM_BUFFER_ID);
  EXPECT_NEAR(1, myOutput.pose.x(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
}

TEST_F(OdometryHandlerTest, TestGetPoseBetweenTimesOutOfRange) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer;
  myBuffer[t1_ros.toSec()] = msg_first;
  myBuffer[t2_ros.toSec()] = msg_second;
  myBuffer[t3_ros.toSec()] = msg_third;
  ros::Time query1, query2, query3;
  query1.fromSec(0.7);
  query2.fromSec(1.3);
  FillGtsamPosCovOdom(myBuffer, myOutput, query1, query2, LIDAR_ODOM_BUFFER_ID);
  EXPECT_NEAR(2, myOutput.pose.x(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
}

TEST_F(OdometryHandlerTest, TestGetPoseBetweenOutsideThreshold) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer;
  myBuffer[t1_ros.toSec()] = msg_first;
  myBuffer[t2_ros.toSec()] = msg_second;
  myBuffer[t3_ros.toSec()] = msg_third;
  ros::Time query1, query2, query3;
  query1.fromSec(0.0);
  query2.fromSec(10.3);
  // This should fail as they are beyond the threshold
  FillGtsamPosCovOdom(myBuffer, myOutput, query1, query2, LIDAR_ODOM_BUFFER_ID);
  EXPECT_FALSE(myOutput.b_has_value);
}

// Test 1m to +y amd 90 deg rotation to the left
TEST_F(OdometryHandlerTest, TestGetPoseBetweenTimesRotationOneStep90) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer;
  myBuffer[t1_ros.toSec()] = msg_first;
  myBuffer[t2_ros.toSec()] = msg_second;
  myBuffer[t3_ros.toSec()] = msg_third;
  myBuffer[t4_ros.toSec()] = msg_fourth;
  myBuffer[t5_ros.toSec()] = msg_fifth;
  FillGtsamPosCovOdom(myBuffer, myOutput, t3_ros, t4_ros, LIDAR_ODOM_BUFFER_ID);
  EXPECT_NEAR(1, myOutput.pose.y(), 1e-5);
  EXPECT_NEAR(M_PI / 2.0f, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
}

// Same test as above, but this would be in -x in the global frame
TEST_F(OdometryHandlerTest, TestGetPoseBetweenTimesRotationCheckLocal) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer;
  myBuffer[t1_ros.toSec()] = msg_first;
  myBuffer[t2_ros.toSec()] = msg_second;
  myBuffer[t3_ros.toSec()] = msg_third;
  myBuffer[t4_ros.toSec()] = msg_fourth;
  myBuffer[t5_ros.toSec()] = msg_fifth;
  FillGtsamPosCovOdom(myBuffer, myOutput, t4_ros, t5_ros, LIDAR_ODOM_BUFFER_ID);
  EXPECT_NEAR(1, myOutput.pose.y(), 1e-5);
  EXPECT_NEAR(M_PI / 2.0f, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
}

// GET ODOM TESTING
// Use messages and test-
TEST_F(OdometryHandlerTest, TestGetOdomDeltaEmptyBuffer) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  // Expect a normal result
  bool result = GetOdomDelta(t2_ros, myOutput);
  EXPECT_FALSE(result);
}

TEST_F(OdometryHandlerTest, TestGetFusedOdomDelta) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  nav_msgs::Odometry msg_first_odom;
  nav_msgs::Odometry msg_second_odom;
  nav_msgs::Odometry msg_third_odom;
  msg_first_odom.pose = msg_first.pose;
  msg_first_odom.header = msg_first.header;
  msg_second_odom.pose = msg_second.pose;
  msg_second_odom.header = msg_second.header;
  msg_third_odom.pose = msg_third.pose;
  msg_third_odom.header = msg_third.header;
  nav_msgs::Odometry::ConstPtr msg_first_odomPtr(
      new nav_msgs::Odometry(msg_first_odom));
  nav_msgs::Odometry::ConstPtr msg_second_odomPtr(
      new nav_msgs::Odometry(msg_second_odom));
  nav_msgs::Odometry::ConstPtr msg_third_odomPtr(
      new nav_msgs::Odometry(msg_third_odom));

  // Call lidar callback
  LidarOdometryCallback(msg_first_odomPtr);
  LidarOdometryCallback(msg_second_odomPtr);
  LidarOdometryCallback(msg_third_odomPtr);
  // First result - should get delta to self
  bool result = GetOdomDelta(t1_ros, myOutput);
  EXPECT_NEAR(0, myOutput.pose.x(), 1e-5);
  EXPECT_NEAR(0.0, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  EXPECT_TRUE(result);
  // Expect a normal result
  result = GetOdomDelta(t2_ros, myOutput);
  EXPECT_NEAR(1, myOutput.pose.x(), 1e-5);
  EXPECT_NEAR(0.0, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  EXPECT_TRUE(result);
  // Expect a normal result
  result = GetOdomDelta(t3_ros, myOutput);
  EXPECT_NEAR(2, myOutput.pose.x(), 1e-5);
  EXPECT_NEAR(0.0, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  EXPECT_TRUE(result);
}

TEST_F(OdometryHandlerTest, TestGetFusedOdomDeltaOffExact) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  nav_msgs::Odometry msg_first_odom;
  nav_msgs::Odometry msg_second_odom;
  nav_msgs::Odometry msg_third_odom;
  msg_first_odom.pose = msg_first.pose;
  msg_first_odom.header = msg_first.header;
  msg_second_odom.pose = msg_second.pose;
  msg_second_odom.header = msg_second.header;
  msg_third_odom.pose = msg_third.pose;
  msg_third_odom.header = msg_third.header;
  nav_msgs::Odometry::ConstPtr msg_first_odomPtr(
      new nav_msgs::Odometry(msg_first_odom));
  nav_msgs::Odometry::ConstPtr msg_second_odomPtr(
      new nav_msgs::Odometry(msg_second_odom));
  nav_msgs::Odometry::ConstPtr msg_third_odomPtr(
      new nav_msgs::Odometry(msg_third_odom));
  // Call lidar callback
  LidarOdometryCallback(msg_first_odomPtr);
  LidarOdometryCallback(msg_second_odomPtr);
  LidarOdometryCallback(msg_third_odomPtr);
  ros::Time query1, query2, query3;
  query1.fromSec(0.7);
  query2.fromSec(1.04);
  query3.fromSec(1.15);
  // First result - should get delta to self
  bool result = GetOdomDelta(query1, myOutput);
  EXPECT_NEAR(0, myOutput.pose.x(), 1e-5);
  EXPECT_NEAR(0.0, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  EXPECT_TRUE(result);
  // Expect a normal result
  result = GetOdomDelta(query2, myOutput);
  EXPECT_NEAR(1, myOutput.pose.x(), 1e-5);
  EXPECT_NEAR(0.0, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  EXPECT_TRUE(result);
  // Expect a normal result
  result = GetOdomDelta(query3, myOutput);
  EXPECT_NEAR(2, myOutput.pose.x(), 1e-5);
  EXPECT_NEAR(0.0, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  EXPECT_TRUE(result);
}

// Expect this to throw false from the function call
TEST_F(OdometryHandlerTest, TestGetFusedOdomDeltaOutOfRange) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  nav_msgs::Odometry msg_first_odom;
  nav_msgs::Odometry msg_second_odom;
  nav_msgs::Odometry msg_third_odom;
  msg_first_odom.pose = msg_first.pose;
  msg_first_odom.header = msg_first.header;
  msg_second_odom.pose = msg_second.pose;
  msg_second_odom.header = msg_second.header;
  msg_third_odom.pose = msg_third.pose;
  msg_third_odom.header = msg_third.header;
  nav_msgs::Odometry::ConstPtr msg_first_odomPtr(
      new nav_msgs::Odometry(msg_first_odom));
  nav_msgs::Odometry::ConstPtr msg_second_odomPtr(
      new nav_msgs::Odometry(msg_second_odom));
  nav_msgs::Odometry::ConstPtr msg_third_odomPtr(
      new nav_msgs::Odometry(msg_third_odom));
  // Call lidar callback
  LidarOdometryCallback(msg_first_odomPtr);
  LidarOdometryCallback(msg_second_odomPtr);
  LidarOdometryCallback(msg_third_odomPtr);
  ros::Time query1, query2;
  query1.fromSec(0.0);
  query2.fromSec(2.0);
  // First result - should get delta to self
  bool result = GetOdomDelta(query1, myOutput);
  EXPECT_NEAR(0, myOutput.pose.x(), 1e-5);
  EXPECT_NEAR(0.0, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_FALSE(myOutput.b_has_value);
  EXPECT_FALSE(result);
  // Expect a normal result
  result = GetOdomDelta(query2, myOutput);
  EXPECT_NEAR(0, myOutput.pose.x(), 1e-5);
  EXPECT_NEAR(0.0, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_FALSE(myOutput.b_has_value);
  EXPECT_FALSE(result);
}

// TEST the function to get the delta to the latest time
TEST_F(OdometryHandlerTest, TestGetFusedOdomDeltaLatestTime) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  nav_msgs::Odometry msg_first_odom;
  nav_msgs::Odometry msg_second_odom;
  nav_msgs::Odometry msg_third_odom;
  msg_first_odom.pose = msg_first.pose;
  msg_first_odom.header = msg_first.header;
  msg_second_odom.pose = msg_second.pose;
  msg_second_odom.header = msg_second.header;
  msg_third_odom.pose = msg_third.pose;
  msg_third_odom.header = msg_third.header;
  nav_msgs::Odometry::ConstPtr msg_first_odomPtr(
      new nav_msgs::Odometry(msg_first_odom));
  nav_msgs::Odometry::ConstPtr msg_second_odomPtr(
      new nav_msgs::Odometry(msg_second_odom));
  nav_msgs::Odometry::ConstPtr msg_third_odomPtr(
      new nav_msgs::Odometry(msg_third_odom));
  // Call lidar callback
  LidarOdometryCallback(msg_first_odomPtr);
  LidarOdometryCallback(msg_second_odomPtr);
  LidarOdometryCallback(msg_third_odomPtr);
  ros::Time query1;
  bool result = GetOdomDeltaLatestTime(query1, myOutput);
  EXPECT_NEAR(2, myOutput.pose.x(), 1e-5);
  EXPECT_NEAR(0.0, myOutput.pose.rotation().yaw(), 1e-5);
  EXPECT_TRUE(myOutput.b_has_value);
  EXPECT_TRUE(result);
  // Need to return the correct time
  EXPECT_NEAR(t3_ros.toSec(), query1.toSec(), 1e-5);
}

TEST_F(OdometryHandlerTest, TestGetDataNoScans) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  nav_msgs::Odometry msg_first_odom;
  nav_msgs::Odometry msg_second_odom;
  nav_msgs::Odometry msg_third_odom;
  msg_first_odom.pose = msg_first.pose;
  msg_first_odom.header = msg_first.header;
  msg_second_odom.pose = msg_second.pose;
  msg_second_odom.header = msg_second.header;
  msg_third_odom.pose = msg_third.pose;
  msg_third_odom.header = msg_third.header;
  nav_msgs::Odometry::ConstPtr msg_first_odomPtr(
      new nav_msgs::Odometry(msg_first_odom));
  nav_msgs::Odometry::ConstPtr msg_second_odomPtr(
      new nav_msgs::Odometry(msg_second_odom));
  nav_msgs::Odometry::ConstPtr msg_third_odomPtr(
      new nav_msgs::Odometry(msg_third_odom));
  // Call lidar callback
  LidarOdometryCallback(msg_first_odomPtr);
  LidarOdometryCallback(msg_second_odomPtr);
  LidarOdometryCallback(msg_third_odomPtr);
  ros::Time query1;
  bool result = GetOdomDeltaLatestTime(query1, myOutput);
  EXPECT_TRUE(result);

  std::shared_ptr<OdomData> factor = std::dynamic_pointer_cast<OdomData>(GetData());
  EXPECT_TRUE(factor->b_has_data);
  EXPECT_FALSE(factor->factors[0].b_has_point_cloud);
  
  OdometryFactor odom_factor = factor->factors[0];
  EXPECT_NEAR(2, odom_factor.transform.x(), 1e-5);
  EXPECT_NEAR(0.0, odom_factor.transform.rotation().yaw(), 1e-5);

  // Need to return the correct time
  EXPECT_NEAR(t1_ros.toSec(), odom_factor.stamps.first.toSec(), 1e-5);
  EXPECT_NEAR(t3_ros.toSec(), odom_factor.stamps.second.toSec(), 1e-5);
}

TEST_F(OdometryHandlerTest, TestGetData) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  nav_msgs::Odometry msg_first_odom;
  nav_msgs::Odometry msg_second_odom;
  nav_msgs::Odometry msg_third_odom;
  msg_first_odom.pose = msg_first.pose;
  msg_first_odom.header = msg_first.header;
  msg_second_odom.pose = msg_second.pose;
  msg_second_odom.header = msg_second.header;
  msg_third_odom.pose = msg_third.pose;
  msg_third_odom.header = msg_third.header;
  nav_msgs::Odometry::ConstPtr msg_first_odomPtr(
      new nav_msgs::Odometry(msg_first_odom));
  nav_msgs::Odometry::ConstPtr msg_second_odomPtr(
      new nav_msgs::Odometry(msg_second_odom));
  nav_msgs::Odometry::ConstPtr msg_third_odomPtr(
      new nav_msgs::Odometry(msg_third_odom));
  // Call lidar callback
  LidarOdometryCallback(msg_first_odomPtr);
  LidarOdometryCallback(msg_second_odomPtr);
  LidarOdometryCallback(msg_third_odomPtr);

  PointCloud msg1;
  PointCloud msg2;
  PointCloud msg3;
  PointCloud msg4;
  pcl_conversions::toPCL(t1_ros, msg1.header.stamp);
  pcl_conversions::toPCL(t2_ros, msg2.header.stamp);
  pcl_conversions::toPCL(t3_ros, msg3.header.stamp);
  pcl_conversions::toPCL(t4_ros, msg4.header.stamp);
  PointCloudConstPtr pc_ptr1(new PointCloud(msg1));
  PointCloudConstPtr pc_ptr2(new PointCloud(msg2));
  PointCloudConstPtr pc_ptr3(new PointCloud(msg3));
  PointCloudConstPtr pc_ptr4(new PointCloud(msg4));
  PointCloudCallback(pc_ptr1);
  PointCloudCallback(pc_ptr2);
  PointCloudCallback(pc_ptr3);
  PointCloudCallback(pc_ptr4);

  bool result = GetOdomDelta(t3_ros, myOutput);
  EXPECT_TRUE(result);

  std::shared_ptr<OdomData> factor = std::dynamic_pointer_cast<OdomData>(GetData());
  EXPECT_TRUE(factor->b_has_data);
  EXPECT_TRUE(factor->factors[0].b_has_point_cloud);
  
  OdometryFactor odom_factor = factor->factors[0];
  EXPECT_NEAR(2, odom_factor.transform.x(), 1e-5);
  EXPECT_NEAR(0.0, odom_factor.transform.rotation().yaw(), 1e-5);

  // Need to return the correct time
  EXPECT_NEAR(t1_ros.toSec(), odom_factor.stamps.first.toSec(), 1e-5);
  EXPECT_NEAR(t3_ros.toSec(), odom_factor.stamps.second.toSec(), 1e-5);
}

TEST_F(OdometryHandlerTest, TestGetDataVLPStop) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  nav_msgs::Odometry msg_first_odom;
  nav_msgs::Odometry msg_second_odom;
  nav_msgs::Odometry msg_third_odom;
  msg_first_odom.pose = msg_first.pose;
  msg_first_odom.header = msg_first.header;
  msg_second_odom.pose = msg_second.pose;
  msg_second_odom.header = msg_second.header;
  msg_third_odom.pose = msg_third.pose;
  msg_third_odom.header = msg_third.header;
  nav_msgs::Odometry::ConstPtr msg_first_odomPtr(
      new nav_msgs::Odometry(msg_first_odom));
  nav_msgs::Odometry::ConstPtr msg_second_odomPtr(
      new nav_msgs::Odometry(msg_second_odom));
  nav_msgs::Odometry::ConstPtr msg_third_odomPtr(
      new nav_msgs::Odometry(msg_third_odom));
  // Call lidar callback
  LidarOdometryCallback(msg_first_odomPtr);
  LidarOdometryCallback(msg_second_odomPtr);
  LidarOdometryCallback(msg_third_odomPtr);

  ros::Time pc_time;
  pc_time.fromSec(t1 - 1.0);
  PointCloud msg1;
  PointCloud msg2;
  PointCloud msg3;
  PointCloud msg4;
  msg1.header.stamp = pc_time.toSec();
  msg2.header.stamp = (pc_time.fromSec(t1 - 0.9)).toSec();
  msg3.header.stamp = (pc_time.fromSec(t1 - 0.8)).toSec();
  msg4.header.stamp = (pc_time.fromSec(t1 - 0.7)).toSec();
  PointCloudConstPtr pc_ptr1(new PointCloud(msg1));
  PointCloudConstPtr pc_ptr2(new PointCloud(msg2));
  PointCloudConstPtr pc_ptr3(new PointCloud(msg3));
  PointCloudConstPtr pc_ptr4(new PointCloud(msg4));
  PointCloudCallback(pc_ptr1);
  PointCloudCallback(pc_ptr2);
  PointCloudCallback(pc_ptr3);
  PointCloudCallback(pc_ptr4);

  ros::Time query1;
  query1.fromSec(t4);
  bool result = GetOdomDelta(t3_ros, myOutput);
  EXPECT_TRUE(result);

  std::shared_ptr<OdomData> factor = std::dynamic_pointer_cast<OdomData>(GetData());
  EXPECT_TRUE(factor->b_has_data);
  EXPECT_FALSE(factor->factors[0].b_has_point_cloud);
  
  OdometryFactor odom_factor = factor->factors[0];
  EXPECT_NEAR(2, odom_factor.transform.x(), 1e-5);
  EXPECT_NEAR(0.0, odom_factor.transform.rotation().yaw(), 1e-5);

  // Need to return the correct time
  EXPECT_NEAR(t1_ros.toSec(), odom_factor.stamps.first.toSec(), 1e-5);
  EXPECT_NEAR(t3_ros.toSec(), odom_factor.stamps.second.toSec(), 1e-5);
}

TEST_F(OdometryHandlerTest, TestGetDataOldOdom) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  nav_msgs::Odometry msg_first_odom;
  nav_msgs::Odometry msg_second_odom;
  nav_msgs::Odometry msg_third_odom;
  msg_first_odom.pose = msg_first.pose;
  msg_first_odom.header = msg_first.header;
  msg_second_odom.pose = msg_second.pose;
  msg_second_odom.header = msg_second.header;
  msg_third_odom.pose = msg_third.pose;
  msg_third_odom.header = msg_third.header;
  nav_msgs::Odometry::ConstPtr msg_first_odomPtr(
      new nav_msgs::Odometry(msg_first_odom));
  nav_msgs::Odometry::ConstPtr msg_second_odomPtr(
      new nav_msgs::Odometry(msg_second_odom));
  nav_msgs::Odometry::ConstPtr msg_third_odomPtr(
      new nav_msgs::Odometry(msg_third_odom));
  // Call lidar callback
  LidarOdometryCallback(msg_first_odomPtr);
  LidarOdometryCallback(msg_second_odomPtr);
  LidarOdometryCallback(msg_third_odomPtr);

  ros::Time pc_time;
  pc_time.fromSec(t3 + 2.0);
  PointCloud msg1;
  PointCloud msg2;
  PointCloud msg3;
  PointCloud msg4;
  msg1.header.stamp = pc_time.toSec();
  msg2.header.stamp = (pc_time.fromSec(t3 + 2.1)).toSec();
  msg3.header.stamp = (pc_time.fromSec(t3 + 2.2)).toSec();
  msg4.header.stamp = (pc_time.fromSec(t3 + 2.3)).toSec();
  PointCloudConstPtr pc_ptr1(new PointCloud(msg1));
  PointCloudConstPtr pc_ptr2(new PointCloud(msg2));
  PointCloudConstPtr pc_ptr3(new PointCloud(msg3));
  PointCloudConstPtr pc_ptr4(new PointCloud(msg4));
  PointCloudCallback(pc_ptr1);
  PointCloudCallback(pc_ptr2);
  PointCloudCallback(pc_ptr3);
  PointCloudCallback(pc_ptr4);

  ros::Time query1;
  query1.fromSec(t4);
  bool result = GetOdomDelta(t3_ros, myOutput);
  EXPECT_TRUE(result);

  std::shared_ptr<OdomData> factor = std::dynamic_pointer_cast<OdomData>(GetData());
  EXPECT_TRUE(factor->b_has_data);
  EXPECT_FALSE(factor->factors[0].b_has_point_cloud);
  
  OdometryFactor odom_factor = factor->factors[0];
  EXPECT_NEAR(2, odom_factor.transform.x(), 1e-5);
  EXPECT_NEAR(0.0, odom_factor.transform.rotation().yaw(), 1e-5);

  // Need to return the correct time
  EXPECT_NEAR(t1_ros.toSec(), odom_factor.stamps.first.toSec(), 1e-5);
  EXPECT_NEAR(t3_ros.toSec(), odom_factor.stamps.second.toSec(), 1e-5);
}

TEST_F(OdometryHandlerTest, TestGetRelativeData) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  nav_msgs::Odometry msg_first_odom;
  nav_msgs::Odometry msg_second_odom;
  nav_msgs::Odometry msg_third_odom;
  msg_first_odom.pose = msg_first.pose;
  msg_first_odom.header = msg_first.header;
  msg_second_odom.pose = msg_second.pose;
  msg_second_odom.header = msg_second.header;
  msg_third_odom.pose = msg_third.pose;
  msg_third_odom.header = msg_third.header;
  nav_msgs::Odometry::ConstPtr msg_first_odomPtr(
      new nav_msgs::Odometry(msg_first_odom));
  nav_msgs::Odometry::ConstPtr msg_second_odomPtr(
      new nav_msgs::Odometry(msg_second_odom));
  nav_msgs::Odometry::ConstPtr msg_third_odomPtr(
      new nav_msgs::Odometry(msg_third_odom));
  // Call lidar callback
  LidarOdometryCallback(msg_first_odomPtr);
  LidarOdometryCallback(msg_second_odomPtr);
  LidarOdometryCallback(msg_third_odomPtr);
  myOutput = GetFusedOdomDeltaBetweenTimes(t2_ros, t3_ros);
  EXPECT_TRUE(myOutput.b_has_value);
  EXPECT_NEAR(1, myOutput.pose.x(), 1e-5);
  EXPECT_NEAR(0.0, myOutput.pose.rotation().yaw(), 1e-5);
}

// TODO: Fix this test as GetPosesAtTimes logic has been changed to handle corner cases
TEST_F(OdometryHandlerTest, TestGetRelativeDataOutOfRange) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput_1;
  GtsamPosCov myOutput_2;
  nav_msgs::Odometry msg_first_odom;
  nav_msgs::Odometry msg_second_odom;
  nav_msgs::Odometry msg_third_odom;
  msg_first_odom.pose = msg_first.pose;
  msg_first_odom.header = msg_first.header;
  msg_second_odom.pose = msg_second.pose;
  msg_second_odom.header = msg_second.header;
  msg_third_odom.pose = msg_third.pose;
  msg_third_odom.header = msg_third.header;
  nav_msgs::Odometry::ConstPtr msg_first_odomPtr(
      new nav_msgs::Odometry(msg_first_odom));
  nav_msgs::Odometry::ConstPtr msg_second_odomPtr(
      new nav_msgs::Odometry(msg_second_odom));
  nav_msgs::Odometry::ConstPtr msg_third_odomPtr(
      new nav_msgs::Odometry(msg_third_odom));
  // Call lidar callback
  // The 1st message will be stored into lidar_odom_value_at_key_.
  // This stored value is used when the first time query is out of range.
  LidarOdometryCallback(msg_first_odomPtr);
  LidarOdometryCallback(msg_second_odomPtr);
  LidarOdometryCallback(msg_third_odomPtr);
  // Corner case 1 (Both queries are out of range)
  ros::Time query1, query2;
  query1.fromSec(0.0);
  query2.fromSec(2.0);
  myOutput_1 = GetFusedOdomDeltaBetweenTimes(query1, query2);
  auto pose_expected_1 = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(2, 0, 0));
  EXPECT_FALSE(myOutput_1.b_has_value);
  // Corner case 2 (One query is out of range (ex) A robot stacks and stay at the same position for a while.)
  // query3 refers to the value of "lidar_odom_value_at_key_" because it's out of range
  // query4 refers to the value associated with the time stamp t2 (=1.05)
  ros::Time query3, query4;
  query3.fromSec(0.0);
  query4.fromSec(1.07);
  myOutput_2 = GetFusedOdomDeltaBetweenTimes(query3, query4);
  auto pose_expected_2 = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(1, 0, 0));
  EXPECT_TRUE((myOutput_2.pose).equals(pose_expected_2));
}

TEST_F(OdometryHandlerTest, TestGetCovariance) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  PoseCovStampedPair pose_cov_stamped_pair;
  PoseCovStamped pose_cov_stamped_1;
  PoseCovStamped pose_cov_stamped_2;
  for (size_t i = 0; i < 36; i++) {
    pose_cov_stamped_1.pose.covariance[i] = 1;
    pose_cov_stamped_2.pose.covariance[i] = 3;
  }
  pose_cov_stamped_pair.first = pose_cov_stamped_1;
  pose_cov_stamped_pair.second = pose_cov_stamped_2;
  gtsam::SharedNoiseModel noise_actual = GetCovariance(pose_cov_stamped_pair);
  gtsam::Matrix66 covariance_expected;
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      covariance_expected(i, j) = 2;
    }
  }
  gtsam::SharedNoiseModel noise_expected =
      gtsam::noiseModel::Gaussian::Covariance(covariance_expected);
  ASSERT_TRUE((*noise_actual).equals(*noise_expected));
}
/*
TEST_F(OdometryHandlerTest, TestGetPosesAtTimes) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create a buffer
  OdomPoseBuffer myBuffer;
  // Create an output
  PoseCovStampedPair myOutput;
  // Odometry-1
  Odometry odom_msg1;
  odom_msg1.pose = msg_first.pose;
  odom_msg1.header = msg_first.header;
  Odometry::ConstPtr my_msg1(new Odometry(odom_msg1));
  InsertMsgInBuffer(my_msg1, myBuffer);
  // Odometry-2
  Odometry odom_msg2;
  odom_msg2.pose = msg_second.pose;
  odom_msg2.header = msg_second.header;
  Odometry::ConstPtr my_msg2(new Odometry(odom_msg2));
  InsertMsgInBuffer(my_msg1, myBuffer);
  // Call the method
  bool result = GetPosesAtTimes(t1_ros, t2_ros, myBuffer, myOutput, LIDAR_ODOM_BUFFER_ID);
  ASSERT_TRUE(result);
}
*/
TEST_F(OdometryHandlerTest, TestGetKeyedScanAtTime) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create keyed scans
  PointCloud msg1;
  PointCloud msg2;
  PointCloud msg3;
  PointCloud msg4;
  PointCloud msg5;
  PointCloud msg6;
  pcl_conversions::toPCL(t1_ros, msg1.header.stamp);
  pcl_conversions::toPCL(t2_ros, msg2.header.stamp);
  pcl_conversions::toPCL(t3_ros, msg3.header.stamp);
  pcl_conversions::toPCL(t4_ros, msg4.header.stamp);
  pcl_conversions::toPCL(t5_ros, msg5.header.stamp);
  PointCloudConstPtr pc_ptr1(new PointCloud(msg1));
  PointCloudConstPtr pc_ptr2(new PointCloud(msg2));
  PointCloudConstPtr pc_ptr3(new PointCloud(msg3));
  PointCloudConstPtr pc_ptr4(new PointCloud(msg4));
  PointCloudConstPtr pc_ptr5(new PointCloud(msg5));
  PointCloudCallback(pc_ptr1);
  PointCloudCallback(pc_ptr2);
  PointCloudCallback(pc_ptr3);
  PointCloudCallback(pc_ptr4);
  PointCloudCallback(pc_ptr5);
  // Create the keyed scan container to be filled by GetKeyedScanAtTime method
  PointCloud point_cloud;
  PointCloud::Ptr my_keyed_scan(new PointCloud(point_cloud));
  bool result = GetKeyedScanAtTime(t1_ros, my_keyed_scan);
  ASSERT_TRUE(result);
}

TEST_F(OdometryHandlerTest, TestGetKeyedScanAtTimeError) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create keyed scans
  PointCloud msg1;
  PointCloud msg2;
  PointCloud msg3;
  PointCloud msg4;
  PointCloud msg5;
  PointCloud msg6;
  pcl_conversions::toPCL(t1_ros, msg1.header.stamp);
  pcl_conversions::toPCL(t2_ros, msg2.header.stamp);
  pcl_conversions::toPCL(t3_ros, msg3.header.stamp);
  pcl_conversions::toPCL(t4_ros, msg4.header.stamp);
  pcl_conversions::toPCL(t5_ros, msg5.header.stamp);
  PointCloudConstPtr pc_ptr1(new PointCloud(msg1));
  PointCloudConstPtr pc_ptr2(new PointCloud(msg2));
  PointCloudConstPtr pc_ptr3(new PointCloud(msg3));
  PointCloudConstPtr pc_ptr4(new PointCloud(msg4));
  PointCloudConstPtr pc_ptr5(new PointCloud(msg5));
  PointCloudCallback(pc_ptr1);
  PointCloudCallback(pc_ptr2);
  PointCloudCallback(pc_ptr3);
  PointCloudCallback(pc_ptr4);
  PointCloudCallback(pc_ptr5);
  // Create the keyed scan container to be filled by GetKeyedScanAtTime method
  PointCloud point_cloud;
  PointCloud::Ptr my_keyed_scan(new PointCloud(point_cloud));
  // Try past the end of the keyed scan buffer
  ros::Time t_test;
  t_test.fromSec(t5 + 5.0);
  bool result = GetKeyedScanAtTime(t_test, my_keyed_scan);
  EXPECT_FALSE(result);

  t_test.fromSec(t1 - 1.0);
  result = GetKeyedScanAtTime(t_test, my_keyed_scan);
  EXPECT_FALSE(result);
}

TEST_F(OdometryHandlerTest, TestClearPreviousPointCloudScans) {
  ros::NodeHandle nh("~");
  oh.Initialize(nh);

  PointCloud msg1;
  pcl_conversions::toPCL(t1_ros, msg1.header.stamp);
  PointCloudConstPtr pc_ptr1(new PointCloud(msg1));

  PointCloud msg2;
  pcl_conversions::toPCL(t2_ros, msg2.header.stamp);
  PointCloudConstPtr pc_ptr2(new PointCloud(msg2));

  PointCloud msg3;
  pcl_conversions::toPCL(t3_ros, msg3.header.stamp);
  PointCloudConstPtr pc_ptr3(new PointCloud(msg3));

  PointCloudCallback(pc_ptr1);
  PointCloudCallback(pc_ptr2);
  PointCloudCallback(pc_ptr3);
  auto ptr_buffer_2 = GetPointCloudBuffer();
  ASSERT_EQ(ptr_buffer_2->size(), 3);
  auto itrTime_2 = ptr_buffer_2->lower_bound(t2);
  ClearPreviousPointCloudScans(itrTime_2);
  ASSERT_EQ(ptr_buffer_2->size(), 2);
  ASSERT_EQ((ptr_buffer_2->rbegin())->first, t3);
}

/* TEST Utilities */

/* TEST ToGtsam */
TEST_F(OdometryHandlerTest, TestToGtsam) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  gu::Transform3 pose;
  pose.translation(0) = 1;
  pose.translation(1) = 0;
  pose.translation(2) = 0;
  pose.rotation(0, 0) = 1;
  pose.rotation(0, 1) = 0;
  pose.rotation(0, 2) = 0;
  pose.rotation(1, 0) = 0;
  pose.rotation(1, 1) = 1;
  pose.rotation(1, 2) = 0;
  pose.rotation(2, 0) = 0;
  pose.rotation(2, 1) = 0;
  pose.rotation(2, 2) = 1;
  gtsam::Pose3 transform_actual = ToGtsam(pose);
  gtsam::Point3 position = gtsam::Point3(1, 0, 0);
  gtsam::Rot3 rotation = gtsam::Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1);
  gtsam::Pose3 transform_expected = gtsam::Pose3(rotation, position);
  ASSERT_TRUE(transform_actual.equals(transform_expected));
}

TEST_F(OdometryHandlerTest, TestEmptyBufferHandling) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create an output
  GtsamPosCov myOutput;
  bool result = GetOdomDelta(t1_ros, myOutput);
  EXPECT_FALSE(result);
  EXPECT_FALSE(myOutput.b_has_value);
  result = GetOdomDeltaLatestTime(t1_ros, myOutput);
  EXPECT_FALSE(result);
  EXPECT_FALSE(myOutput.b_has_value);
  myOutput = GetFusedOdomDeltaBetweenTimes(t1_ros, t2_ros);
  EXPECT_FALSE(myOutput.b_has_value);
}

TEST_F(OdometryHandlerTest, InsertMsgInBuffer) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  // Create a buffer
  OdomPoseBuffer myBuffer;
  // Create a message
  Odometry odom_msg;
  odom_msg.pose = msg_first.pose;
  odom_msg.header = msg_first.header;
  Odometry::ConstPtr my_msg(new Odometry(odom_msg));
  // // Call the method
  bool result = InsertMsgInBuffer(my_msg, myBuffer);
  // Check result is correct
  ASSERT_TRUE(result);
}

TEST_F(OdometryHandlerTest, TestFillGtsamPosCovOdom) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  OdomPoseBuffer odom_buffer;
  // Odometry-1
  Odometry odom_msg1;
  odom_msg1.pose = msg_first.pose;
  odom_msg1.header = msg_first.header;
  for (size_t i = 0; i < 36; i++) {
    msg_first.pose.covariance[i] = 1;
  }
  odom_msg1.pose.covariance = msg_first.pose.covariance;
  Odometry::ConstPtr odom1(new Odometry(odom_msg1));
  InsertMsgInBuffer(odom1, odom_buffer);
  // Odometry-2
  Odometry odom_msg2;
  odom_msg2.pose = msg_second.pose;
  odom_msg2.header = msg_second.header;
  for (size_t i = 0; i < 36; i++) {
    msg_second.pose.covariance[i] = 3;
  }
  odom_msg2.pose.covariance = msg_second.pose.covariance;
  Odometry::ConstPtr odom2(new Odometry(odom_msg2));
  InsertMsgInBuffer(odom2, odom_buffer);
  // FillGtsamPosCovOdom
  GtsamPosCov odom_delta_actual;
  FillGtsamPosCovOdom(odom_buffer, odom_delta_actual, t1_ros, t2_ros, LIDAR_ODOM_BUFFER_ID);
  // Verification
  GtsamPosCov odom_delta_expected;
  gtsam::Point3 position = gtsam::Point3(1,0,0);
  gtsam::Rot3 rotation = gtsam::Rot3(1,0,0,0,1,0,0,0,1);
  odom_delta_expected.pose = gtsam::Pose3(rotation, position);
  gtsam::Matrix66 covariance_expected;
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      covariance_expected(i,j) = 3;
    }
  }
  gtsam::SharedNoiseModel noise_expected =
      gtsam::noiseModel::Gaussian::Covariance(covariance_expected);
  ASSERT_TRUE(odom_delta_actual.b_has_value);
  ASSERT_TRUE((odom_delta_actual.pose).equals(odom_delta_expected.pose));
  ASSERT_TRUE((*(odom_delta_actual.covariance)).equals(*noise_expected));
}

//------------------------------------------------------------------------------

TEST_F(OdometryHandlerTest, TestCalculatePoseDelta) {
  ros::NodeHandle nh("~");
  system("rosparam set ts_threshold 0.6");
  oh.Initialize(nh);
  OdomPoseBuffer myBuffer;
  GtsamPosCov my_fused_odom;
  my_fused_odom.pose = gtsam::Pose3();
  double delta = CalculatePoseDelta(my_fused_odom);
  EXPECT_EQ(delta, 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_odometry_handler");
  return RUN_ALL_TESTS();
}

/*
UNUSED

//
----------------------------------------------------------------------------------------
TEST_F(OdometryHandlerTest, TestCheckBufferSize) {
  std::vector<PoseCovStamped> myBuffer;
  PoseCovStamped my_msg;
  myBuffer.push_back(my_msg);
  int size = CheckBufferSize(myBuffer);
  EXPECT_EQ(size, 1);
}
// -----------------------------------------------------------------------------

std::pair<ros::Time, ros::Time> GetTimeStamps(PoseCovStampedPair
pose_cov_stamped_pair) { return oh.GetTimeStamps(pose_cov_stamped_pair);
}

TEST_F(OdometryHandlerTest, TestGetTimeStamps) {
  double t1 = 1.0;
  double t2 = 2.0;
  ros::Time t1_ros;
  ros::Time t2_ros;
  t1_ros.fromSec(t1);
  t2_ros.fromSec(t2);
  PoseCovStampedPair pose_cov_stamped_pair;
  pose_cov_stamped_pair.first.header.stamp = t1_ros;
  pose_cov_stamped_pair.second.header.stamp = t2_ros;
  std::pair<ros::Time, ros::Time> time_stamp_pair_actual =
    GetTimeStamps(pose_cov_stamped_pair);
  EXPECT_EQ(time_stamp_pair_actual.first, t1_ros);
  EXPECT_EQ(time_stamp_pair_actual.second, t2_ros);
}
*/