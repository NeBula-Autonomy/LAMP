/**
 *  @brief Test cases for talker class
 *
 *  This file shows an example usage of gtest.
 */

#include <gtest/gtest.h>
#include <factor_handlers/OdometryHandler.h>

class OdometryHandlerTest : public ::testing::Test {

public:

  OdometryHandlerTest() {
    // Load Params
    system("rosparam load $(rospack find factor_handlers)/config/odom_parameters.yaml");
  }

  OdometryHandler oh;

protected:

    // Odometry Callbacks ------------------------------------------------
    void LidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      oh.LidarOdometryCallback(msg);
    } 

    void VisualOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      oh.LidarOdometryCallback(msg);
    }  

    void WheelOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      oh.LidarOdometryCallback(msg);
    } 

    // Utilities 
    template <typename T>
    int CheckBufferSize(const std::vector<T>& x) {
      return oh.CheckBufferSize<T>(x);
    }

    template <typename T1, typename T2>
    bool InsertMsgInBuffer(typename T1::ConstPtr& msg, std::vector<T2>& buffer) {
      return oh.InsertMsgInBuffer<T1, T2>(msg, buffer);
    }

    void FillGtsamPosCovOdom(const OdomPoseBuffer& odom_buffer, 
                              GtsamPosCov& measurement,
                              const ros::Time t1,
                              const ros::Time t2) const {
      return oh.FillGtsamPosCovOdom(odom_buffer, measurement, t1, t2);
    }

    gtsam::Pose3 GetTransform(PoseCovStampedPair pose_cov_stamped_pair) {
      return oh.GetTransform(pose_cov_stamped_pair);
    }

    gtsam::SharedNoiseModel GetCovariance(PoseCovStampedPair pose_cov_stamped_pair) {
      return oh.GetCovariance(pose_cov_stamped_pair);
    }

    gtsam::Pose3 ToGtsam(const gu::Transform3& pose) const{
      return oh.ToGtsam(pose);
    }

    bool GetPoseAtTime(ros::Time t, const OdomPoseBuffer& odom_buffer, PoseCovStamped& output) {
      return oh.GetPoseAtTime(t, odom_buffer, output);
    }

    bool GetPosesAtTimes(ros::Time t1, ros::Time t2, const OdomPoseBuffer& odom_buffer, PoseCovStampedPair& output_poses) {
      return oh.GetPosesAtTimes(t1, t2, odom_buffer, output_poses);
    }

    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
      return oh.PointCloudCallback(msg);
    }


    bool GetKeyedScanAtTime(const ros::Time& stamp, PointCloud::Ptr& msg) {
      return oh.GetKeyedScanAtTime(stamp, msg);
    }

    std::vector<geometry_msgs::PoseWithCovarianceStamped> lidar_odometry_buffer_ = oh.lidar_odometry_buffer_;

  private:    

};

// Test we pass ----------------------------------------------------------------------

/* TEST Initialize */ 
// TEST_F(OdometryHandlerTest, Initialization) {
//    ros::NodeHandle nh;
//    bool result = oh.Initialize(nh);
//    ASSERT_TRUE(result);
// }

/* TEST CheckBufferSize */ 
TEST_F(OdometryHandlerTest, TestCheckBufferSize) {
  std::vector<PoseCovStamped> myBuffer;
  PoseCovStamped my_msg;
  myBuffer.push_back(my_msg);
  int size = CheckBufferSize(myBuffer);
  EXPECT_EQ(size, 1);
}


// Getters ------------------------------------------------------------

/* TEST GetTransform */
TEST_F (OdometryHandlerTest, TestGetTransform) {
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
  gtsam::Point3 position = gtsam::Point3(1,0,0);
  gtsam::Rot3 rotation = gtsam::Rot3(1,0,0,0,1,0,0,0,1);
  gtsam::Pose3 transform_expected = gtsam::Pose3(rotation, position);
  ASSERT_TRUE(transform_actual.equals(transform_expected));
}

/* TEST ToGtsam */
TEST_F(OdometryHandlerTest, TestToGtsam) {
  gu::Transform3 pose;
  pose.translation(0) = 1;
  pose.translation(1) = 0;
  pose.translation(2) = 0;
  pose.rotation(0,0) = 1;
  pose.rotation(0,1) = 0;
  pose.rotation(0,2) = 0;
  pose.rotation(1,0) = 0;
  pose.rotation(1,1) = 1;
  pose.rotation(1,2) = 0;
  pose.rotation(2,0) = 0;
  pose.rotation(2,1) = 0;
  pose.rotation(2,2) = 1;
  gtsam::Pose3 transform_actual = ToGtsam(pose);

  gtsam::Point3 position = gtsam::Point3(1,0,0);
  gtsam::Rot3 rotation = gtsam::Rot3(1,0,0,0,1,0,0,0,1);
  gtsam::Pose3 transform_expected = gtsam::Pose3(rotation, position);

  ASSERT_TRUE(transform_actual.equals(transform_expected));
}

TEST_F(OdometryHandlerTest, TestGetPoseAtTime) {
  double t1 = 1.00;
  double t2 = 1.05;
  double t3 = 1.10;
  ros::Time t1_ros;
  ros::Time t2_ros;
  ros::Time t3_ros;
  t1_ros.fromSec(t1);
  t2_ros.fromSec(t2);
  t3_ros.fromSec(t3);

  // Create an output
  PoseCovStamped myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer; 
  // Create two messages
  geometry_msgs::PoseWithCovarianceStamped msg_first; 
  geometry_msgs::PoseWithCovarianceStamped msg_second;
  geometry_msgs::PoseWithCovarianceStamped msg_third;

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

  // Push messages to buffer
  myBuffer.push_back(msg_first); 
  myBuffer.push_back(msg_second); 
  myBuffer.push_back(msg_third);
  // std::cout<<myBuffer.size()<<std::endl;
  // for (size_t i=0; i<2; ++i){
  //   std::cout<< myBuffer[i].header.stamp.toSec() << std::endl;
  // }
  // std::cout<< myBuffer[0].header.stamp.toSec() << std::endl;
  // std::cout<< myBuffer[1].header.stamp.toSec() << std::endl;
  // std::cout<< myBuffer[2].header.stamp.toSec() << std::endl;
  // std::cout<< myBuffer[4].header.stamp.toSec() << std::endl;
  bool result = GetPoseAtTime(t3_ros, myBuffer, myOutput);
  EXPECT_NEAR(msg_third.pose.pose.position.x, myOutput.pose.pose.position.x, 1e-5);
  ASSERT_TRUE(result);
  // EXPECT_EQ(myBuffer[1].header.stamp.toSec(),2);
}

TEST_F(OdometryHandlerTest, TestGetPosesAtTimes) {
  double t1 = 1.0;
  double t2 = 2.0;
  double t3 = 3.0;
  ros::Time t1_ros;
  ros::Time t2_ros;
  ros::Time t3_ros;
  t1_ros.fromSec(t1);
  t2_ros.fromSec(t2);
  t3_ros.fromSec(t3);

  // Create an output
  PoseCovStampedPair myOutput;
  // Create a buffer
  OdomPoseBuffer myBuffer; 
  // Create two messages
  geometry_msgs::PoseWithCovarianceStamped msg_first; 
  geometry_msgs::PoseWithCovarianceStamped msg_second;
  geometry_msgs::PoseWithCovarianceStamped msg_third;

  // Fill the two messages
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

  // Push messages to buffer
  myBuffer.push_back(msg_first); 
  myBuffer.push_back(msg_second); 
  myBuffer.push_back(msg_third);

  bool result = GetPosesAtTimes(t1_ros, t2_ros, myBuffer, myOutput);
  ASSERT_TRUE(result);
}

TEST_F(OdometryHandlerTest, TestGetKeyedScanAtTime) {

  // Create time stamps
  double t1 = 1.0;
  double t2 = 2.0;
  double t3 = 3.0;
  double t4 = 4.0;
  double t5 = 5.0;
  double t6 = 6.0;
  ros::Time t1_ros;
  ros::Time t2_ros;
  ros::Time t3_ros;
  ros::Time t4_ros;
  ros::Time t5_ros;
  ros::Time t6_ros;
  t1_ros.fromSec(t1);
  t2_ros.fromSec(t2);
  t3_ros.fromSec(t3);
  t4_ros.fromSec(t4);
  t5_ros.fromSec(t5);
  t6_ros.fromSec(t6);

  // Create keyed scans
  sensor_msgs::PointCloud2 msg1;
  sensor_msgs::PointCloud2 msg2;
  sensor_msgs::PointCloud2 msg3;
  sensor_msgs::PointCloud2 msg4;
  sensor_msgs::PointCloud2 msg5;
  sensor_msgs::PointCloud2 msg6;

  msg1.header.stamp = t1_ros;
  msg2.header.stamp = t2_ros;
  msg3.header.stamp = t3_ros;
  msg4.header.stamp = t4_ros;
  msg5.header.stamp = t5_ros;
  msg6.header.stamp = t6_ros;

  sensor_msgs::PointCloud2::ConstPtr pc_ptr1(new sensor_msgs::PointCloud2(msg1));
  sensor_msgs::PointCloud2::ConstPtr pc_ptr2(new sensor_msgs::PointCloud2(msg2));
  sensor_msgs::PointCloud2::ConstPtr pc_ptr3(new sensor_msgs::PointCloud2(msg3));
  sensor_msgs::PointCloud2::ConstPtr pc_ptr4(new sensor_msgs::PointCloud2(msg4));
  sensor_msgs::PointCloud2::ConstPtr pc_ptr5(new sensor_msgs::PointCloud2(msg5));
  sensor_msgs::PointCloud2::ConstPtr pc_ptr6(new sensor_msgs::PointCloud2(msg6));

  
  PointCloudCallback(pc_ptr1);
  PointCloudCallback(pc_ptr2);
  PointCloudCallback(pc_ptr3);
  PointCloudCallback(pc_ptr4);
  PointCloudCallback(pc_ptr5);
  PointCloudCallback(pc_ptr6);

  // Create the keyed scan to be filled by GetKeyedScanAtTime method
  PointCloud::Ptr my_keyed_scan;
  bool result = GetKeyedScanAtTime(t3_ros, my_keyed_scan);
  ASSERT_TRUE(result);
}

    // PointCloud current_pointcloud;
    // pcl::fromROSMsg(*msg, current_pointcloud);


// Test we pass but need more testing/implementation ---------------------------------


// Test we don't pass ----------------------------------------------------------------

/* TEST InsertMsgInBuffer */
// TEST_F(OdometryHandlerTest, InsertMsgInBuffer) {
//    // Create a buffer
//     std::vector<PoseCovStamped> myBuffer;
//     // Create a message
//     Odometry::ConstPtr msg;
//     // Call the method
//     bool result = InsertMsgInBuffer<Odometry, PoseCovStamped>(msg, myBuffer);
//    // Check result is correct
//    ASSERT_TRUE(result);
// }

TEST_F(OdometryHandlerTest, TestGetCovariance) {
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
      covariance_expected(i,j) = 2;
    }
  }
  gtsam::SharedNoiseModel noise_expected =
      gtsam::noiseModel::Gaussian::Covariance(covariance_expected);
  // ASSERT_TRUE((*noise_actual).equals(*noise_expected));
}

/* TEST FillGtsamPosCovOdom */
TEST_F(OdometryHandlerTest, TestFillGtsamPosCovOdom) {
  double t1 = 1.0;
  double t2 = 2.0;
  ros::Time t1_ros;
  ros::Time t2_ros;
  t1_ros.fromSec(t1);
  t2_ros.fromSec(t2);
  OdomPoseBuffer odom_buffer;
  // Odometry-1
  PoseCovStamped odom1;
  odom1.header.stamp = t1_ros;
  odom1.pose.pose.position.x = 0;
  odom1.pose.pose.position.y = 0;
  odom1.pose.pose.position.z = 0;
  odom1.pose.pose.orientation.x = 0;
  odom1.pose.pose.orientation.y = 0;
  odom1.pose.pose.orientation.z = 0;
  odom1.pose.pose.orientation.w = 1;
  for (size_t i = 0; i < 36; i++) {
    odom1.pose.covariance[i] = 1;
  }
  odom_buffer.push_back(odom1);
  // Odometry-2
  PoseCovStamped odom2;
  odom2.header.stamp = t2_ros;
  odom2.pose.pose.position.x = 1;
  odom2.pose.pose.position.y = 0;
  odom2.pose.pose.position.z = 0;
  odom2.pose.pose.orientation.x = 0;
  odom2.pose.pose.orientation.y = 0;
  odom2.pose.pose.orientation.z = 0;
  odom2.pose.pose.orientation.w = 1;
  for (size_t i = 0; i < 36; i++) {
    odom2.pose.covariance[i] = 3;
  }
  odom_buffer.push_back(odom2);
  // FillGtsamPosCovOdom
  GtsamPosCov odom_delta_actual;
  FillGtsamPosCovOdom(odom_buffer, odom_delta_actual, t1_ros, t2_ros);
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
  // ASSERT_TRUE(odom_delta_actual.b_has_value);
  // ASSERT_TRUE((odom_delta_actual.pose).equals(odom_delta_expected.pose));
  // ASSERT_TRUE((*(odom_delta_actual.covariance)).equals(*noise_expected));
}


// Initialize: Done

// LoadParameters

// RegisterCallbacks

// GetData: 

// GetOdomDelta:

// GetKeyedScanAtTime: 

// GetFusedOdomDeltaBetweenTimes

// LidarOdometryCallback

// VisualOdometryCallback

// WheelOdometryCallback

// PointCloudCallback

// CheckBufferSize: Done (Kamak)

// InsertMsgInBuffer: Done

// FillGtsamPosCovOdom

// CalculatePoseDelta(gtsam version): (Nobuhiro)

// ClearOdometryBuffers

// ResetFactorData

// GetTransform: Done (Nobuhiro)

// GetCovariance: Done (Nobuhiro)

// GetTimeStamps: Done (Nobuhiro)

// ToGtsam: (Kamak)

// GetPoseAtTime: (Kamak)

// GetPosesAtTimes: (Kamak)


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_odometry_handler");
  return RUN_ALL_TESTS();
}

/*
UNUSED

unit tests
// CalculatePoseDelta: Done (Nobuhiro)
// GetDeltaBetweenPoses: (Kamak)
// GetDeltaBetweenTimes: (Kamak)
    

double CalculatePoseDelta(OdomPoseBuffer& odom_buffer){
  return oh.CalculatePoseDelta(odom_buffer);
}



TEST_F (OdometryHandlerTest, TestCalculatePoseDelta){
  // Create a buffer
  OdomPoseBuffer myBuffer; 
  // Create two messages
  geometry_msgs::PoseWithCovarianceStamped msg_first; 
  geometry_msgs::PoseWithCovarianceStamped msg_second;
  // Fill the two messages
  msg_first.pose.pose.position.x = 1; 
  msg_first.pose.pose.position.y = 0; 
  msg_first.pose.pose.position.z = 0; 
  msg_first.pose.pose.orientation.x = 0;
  msg_first.pose.pose.orientation.y = 0;
  msg_first.pose.pose.orientation.z = 0;
  msg_first.pose.pose.orientation.w = 1;
  msg_second.pose.pose.position.x = 0; 
  msg_second.pose.pose.position.y = 0; 
  msg_second.pose.pose.position.z = 0;
  msg_second.pose.pose.orientation.x = 0;
  msg_second.pose.pose.orientation.y = 0;
  msg_second.pose.pose.orientation.z = 0;
  msg_second.pose.pose.orientation.w = 1;
  // Push messages to buffer
  myBuffer.push_back(msg_first); 
  myBuffer.push_back(msg_second);   
  // Call the method to test 
  int size = CheckBufferSize(myBuffer);
  EXPECT_EQ(size, 2);
  double delta = CalculatePoseDelta(myBuffer);   
  EXPECT_EQ(delta, 1);
}


std::pair<ros::Time, ros::Time> GetTimeStamps(PoseCovStampedPair pose_cov_stamped_pair) {
  return oh.GetTimeStamps(pose_cov_stamped_pair);
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