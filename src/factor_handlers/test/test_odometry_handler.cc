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
    bool InsertMsgInBuffer(const typename T1::ConstPtr& msg, std::vector<T2>& buffer) {
      return oh.InsertMsgInBuffer<T1, T2>(msg, buffer);
    }

    // Utilities
    // template <typename TYPE>
    // int CheckBufferSize(const std::vector<TYPE>& x) {
    //   return myOdometryHandler.CheckBufferSize<TYPE>(x);
    // }

    double CalculatePoseDelta(OdomPoseBuffer& odom_buffer){
      return oh.CalculatePoseDelta(odom_buffer);
    }   

    std::vector<geometry_msgs::PoseWithCovarianceStamped> lidar_odometry_buffer_ = oh.lidar_odometry_buffer_;

  private:    

};

// Test we pass ----------------------------------------------------------------------

/* TEST Initialize */ 
TEST_F(OdometryHandlerTest, Initialization) {
   ros::NodeHandle nh, pnh("~");
   bool result = oh.Initialize(nh);
   ASSERT_TRUE(result);
}

/* TEST InsertMsgInBuffer */
TEST_F(OdometryHandlerTest, InsertMsgInBuffer) {
   // Create a buffer
    std::vector<PoseCovStamped> myBuffer;
    // Create a message
    Odometry::ConstPtr msg;
    // Call the method
    bool result = InsertMsgInBuffer<Odometry, PoseCovStamped>(msg, myBuffer);
   // Check result is correct
   ASSERT_TRUE(result);
}

/* TEST CheckBufferSize - TODO: Not passing the test*/ 
TEST_F(OdometryHandlerTest, TestCheckBufferSize) {
  std::vector<PoseCovStamped> myBuffer;
  PoseCovStamped my_msg;
  myBuffer.push_back(my_msg);
  int size = CheckBufferSize(myBuffer);
  EXPECT_EQ(size, 1);
}

/* TEST CalculatePoseDelta*/
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

// GetTransform function unit test is done, will be pasted here later, Nobuhiro

// Nobuhiro is working on GetCovariance function unit test

// Nobuhiro is working on GetTimeStam function unit test


// Test we pass but need more testing/implementation ---------------------------------


// Test we don't pass ----------------------------------------------------------------



int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_odometry_handler");
  return RUN_ALL_TESTS();
}