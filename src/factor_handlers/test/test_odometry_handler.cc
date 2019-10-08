/**
 *  @brief Test cases for talker class
 *
 *  This file shows an example usage of gtest.
 */

#include <gtest/gtest.h>
#include <factor_handlers/OdometryHandler.h>

typedef geometry_msgs::PoseWithCovarianceStamped PoseCovStamped;

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

    double CalculatePoseDelta(std::vector<geometry_msgs::PoseWithCovarianceStamped>& odom_buffer){
      return oh.CalculatePoseDelta(odom_buffer);
    }    

    std::vector<geometry_msgs::PoseWithCovarianceStamped> lidar_odometry_buffer_ = oh.lidar_odometry_buffer_;

  private:    

};

// Test we pass ----------------------------------------------------------------------

/*
TEST Initialize method 
*/
TEST_F(OdometryHandlerTest, Initialization) {
   ros::NodeHandle nh, pnh("~");
   bool result = oh.Initialize(nh);
   ASSERT_TRUE(result);
}
 
TEST_F(OdometryHandlerTest, CheckBufferSize) {
   std::vector<PoseCovStamped> myBuffer;
   PoseCovStamped my_msg;
   myBuffer.push_back(my_msg);
   int size = CheckBufferSize(myBuffer);
   EXPECT_EQ(size, 1);
}



/*
TEST CheckBufferSize method 
  Create a buffer 
  Create a message 
  Push message in the buffer 
  Check the resulting buffer size 
*/


// Test we pass but need more testing/implementation ---------------------------------


// Test we don't pass ----------------------------------------------------------------
/*

TEST CalculatePoseDelta method 
  Create a buffer 
  Fill the buffer with two messages 
  Invoke the CalculatePoseDelta method

TEST_F (OdometryHandlerTest, TestCalculatePoseDelta){
  // Create a buffer
  std::vector<geometry_msgs::PoseWithCovarianceStamped> myBuffer; 
  // Create two messages
  geometry_msgs::PoseWithCovarianceStamped msg_first; 
  geometry_msgs::PoseWithCovarianceStamped msg_second;
  // Fill the two messages
  msg_first.pose.pose.position.x = 1; 
  msg_first.pose.pose.position.y = 0; 
  msg_first.pose.pose.position.z = 0; 
  msg_second.pose.pose.position.x = 0; 
  msg_second.pose.pose.position.y = 0; 
  msg_second.pose.pose.position.z = 0;
  // Push messages to buffer
  myBuffer.push_back(msg_first); 
  myBuffer.push_back(msg_second);   
  // Call the method to test 
  int size = CheckBufferSize(myBuffer);
  EXPECT_EQ(size, 2);
  // double delta = CalculatePoseDelta(myBuffer);   
  // EXPECT_EQ(delta, 1);
}


TEST_F (OdometryHandlerTest, TestGetKeyedScanAtTime) {
  
}

// TEST_F(OdometryHandlerTest, InsertMsgInBuffer) {
//   // Create a buffer
//   std::vector<PoseCovStamped> myBuffer;
//   nav_msgs::Odometry msg;
//   bool result = InsertMsgInBuffer<nav_msgs::Odometry, PoseCovStamped>(msg, myBuffer);
//   ASSERT_TRUE(result);
// }

/*
TEST LidarOdometryCallback
  Create a pointer to the Odometry message 
  Trigger the callback 
  Check that the callback successfully pushed the received message in the buffer 
*/


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_odometry_handler");
  return RUN_ALL_TESTS();
}






