/**
 *  @brief Test cases for talker class
 *
 *  This file shows an example usage of gtest.
 */

#include <gtest/gtest.h>
#include <factor_handlers/OdometryHandler.h>

class OdometryHandlerTest : public ::testing::Test {
  
  protected: 

    // Odometry Callbacks ------------------------------------------------   
    void LidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      myOdometryHandler.LidarOdometryCallback(msg);
    } 

    void VisualOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      myOdometryHandler.LidarOdometryCallback(msg);
    }  
    
    void WheelOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      myOdometryHandler.LidarOdometryCallback(msg);
    } 

    // Utilities 
    template <typename TYPE>
    int CheckBufferSize(std::vector<TYPE> const& x) {
      return myOdometryHandler.CheckBufferSize(x);
    }

    int CheckMyBufferSize(const std::vector<geometry_msgs::PoseWithCovarianceStamped>& x) {
      return myOdometryHandler.CheckMyBufferSize(x);
    }

    double CalculatePoseDelta(std::vector<geometry_msgs::PoseWithCovarianceStamped>& odom_buffer){
      return myOdometryHandler.CalculatePoseDelta(odom_buffer);
    }    

    std::vector<geometry_msgs::PoseWithCovarianceStamped> lidar_odometry_buffer_ = myOdometryHandler.lidar_odometry_buffer_;

  private: 
    OdometryHandler myOdometryHandler; 

};



/*
TEST LidarOdometryCallback
  Create a pointer to the Odometry message 
  Trigger the callback 
  Check that the callback successfully pushed the received message in the buffer 
*/
TEST_F (OdometryHandlerTest, TestLidarOdometryCallback){
  nav_msgs::Odometry::ConstPtr msg_p;
  LidarOdometryCallback(msg_p);
  EXPECT_EQ(CheckMyBufferSize(lidar_odometry_buffer_), 1);

  nav_msgs::Odometry::ConstPtr msg_pp;
  LidarOdometryCallback(msg_pp);
  EXPECT_EQ(CheckMyBufferSize(lidar_odometry_buffer_), 2);

}



/*
TEST CheckMyBufferSize method 
  Create a buffer 
  Create a message 
  Push message in the buffer 
  Check the resulting buffer size 
*/
TEST_F (OdometryHandlerTest, TestCheckMyBufferSize) {

  int N = 10;

  // Create a buffer 
  std::vector<geometry_msgs::PoseWithCovarianceStamped> pose_buffer;
  // Create a message
  geometry_msgs::PoseWithCovarianceStamped pose;

  for (size_t x=0; x<N; x++){    
    // Push the message in the buffer
    pose_buffer.push_back(pose);   
    std::cout << x << std::endl; 
  }

  // Compute current buffer size
  int buffer_size = CheckMyBufferSize(pose_buffer);
  // int buffer_size = CheckBufferSize<geometry_msgs::PoseStamped>(pose_stamped_buffer);
  // Check that the result is the expected
  
  EXPECT_EQ(buffer_size, N);
  
}

/*
TEST CalculatePoseDelta method 
  Create a buffer 
  Fill the buffer with two messages 
  Invoke the CalculatePoseDelta method
*/
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
  double delta = CalculatePoseDelta(myBuffer);   
  EXPECT_EQ(delta, 1);
  }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_odometry_handler");
  return RUN_ALL_TESTS();
}