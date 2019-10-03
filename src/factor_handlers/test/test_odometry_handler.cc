/**
 *  @brief Test cases for talker class
 *
 *  This file shows an example usage of gtest.
 */

#include <gtest/gtest.h>
#include <factor_handlers/OdometryHandler.h>

class OdometryHandlerTest : public ::testing::Test {
  
  protected: 

    // Odometry Callbacks
   
    void LidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      myOdometryHandler.LidarOdometryCallback(msg);
    } 

    void VisualOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      myOdometryHandler.LidarOdometryCallback(msg);
    }  

    void WheelOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      myOdometryHandler.LidarOdometryCallback(msg);
    } 

    // Check Buffer Size Callbacks

    template <typename TYPE>
    int CheckBufferSize(std::vector<TYPE> const& x) {
      return myOdometryHandler.CheckBufferSize(x);
    }

    int CheckMyBufferSize(const std::vector<geometry_msgs::PoseWithCovarianceStamped>& x) {
      return myOdometryHandler.CheckMyBufferSize(x);
    }

  private: 
    OdometryHandler myOdometryHandler; 

};

TEST_F (OdometryHandlerTest, TestCheckMyBufferSize) {

  int N = 10;

  // Create a buffer 
  std::vector<geometry_msgs::PoseWithCovarianceStamped> pose_stamped_buffer;
  // Create a message
  geometry_msgs::PoseWithCovarianceStamped pose_stamped;

  for (size_t x=0; x<N; x++){    
    // Push the message in the buffer
    pose_stamped_buffer.push_back(pose_stamped);   
    std::cout << x << std::endl; 
  }

  // Compute current buffer size
  int buffer_size = CheckMyBufferSize(pose_stamped_buffer);
  // int buffer_size = CheckBufferSize<geometry_msgs::PoseStamped>(pose_stamped_buffer);
  // Check that the result is the expected
  
  EXPECT_EQ(buffer_size, N);
  
}


// TEST_F (OdometryHandlerTest, TestCheckBufferSize) {

//   int N = 10;

//   // Create a buffer 
//   std::vector<geometry_msgs::PoseStamped> pose_stamped_buffer;
//   // Create a message
//   geometry_msgs::PoseStamped pose_stamped;

//   for (size_t x=0; x<N; x++){    
//     // Push the message in the buffer
//     pose_stamped_buffer.push_back(pose_stamped);   
//     std::cout << x << std::endl; 
//   }

//   // Compute current buffer size
//   int buffer_size = CheckBufferSize<geometry_msgs::PoseStamped>(pose_stamped_buffer);

//   const std::vector<geometry_msgs::PoseStamped>& x


//   // Check that the result is the expected
  
//   //EXPECT_EQ(buffer_size, N);
//   EXPECT_EQ(0,0);
  
// }

// TEST_F(OdometryHandlerTest, TestLidarOdometryCallback) {   

//   nav_msgs::Odometry::ConstPtr msg_p;
//   LidarOdometryCallback(msg_p);
//   EXPECT_EQ(CheckBufferSize<geometry_msgs::PoseStamped>(msg_p), 1);

//   nav_msgs::Odometry::ConstPtr msg_pp; 
//   LidarOdometryCallback(msg_pp);

//   EXPECT_EQ(CheckBufferSize<geometry_msgs::PoseStamped>(msg_pp), 2);
// }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_odometry_handler");
  return RUN_ALL_TESTS();
}