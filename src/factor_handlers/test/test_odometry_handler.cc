/**
 *  @brief Test cases for talker class
 *
 *  This file shows an example usage of gtest.
 */

#include <gtest/gtest.h>
#include <factor_handlers/OdometryHandler.h>

using namespace std;

class OdometryHandlerTest : public ::testing::Test {
  
  protected:

    void LidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      myOdometryHandler.LidarOdometryCallback(msg);
      }

    int Foo(int a) {
      myOdometryHandler.Foo(a);
    }       

    int CheckLidarOdometryBufferSize() {
      myOdometryHandler.CheckLidarOdometryBufferSize();
    }

  private: 
    OdometryHandler myOdometryHandler; 
};




TEST_F(OdometryHandlerTest, TestLidarOdometryCallback) {   

  // Construct a OdometryHandler Object
  // OdometryHandler myOdometryHandler; 
  
  // Check that result is correct  
  EXPECT_EQ(CheckLidarOdometryBufferSize(), 0);

  // Create a message to trigger the Callback 
  // const int * intPtr1; // Declares a pointer whose contents cannot be changed.

  //const nav_msgs::Odometry OdometryMsg;
  
  //LidarOdometryCallback(OdometryMsg);


  EXPECT_EQ(Foo(2),2)


  // Check that result is correct  
  //EXPECT_EQ(CheckLidarOdometryBufferSize(), 0);

  //myOdometryHandler.LidarOdometryCallback()

}




// TEST(OdomHandlerTest, EnqueueLidarOdometryMsgs) {
//   ros::NodeHandle nh, pnh("~");
//   OdometryHandler myOdometryHandler; 
//   // myOdometryHandler.Initialize(nh);
//   // std::string phrase("Hello World");
//   // std_msgs::String msg = talker.composeGreetingMsg(phrase);
//   // EXPECT_EQ(phrase, msg.data);
// }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_odometry_handler");
  return RUN_ALL_TESTS();
}




