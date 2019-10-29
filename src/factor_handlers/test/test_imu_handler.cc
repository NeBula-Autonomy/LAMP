#include <factor_handlers/ImuHandler.h>
#include <gtest/gtest.h>

class ImuHandlerTest : public ::testing::Test {

  public:

    ImuHandlerTest() {      
      // Load Params
      system("rosparam load $(rospack find "
             "factor_handlers)/config/imu_parameters.yaml");   

      tolerance_ = 1e-5;

      double t1 = 1.00;
      double t2 = 1.05;
      double t3 = 1.10;

      t1_ros.fromSec(t1);
      t2_ros.fromSec(t2);
      t3_ros.fromSec(t3);

      // 0 degree Pitch
      msg_first.header.stamp = t1_ros;
      msg_first.orientation.x = 0;
      msg_first.orientation.y = 0;
      msg_first.orientation.z = 0;
      msg_first.orientation.w = 1;    

      // 10 degree Pitch
      msg_second.header.stamp = t2_ros;
      msg_second.orientation.x = 0;
      msg_second.orientation.y = 0.0871557;
      msg_second.orientation.z = 0;
      msg_second.orientation.w = 0.9961947;   

      // 20 degree Pitch
      msg_third.header.stamp = t3_ros;
      msg_third.orientation.x = 0;
      msg_third.orientation.y = 0.1736482;
      msg_third.orientation.z = 0;
      msg_third.orientation.w = 0.9848078;     
    }   

    ImuHandler ih;

    double tolerance_;

  protected: 
  
    ImuMessage msg_first;
    ImuMessage msg_second;
    ImuMessage msg_third;

    ros::Time t1_ros;
    ros::Time t2_ros;
    ros::Time t3_ros;

  private:

};

/* TEST Initialize */
TEST_F(ImuHandlerTest, TestInitialize) {
  ros::NodeHandle nh;
  bool result = ih.Initialize(nh);
  ASSERT_TRUE(result);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_imu_handler");
  return RUN_ALL_TESTS();
}