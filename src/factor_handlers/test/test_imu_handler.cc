#include <factor_handlers/ImuHandler.h>
#include <gtest/gtest.h>

class ImuHandlerTest : public ::testing::Test {

    public:

    ImuHandlerTest() {
      // Load Params
      system("rosparam load $(rospack find "
             "factor_handlers)/config/imu_parameters.yaml");   
    }

    ImuHandler ih;

    protected:  

    private:

};

// TODO: Add unit tests

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_imu_handler");
  return RUN_ALL_TESTS();
}

