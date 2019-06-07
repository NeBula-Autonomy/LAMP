/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for blam_slam class
 *
 */

#include <gtest/gtest.h>

#include "blam_slam/BlamSlam.h"

/**
 * Test setup
 */
class BlamSlamTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    test_variable = new BlamSlam();
  }

  virtual void TearDown() {
  }
  ros::NodeHandle nh;
  BlamSlam* test_variable;
};

/**
 * Dummy unit test for now
 */
TEST_F(BlamSlamTest, RegisterLogCallbacks) {
  bool log_var = test_variable->Initialize(nh, true);
  EXPECT_EQ(log_var, true);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "BlamSlamTest");
  return RUN_ALL_TESTS();
}
