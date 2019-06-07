/**
 *  @brief Test cases for talker class
 *
 *  This shows an example usage of gtest.
 *  The fixture pattern is used to execute the common operations
 *  before and after each test case.
 */

#include <gtest/gtest.h>

#include "blam_slam/BlamSlam.h"

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

TEST_F(BlamSlamTest, RegisterLogCallbacks) {
  bool log_var = test_variable->Initialize(nh, true);
  EXPECT_EQ(log_var, true);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "BlamSlamTest");
  return RUN_ALL_TESTS();
}
