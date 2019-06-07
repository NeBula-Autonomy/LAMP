/**
 *  @brief Test cases for talker class
 *
 *  This shows an example usage of gtest.
 *  The fixture pattern is used to execute the common operations
 *  before and after each test case.
 */

#include <gtest/gtest.h>
#include "measurement_synchronizer/MeasurementSynchronizer.h"

class MeasurementSynchronizerTest : public ::testing::Test {
protected:
  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "MeasurementSynchronizerTest");
  return RUN_ALL_TESTS();
}
