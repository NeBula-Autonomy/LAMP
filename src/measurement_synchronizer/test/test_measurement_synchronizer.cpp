/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for measurement_synchronizer class
 *
 */

#include "measurement_synchronizer/MeasurementSynchronizer.h"
#include <gtest/gtest.h>

class MeasurementSynchronizerTest : public ::testing::Test {
protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "MeasurementSynchronizerTest");
  return RUN_ALL_TESTS();
}
