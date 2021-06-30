#include <factor_handlers/StationaryHandler.h>
#include <gtest/gtest.h>

class StationaryHandlerTest : public ::testing::Test {
 public:
  StationaryHandlerTest() : tolerance_(1e-5) {
    // Load Params
    system(
        "rosparam load $(rospack find "
        "lamp)/config/precision_parameters.yaml");
  }

  StationaryHandler sh_;

  double tolerance_;

 protected:
  std::shared_ptr<FactorData> getData() { return sh_.GetData(); }

  bool SetKeyForImuAttitude(const gtsam::Symbol& key) {
    return sh_.SetKeyForImuAttitude(key);
  }

  void stationaryCallback(const StationaryMessage::ConstPtr& msg) {
    sh_.StationaryCallback(msg);
  }

  // Quaternions
  bool getGravityAtTime(const ros::Time& stamp,
                        geometry_msgs::Vector3& gravity) {
    return sh_.GetGravityAtTime(stamp, gravity);
  }

  // Factors
  gtsam::Pose3AttitudeFactor createAttitudeFactor(
      const geometry_msgs::Vector3& gravity_vec) {
    return sh_.CreateAttitudeFactor(gravity_vec);
  }

  void resetFactorData() { sh_.ResetFactorData(); }
};

/* TEST Initialize */
TEST_F(StationaryHandlerTest, TestInitialize) {
  ros::NodeHandle nh;
  bool result = sh_.Initialize(nh);
  ASSERT_TRUE(result);
}

TEST_F(StationaryHandlerTest, CreateAttitudeFactor) {
  ros::NodeHandle nh;
  sh_.Initialize(nh);

  SetKeyForImuAttitude(gtsam::Symbol('a', 0));

  geometry_msgs::Vector3 g_vec;
  g_vec.x = 0;
  g_vec.y = 0.01;
  g_vec.z = 0.99;
  gtsam::Pose3AttitudeFactor factor = createAttitudeFactor(g_vec);

  EXPECT_EQ(gtsam::Symbol('a', 0), factor.key());
  EXPECT_NEAR(0, factor.nZ().unitVector()[0], tolerance_);
  EXPECT_NEAR(0, factor.nZ().unitVector()[1], tolerance_);
  EXPECT_NEAR(1, factor.nZ().unitVector()[2], tolerance_);
  EXPECT_NEAR(0, factor.bRef().unitVector()[0], tolerance_);
  EXPECT_NEAR(0.0101, factor.bRef().unitVector()[1], tolerance_);
  EXPECT_NEAR(0.99995, factor.bRef().unitVector()[2], tolerance_);

  // Check error
  gtsam::Pose3 test_pose_1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3());
  gtsam::Pose3 test_pose_2 =
      gtsam::Pose3(gtsam::Rot3(0.9999872, 0.0050503, 0, 0), gtsam::Point3());

  gtsam::Vector nRef_2 =
      gtsam::Rot3(0.9999872, 0.0050503, 0, 0) * factor.bRef().unitVector();
  gtsam::Vector error_2_exp = factor.nZ().unitVector() - nRef_2;

  gtsam::Vector error_1 = factor.evaluateError(test_pose_1);
  gtsam::Vector error_2 = factor.evaluateError(test_pose_2);

  EXPECT_NEAR(0.0101, error_1[0], tolerance_);
  EXPECT_NEAR(0, error_1[1], tolerance_);
  EXPECT_NEAR(0, error_1[2], tolerance_);

  EXPECT_NEAR(error_2_exp[0], error_2[0], tolerance_);
  EXPECT_NEAR(error_2_exp[1], error_2[1], tolerance_);
  EXPECT_NEAR(error_2_exp[2], error_2[2], tolerance_);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_stationary_handler");
  return RUN_ALL_TESTS();
}