/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include "factor_handlers/ArtifactHandler.h"

class TestArtifactHandler : public ::testing::Test{
  public: 
    ros::NodeHandle nh;
    ArtifactHandler art_handle;
    TestArtifactHandler(){
      art_handle.use_artifact_loop_closure_ = true;
    }
    ~TestArtifactHandler(){}
    bool LoadParameters() {return art_handle.LoadParameters(nh);}
    bool RegisterCallbacks() {return art_handle.RegisterCallbacks(nh, false);};
    bool UpdateGlobalPose(gtsam::Symbol artifact_key ,gtsam::Pose3 global_pose) {art_handle.UpdateGlobalPose(artifact_key, global_pose);};
    ArtifactData& GetArtifactData() {return art_handle.artifact_data_;};
    FactorData* GetData() {return art_handle.GetData();};
    bool ArtifactCallback(core_msgs::Artifact msg) {art_handle.ArtifactCallback(msg);};
    std::unordered_map<long unsigned int, ArtifactInfo> GetKeyInfoMap() {return art_handle.artifact_key2info_hash_;};
    std::unordered_map<std::string, gtsam::Symbol> GetStringKeyMap() {return art_handle.artifact_id2key_hash;};
    void AddArtifactKey(gtsam::Symbol key) {art_handle.artifact_id2key_hash["distal"] = key;};
    std::string getID(gtsam::Symbol key) {return art_handle.GetArtifactID(key);};
    void AddArtifact(const gtsam::Symbol artifact_key, ros::Time time_stamp, const gtsam::Pose3 transform, const gtsam::SharedNoiseModel noise) {art_handle.AddArtifactData(artifact_key, time_stamp, transform, noise);};
    Eigen::Vector3d ComputeTrans(const core_msgs::Artifact& msg) {return art_handle.ComputeTransform(msg);};
    void ClearData() {art_handle.ClearArtifactData();};
};

TEST_F(TestArtifactHandler, ArtifactInfoInitialize) {
  // Constructor
  ArtifactInfo art_;

  // Check if ArtifactInfo is initialised correctly
  EXPECT_EQ(art_.num_updates, 0);
  EXPECT_EQ(art_.id, "");
  EXPECT_EQ(art_.global_pose.translation().vector(),
            Eigen::Vector3d(0.0, 0.0, 0.0));
}

TEST_F(TestArtifactHandler, LoadParameters) {
  ros::param::set("b_artifacts_in_global", false);
  ros::param::set("use_artifact_loop_closure", true);
  ros::param::set("artifact_prefix", "Artifact");
  ASSERT_TRUE(LoadParameters());
}

TEST_F(TestArtifactHandler, RegisterCallbacks) {
  ASSERT_TRUE(RegisterCallbacks());
}

// TODO Something wrong in this. The artifact_key is missing something
TEST_F(TestArtifactHandler, UpdateGlobalPose) {
  // Key is 1
  gtsam::Symbol artifact_key = 1;
  // Global pose
  gtsam::Pose3 global_pose =
      gtsam::Pose3(gtsam::Rot3(0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1),
                   gtsam::Point3(1.0, 1.0, 1.0));
  // global_pose.translation().vector() = Eigen::Vector3d (1.0,1.0,1.0);
  // Add Artifact Info to the key hash
  ArtifactInfo art_info("distal");
  auto& ArtifactKeyToInfo = art_handle.GetArtifactKey2InfoHash();
  ArtifactKeyToInfo[artifact_key] = art_info;
  // Update global pose
  UpdateGlobalPose(artifact_key, global_pose);
  // Check if translation part of pose is updated
  EXPECT_EQ(ArtifactKeyToInfo[artifact_key].global_pose.translation().vector(),
            Eigen::Vector3d(1.0, 1.0, 1.0));
}

TEST_F(TestArtifactHandler, ComputeTransform) {
  // Construct the message
  core_msgs::Artifact msg;
  msg.parent_id = "distal";
  msg.confidence = 0.9;
  msg.id = "distal";
  msg.point.point.x = 0.9;
  msg.point.point.y = 0.3;
  msg.point.point.z = 0.5;
  msg.label = "backpack";

  // Check the value
  EXPECT_EQ(ComputeTrans(msg), Eigen::Vector3d(0.9,0.3,0.5));
}

TEST_F(TestArtifactHandler, ClearArtifactData) {
  // Add to the transform
  gtsam::Pose3 global_pose = gtsam::Pose3(gtsam::Rot3(0.1, 0,0,
                                                      0, 0.1,0,
                                                      0,0, 0.1), 
                                          gtsam::Point3 (1.0,1.0,1.0));

  // Add the artifacts message timsetamp 
  ros::Time ros_time = ros::Time(0.1);
  // Get covariance
  Eigen::VectorXd sig (6);
  sig << 0.3,0.3,0.3,0.3,0.3,0.3;
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(sig);
  // Add the data
  AddArtifact(gtsam::Symbol('l',0), ros_time, global_pose, noise);
  // Get the data  
  ArtifactData& artifact_data = GetArtifactData();
  // Check size of new artifact values
  EXPECT_EQ(artifact_data.factors.size(), 1);
  // Clear the values
  ClearData();
  // Check size of new artifact values
  EXPECT_EQ(artifact_data.factors.size(), 0);
}

TEST_F(TestArtifactHandler, GetData)
{
  ArtifactData& artifact_data = GetArtifactData();
  // Set the new data flag
  artifact_data.b_has_data = true;
  // Set the type to artifact
  artifact_data.type = "artifact";
  // Add to the transform
  gtsam::Pose3 global_pose =
      gtsam::Pose3(gtsam::Rot3(0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1),
                   gtsam::Point3(1.0, 1.0, 1.0));

  // Add the new artifact
  ArtifactFactor artifact; 
  artifact.position = global_pose.translation();
  artifact.stamp = ros::Time(0.1);
  artifact.key = 1;
  artifact_data.factors.push_back(artifact);

  // Get the data and check if we get the data back
  ArtifactData* stored_data = dynamic_cast<ArtifactData*>(GetData());
  ArtifactFactor artifact_factor = stored_data->factors[0];

  // Check the data
  EXPECT_EQ(stored_data->type, "artifact");
  ASSERT_TRUE(stored_data->b_has_data);
  EXPECT_EQ(artifact_factor.key, 1);
  EXPECT_EQ(artifact_factor.stamp, ros::Time(0.1));
}

TEST_F(TestArtifactHandler, GetArtifactID) {
  gtsam::Symbol key ('l',0);
  EXPECT_EQ(getID(key), "");
  AddArtifactKey(key);
  EXPECT_EQ(getID(key), "distal");
}

TEST_F(TestArtifactHandler, AddArtifactData) {
  // Add to the transform
  gtsam::Pose3 global_pose = gtsam::Pose3(gtsam::Rot3(0.1, 0,0,
                                                      0, 0.1,0,
                                                      0,0, 0.1), 
                                          gtsam::Point3 (1.0,1.0,1.0));

  // Add the artifacts message timsetamp 
  ros::Time ros_time = ros::Time(0.1);
  // Get covariance
  Eigen::VectorXd sig (6);
  sig << 0.3,0.3,0.3,0.3,0.3,0.3;
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(sig);
  // Add the data
  AddArtifact(gtsam::Symbol('l',0), ros_time, global_pose, noise);
  // Get the data  
  ArtifactData& factor = GetArtifactData();
  ArtifactFactor artifact_factor = factor.factors[0];
  // Check if data is new
  EXPECT_TRUE(factor.b_has_data);
  EXPECT_EQ(artifact_factor.key, gtsam::Symbol('l',0));
  EXPECT_EQ(artifact_factor.stamp, ros::Time(0.1));
  EXPECT_EQ(artifact_factor.position.vector(), Eigen::Vector3d(1.0,1.0,1.0));
}

TEST_F(TestArtifactHandler, ArtifactCallback) {
  // Construct the message
  core_msgs::Artifact msg;
  msg.parent_id = "distal";
  msg.confidence = 0.9;
  msg.id = "distal";
  msg.point.point.x = 0.9;
  msg.point.point.y = 0.3;
  msg.point.point.z = 0.5;
  msg.label = "backpack";
  // Trigger the callback
  ArtifactCallback(msg);
  // Get the data
  ArtifactData stored_data = GetArtifactData();
  ArtifactFactor artifact_factor = stored_data.factors[0];
  // Check if data is flowing correctly
  // TODO Check next line
  EXPECT_EQ(stored_data.type, "artifact");
  ASSERT_TRUE(stored_data.b_has_data);
  EXPECT_EQ(artifact_factor.key.index(), 0);
  EXPECT_EQ(artifact_factor.stamp, ros::Time(0.0));
  EXPECT_EQ(artifact_factor.position.vector(), Eigen::Vector3d (0.9,0.3,0.5));
  // Check if maps are filled
  gtsam::Pose3 global_pose =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.9, 0.3, 0.5));
  // Update the global pose
  UpdateGlobalPose(artifact_factor.key, global_pose);
  // Get the ArtifactInfo
  auto KeyInfoMap = GetKeyInfoMap();
  // Is the data in ArtifactInfo correct
  EXPECT_EQ(KeyInfoMap[artifact_factor.key].global_pose.translation().vector(), Eigen::Vector3d (0.9,0.3,0.5));
  
  // Construct a new message
  msg.parent_id = "distal";
  msg.confidence = 0.8;
  msg.id = "distal";
  msg.point.point.x = 0.3;
  msg.point.point.y = 0.3;
  msg.point.point.z = 0.3;
  msg.label = "backpack";
  // Trigger the callback
  ArtifactCallback(msg);
  // Get the data
  ArtifactData* internal_data = dynamic_cast<ArtifactData*>(GetData());
  artifact_factor = internal_data->factors[1];
  // Check data
  EXPECT_EQ(stored_data.type, "artifact");
  ASSERT_TRUE(stored_data.b_has_data);
  EXPECT_EQ(artifact_factor.key.index(), 0);
  EXPECT_EQ(artifact_factor.stamp, ros::Time(0.0));
  EXPECT_EQ(artifact_factor.position.vector(),
            Eigen::Vector3d(0.3, 0.3, 0.3));

  // Update the global pose
  UpdateGlobalPose(artifact_factor.key, global_pose);
  // Get the Key Node
  auto StringKeyMap = GetStringKeyMap();
  // Is the data in ArtifactInfo correct
  EXPECT_EQ(StringKeyMap.size(), 1);
  // Get the ArtifactInfo
  KeyInfoMap = GetKeyInfoMap();
  // Is the data in ArtifactInfo correct
  EXPECT_EQ(KeyInfoMap.size(), 1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_artifact_handler");
  return RUN_ALL_TESTS();
}
