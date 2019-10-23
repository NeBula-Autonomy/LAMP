/**
 *  @brief Testing the AprilTagHandler class
 *
 */

#include <gtest/gtest.h>

#include "factor_handlers/AprilTagHandler.h"

class TestAprilTagHandler : public ::testing::Test{
  public: 
    /**
     * Data
     */
    // Node handle
    ros::NodeHandle nh;
    // Class handler
    AprilTagHandler apt_handle;
    // Message
    core_msgs::AprilTag msg;

    /**
     * Methods
     */
    TestAprilTagHandler(){
      apt_handle.use_artifact_loop_closure_ = true;

      // Construct messages
      msg.header.stamp = ros::Time(1.0);
      msg.id = "distal";

      // Fill covariance
      msg.covariance[0] = 0.3;
      msg.covariance[1] = 0;
      msg.covariance[2] = 0;
      msg.covariance[3] = 0;
      msg.covariance[4] = 0.3;
      msg.covariance[5] = 0;
      msg.covariance[6] = 0;
      msg.covariance[7] = 0;
      msg.covariance[8] = 0.3;

      msg.point.point.x = 1.0;
      msg.point.point.y = 2.0;
      msg.point.point.z = 3.0;

      // Set parameters
      // rosparam::set()
    }

    ~TestAprilTagHandler(){}
    void AprilTagCallback(core_msgs::AprilTag msg) {
      apt_handle.AprilTagCallback(msg);
    }

    FactorData* GetData() {
      return apt_handle.GetData();
    }

    core_msgs::Artifact ConvertAprilTagMsgToArtifactMsg(core_msgs::AprilTag msg) {
      return apt_handle.ConvertAprilTagMsgToArtifactMsg(msg);
    }

    bool LoadParameters() {
      return apt_handle.LoadParameters(nh);
    }

    bool Initialize() {
      return apt_handle.Initialize(nh);
    }

    std::unordered_map<long unsigned int, ArtifactInfo> GetKeyInfoMap() {return apt_handle.artifact_key2info_hash_;};
    std::unordered_map<std::string, gtsam::Symbol> GetStringKeyMap() {return apt_handle.artifact_id2key_hash;};

    gtsam::Pose3 GetGroundTruth(gtsam::Symbol key){
      return apt_handle.GetGroundTruthData(key);
    }
};

// Check if callback working correctly.
TEST_F(TestAprilTagHandler, AprilTagCallback) {
  // Load parameters
  system("rosparam load $(rospack find "
             "factor_handlers)/config/april_parameters.yaml");
  LoadParameters();

  // Callback
  AprilTagCallback(msg);

  // Check the results 
  AprilTagData* new_data = dynamic_cast<AprilTagData*>(GetData());

  // Is the data new
  EXPECT_TRUE(new_data->b_has_data);

  // Is the data April tag
  EXPECT_EQ(new_data->type, "april");

  // One message should be present
  EXPECT_EQ(new_data->factors.size(), 1);

  // Check the details of the message
  EXPECT_EQ(new_data->factors[0].key.chr(), 'T');
  EXPECT_EQ(new_data->factors[0].key.index(), 0);
  EXPECT_EQ(new_data->factors[0].stamp, ros::Time(1.0));
  EXPECT_EQ(new_data->factors[0].position, gtsam::Point3(1.0,2.0,3.0));
  
  // Get the covariance and check
  gtsam::SharedGaussian noise = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(new_data->factors[0].covariance);
  EXPECT_NEAR(noise->covariance()(5,5), 0.3, 1e-5);

  // Check if artifactkey2infohash is updated
  std::unordered_map<long unsigned int, ArtifactInfo> Info = GetKeyInfoMap();
  EXPECT_EQ(Info.size(),1);
  EXPECT_EQ(Info[gtsam::Symbol('T',0)].num_updates,1);
  EXPECT_EQ(Info[gtsam::Symbol('T',0)].id,msg.id);
  EXPECT_EQ(Info[gtsam::Symbol('T',0)].msg.parent_id,msg.id);
  EXPECT_EQ(Info[gtsam::Symbol('T',0)].msg.header.stamp,msg.header.stamp);
  EXPECT_EQ(Info[gtsam::Symbol('T',0)].msg.point.point.x,msg.point.point.x);
  EXPECT_EQ(Info[gtsam::Symbol('T',0)].msg.point.point.y,msg.point.point.y);
  EXPECT_EQ(Info[gtsam::Symbol('T',0)].msg.point.point.z,msg.point.point.z);
  EXPECT_EQ(Info[gtsam::Symbol('T',0)].msg.covariance,msg.covariance);

  // Check if the ground truth is updated
  EXPECT_EQ(Info[gtsam::Symbol('T',0)].global_position,gtsam::Point3(0.1,0.1,0.1));

  // Check if artifactid2keyhash is updated
  std::unordered_map<std::string, gtsam::Symbol> idkeymap = GetStringKeyMap();
  EXPECT_EQ(idkeymap[msg.id],gtsam::Symbol('T',0));
}

// Check if April tag message is being converted into artifacts correctly
TEST_F(TestAprilTagHandler, ConvertAprilTagMsgToArtifactMsg) {
  core_msgs::Artifact artifact_msg = ConvertAprilTagMsgToArtifactMsg(msg);
  
  // Check header
  EXPECT_EQ(artifact_msg.header.stamp, msg.header.stamp);
  // Check april tags name
  EXPECT_EQ(artifact_msg.name, msg.name);
  // Check label
  EXPECT_EQ(artifact_msg.label, "");
  // Check sequence
  EXPECT_EQ(artifact_msg.seq, 0);
  // Check id 
  EXPECT_EQ(artifact_msg.id, "");
  // Check parent id
  EXPECT_EQ(artifact_msg.parent_id, msg.id);
  // Check hostspot name
  EXPECT_EQ(artifact_msg.hotspot_name, "");
  // Check pose value
  EXPECT_EQ(artifact_msg.point.point.x, msg.point.point.x);
  EXPECT_EQ(artifact_msg.point.point.y, msg.point.point.y);
  EXPECT_EQ(artifact_msg.point.point.z, msg.point.point.z);

  // Check confidence
  EXPECT_EQ(artifact_msg.confidence, 1.0);
  // Check the covariance
  EXPECT_EQ(artifact_msg.covariance, msg.covariance);
}

// Check if ground truth is being extracted.
TEST_F(TestAprilTagHandler, GetGroundTruthData) {
  // Load parameters
  system("rosparam load $(rospack find "
             "factor_handlers)/config/april_parameters.yaml");
  LoadParameters();

  // Callback
  AprilTagCallback(msg);

  // Check if we can retrieve the ground truth
  EXPECT_EQ(GetGroundTruth(gtsam::Symbol('T',0)).translation(), gtsam::Point3(0.1,0.1,0.1));
}

// Check if parameters are being loaded properly. 
TEST_F(TestAprilTagHandler, LoadParameters) {
  system("rosparam load $(rospack find "
             "factor_handlers)/config/april_parameters.yaml");
  EXPECT_TRUE(LoadParameters());
}

// Check if initialization is done properly.
TEST_F(TestAprilTagHandler, Initialize) {
  system("rosparam load $(rospack find "
             "factor_handlers)/config/april_parameters.yaml");
  EXPECT_TRUE(Initialize());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_april_tag_handler");
  return RUN_ALL_TESTS();
}