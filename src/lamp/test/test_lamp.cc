/**
 *  @brief Testing the Lamp Robot class
 *
 */

#include <gtest/gtest.h>

#include "lamp/LampBaseStation.h"
#include "lamp/LampRobot.h"

class TestLampRobot : public ::testing::Test {
public:
  PointCloud::Ptr data;

  TestLampRobot() : data(new PointCloud(2, 2)) {
    // Load params
    system("rosparam load $(rospack find "
           "lamp)/config/precision_parameters.yaml");
    system("rosparam load $(rospack find lamp)/config/lamp_frames.yaml");
    system("rosparam load $(rospack find lamp)/config/lamp_rates.yaml");
    system("rosparam load $(rospack find lamp)/config/lamp_init_noise.yaml");
    system("rosparam load $(rospack find lamp)/config/lamp_settings.yaml");

    system("rosparam set artifact_prefix 'l'");

    system("rosparam load $(rospack find "
           "point_cloud_filter)/config/parameters.yaml");
    system("rosparam load $(rospack find "
           "point_cloud_mapper)/config/parameters.yaml");
    system("rosparam load $(rospack find "
           "factor_handlers)/config/odom_parameters.yaml");
    system("rosparam load $(rospack find "
           "factor_handlers)/config/april_parameters.yaml");
    system("rosparam load $(rospack find "
           "factor_handlers)/config/imu_parameters.yaml");

    // Create data in the point cloud
    int n_points = 2;
    for (int i = 0; i < n_points; i++) {
      for (int j = 0; j < n_points; j++) {
        data->at(j, i).x = (float)j / (n_points - 1);
        data->at(j, i).y = (float)i / (n_points - 1);
        data->at(j, i).z = 0.0f;
      }
    }
  }
  ~TestLampRobot() {}

  LampRobot lr;

  // Pass-through functions
  bool SetInitialKey() {
    return lr.SetInitialKey();
  }
  bool SetFactorPrecisions() {
    return lr.SetFactorPrecisions();
  }
  bool SetInitialPosition() {
    return lr.SetInitialPosition();
  }
  int GetValuesSize() {
    return lr.graph().GetValues().size();
  }
  gtsam::Symbol GetKeyAtTime(const ros::Time& stamp) {
    return lr.graph().GetKeyAtTime(stamp);
  }
  gtsam::Symbol GetClosestKeyAtTime(const ros::Time& stamp) {
    return lr.graph().GetClosestKeyAtTime(stamp);
  }
  gtsam::SharedNoiseModel SetFixedNoiseModels(std::string type) {
    return lr.SetFixedNoiseModels(type);
  }
  void TrackFactor(gtsam::Symbol key_from,
                   gtsam::Symbol key_to,
                   int type,
                   gtsam::Pose3 pose,
                   gtsam::SharedNoiseModel covariance) {
    std::cout << "Tracking factor between " << key_from << " and " << key_to
              << std::endl;
    lr.graph().TrackFactor(key_from, key_to, type, pose, covariance);
    std::cout << "Tracked factor between " << key_from << " and " << key_to
              << std::endl;
  }
  void TrackPrior(gtsam::Symbol key,
                  gtsam::Pose3 pose,
                  gtsam::SharedNoiseModel covariance) {
    lr.graph().TrackPrior(key, pose, covariance);
  }
  void TrackNode(const ros::Time& stamp,
                 gtsam::Symbol key,
                 gtsam::Pose3 pose,
                 gtsam::SharedNoiseModel covariance) {
    lr.graph().TrackNode(stamp, key, pose, covariance);
  }

  // Access functions
  void AddStampToOdomKey(ros::Time stamp, gtsam::Symbol key) {
    lr.graph().stamp_to_odom_key[stamp.toSec()] = key;
  }
  void AddKeyedStamp(gtsam::Symbol key, ros::Time stamp) {
    lr.graph().keyed_stamps[key] = stamp;
  }
  void SetTimeThreshold(double threshold) {
    lr.graph().time_threshold = threshold;
  }
  void SetPrefix(char c) {
    lr.graph().prefix = c;
  }
  void InsertValues(gtsam::Symbol key, gtsam::Pose3 pose) {
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    lr.graph().TrackNode(ros::Time::now(), key, pose, noise);
  }

  void LaserLoopClosureCallback(const pose_graph_msgs::PoseGraphConstPtr msg) {
    lr.LaserLoopClosureCallback(msg);
  }
  bool ReGenerateMapPointCloud() {
    lr.ReGenerateMapPointCloud();
  }
  bool AddTransformedPointCloudToMap(const gtsam::Symbol key) {
    lr.AddTransformedPointCloudToMap(key);
  }

  bool GetOptFlag() {
    return lr.b_run_optimization_;
  }

  void setArtifactInGlobal(bool value) {
    lr.b_artifacts_in_global_ = value;
  }

  void LidarCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    lr.odometry_handler_.LidarOdometryCallback(msg);
  }

  void setFixedCovariance(bool value) {
    lr.b_use_fixed_covariances_ = value;
  }

  void ProcessArtifacts(std::shared_ptr<FactorData> data) {
    lr.ProcessArtifactData(data);
  }

  void ProcessAprilTags(std::shared_ptr<FactorData> data) {
    lr.ProcessAprilTagData(data);
  }

  std::unordered_map<long unsigned int, ArtifactInfo>& GetInfoHash() {
    return lr.april_tag_handler_.GetArtifactKey2InfoHash();
  }

  std::unordered_map<long unsigned int, ArtifactInfo>& GetArtifactInfoHash() {
    return lr.artifact_handler_.GetArtifactKey2InfoHash();
  }

  std::unordered_map<std::string, gtsam::Symbol>& GetIdKeyHash() {
    return lr.artifact_handler_.artifact_id2key_hash;
  }

  std::vector<gtsam::Symbol>& GetNewKeys() {
    return lr.artifact_handler_.new_keys_;
  }

  int GetLargestArtifactId() {
    return lr.artifact_handler_.largest_artifact_id_;
  }

  std::unordered_map<std::string, gtsam::Symbol>& GetAprilIdKeyHash() {
    return lr.april_tag_handler_.artifact_id2key_hash;
  }

  std::vector<gtsam::Symbol>& GetAprilNewKeys() {
    return lr.april_tag_handler_.new_keys_;
  }

  int GetAprilLargestArtifactId() {
    return lr.april_tag_handler_.largest_artifact_id_;
  }

  void ConvertGlobalToRelative(const ros::Time stamp,
                               const gtsam::Pose3 pose_global,
                               gtsam::Pose3& pose_relative) {
    lr.ConvertGlobalToRelative(stamp, pose_global, pose_relative);
  }

  // Other utilities

  const gtsam::Values& GetValues() const {
    return lr.graph().GetValues();
  }

  void AddToKeyScans(gtsam::Symbol key, PointCloud::ConstPtr scan) {
    lr.graph().keyed_scans.insert(
        std::pair<gtsam::Symbol, PointCloud::ConstPtr>(key, scan));
  }

  const gtsam::NonlinearFactorGraph& GetNfg() const {
    return lr.graph().GetNfg();
  }
  const EdgeSet& GetEdges() const {
    return lr.graph().GetEdges();
  }
  const NodeSet& GetNodes() const {
    return lr.graph().GetNodes();
  }
  const EdgeSet& GetPriors() const {
    return lr.graph().GetPriors();
  }
  const EdgeSet& GetNewEdges() const {
    return lr.graph().GetNewEdges();
  }
  const NodeSet& GetNewNodes() const {
    return lr.graph().GetNewNodes();
  }
  const EdgeSet& GetNewPriors() const {
    return lr.graph().GetNewPriors();
  }

  gtsam::Pose3 GetPose(gtsam::Key key) const {
    return lr.graph().GetPose(key);
  }
  const EdgeMessage* FindEdge(gtsam::Key key_from, gtsam::Key key_to) const {
    return lr.graph().FindEdge(key_from, key_to);
  }

  PointCloud::Ptr GetMapPC() {
    return lr.mapper_.GetMapData();
  }
  // Other utilities

protected:
  // Tolerance on EXPECT_NEAR assertions
  double tolerance_ = 1e-5;

private:
};

TEST_F(TestLampRobot, TestSetInitialPositionNoParam) {
  // del params
  ros::param::del("fiducial_calibration/position/y");
  ros::param::del("fiducial_calibration/position/x");
  ros::param::del("fiducial_calibration/position/z");
  ros::param::del("fiducial_calibration/orientation/x");
  ros::param::del("fiducial_calibration/orientation/y");
  ros::param::del("fiducial_calibration/orientation/z");
  ros::param::del("fiducial_calibration/orientation/w");
  ros::param::del("init/position_sigma/x");
  ros::param::del("init/position_sigma/y");
  ros::param::del("init/position_sigma/z");
  ros::param::del("init/orientation_sigma/roll");
  ros::param::del("init/orientation_sigma/pitch");
  ros::param::del("init/orientation_sigma/yaw");

  EXPECT_FALSE(SetInitialPosition());

  EXPECT_EQ(GetValuesSize(), 0);
}

TEST_F(TestLampRobot, TestSetInitialPosition) {
  ros::Time::init();

  // Set params
  ros::param::set("fiducial_calibration/position/x", 1.0);
  ros::param::set("fiducial_calibration/position/y", 1.0);
  ros::param::set("fiducial_calibration/position/z", 1.0);
  ros::param::set("fiducial_calibration/orientation/x", 0.0);
  ros::param::set("fiducial_calibration/orientation/y", 0.0);
  ros::param::set("fiducial_calibration/orientation/z", 0.0);
  ros::param::set("fiducial_calibration/orientation/w", 1.0);

  ros::param::set("init/position_sigma/x", 1.0);
  ros::param::set("init/position_sigma/y", 1.0);
  ros::param::set("init/position_sigma/z", 1.0);
  ros::param::set("init/orientation_sigma/roll", 1.0);
  ros::param::set("init/orientation_sigma/pitch", 1.0);
  ros::param::set("init/orientation_sigma/yaw", 1.0);

  EXPECT_TRUE(SetInitialPosition());
  EXPECT_EQ(GetValuesSize(), 1);
}

/**
 * Call ProcessArtifacts with this new artifact. New artifact added
 *                                                  |-----------------------------------------------------|
 *Graph loop closure a0       a1                          l1         a2 l1 a3 l1
 *V                                    Node symbols Odom     Odom Artifact Odom
 *Artifact    Odom              Artifact position                        Type of
 *measurement g(0,0,0) g(2,0,0)    g(2.4,0,0)   r(9.7,0,0)    g(4,0,0)
 *g(4.4,0,0) r(7.8,0,0)  g(6,0,0)          g(12.1,0,0) global(g)/relative(r)
 *position
 *o--------->o------------|------------------------->o------------------------------>o---------------------------------
 *Graph odom
 * -|--------|------------|------------|------------|-------------|---------|--------|--------------------|--------------
 *1D line 0.05     0.1         0.109        0.11          0.15         0.159
 *0.16     2.0                                                        Time
 */
// CAUSING TESTING ISSUES TODO

/*
TEST_F(TestLampRobot, TestProcessArtifactData) {
  // Construct the new Artifact data
  std::shared_ptr<ArtifactData> new_data = std::make_shared<ArtifactData>();
  new_data->b_has_data = true;
  new_data->type = "artifact";

  ArtifactFactor new_factor;
  new_factor.key = gtsam::Symbol('l', 1);
  new_factor.position = gtsam::Point3(9.7, 0, 0);
  gtsam::Vector6 sig;
  sig << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(sig);
  new_factor.covariance = noise;
  new_factor.stamp = ros::Time(0.11);

  // Add the new factor
  new_data->factors.push_back(new_factor);

  // Set the global flag
  setArtifactInGlobal(false);
  setFixedCovariance(false);

  // Add to values
  AddStampToOdomKey(ros::Time(0.05), gtsam::Symbol('a', 0));
  InsertValues(gtsam::Symbol('a', 0),
               gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)));
  AddStampToOdomKey(ros::Time(0.1), gtsam::Symbol('a', 1));
  InsertValues(gtsam::Symbol('a', 1),
               gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0, 0)));
  AddStampToOdomKey(ros::Time(0.15), gtsam::Symbol('a', 2));
  InsertValues(gtsam::Symbol('a', 2),
               gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 0, 0)));
  AddStampToOdomKey(ros::Time(0.2), gtsam::Symbol('a', 3));
  InsertValues(gtsam::Symbol('a', 3),
               gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(6.0, 0, 0)));

  AddKeyedStamp(gtsam::Symbol('a', 0), ros::Time(0.05));
  AddKeyedStamp(gtsam::Symbol('a', 1), ros::Time(0.1));
  AddKeyedStamp(gtsam::Symbol('a', 2), ros::Time(0.15));
  AddKeyedStamp(gtsam::Symbol('a', 3), ros::Time(0.2));

  // Construct the odometry message for a0 (the nearest key)
  nav_msgs::Odometry a0_value;
  a0_value.header.stamp = ros::Time(0.05);
  geometry_msgs::PoseWithCovariance msg_pose;
  a0_value.pose = msg_pose;
  nav_msgs::Odometry::ConstPtr a0_odom(new nav_msgs::Odometry(a0_value));

  // Call Lidar callback
  LidarCallback(a0_odom);

  // New message at 0.1
  nav_msgs::Odometry l1_value;
  l1_value.header.stamp = ros::Time(0.1);
  l1_value.pose.pose.position.x = 2.0;
  l1_value.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry::ConstPtr a1_odom(new nav_msgs::Odometry(l1_value));

  // Call Lidar callback
  LidarCallback(a1_odom);

  // New message at 0.109 (not gets added to pose graph)
  l1_value.header.stamp = ros::Time(0.109);
  l1_value.pose.pose.position.x = 2.4;
  l1_value.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry::ConstPtr a1l1_odom(new nav_msgs::Odometry(l1_value));

  // Call Lidar callback
  LidarCallback(a1l1_odom);

  // New message at 0.15
  l1_value.header.stamp = ros::Time(0.15);
  l1_value.pose.pose.position.x = 4.0;
  l1_value.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry::ConstPtr a2_odom(new nav_msgs::Odometry(l1_value));

  // Call Lidar callback
  LidarCallback(a2_odom);

  // New message at 0.2
  l1_value.header.stamp = ros::Time(0.2);
  l1_value.pose.pose.position.x = 6.0;
  l1_value.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry::ConstPtr a3_odom(new nav_msgs::Odometry(l1_value));

  // Call Lidar callback
  LidarCallback(a3_odom);

  // Construct Artifact Info
  ArtifactInfo temp_info;
  temp_info.id = "artifact";
  temp_info.num_updates = 1;
  temp_info.global_position = gtsam::Point3(9.7, 0, 0);

  // Add the factor in InfoHash
  std::unordered_map<long unsigned int, ArtifactInfo>& info_map =
      GetArtifactInfoHash();
  info_map[new_factor.key] = temp_info;

  // Add to idkey hash
  std::unordered_map<std::string, gtsam::Symbol>& key_hash = GetIdKeyHash();
  key_hash["artifact"] = new_factor.key;

  // Check if the artifact is in the Info Hash
  info_map = GetArtifactInfoHash();
  EXPECT_EQ(info_map.size(), 1);

  // Check if the artifact is in the Key Hash
  key_hash = GetIdKeyHash();
  EXPECT_EQ(key_hash.size(), 1);

  // Add to new keys
  std::vector<gtsam::Symbol>& temp_keys = GetNewKeys();
  temp_keys.push_back(new_factor.key);

  // Check if the new_keys is inserted
  temp_keys = GetNewKeys();
  EXPECT_EQ(temp_keys.size(), 1);

  // Call the ProcessArtifactData. Adding a new artifact
  ProcessArtifacts(new_data);

  // Check if the artifact is still in the Info Hash
  info_map = GetArtifactInfoHash();
  EXPECT_EQ(info_map.size(), 1);

  // Check if the artifact is still in the Key Hash
  key_hash = GetIdKeyHash();
  EXPECT_EQ(key_hash.size(), 1);

  // Check if the new_keys is cleared
  temp_keys = GetNewKeys();
  EXPECT_EQ(temp_keys.size(), 0);

  // As this is a new artifact optimization should be false
  EXPECT_FALSE(GetOptFlag());
  // Check if l1 is added to values
  EXPECT_TRUE(GetValues().exists(gtsam::Symbol('l', 1)));
  // Check the position of the artifact
  EXPECT_EQ(GetPose(gtsam::Symbol('l', 1)).translation(),
            gtsam::Point3(12.1, 0, 0));

  // New message at 0.159 (not gets added to pose graph)
  l1_value.header.stamp = ros::Time(0.159);
  l1_value.pose.pose.position.x = 4.4;
  l1_value.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry::ConstPtr a2l1_odom(new nav_msgs::Odometry(l1_value));

  // Call Lidar callback
  LidarCallback(a2l1_odom);

  // Change time and send the message again
  new_data->b_has_data = true;
  new_data->factors[0].stamp = ros::Time(0.16);
  new_data->factors[0].position = gtsam::Point3(7.8, 0, 0);

  // Call the ProcessArtifactData. Adding an old artifact
  ProcessArtifacts(new_data);

  // As this is a new artifact optimization should be false
  EXPECT_TRUE(GetOptFlag());
  // Check if l1 is added to values
  EXPECT_TRUE(GetValues().exists(gtsam::Symbol('l', 1)));
  // Check the position of the artifact
  EXPECT_EQ(GetPose(gtsam::Symbol('l', 1)).translation(),
            gtsam::Point3(12.1, 0, 0));
  // Check the loop closure factor
  gtsam::NonlinearFactorGraph graph = GetNfg();
  std::vector<gtsam::Symbol> other_keys;

  for (auto factor : graph) {
    if (factor->find(gtsam::Symbol('l', 1)) != factor->end()) {
      if (factor->keys()[0] == gtsam::Symbol('l', 1)) {
        other_keys.push_back(factor->keys()[1]);
      } else {
        other_keys.push_back(factor->keys()[0]);
      }
    }
  }
  // Check if a1 is present in a factor with l1
  EXPECT_TRUE(find(other_keys.begin(),
                   other_keys.end(),
                   gtsam::Symbol('a', 1)) != other_keys.end());

  // Check if a2 is present in a factor with l1
  EXPECT_TRUE(find(other_keys.begin(),
                   other_keys.end(),
                   gtsam::Symbol('a', 2)) != other_keys.end());

  // Add a new failed factor
  new_data->factors.clear();
  new_data->b_has_data = true;
  new_data->type = "survivor";
  new_factor.key = gtsam::Symbol('l', 2);
  new_factor.position = gtsam::Point3(0.9, 0, 0);
  new_factor.covariance = noise;
  // Time greater than threshold
  new_factor.stamp = ros::Time(20.15);

  // Add the new factor
  new_data->factors.push_back(new_factor);

  // Construct Artifact Info
  temp_info.id = "survivor";
  temp_info.num_updates = 1;
  temp_info.global_position = gtsam::Point3(0.9, 0, 0);

  // Add the factor in InfoHash
  info_map = GetArtifactInfoHash();
  info_map[new_factor.key] = temp_info;

  // Add to idkey hash
  key_hash = GetIdKeyHash();
  key_hash["survivor"] = new_factor.key;

  // Add to new keys
  temp_keys = GetNewKeys();
  temp_keys.push_back(new_factor.key);

  // Call Process Artifacts should fail
  ProcessArtifacts(new_data);

  // Check if the artifact is still in the Info Hash
  info_map = GetArtifactInfoHash();
  EXPECT_EQ(info_map.size(), 1);

  // Check if the artifact is still in the Key Hash
  key_hash = GetIdKeyHash();
  EXPECT_EQ(key_hash.size(), 1);

  // Check if the new_keys is cleared
  temp_keys = GetNewKeys();
  EXPECT_EQ(temp_keys.size(), 0);

  // Check the largest artifact id
  int largest_id = GetLargestArtifactId();
  EXPECT_EQ(largest_id, 2);
}

*/

/** Check Process April tag data.
 * Same as Artifacts unit test. The two main purpose here is
 *      Optimization is done for new april as well
 *      Prior is added when the april is new
 *  Call ProcessAprilTags with April tags. New April tag added.
 * Second message for same april arrives and only between factor added
 *                                                  |-----------------------------------------------------|
 *Graph loop closure a0       a1                          l1         a2 l1 a3 l1
 *V                                                       Node symbols Odom Odom
 *April         Odom                 April       Odom              April
 *measured position    April ground position            Type of measurement
 * g(0,0,0) g(2,0,0)    g(2.4,0,0)   r(9.7,0,0)    g(4,0,0)  g(4.4,0,0)
 *r(7.8,0,0)  g(6,0,0)          g(12.1,0,0)                g(12.2,0,0)
 *global(g)/relative(r) position
 *o--------->o------------|------------------------->o------------------------------>o-----------------------------------------------------
 *Graph odom
 * -|--------|------------|------------|------------|-------------|---------|--------|--------------------|---------------------------|----
 *1D line 0.05     0.1         0.109        0.11          0.15         0.159
 *0.16     2.0 Time QUESTION: Check if global flag needs to be turned on in case
 *of april.
 * TODO: Reflect the new non-sequential unit test in the above graph for both
 *Artifact and april tag handler
 * TODO: I dont think that the position in these new non sequential factor
 *matters. Check it once.
 */

/*
TEST_F(TestLampRobot, TestProcessAprilTagData) {
  // Construct the new April tag data
  std::shared_ptr<AprilTagData> new_data = std::make_shared<AprilTagData>();
  new_data->b_has_data = true;
  new_data->type = "april";

  AprilTagFactor new_factor;
  new_factor.key = gtsam::Symbol('l', 1);
  new_factor.position = gtsam::Point3(9.7, 0, 0);
  gtsam::Vector6 sig;
  sig << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(sig);
  new_factor.covariance = noise;
  new_factor.stamp = ros::Time(0.11);

  // Add the new factor
  new_data->factors.push_back(new_factor);

  // Set the global flag
  setArtifactInGlobal(false);
  setFixedCovariance(false);

  // Add ground truth value for April Tag l1
  std::unordered_map<long unsigned int, ArtifactInfo>& info_hash =
      GetInfoHash();
  info_hash[gtsam::Symbol('l', 1)].id = "distal";
  info_hash[gtsam::Symbol('l', 1)].num_updates = 1;
  info_hash[gtsam::Symbol('l', 1)].global_position =
      gtsam::Point3(12.2, 0.0, 0.0);

  // Add to values
  AddStampToOdomKey(ros::Time(0.05), gtsam::Symbol('a', 0));
  InsertValues(gtsam::Symbol('a', 0),
               gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)));
  AddStampToOdomKey(ros::Time(0.1), gtsam::Symbol('a', 1));
  InsertValues(gtsam::Symbol('a', 1),
               gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0, 0)));
  AddStampToOdomKey(ros::Time(0.15), gtsam::Symbol('a', 2));
  InsertValues(gtsam::Symbol('a', 2),
               gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 0, 0)));
  AddStampToOdomKey(ros::Time(0.2), gtsam::Symbol('a', 3));
  InsertValues(gtsam::Symbol('a', 3),
               gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(6.0, 0, 0)));

  AddKeyedStamp(gtsam::Symbol('a', 0), ros::Time(0.05));
  AddKeyedStamp(gtsam::Symbol('a', 1), ros::Time(0.1));
  AddKeyedStamp(gtsam::Symbol('a', 2), ros::Time(0.15));
  AddKeyedStamp(gtsam::Symbol('a', 3), ros::Time(0.2));

  // Construct the odometry message for a0 (the nearest key)
  nav_msgs::Odometry a0_value;
  a0_value.header.stamp = ros::Time(0.05);
  geometry_msgs::PoseWithCovariance msg_pose;
  a0_value.pose = msg_pose;
  nav_msgs::Odometry::ConstPtr a0_odom(new nav_msgs::Odometry(a0_value));

  // Call Lidar callback
  LidarCallback(a0_odom);

  // New message at 0.1
  nav_msgs::Odometry l1_value;
  l1_value.header.stamp = ros::Time(0.1);
  l1_value.pose.pose.position.x = 2.0;
  l1_value.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry::ConstPtr a1_odom(new nav_msgs::Odometry(l1_value));

  // Call Lidar callback
  LidarCallback(a1_odom);

  // New message at 0.109 (not gets added to pose graph)
  l1_value.header.stamp = ros::Time(0.109);
  l1_value.pose.pose.position.x = 2.4;
  l1_value.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry::ConstPtr a1l1_odom(new nav_msgs::Odometry(l1_value));

  // Call Lidar callback
  LidarCallback(a1l1_odom);

  // New message at 0.15
  l1_value.header.stamp = ros::Time(0.15);
  l1_value.pose.pose.position.x = 4.0;
  l1_value.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry::ConstPtr a2_odom(new nav_msgs::Odometry(l1_value));

  // Call Lidar callback
  LidarCallback(a2_odom);

  // New message at 0.2
  l1_value.header.stamp = ros::Time(0.2);
  l1_value.pose.pose.position.x = 6.0;
  l1_value.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry::ConstPtr a3_odom(new nav_msgs::Odometry(l1_value));

  // Call Lidar callback
  LidarCallback(a3_odom);

  // Check if l1 is added to values
  EXPECT_FALSE(GetValues().exists(gtsam::Symbol('l', 1)));

  // Add to idkey hash
  std::unordered_map<std::string, gtsam::Symbol>& key_hash =
      GetAprilIdKeyHash();
  key_hash["distal"] = new_factor.key;

  // Check if the artifact is in the Info Hash
  info_hash = GetInfoHash();
  EXPECT_EQ(info_hash.size(), 1);

  // Check if the artifact is in the Key Hash
  key_hash = GetAprilIdKeyHash();
  EXPECT_EQ(key_hash.size(), 1);

  // Add to new keys
  std::vector<gtsam::Symbol>& temp_keys = GetAprilNewKeys();
  temp_keys.push_back(new_factor.key);

  // Check if the new_keys is inserted
  temp_keys = GetAprilNewKeys();
  EXPECT_EQ(temp_keys.size(), 1);

  // Call the ProcessAprilTagData. Adding a new April Tag
  ProcessAprilTags(new_data);

  // April tags always lead to optimization
  EXPECT_TRUE(GetOptFlag());
  // Check if l1 is added to values
  EXPECT_TRUE(GetValues().exists(gtsam::Symbol('l', 1)));
  // Check the position of the April Tag
  EXPECT_EQ(GetValues().at<gtsam::Pose3>(gtsam::Symbol('l', 1)).translation(),
            gtsam::Point3(12.1, 0, 0));
  // Check the graph for prior
  gtsam::NonlinearFactorGraph graph = GetNfg();

  // Number of factors for l1 (One should be prior and one between factor)
  int count = 0;
  for (auto factor : graph) {
    if (factor->find(gtsam::Symbol('l', 1)) != factor->end()) {
      count = count + 1;
    }
  }
  EXPECT_EQ(count, 2);

  // New message at 0.159 (not gets added to pose graph)
  l1_value.header.stamp = ros::Time(0.159);
  l1_value.pose.pose.position.x = 4.4;
  l1_value.pose.pose.orientation.w = 1.0;

  nav_msgs::Odometry::ConstPtr a2l1_odom(new nav_msgs::Odometry(l1_value));

  // Call Lidar callback
  LidarCallback(a2l1_odom);

  // Change time and send the message again
  new_data->b_has_data = true;
  new_data->factors[0].stamp = ros::Time(0.16);
  new_data->factors[0].position = gtsam::Point3(7.8, 0, 0);

  // Call the ProcessAprilTagData. Adding an old April Tag
  ProcessAprilTags(new_data);

  // April tags always optimize
  EXPECT_TRUE(GetOptFlag());
  // Check if l1 is added to values
  EXPECT_TRUE(GetValues().exists(gtsam::Symbol('l', 1)));
  // Check the position of the April Tag
  EXPECT_EQ(GetPose(gtsam::Symbol('l', 1)).translation(),
            gtsam::Point3(12.1, 0, 0));
  // Check the loop closure factor
  graph = GetNfg();
  std::vector<gtsam::Symbol> other_keys;

  for (auto factor : graph) {
    if (factor->find(gtsam::Symbol('l', 1)) != factor->end()) {
      if (factor->keys()[0] == gtsam::Symbol('l', 1)) {
        other_keys.push_back(factor->keys()[1]);
      } else {
        other_keys.push_back(factor->keys()[0]);
      }
    }
  }
  // Check if a1 is present in a factor with l1
  EXPECT_TRUE(find(other_keys.begin(),
                   other_keys.end(),
                   gtsam::Symbol('a', 1)) != other_keys.end());

  // Check if a2 is present in a factor with l1
  EXPECT_TRUE(find(other_keys.begin(),
                   other_keys.end(),
                   gtsam::Symbol('a', 2)) != other_keys.end());

  // Number of factors for l1 (One should be prior and two between factors)
  count = 0;
  for (auto factor : graph) {
    if (factor->find(gtsam::Symbol('l', 1)) != factor->end()) {
      count = count + 1;
    }
  }
  EXPECT_EQ(count, 3);

  // Add a new failed factor
  new_data->factors.clear();
  new_data->b_has_data = true;
  new_data->type = "april";
  new_factor.key = gtsam::Symbol('l', 2);
  new_factor.position = gtsam::Point3(0.9, 0, 0);
  new_factor.covariance = noise;
  // Time greater than threshold
  new_factor.stamp = ros::Time(20.15);

  // Add the new factor
  new_data->factors.push_back(new_factor);

  // Construct Artifact Info
  ArtifactInfo temp_info;
  temp_info.id = "calib";
  temp_info.num_updates = 1;
  temp_info.global_position = gtsam::Point3(0.9, 0, 0);

  // Add the factor in InfoHash
  info_hash = GetInfoHash();
  info_hash[new_factor.key] = temp_info;

  // Add to idkey hash
  key_hash = GetAprilIdKeyHash();
  key_hash["calib"] = new_factor.key;

  // Add to new keys
  temp_keys = GetAprilNewKeys();
  temp_keys.push_back(new_factor.key);

  // Call Process Artifacts should fail
  ProcessAprilTags(new_data);

  // Check if the artifact is still in the Info Hash
  info_hash = GetInfoHash();
  EXPECT_EQ(info_hash.size(), 1);

  // Check if the artifact is still in the Key Hash
  key_hash = GetAprilIdKeyHash();
  EXPECT_EQ(key_hash.size(), 1);

  // Check if the new_keys is cleared
  temp_keys = GetAprilNewKeys();
  EXPECT_EQ(temp_keys.size(), 0);

  // Check the largest artifact id
  int largest_id = GetAprilLargestArtifactId();
  EXPECT_EQ(largest_id, 2);
}

*/

TEST_F(TestLampRobot, NonSequentialKeys) {
  // Construct the new Artifact data
  std::shared_ptr<ArtifactData> new_data = std::make_shared<ArtifactData>();
  new_data->b_has_data = true;
  new_data->type = "artifact";

  ArtifactFactor new_factor;
  new_factor.key = gtsam::Symbol('l', 1);
  new_factor.position = gtsam::Point3(9.7, 0, 0);
  gtsam::Vector6 sig;
  sig << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(sig);
  new_factor.covariance = noise;
  new_factor.stamp = ros::Time(0.11);

  // Add the new factor
  new_data->factors.push_back(new_factor);

  // Construct Artifact Info
  ArtifactInfo temp_info;
  temp_info.id = "artifact";
  temp_info.num_updates = 1;
  temp_info.global_position = gtsam::Point3(9.7, 0, 0);

  // Add the factor in InfoHash
  std::unordered_map<long unsigned int, ArtifactInfo>& info_map =
      GetArtifactInfoHash();
  info_map[new_factor.key] = temp_info;

  // Add to idkey hash
  std::unordered_map<std::string, gtsam::Symbol>& key_hash = GetIdKeyHash();
  key_hash["artifact"] = new_factor.key;

  // Add to new keys
  std::vector<gtsam::Symbol>& temp_keys = GetNewKeys();
  temp_keys.push_back(new_factor.key);

  // Check if the artifact is still in the Info Hash
  info_map = GetArtifactInfoHash();
  EXPECT_EQ(info_map.size(), 1);

  // Check if the artifact is still in the Key Hash
  key_hash = GetIdKeyHash();
  EXPECT_EQ(key_hash.size(), 1);

  // Check if the new_keys is cleared
  temp_keys = GetNewKeys();
  EXPECT_EQ(temp_keys.size(), 1);

  // Set the global flag
  setArtifactInGlobal(true);
  setFixedCovariance(false);

  // Call the ProcessArtifactData. Adding a new artifact
  ProcessArtifacts(new_data);

  // Check if the artifact is still in the Info Hash
  info_map = GetArtifactInfoHash();
  EXPECT_EQ(info_map.size(), 0);

  // Check if the artifact is still in the Key Hash
  key_hash = GetIdKeyHash();
  EXPECT_EQ(key_hash.size(), 0);

  // Check if the new_keys is cleared
  temp_keys = GetNewKeys();
  EXPECT_EQ(temp_keys.size(), 0);

  // Turn off global flag
  setArtifactInGlobal(false);

  // Add the factor in InfoHash
  info_map = GetArtifactInfoHash();
  info_map[new_factor.key] = temp_info;

  // Add to idkey hash
  key_hash = GetIdKeyHash();
  key_hash["artifact"] = new_factor.key;

  // Add to new keys
  temp_keys = GetNewKeys();
  temp_keys.push_back(new_factor.key);

  // Check if the artifact is still in the Info Hash
  info_map = GetArtifactInfoHash();
  EXPECT_EQ(info_map.size(), 1);

  // Check if the artifact is still in the Key Hash
  key_hash = GetIdKeyHash();
  EXPECT_EQ(key_hash.size(), 1);

  // Check if the new_keys is cleared
  temp_keys = GetNewKeys();
  EXPECT_EQ(temp_keys.size(), 1);

  // Call the ProcessArtifactData. Adding a new artifact
  ProcessArtifacts(new_data);

  // Check if the artifact is still in the Info Hash
  info_map = GetArtifactInfoHash();
  EXPECT_EQ(info_map.size(), 0);

  // Check if the artifact is still in the Key Hash
  key_hash = GetIdKeyHash();
  EXPECT_EQ(key_hash.size(), 0);

  // Check if the new_keys is cleared
  temp_keys = GetNewKeys();
  EXPECT_EQ(temp_keys.size(), 0);
}

TEST_F(TestLampRobot, SetInitialKey) {
  // Set string
  std::string prefix = "a";
  ros::param::set("robot_prefix", prefix);

  // Set key (with Friend Class)
  SetInitialKey();

  // Retrieve result
  gtsam::Symbol key_gtsam = lr.GetInitialKey();
  std::string key_string = std::string(key_gtsam);
  // std::string key_string = gtsam::DefaultKeyFormatter(key_gtsam);

  std::cout << "Initial key is" << key_string << std::endl;

  EXPECT_EQ(std::string("a0"), key_string);
}

TEST_F(TestLampRobot, ConvertGlobalToRelative) {
  // Ros time to search for current key
  const ros::Time stamp = ros::Time(5.0);
  // Global pose of the artifact
  const gtsam::Pose3 pose_global =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(5.0, 0.0, 0.0));
  // Final relative pose between artifact and the current key
  gtsam::Pose3 pose_relative;
  // Add current key to map
  AddStampToOdomKey(ros::Time(4.0), gtsam::Symbol('a', 0));
  // Add value/pose for the current key
  InsertValues(gtsam::Symbol('a', 0),
               gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(420.0, 0.0, 0.0)));
  // Convert global to relative pose
  ConvertGlobalToRelative(stamp, pose_global, pose_relative);
  // Check
  EXPECT_EQ(pose_relative.translation().vector(),
            gtsam::Point3(-415.0, 0.0, 0.0));
}

TEST_F(TestLampRobot, SetFactorPrecisions) {
  // Set all parameter values
  ros::param::set("manual_lc_rot_precision", 1.0);
  ros::param::set("manual_lc_trans_precision", 1.0);
  ros::param::set("laser_lc_rot_sigma", 1.0);
  ros::param::set("laser_lc_trans_sigma", 1.0);
  ros::param::set("artifact_rot_precision", 1.0);
  ros::param::set("artifact_trans_precision", 1.0);
  ros::param::set("point_estimate_precision", 1.0);
  ros::param::set("fiducial_trans_precision", 1.0);
  ros::param::set("fiducial_rot_precision", 1.0);

  EXPECT_TRUE(SetFactorPrecisions());
}

TEST_F(TestLampRobot, Initialization) {
  ros::NodeHandle nh, pnh("~");

  bool result = lr.Initialize(nh);

  EXPECT_TRUE(result);
}

TEST_F(TestLampRobot, GetKeyAtTime) {
  ros::Time::init();

  SetTimeThreshold(0.001);

  // Build map
  AddStampToOdomKey(ros::Time(0.0), gtsam::Symbol('a', 0));
  AddStampToOdomKey(ros::Time(0.5), gtsam::Symbol('a', 1));
  AddStampToOdomKey(ros::Time(1.0), gtsam::Symbol('a', 2));
  AddStampToOdomKey(ros::Time(1.5), gtsam::Symbol('a', 3));
  AddStampToOdomKey(ros::Time(2.0), gtsam::Symbol('a', 4));

  // Exact matches
  EXPECT_EQ(gtsam::Symbol('a', 0), GetKeyAtTime(ros::Time(0.0)));
  EXPECT_EQ(gtsam::Symbol('a', 1), GetKeyAtTime(ros::Time(0.5)));
  EXPECT_EQ(gtsam::Symbol('a', 2), GetKeyAtTime(ros::Time(1.0)));
  EXPECT_EQ(gtsam::Symbol('a', 3), GetKeyAtTime(ros::Time(1.5)));
  EXPECT_EQ(gtsam::Symbol('a', 4), GetKeyAtTime(ros::Time(2.0)));

  // Mismatches
  EXPECT_EQ(utils::GTSAM_ERROR_SYMBOL, GetKeyAtTime(ros::Time(0.0000001)));
  EXPECT_EQ(utils::GTSAM_ERROR_SYMBOL, GetKeyAtTime(ros::Time(0.4990001)));
  EXPECT_EQ(utils::GTSAM_ERROR_SYMBOL, GetKeyAtTime(ros::Time(0.9999999)));
  EXPECT_EQ(utils::GTSAM_ERROR_SYMBOL, GetKeyAtTime(ros::Time(1.5009999)));
  EXPECT_EQ(utils::GTSAM_ERROR_SYMBOL, GetKeyAtTime(ros::Time(1.9999000)));
}

TEST_F(TestLampRobot, GetKeyAtTimeEmpty) {
  ros::Time::init();
  SetTimeThreshold(0.001);

  // Expect error keys from invalid inputs
  EXPECT_EQ(utils::GTSAM_ERROR_SYMBOL, GetKeyAtTime(ros::Time(0.0)));
  EXPECT_EQ(utils::GTSAM_ERROR_SYMBOL, GetKeyAtTime(ros::Time(1.0)));
}

TEST_F(TestLampRobot, GetClosestKeyAtTime) {
  ros::Time::init();

  // Set large threshold
  SetTimeThreshold(1000.0);

  // Check single key
  AddStampToOdomKey(ros::Time(40.0), gtsam::Symbol('a', 0));
  EXPECT_EQ(gtsam::Symbol('a', 0), GetClosestKeyAtTime(ros::Time(500.0)));

  // Add more keys
  AddStampToOdomKey(ros::Time(50.0), gtsam::Symbol('a', 1));
  AddStampToOdomKey(ros::Time(60.0), gtsam::Symbol('a', 2));
  AddStampToOdomKey(ros::Time(80.0), gtsam::Symbol('a', 3));
  AddStampToOdomKey(ros::Time(100.0), gtsam::Symbol('a', 4));

  // Exact matches
  // EXPECT_EQ(gtsam::Symbol('a', 0), GetClosestKeyAtTime(ros::Time(0.0))); //
  // TODO - fix this
  EXPECT_EQ(gtsam::Symbol('a', 1), GetClosestKeyAtTime(ros::Time(50.0)));
  EXPECT_EQ(gtsam::Symbol('a', 2), GetClosestKeyAtTime(ros::Time(60.0)));
  EXPECT_EQ(gtsam::Symbol('a', 3), GetClosestKeyAtTime(ros::Time(80.0)));
  EXPECT_EQ(gtsam::Symbol('a', 4), GetClosestKeyAtTime(ros::Time(100.0)));

  // Closest key
  EXPECT_EQ(gtsam::Symbol('a', 0), GetClosestKeyAtTime(ros::Time(0.5)));
  EXPECT_EQ(gtsam::Symbol('a', 1), GetClosestKeyAtTime(ros::Time(54.0)));
  EXPECT_EQ(gtsam::Symbol('a', 2), GetClosestKeyAtTime(ros::Time(63.43)));
  EXPECT_EQ(gtsam::Symbol('a', 3), GetClosestKeyAtTime(ros::Time(75.0)));
  EXPECT_EQ(gtsam::Symbol('a', 4), GetClosestKeyAtTime(ros::Time(99.9)));
  EXPECT_EQ(gtsam::Symbol('a', 4), GetClosestKeyAtTime(ros::Time(1000.0)));
}

TEST_F(TestLampRobot, GetClosestKeyAtTimeException) {
  ros::Time::init();

  SetTimeThreshold(1.0);

  // Check single key
  AddStampToOdomKey(ros::Time(40.0), gtsam::Symbol('a', 0));
  EXPECT_EQ(utils::GTSAM_ERROR_SYMBOL, GetClosestKeyAtTime(ros::Time(500.0)));
  EXPECT_EQ(utils::GTSAM_ERROR_SYMBOL, GetClosestKeyAtTime(ros::Time(0.0)));

  // Add more keys
  AddStampToOdomKey(ros::Time(50.0), gtsam::Symbol('a', 1));
  AddStampToOdomKey(ros::Time(60.0), gtsam::Symbol('a', 2));
  AddStampToOdomKey(ros::Time(80.0), gtsam::Symbol('a', 3));
  AddStampToOdomKey(ros::Time(100.0), gtsam::Symbol('a', 4));

  // Exact matches
  // EXPECT_EQ(gtsam::Symbol('a', 0), GetClosestKeyAtTime(ros::Time(0.0))); //
  // TODO - fix this
  EXPECT_EQ(utils::GTSAM_ERROR_SYMBOL, GetClosestKeyAtTime(ros::Time(55.0)));
  EXPECT_EQ(gtsam::Symbol('a', 2), GetClosestKeyAtTime(ros::Time(60.5)));
}

TEST_F(TestLampRobot, TestDuplicateFactors) {
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  TrackFactor(0lu, 1lu, 5, gtsam::Pose3(), noise);
  TrackFactor(0lu, 2lu, 5, gtsam::Pose3(), noise);
  TrackFactor(0lu, 1lu, 5, gtsam::Pose3(), noise);
  TrackFactor(1lu, 1lu, 5, gtsam::Pose3(), noise);
  TrackFactor(2lu, 1lu, 5, gtsam::Pose3(), noise);
  TrackFactor(1lu, 1lu, 3, gtsam::Pose3(), noise);

  EXPECT_EQ(GetEdges().size(), 5lu);
}

TEST_F(TestLampRobot, TestDuplicateNodes) {
  ros::Time::init();
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  TrackNode(ros::Time(10.0), gtsam::Symbol('a', 2), gtsam::Pose3(), noise);
  TrackNode(ros::Time(20.0), gtsam::Symbol('b', 2), gtsam::Pose3(), noise);
  TrackNode(ros::Time(30.0), gtsam::Symbol('a', 2), gtsam::Pose3(), noise);
  TrackNode(ros::Time(40.0), gtsam::Symbol('a', 1), gtsam::Pose3(), noise);
  TrackNode(ros::Time(50.0), gtsam::Symbol('b', 2), gtsam::Pose3(), noise);

  EXPECT_EQ(GetNodes().size(), 3lu);
}

TEST_F(TestLampRobot, TestDuplicatePriors) {
  ros::Time::init();
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  TrackPrior(gtsam::Symbol('a', 2), gtsam::Pose3(), noise);
  TrackPrior(gtsam::Symbol('b', 2), gtsam::Pose3(), noise);
  TrackPrior(gtsam::Symbol('a', 2), gtsam::Pose3(), noise);
  TrackPrior(gtsam::Symbol('a', 1), gtsam::Pose3(), noise);
  TrackPrior(gtsam::Symbol('b', 2), gtsam::Pose3(), noise);

  EXPECT_EQ(GetPriors().size(), 3lu);
}

TEST_F(TestLampRobot, ConvertPoseGraphToMsg) {
  ros::Time::init();

  // NOTE: this test does not use a full valid graph

  // Set up the robot prefix for odom nodes
  SetPrefix('a');

  // Initialize some keys and poses that are used for nodes, edges and priors.

  const gtsam::Symbol key0('a', 100);
  const gtsam::Symbol key1('a', 101);
  const gtsam::Symbol key2('m', 0);

  gtsam::Pose3 tf0(gtsam::Rot3(sqrt(0.5), 0, 0, sqrt(0.5)),
                   gtsam::Point3(420.0, 69.0, 0.0));
  gtsam::Pose3 tf1(gtsam::Rot3(sqrt(0.3), sqrt(0.3), sqrt(0.4), 0.0),
                   gtsam::Point3(10.0, -1.0, 1000.0));
  gtsam::Pose3 tf2(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(500.0, 433.5, -2.5));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  // Test values
  InsertValues(key0, tf0);
  InsertValues(key1, tf1);
  InsertValues(key2, tf2);

  // Test edges
  TrackFactor(key0, key1, pose_graph_msgs::PoseGraphEdge::ODOM, tf2, noise);
  TrackFactor(key2, key0, pose_graph_msgs::PoseGraphEdge::ARTIFACT, tf1, noise);

  // Test priors
  AddKeyedStamp(key0, ros::Time(67589467.0));
  TrackPrior(key0, tf0, noise);

  float x, y, z;
  for (const auto& n : GetNodes()) {
    x = n.pose.position.x;
    y = n.pose.position.y;
    z = n.pose.position.z;
    std::cout << "\toriginal node " << n.key << ": (" << x << ", " << y << ", "
              << z << ")\n";
  }
  for (const auto& e : GetEdges()) {
    x = e.pose.position.x;
    y = e.pose.position.y;
    z = e.pose.position.z;
    std::cout << "\toriginal edge from " << e.key_from << " to " << e.key_to
              << ": (" << x << ", " << y << ", " << z << ")\n";
  }
  for (const auto& e : GetPriors()) {
    x = e.pose.position.x;
    y = e.pose.position.y;
    z = e.pose.position.z;
    std::cout << "\toriginal prior " << e.key_from << ": (" << x << ", " << y
              << ", " << z << ")\n";
  }

  std::cout << "Converting graph to message.\n";

  // Convert pose-graph to message
  pose_graph_msgs::PoseGraphConstPtr g = lr.graph().ToMsg();

  for (const auto& n : g->nodes) {
    x = n.pose.position.x;
    y = n.pose.position.y;
    z = n.pose.position.z;
    std::cout << "\tmsg node " << n.key << ": (" << x << ", " << y << ", " << z
              << ")\n";
  }

  for (const auto& e : g->edges) {
    x = e.pose.position.x;
    y = e.pose.position.y;
    z = e.pose.position.z;
    std::cout << "\tmsg edge from " << e.key_from << " to " << e.key_to << ": ("
              << x << ", " << y << ", " << z << ")\n";
  }

  // Compare message sizes
  EXPECT_EQ(3, g->nodes.size());
  // 2 factors plus 1 prior
  EXPECT_EQ(3, g->edges.size());

  // Convert message back to PoseGraph struct
  std::cout << "Converting message to graph.\n";
  PoseGraph graph;
  graph.UpdateFromMsg(g);
  for (const auto& n : graph.GetNodes()) {
    x = n.pose.position.x;
    y = n.pose.position.y;
    z = n.pose.position.z;
    std::cout << "\tconverted node " << n.key << ": (" << x << ", " << y << ", "
              << z << ")\n";
  }
  for (const auto& e : graph.GetEdges()) {
    x = e.pose.position.x;
    y = e.pose.position.y;
    z = e.pose.position.z;
    std::cout << "\tconverted edge from " << e.key_from << " to " << e.key_to
              << ": (" << x << ", " << y << ", " << z << ")\n";
  }
  for (const auto& e : graph.GetPriors()) {
    x = e.pose.position.x;
    y = e.pose.position.y;
    z = e.pose.position.z;
    std::cout << "\tconverted prior " << e.key_from << ": (" << x << ", " << y
              << ", " << z << ")\n";
  }

  // Node a100 - check all information
  gtsam::Pose3 pose0 = graph.GetPose(key0);
  gtsam::Pose3 actual0 = GetPose(key0);
  std::cout << "Pose0:   " << pose0 << std::endl;
  std::cout << "Actual0: " << actual0 << std::endl;
  EXPECT_EQ(pose0.equals(actual0, tolerance_), true);

  // Node a101 - check all information
  gtsam::Pose3 pose1 = graph.GetPose(key1);
  gtsam::Pose3 actual1 = GetPose(key1);
  std::cout << "Pose1:   " << pose1 << std::endl;
  std::cout << "Actual1: " << actual1 << std::endl;
  EXPECT_EQ(pose1.equals(actual1, tolerance_), true);

  // Node m0 - check all information
  gtsam::Pose3 pose2 = graph.GetPose(key2);
  gtsam::Pose3 actual2 = GetPose(key2);
  std::cout << "Pose2:   " << pose2 << std::endl;
  std::cout << "Actual2: " << actual2 << std::endl;
  EXPECT_EQ(pose2.equals(actual2, tolerance_), true);

  // Odom edge
  const auto* edge0 = graph.FindEdge(key0, key1);
  auto edge0_tf = utils::MessageToPose(*edge0);
  auto edge0_noise = utils::MessageToCovariance(*edge0);
  EXPECT_EQ(edge0->type, pose_graph_msgs::PoseGraphEdge::ODOM);
  EXPECT_EQ(edge0_tf.equals(tf2, tolerance_), true);
  EXPECT_EQ(edge0_noise->equals(*noise, tolerance_), true);

  // Artifact edge
  const auto* edge1 = graph.FindEdge(key2, key0);
  auto edge1_tf = utils::MessageToPose(*edge1);
  auto edge1_noise = utils::MessageToCovariance(*edge1);
  EXPECT_EQ(edge1->type, pose_graph_msgs::PoseGraphEdge::ARTIFACT);
  EXPECT_EQ(edge1_tf.equals(tf1, tolerance_), true);
  EXPECT_EQ(edge1_noise->equals(*noise, tolerance_), true);

  // Prior factor
  const auto* prior = graph.FindPrior(key0);
  auto prior_tf = utils::MessageToPose(*prior);
  auto prior_noise = utils::MessageToCovariance(*prior);
  EXPECT_EQ(prior->type, pose_graph_msgs::PoseGraphEdge::PRIOR);
  EXPECT_EQ(prior_tf.equals(tf0, tolerance_), true);
  EXPECT_EQ(prior_noise->equals(*noise, tolerance_), true);
}

TEST_F(TestLampRobot, SaveLoadPoseGraph) {
  ros::Time::init();

  // NOTE: this test does not use a full valid graph

  // Set up the robot prefix for odom nodes
  SetPrefix('a');

  // Initialize some keys and poses that are used for nodes, edges and priors.

  const gtsam::Symbol key0('a', 100);
  const gtsam::Symbol key1('a', 101);
  const gtsam::Symbol key2('m', 0);

  gtsam::Pose3 tf0(gtsam::Rot3(sqrt(0.5), 0, 0, sqrt(0.5)),
                   gtsam::Point3(420.0, 69.0, 0.0));
  gtsam::Pose3 tf1(gtsam::Rot3(sqrt(0.3), sqrt(0.3), sqrt(0.4), 0.0),
                   gtsam::Point3(10.0, -1.0, 1000.0));
  gtsam::Pose3 tf2(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(500.0, 433.5, -2.5));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  // Test values
  InsertValues(key0, tf0);
  InsertValues(key1, tf1);
  InsertValues(key2, tf2);

  // Test edges
  TrackFactor(key0, key1, pose_graph_msgs::PoseGraphEdge::ODOM, tf2, noise);
  TrackFactor(key2, key0, pose_graph_msgs::PoseGraphEdge::ARTIFACT, tf1, noise);

  // Test priors
  AddKeyedStamp(key0, ros::Time(67589467.0));
  TrackPrior(key0, tf0, noise);

  float x, y, z;
  for (const auto& n : GetNodes()) {
    x = n.pose.position.x;
    y = n.pose.position.y;
    z = n.pose.position.z;
    std::cout << "\toriginal node " << n.key << ": (" << x << ", " << y << ", "
              << z << ")\n";
  }
  for (const auto& e : GetEdges()) {
    x = e.pose.position.x;
    y = e.pose.position.y;
    z = e.pose.position.z;
    std::cout << "\toriginal edge from " << e.key_from << " to " << e.key_to
              << ": (" << x << ", " << y << ", " << z << ")\n";
  }
  for (const auto& e : GetPriors()) {
    x = e.pose.position.x;
    y = e.pose.position.y;
    z = e.pose.position.z;
    std::cout << "\toriginal prior " << e.key_from << ": (" << x << ", " << y
              << ", " << z << ")\n";
  }

  std::cout << "Saving graph to file.\n";
  EXPECT_EQ(lr.graph().Save("graph.zip"), true);

  PoseGraph graph;
  std::cout << "Load graph from file.\n";
  EXPECT_EQ(graph.Load("graph.zip"), true);

  for (const auto& n : graph.GetNodes()) {
    x = n.pose.position.x;
    y = n.pose.position.y;
    z = n.pose.position.z;
    std::cout << "\tconverted node " << n.key << ": (" << x << ", " << y << ", "
              << z << ")\n";
  }
  for (const auto& e : graph.GetEdges()) {
    x = e.pose.position.x;
    y = e.pose.position.y;
    z = e.pose.position.z;
    std::cout << "\tconverted edge from " << e.key_from << " to " << e.key_to
              << ": (" << x << ", " << y << ", " << z << ")\n";
  }
  for (const auto& e : graph.GetPriors()) {
    x = e.pose.position.x;
    y = e.pose.position.y;
    z = e.pose.position.z;
    std::cout << "\tconverted prior " << e.key_from << ": (" << x << ", " << y
              << ", " << z << ")\n";
  }

  // Node a100 - check all information
  gtsam::Pose3 pose0 = graph.GetPose(key0);
  gtsam::Pose3 actual0 = GetPose(key0);
  std::cout << "Pose0:   " << pose0 << std::endl;
  std::cout << "Actual0: " << actual0 << std::endl;
  EXPECT_EQ(pose0.equals(actual0, tolerance_), true);

  // Node a101 - check all information
  gtsam::Pose3 pose1 = graph.GetPose(key1);
  gtsam::Pose3 actual1 = GetPose(key1);
  std::cout << "Pose1:   " << pose1 << std::endl;
  std::cout << "Actual1: " << actual1 << std::endl;
  EXPECT_EQ(pose1.equals(actual1, tolerance_), true);

  // Node m0 - check all information
  gtsam::Pose3 pose2 = graph.GetPose(key2);
  gtsam::Pose3 actual2 = GetPose(key2);
  std::cout << "Pose2:   " << pose2 << std::endl;
  std::cout << "Actual2: " << actual2 << std::endl;
  EXPECT_EQ(pose2.equals(actual2, tolerance_), true);

  // Odom edge
  const auto* edge0 = graph.FindEdge(key0, key1);
  auto edge0_tf = utils::MessageToPose(*edge0);
  auto edge0_noise = utils::MessageToCovariance(*edge0);
  EXPECT_EQ(edge0->type, pose_graph_msgs::PoseGraphEdge::ODOM);
  EXPECT_EQ(edge0_tf.equals(tf2, tolerance_), true);
  EXPECT_EQ(edge0_noise->equals(*noise, tolerance_), true);

  // Artifact edge
  const auto* edge1 = graph.FindEdge(key2, key0);
  auto edge1_tf = utils::MessageToPose(*edge1);
  auto edge1_noise = utils::MessageToCovariance(*edge1);
  EXPECT_EQ(edge1->type, pose_graph_msgs::PoseGraphEdge::ARTIFACT);
  EXPECT_EQ(edge1_tf.equals(tf1, tolerance_), true);
  EXPECT_EQ(edge1_noise->equals(*noise, tolerance_), true);

  // Prior factor
  const auto* prior = graph.FindPrior(key0);
  auto prior_tf = utils::MessageToPose(*prior);
  auto prior_noise = utils::MessageToCovariance(*prior);
  EXPECT_EQ(prior->type, pose_graph_msgs::PoseGraphEdge::PRIOR);
  EXPECT_EQ(prior_tf.equals(tf0, tolerance_), true);
  EXPECT_EQ(prior_noise->equals(*noise, tolerance_), true);
}

// TODO - work out how to pass around SharedNoiseModels
TEST_F(TestLampRobot, TestSetFixedCovariancesOdom) {
  double attitude_sigma;
  double position_sigma;

  ros::NodeHandle nh, pnh("~");

  lr.Initialize(nh);

  ros::param::get("attitude_sigma", attitude_sigma);
  ros::param::get("position_sigma", position_sigma);

  // Set the paramters
  gtsam::Vector sigma_out =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(
          SetFixedNoiseModels("odom"))
          ->sigmas();
  // Diagonal noise model does not have a covariance call - just Sigmas
  // Casting to something that it is not and calling functions will work, but
  // will point to garbage

  EXPECT_NEAR(sigma_out[0], attitude_sigma, tolerance_);
  EXPECT_NEAR(sigma_out[3], position_sigma, tolerance_);
}

// TODO - work out how to pass around SharedNoiseModels
TEST_F(TestLampRobot, TestSetFixedPrecisionsArtifacts) {
  double rot_precision;
  double trans_precision;

  ros::NodeHandle nh, pnh("~");

  lr.Initialize(nh);

  ros::param::get("artifact_rot_precision", rot_precision);
  ros::param::get("artifact_trans_precision", trans_precision);

  // Set the paramters
  gtsam::Vector precision_out =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(
          SetFixedNoiseModels("artifact"))
          ->precisions();

  EXPECT_NEAR(precision_out[0], rot_precision, tolerance_);
  EXPECT_NEAR(precision_out[3], trans_precision, tolerance_);
}

TEST_F(TestLampRobot, TestSetFixedCovariancesLoopClosure) {
  double laser_lc_rot_sigma;
  double laser_lc_trans_sigma;

  ros::NodeHandle nh, pnh("~");

  lr.Initialize(nh);

  ros::param::get("laser_lc_rot_sigma", laser_lc_rot_sigma);
  ros::param::get("laser_lc_trans_sigma", laser_lc_trans_sigma);

  // Set the paramters
  gtsam::Vector sigma_out =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(
          SetFixedNoiseModels("laser_loop_closure"))
          ->sigmas();
  // Diagonal noise model does not have a covariance call - just Sigmas
  // Casting to something that it is not and calling functions will work, but
  // will point to garbage

  EXPECT_NEAR(sigma_out[0], laser_lc_rot_sigma, tolerance_);
  EXPECT_NEAR(sigma_out[3], laser_lc_trans_sigma, tolerance_);
}

TEST_F(TestLampRobot, TestSetFixedCovariancesError) {
  // Set the paramters
  EXPECT_ANY_THROW(SetFixedNoiseModels("something_wrong"));
}

TEST_F(TestLampRobot, TestLaserLoopClosure) {
  ros::NodeHandle nh, pnh("~");

  // bool result = lr.Initialize(nh);

  // Create pose graph edge
  pose_graph_msgs::PoseGraph pg_msg;
  pose_graph_msgs::PoseGraphEdge edge;

  edge.key_from = 0;
  edge.key_to = 1;
  edge.type = pose_graph_msgs::PoseGraphEdge::LOOPCLOSE;
  edge.pose.position.x = 1.0;
  edge.pose.position.y = 2.0;
  edge.pose.position.z = 3.0;

  pg_msg.edges.push_back(edge);

  pose_graph_msgs::PoseGraphConstPtr pg_ptr(
      new pose_graph_msgs::PoseGraph(pg_msg));

  LaserLoopClosureCallback(pg_ptr);

  EdgeSet edges_info = GetEdges();

  gtsam::NonlinearFactorGraph nfg = GetNfg();

  // Check opt flag
  EXPECT_TRUE(GetOptFlag());

  // Check nfg
  EXPECT_EQ(nfg.size(), 1); // only loop closure

  // TODO - more checks

  // Check edges track
  const auto edge_pos = edges_info.begin()->pose.position;
  EXPECT_NEAR(edge_pos.x, edge.pose.position.x, tolerance_);
  EXPECT_NEAR(edge_pos.y, edge.pose.position.y, tolerance_);
  EXPECT_NEAR(edge_pos.z, edge.pose.position.z, tolerance_);
}

TEST_F(TestLampRobot, TestPointCloudTransform) {
  // Add the scan and values to the graph
  ros::NodeHandle nh, pnh("~");
  lr.Initialize(nh);

  // Scan
  gtsam::Symbol key = gtsam::Symbol('a', 1);
  AddToKeyScans(key, data);

  // Values
  InsertValues(gtsam::Symbol('a', 1),
               gtsam::Pose3(gtsam::Rot3(sqrt(0.5), 0, 0, sqrt(0.5)),
                            gtsam::Point3(0.0, 0.0, 0.0)));

  // Test the function
  ReGenerateMapPointCloud();

  // Output
  PointCloud::Ptr pc_out = GetMapPC();

  // Transform the main point cloud
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  Eigen::Vector3f rotVec(0.0, 0.0, 1);
  transform.rotate(Eigen::AngleAxisf(M_PI / 2.0f, rotVec));

  pcl::transformPointCloud(*data, *data, transform);

  // Compare
  for (int i; i < 4; i++) {
    std::cout << "Points are " << data->at(i) << " and " << pc_out->at(i)
              << std::endl;
    EXPECT_NEAR(data->at(i).x, pc_out->at(i).x, tolerance_);
    EXPECT_NEAR(data->at(i).y, pc_out->at(i).y, tolerance_);
    EXPECT_NEAR(data->at(i).z, pc_out->at(i).z, tolerance_);
  }
}

TEST_F(TestLampRobot, TestPointCloudTransformSingle) {
  // Add the scan and values to the graph
  ros::NodeHandle nh, pnh("~");
  lr.Initialize(nh);

  // Scan
  gtsam::Symbol key = gtsam::Symbol('a', 1);
  AddToKeyScans(key, data);

  // Values
  InsertValues(gtsam::Symbol('a', 1),
               gtsam::Pose3(gtsam::Rot3(sqrt(0.5), 0, 0, sqrt(0.5)),
                            gtsam::Point3(0.0, 0.0, 0.0)));

  // Test the function
  AddTransformedPointCloudToMap(key);

  // Output
  PointCloud::Ptr pc_out = GetMapPC();

  // Transform the main point cloud
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  Eigen::Vector3f rotVec(0.0, 0.0, 1);
  transform.rotate(Eigen::AngleAxisf(M_PI / 2.0f, rotVec));

  pcl::transformPointCloud(*data, *data, transform);

  // Compare
  for (int i; i < 4; i++) {
    std::cout << "Points are " << data->at(i) << " and " << pc_out->at(i)
              << std::endl;
    EXPECT_NEAR(data->at(i).x, pc_out->at(i).x, tolerance_);
    EXPECT_NEAR(data->at(i).y, pc_out->at(i).y, tolerance_);
    EXPECT_NEAR(data->at(i).z, pc_out->at(i).z, tolerance_);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lamp_robot");
  return RUN_ALL_TESTS();
}
