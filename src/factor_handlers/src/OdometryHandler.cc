/*
 * Copyright Notes
 *
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
 *          Kamak Ebadi         (kamak.ebadi@jpl.nasa.gov)
 *          Nobuhiro Funabiki   (nobuhiro.funabiki@jpl.nasa.gov)
*/



// Includes
#include <factor_handlers/OdometryHandler.h>

namespace pu = parameter_utils;

// Constructor & Destructors ----------------------------------------------------------------------------

OdometryHandler::OdometryHandler()
  : keyed_scan_time_diff_limit_(0.2),
    pc_buffer_size_limit_(10),
    translation_threshold_(1.0),
    ts_threshold_(0.1), 
    query_timestamp_first_(0) {
  ROS_INFO("Odometry Handler Class Constructor");
}

OdometryHandler::~OdometryHandler() {
    ROS_INFO("Odometry Handler Class Destructor");
}

// Initialize -------------------------------------------------------------------------------------------

bool OdometryHandler::Initialize(const ros::NodeHandle& n){
    
    name_ = ros::names::append(n.getNamespace(), "OdometryHandler");

    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }

    if (!RegisterCallbacks(n)) {
        ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
        return false;
    }    

    return true;
}

bool OdometryHandler::LoadParameters(const ros::NodeHandle& n) {
  ROS_INFO("LoadParameters method called in OdometryHandler");

  // Thresholds to add new factors
  if (!pu::Get("translation_threshold", translation_threshold_))
    return false;

  // Point Cloud buffer param
  if (!pu::Get("keyed_scan_time_diff_limit", keyed_scan_time_diff_limit_))
    return false;
  if (!pu::Get("pc_buffer_size_limit", pc_buffer_size_limit_))
    return false;

  // Timestamp threshold used in GetPoseAtTime method to return true to the caller
  if (!pu::Get("ts_threshold", ts_threshold_))
    return false;

  return true;
}

bool OdometryHandler::RegisterCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering online callbacks in OdometryHandler",
           name_.c_str());
  ros::NodeHandle nl(n);
  // TODO - check what is a reasonable buffer size
  lidar_odom_sub_ = nl.subscribe(
      "lio_odom", 1000, &OdometryHandler::LidarOdometryCallback, this);
  visual_odom_sub_ = nl.subscribe(
      "vio_odom", 1000, &OdometryHandler::VisualOdometryCallback, this);
  wheel_odom_sub_ = nl.subscribe(
      "wio_odom", 1000, &OdometryHandler::WheelOdometryCallback, this);

  // Point Cloud callback
  point_cloud_sub_ = nl.subscribe(
      "pcld", 10, &OdometryHandler::PointCloudCallback, this);

  return true;
}

// Callbacks --------------------------------------------------------------------------------------------

void OdometryHandler::LidarOdometryCallback(const Odometry::ConstPtr& msg) {    
    ROS_INFO("LidarOdometryCallback");
    
    if (InsertMsgInBuffer<Odometry, PoseCovStamped>(msg, lidar_odometry_buffer_)) {
        ROS_WARN("OdometryHanlder - LidarOdometryCallback - Unable to store message in buffer");
    }
}

void OdometryHandler::VisualOdometryCallback(const Odometry::ConstPtr& msg) {    
    ROS_INFO("VisualOdometryCallback");
    if (InsertMsgInBuffer<Odometry, PoseCovStamped>(msg, visual_odometry_buffer_)) {
        ROS_WARN("OdometryHanlder - VisualOdometryCallback - Unable to store message in buffer");
    }
}

void OdometryHandler::WheelOdometryCallback(const Odometry::ConstPtr& msg) {    
    ROS_INFO("WheelOdometryCallback");
    if (InsertMsgInBuffer<Odometry, PoseCovStamped>(msg, wheel_odometry_buffer_)) {
        ROS_WARN("OdometryHanlder - WheelOdometryCallback - Unable to store message in buffer");
    }
}

void OdometryHandler::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    double current_timestamp = msg->header.stamp.toSec();
    PointCloud current_pointcloud;
    pcl::fromROSMsg(*msg, current_pointcloud);
    point_cloud_buffer_.insert({current_timestamp, current_pointcloud});
    // Clear start of buffer if buffer is too large
    if (point_cloud_buffer_.size() > pc_buffer_size_limit_) {
      // Clear the first entry in the buffer
      point_cloud_buffer_.erase(point_cloud_buffer_.begin());
    }
}

// Interfaces  --------------------------------------------------------------------------------------------

void OdometryHandler::GetOdomDelta(const ros::Time t_now, GtsamPosCov& delta_pose) {
  // This is dynamically set by GetData and represents the timestamp of the last created node
  ros::Time t1 = query_timestamp_first_;  
  fused_odom_ = GetFusedOdomDeltaBetweenTimes(t1, t_now);
  delta_pose = fused_odom_;
}

FactorData OdometryHandler::GetData(){  
  if (query_timestamp_first_.toSec()==0){
    // If we never received a query before, store current time as query_timestamp_first
    query_timestamp_first_ = ros::Time::now();
    // If we stored the first query timestamp, we're sure we don't have any query_timestamp_second_ so we return empty factors
    factors_.b_has_data = false; 
    ROS_WARN("OdometryHandler - Queried for the first time, return empty factors");
    fused_odom_.pose = gtsam::Pose3(); // TODO: Make sure this is needed at runtime 
    return factors_;
  }
  else {
    ROS_INFO("Odometry Handler - Perform Fusion Logic");
    // }
    if (CalculatePoseDelta(fused_odom_) > 1.0) {
      // TODO: The time queries need to be changed
      // auto t2 = GetClosestLidarTime(ros::Time::now());
      auto t2 = ros::Time::now();
      fused_odom_ = GetFusedOdomDeltaBetweenTimes(query_timestamp_first_, t2);
      factors_.b_has_data = true; // TODO: Do this only if Fusion Logic output exceeds threshold
      factors_.transforms.push_back(fused_odom_.pose);
      factors_.covariances.push_back(fused_odom_.covariance);
      factors_.time_stamps.push_back(TimeStampedPair(query_timestamp_first_, t2));
      query_timestamp_first_ = t2;
      ResetFactorData();
    }
    else {
      factors_.b_has_data = false;
    }
    return factors_;
  }
}

bool OdometryHandler::GetKeyedScanAtTime(const ros::Time& stamp, PointCloud::Ptr& msg) {
  // TODO: This function should be impletented as a template function in the base class
  // TODO: For example, template <typename TYPE> GetKeyedValueAtTime(ros::Time& stamp, TYPE& msg)
  // Return false if there are not point clouds in the buffer
  if (point_cloud_buffer_.size() == 0)
    return false;

  // Search to get the lower-bound - the first entry that is not less than the
  // input timestamp
  auto itrTime = point_cloud_buffer_.lower_bound(stamp.toSec());
  auto time2 = itrTime->first;

  // If this gives the start of the buffer, then take that point cloud
  if (itrTime == point_cloud_buffer_.begin()) {
    *msg = itrTime->second;
    return true;
  }

  // Check if it is past the end of the buffer - if so, then take the last point
  // cloud
  if (itrTime == point_cloud_buffer_.end()) {
    ROS_WARN("Timestamp past the end of the point cloud buffer");
    itrTime--;
    *msg = itrTime->second;
    ROS_INFO_STREAM("input time is " << stamp.toSec()
                                     << "s, and latest time is "
                                     << itrTime->first << " s");
    return true;
  }

  // Otherwise = step back by 1 to get the time before the input time (time1,
  // stamp, time2)
  double time1 = std::prev(itrTime, 1)->first;

  double time_diff;

  // If closer to time2, then use that
  if (time2 - stamp.toSec() < stamp.toSec() - time1) {
    *msg = itrTime->second;

    time_diff = time2 - stamp.toSec();
  } else {
    // Otherwise use time1
    *msg = std::prev(itrTime, 1)->second;

    time_diff = stamp.toSec() - time1;
  }

  // Clear the point cloud buffer
  point_cloud_buffer_.clear();

  // Check if the time difference is too large
  if (time_diff >
      keyed_scan_time_diff_limit_) { // TODO make this threshold a parameter
    ROS_WARN("Time difference between request and latest point cloud is too "
             "large, returning no point cloud");
    ROS_INFO_STREAM("Time difference is " << keyed_scan_time_diff_limit_
                                          << "s");
    return false;
  }

  // Return true - have a point cloud for the timestamp
  return true;
}

// Utilities ---------------------------------------------------------------------------------------------

GtsamPosCov OdometryHandler::GetFusedOdomDeltaBetweenTimes(const ros::Time t1, const ros::Time t2) {
  // Returns the fused GtsamPosCov delta between t1 and t2
  GtsamPosCov output_odom, lidar_odom, visual_odom, wheel_odom;
  FillGtsamPosCovOdom(lidar_odometry_buffer_, lidar_odom, t1, t2);
  FillGtsamPosCovOdom(visual_odometry_buffer_, visual_odom, t1, t2);
  FillGtsamPosCovOdom(wheel_odometry_buffer_, wheel_odom, t1, t2);
  if (lidar_odom.b_has_value == true) {
    // 
  }
  if (visual_odom.b_has_value == true) {
    // 
  }
  if (wheel_odom.b_has_value == true) {
    // 
  }
  // TODO: For the first implementation, pure lidar-based odometry is used.
  output_odom = lidar_odom;
  return output_odom;
}

void OdometryHandler::FillGtsamPosCovOdom(const OdomPoseBuffer& odom_buffer, 
                                          GtsamPosCov& pure_odom,
                                          const ros::Time t1,
                                          const ros::Time t2) {
  /*
  Receives odometric buffer, search within it a pair of poses for the given timestamp, 
  Computes the relative transformation between the two poses and fills the GtsamPosCov struct
  */
  PoseCovStampedPair poses;
  if (GetPosesAtTimes(t1, t2, odom_buffer, poses)) {
    pure_odom.b_has_value = true;
    pure_odom.pose = GetTransform(poses);
    pure_odom.covariance = GetCovariance(poses);
  }
  else {
    pure_odom.b_has_value = false;
  }
}

void OdometryHandler::ResetFactorData() {
  // TODO: ResetFactors() clear all fields of private class member
  factors_.b_has_data = false;
  factors_.type = "odom";
  factors_.transforms.clear();
  factors_.covariances.clear();
  factors_.time_stamps.clear();
  factors_.artifact_key.clear();
}

ros::Time OdometryHandler::GetClosestLidarTime(ros::Time time) {
  ros::Time closest_time;  
  // TODO: Implement
  return closest_time;
}

void OdometryHandler::ClearOdometryBuffers() {
  // TODO: The last few elements should be kept in buffer just in case
  lidar_odometry_buffer_.clear();
  visual_odometry_buffer_.clear();
  wheel_odometry_buffer_.clear();
}

// CalculatePoseDelta ---------------------------------------------------------------------------------------------

double OdometryHandler::CalculatePoseDelta(GtsamPosCov gtsam_pos_cov) {
  auto pose = gtsam_pos_cov.pose;
  return pose.translation().norm();
}


// Getters -----------------------------------------------------------------------------------------------

bool OdometryHandler::GetPoseAtTime(ros::Time t, const OdomPoseBuffer& odom_buffer, PoseCovStamped& output) {
  // Create a PoseCovStamped message
  PoseCovStamped myPoseCovStamped;
  // Given a query timestamp 
  auto query_timestamp = t.toSec();
  // Declare a big timestamp difference 
  double min_ts_diff = 1000; //TODO:: make it a parameter
  // Iterate through the vector to find the element of interest 
  for (size_t i=0; i<odom_buffer.size(); ++i){
        double cur_ts_diff = odom_buffer[i].header.stamp.toSec() - query_timestamp;
    if (fabs(cur_ts_diff)<fabs(min_ts_diff)){
      myPoseCovStamped = odom_buffer[i];
      min_ts_diff = cur_ts_diff; 
    }
    // Here we've selected the most likely element we were searching for, make sure everything is correct    
  }
  if (fabs(min_ts_diff)<fabs(ts_threshold_)){
      // If everything is fine, we fill the output message and return true to the caller
    output = myPoseCovStamped; 
    return true; 
  }
  else{
    return false; 
  }
}

bool OdometryHandler::GetPosesAtTimes(ros::Time t1, ros::Time t2, const OdomPoseBuffer& odom_buffer, PoseCovStampedPair& output_poses) {
  PoseCovStamped first_pose; 
  PoseCovStamped second_pose; 
  if (GetPoseAtTime(t1, odom_buffer, first_pose)){
    if (GetPoseAtTime(t2, odom_buffer, second_pose)) {
      output_poses = std::make_pair(first_pose, second_pose);
      return true;
    }
  }
  else {
    return false;
  }
}

gtsam::Pose3 OdometryHandler::GetTransform(PoseCovStampedPair pose_cov_stamped_pair) {
    // Gets the transform between two pose stamped - the delta
    auto pose_first = gr::FromROS(pose_cov_stamped_pair.first.pose.pose); 
    auto pose_end = gr::FromROS(pose_cov_stamped_pair.second.pose.pose); 
    auto pose_delta = gu::PoseDelta(pose_first, pose_end);
    gtsam::Pose3 output = ToGtsam(pose_delta);
    return output;
}

gtsam::SharedNoiseModel OdometryHandler::GetCovariance(PoseCovStampedPair pose_cov_stamped_pair) {
  // TODO check which frame the covariances are in - ideally we have incremental
  // in the relative frame If the covariances are absolute

  gtsam::Matrix66 covariance;
  for (size_t i = 0; i < pose_cov_stamped_pair.second.pose.covariance.size();
       i++) {
    size_t row = static_cast<size_t>(i / 6);
    size_t col = i % 6;
    covariance(row, col) = pose_cov_stamped_pair.second.pose.covariance[i] -
        pose_cov_stamped_pair.first.pose.covariance[i];
  }
  gtsam::SharedNoiseModel noise =
      gtsam::noiseModel::Gaussian::Covariance(covariance);

  return noise;
}

PoseCovStamped OdometryHandler::GetDeltaBetweenPoses(const PoseCovStampedPair& input_poses){
  // TODO: Get the first and second pose, find the transformation between the two and return it 
  PoseCovStamped output; 
  PoseCovStamped first_pose = input_poses.first;
  PoseCovStamped second_pose = input_poses.second;
  // TODO: Integrate into geometry_utils a method that provide us the DeltaBetweenPoses preserving the covariance information
  return output; 
}

// Converters -------------------------------------------------------------------------------------------

gtsam::Pose3 OdometryHandler::ToGtsam(const gu::Transform3& pose) const {
  
  gtsam::Vector3 t;

  t(0) = pose.translation(0);
  t(1) = pose.translation(1);
  t(2) = pose.translation(2);

  gtsam::Rot3 r(pose.rotation(0, 0), pose.rotation(0, 1), pose.rotation(0, 2),
         pose.rotation(1, 0), pose.rotation(1, 1), pose.rotation(1, 2),
         pose.rotation(2, 0), pose.rotation(2, 1), pose.rotation(2, 2));

  return gtsam::Pose3(r, t);
}

/*
DOCUMENTATION 

        nav_msgs/Odometry Message
            Header header
            string child_frame_id
            geometry_msgs/PoseWithCovariance pose
            geometry_msgs/TwistWithCovariance twist

        geometry_msgs/PoseWithCovariance
            Pose pose
            float64[36] covariance

        geometry_msgs/PoseWithCovarianceStamped
            Header header
            PoseWithCovariance pose

        struct FactorData {
            bool b_has_data; // False if there is no data
            std::string type; // odom, artifact, loop clsoure
            // Vector for possible multiple factors
            std::vector<gtsam::Pose3> transforms; // The transform (for odom, loop closures etc.) and pose for TS
            std::vector<Mat1212> covariances; // Covariances for each transform 
            std::vector<std::pair<ros::Time, ros::Time>> time_stamps; // Time when the measurement as acquired
            std::vector<gtsam::Key> artifact_key; // key for the artifacts
        };
*/

/*
UNUSED ----------------------------------------------------------------------------------------------------------------



void OdometryHandler::CheckOdometryBuffer(OdomPoseBuffer& odom_buffer) {
    if (CheckBufferSize<PoseCovStamped>(odom_buffer) > 2) {
      double translation = CalculatePoseDelta(odom_buffer); 
      if (translation > translation_threshold_) {
        ROS_INFO_STREAM("Moved more than threshold: " << translation_threshold_
                                                      << " m (" << translation
                                                      << " m)");
        PrepareFactor(odom_buffer);
        }
    }     
}

void OdometryHandler::PrepareFactor(OdomPoseBuffer& odom_buffer) {
  // Make a pair between the first and last elements in the odom buffer
  auto first_odom_element = odom_buffer.begin();
  auto last_odom_element = std::prev(odom_buffer.end());
  auto pose_cov_stamped_pair = std::make_pair(*first_odom_element, *last_odom_element);
  MakeFactor(pose_cov_stamped_pair);
  // After MakeFactor has finished its job, reset the buffer and add last_odom_element as first element
  odom_buffer.clear();
  odom_buffer.push_back(*last_odom_element);
}

void OdometryHandler::MakeFactor(PoseCovStampedPair pose_cov_stamped_pair) {
    //Makes a new factor by filling the fields of FactorData
    factors_.b_has_data = true;
    factors_.type = "odom";
    factors_.transforms.push_back(GetTransform(pose_cov_stamped_pair));
    factors_.covariances.push_back(GetCovariance(pose_cov_stamped_pair));
    factors_.time_stamps.push_back(GetTimeStamps(pose_cov_stamped_pair));
}

std::pair<ros::Time, ros::Time> OdometryHandler::GetTimeStamps(PoseCovStampedPair pose_cov_stamped_pair) {
  // Create a pair of the timestamps from and to - to be used in lamp to reference nodes
  // Get the timestamps of interest from the received pair
  ros::Time first_timestamp = pose_cov_stamped_pair.first.header.stamp;
  ros::Time second_timestamp = pose_cov_stamped_pair.second.header.stamp;
  std::pair<ros::Time, ros::Time> timestamp_pair;
  timestamp_pair.first = first_timestamp;
  timestamp_pair.second = second_timestamp;
  return timestamp_pair;
}
*/