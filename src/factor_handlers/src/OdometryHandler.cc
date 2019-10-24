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

// Constructor & Destructors
// ----------------------------------------------------------------------------

OdometryHandler::OdometryHandler()
  : keyed_scan_time_diff_limit_(0.2),
    pc_buffer_size_limit_(10),
    translation_threshold_(1.0),
    ts_threshold_(0.1),
    query_timestamp_first_(0),
    b_is_first_query_(true), 
    max_buffer_size_(6000),
    min_buffer_size_(3000)
    {
      InitializePoseCovStampedMsgValue(lidar_odom_value_at_key_);
      InitializePoseCovStampedMsgValue(visual_odom_value_at_key_);
      InitializePoseCovStampedMsgValue(wheel_odom_value_at_key_);
    }

OdometryHandler::~OdometryHandler() {
}

// Initialize
// -------------------------------------------------------------------------------------------

bool OdometryHandler::Initialize(const ros::NodeHandle& n) {
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

  // Thresholds to add new factors
  if (!pu::Get("translation_threshold", translation_threshold_)) return false;

  // Point Cloud buffer param
  if (!pu::Get("keyed_scan_time_diff_limit", keyed_scan_time_diff_limit_)) return false;
  if (!pu::Get("pc_buffer_size_limit", pc_buffer_size_limit_)) return false;

  // Timestamp threshold used in GetPoseAtTime method to return true to the caller
  if (!pu::Get("ts_threshold", ts_threshold_)) return false;
  
  // Specify a maximum and minimum buffer size to store history of Odometric data stream 
  if (!pu::Get("max_buffer_size", max_buffer_size_)) return false;
  if (!pu::Get("min_buffer_size", min_buffer_size_)) return false;
  
  return true;
}

bool OdometryHandler::RegisterCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering online callbacks in OdometryHandler",
           name_.c_str());
  ros::NodeHandle nl(n);
  // TODO - check what is a reasonable buffer size
  lidar_odom_sub_ = nl.subscribe(
      "lio_odom", 10, &OdometryHandler::LidarOdometryCallback, this);
  visual_odom_sub_ = nl.subscribe(
      "vio_odom", 10, &OdometryHandler::VisualOdometryCallback, this);
  wheel_odom_sub_ = nl.subscribe(
      "wio_odom", 10, &OdometryHandler::WheelOdometryCallback, this);

  // Point Cloud callback
  point_cloud_sub_ =
      nl.subscribe("pcld", 10, &OdometryHandler::PointCloudCallback, this);

  return true;
}

// Callbacks
// --------------------------------------------------------------------------------------------

void OdometryHandler::LidarOdometryCallback(const Odometry::ConstPtr& msg) {
  
  // If buffer size exceeds limit, clear the buffer, keeping "min_buffer_size_" of data
  if (CheckBufferSize<double, PoseCovStamped>(lidar_odometry_buffer_)>max_buffer_size_){
    ClearOdometryBuffer(lidar_odometry_buffer_, LIDAR_ODOM_BUFFER_ID);
  }
  // Insert message in buffer 
  if (!InsertMsgInBuffer(msg, lidar_odometry_buffer_)) {
    ROS_WARN("OdometryHanlder - LidarOdometryCallback - Unable to store "
             "message in buffer");
  }
}

void OdometryHandler::VisualOdometryCallback(const Odometry::ConstPtr& msg) {
  
  // If buffer size exceeds limit, clear the buffer, keeping "min_buffer_size_" of data
  if (CheckBufferSize<double, PoseCovStamped>(visual_odometry_buffer_)>max_buffer_size_){
    ClearOdometryBuffer(visual_odometry_buffer_, VISUAL_ODOM_BUFFER_ID);
  }
  // Insert message in buffer
  if (!InsertMsgInBuffer(msg, visual_odometry_buffer_)) {
    ROS_WARN("OdometryHanlder - VisualOdometryCallback - Unable to store "
             "message in buffer");
  }
}

void OdometryHandler::WheelOdometryCallback(const Odometry::ConstPtr& msg) {

  // If buffer size exceeds limit, clear the buffer, keeping "min_buffer_size_" of data
  if (CheckBufferSize<double, PoseCovStamped>(wheel_odometry_buffer_)>max_buffer_size_){
    ClearOdometryBuffer(wheel_odometry_buffer_, WHEEL_ODOM_BUFFER_ID);
  }
  // Insert message in buffer
  if (!InsertMsgInBuffer(msg, wheel_odometry_buffer_)) {
    ROS_WARN("OdometryHanlder - WheelOdometryCallback - Unable to store "
             "message in buffer");
  }
}

void OdometryHandler::PointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
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

// Utilities
// ---------------------------------------------------------------------------------------------
void OdometryHandler::InitializePoseCovStampedMsgValue(PoseCovStamped& msg) {
  msg.pose.pose.position.x = 0;
  msg.pose.pose.position.y = 0;
  msg.pose.pose.position.z = 0;
  msg.pose.pose.orientation.x = 0;
  msg.pose.pose.orientation.y = 0;
  msg.pose.pose.orientation.z = 0;
  msg.pose.pose.orientation.w = 1;
}

bool OdometryHandler::InsertMsgInBuffer(const Odometry::ConstPtr& odom_msg,
                                        OdomPoseBuffer& buffer) {
  auto initial_size = buffer.size();
  PoseCovStamped current_msg;
  current_msg.header = odom_msg->header;
  current_msg.pose = odom_msg->pose;
  auto current_time = odom_msg->header.stamp.toSec();
  buffer.insert({current_time, current_msg});
  auto final_size = buffer.size();
  if (final_size == (initial_size + 1)) {
    // Msg insertion was successful, return true to the caller
    return true;
  } else {
    return false;
  }
}

bool OdometryHandler::GetOdomDelta(const ros::Time t_now,
                                   GtsamPosCov& delta_pose) {
  // Check odometry buffer size - return false otherwise
  if (!CheckOdomSize()) {
    ROS_WARN("Buffers are empty, returning no data (GetOdomDelta)");
    return false;
  }

  // query_timestamp_first_ is dynamically set by GetData but is initialized
  // here It represents the timestamp of the last created node

  if (b_is_first_query_) {
    // Get the first time from the lidar scan
    query_timestamp_first_.fromSec(lidar_odometry_buffer_.begin()->first);

    ROS_INFO_STREAM("First query to Odometry Handler, Input timestamp is "
                    << t_now.toSec() << ". setting first timestamp to "
                    << query_timestamp_first_.toSec());
    b_is_first_query_ = false;
    // Set Fused odom to zero pose - no movement yet
    fused_odom_.pose =
        gtsam::Pose3(); // TODO: Make sure this is needed at runtime
  }

  // ROS_INFO_STREAM("Delta between times is: "
  //                 << (query_timestamp_first_.toSec() - t_now.toSec()));
  fused_odom_ = GetFusedOdomDeltaBetweenTimes(query_timestamp_first_, t_now);

  if (!fused_odom_.b_has_value) {
    ROS_ERROR("No valid return from GetFusedOdomDelta");
    ROS_INFO_STREAM("Earliest timestamp in buffer is "
                    << lidar_odometry_buffer_.begin()->first);
    ROS_INFO_STREAM("Latest timestamp in buffer is "
                    << lidar_odometry_buffer_.rbegin()->first);
    ROS_INFO_STREAM("Input times are " << query_timestamp_first_.toSec()
                                       << " and " << t_now.toSec());
    return false;
  }

  // TODO - unit test to see what happens at the start when
  // query_timestamp_fist is very close to t_now

  // Fill in what is needed for the output
  delta_pose = fused_odom_;

  return true;
}

// For when the normal OdomDelta fails - call this to get the latest - from
// lidar timestamps Return the timestamp for use in LAMP
bool OdometryHandler::GetOdomDeltaLatestTime(ros::Time& t_latest,
                                             GtsamPosCov& delta_pose) {
  if (!CheckOdomSize()) {
    ROS_WARN("Buffers are empty, returning no data (GetOdomDeltaLatestTime)");
    return false;
  }
  // Get the latest time (rbegin is the last entry in the map)
  t_latest.fromSec(lidar_odometry_buffer_.rbegin()->first);

  // Get the delta as normal
  return GetOdomDelta(t_latest, delta_pose);
}

FactorData* OdometryHandler::GetData() {
  // Main interface with lamp for getting factor information
  OdomData* output_data = new OdomData(factors_);
  output_data->b_has_data = false;

  if (!CheckOdomSize()) {
    ROS_WARN("Buffers are emptyin GetData Call, returning no data");
    return output_data;
  }

  GtsamPosCov fused_odom_for_factor;

  if (CalculatePoseDelta(fused_odom_) > translation_threshold_) {
    // ROS_INFO("Adding a new node");
    // Get the most recent lidar timestamp
    ros::Time t2;
    t2.fromSec(lidar_odometry_buffer_.rbegin()->first);
    // GetClosestLidarTime(ros::Time::now(), t2); TODO: Check how we're computing t2

    // Get the updated fused odom between the two lidar-linked timestamps
    fused_odom_for_factor =
        GetFusedOdomDeltaBetweenTimes(query_timestamp_first_, t2);

    if (!fused_odom_for_factor.b_has_value) {
      ROS_ERROR("Issues getting delta for factor. Returning no data");
      return output_data;
    }

    // Fill factors data
    output_data->b_has_data =
        true; // TODO: Do this only if Fusion Logic output exceeds threshold

    // Make the new factor data
    OdometryFactor new_odom;
    new_odom.transform = fused_odom_for_factor.pose;
    new_odom.covariance = fused_odom_for_factor.covariance;
    new_odom.stamps = TimeStampedPair(query_timestamp_first_, t2);
    output_data->factors.push_back(new_odom);

    // Update the query timestamp to the time of the new node
    // TODO - update name to link to node/factor creation
    query_timestamp_first_ = t2;

    SetOdomValuesAtKey(t2);

    // Clear the stored data, now that it has been processed
    // This will clear factors_ - hence we have created output_data in this
    // function
    ResetFactorData();
  }

  return output_data;
}

bool OdometryHandler::GetKeyedScanAtTime(const ros::Time& stamp,
                                         PointCloud::Ptr& msg) {
  // TODO: This function should be impletented as a template function in the
  // base class
  // TODO: For example, template <typename TYPE> GetKeyedValueAtTime(ros::Time&
  // stamp, TYPE& msg) Return false if there are not point clouds in the buffer
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
    itrTime--;
    *msg = itrTime->second;
    if (stamp.toSec() - itrTime->first > ts_threshold_){
      ROS_WARN("Timestamp past the end of the point cloud buffer [GetKeyedScan]");
      ROS_INFO_STREAM("input time is " << stamp.toSec()
                                      << "s, and latest time is "
                                      << itrTime->first << " s [GetKeyedScan]");
    }
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

// Utilities
// ---------------------------------------------------------------------------------------------

GtsamPosCov OdometryHandler::GetFusedOdomDeltaBetweenTimes(const ros::Time t1,
                                                           const ros::Time t2) {
  // TODO - Interpolate here rather than just getting the closest times
  GtsamPosCov output_odom;
  output_odom.b_has_value = false;

  // Returns the fused GtsamPosCov delta between t1 and t2
  if (!CheckOdomSize()) {
    ROS_WARN(
        "Buffers are empty, returning no data (GetFusedOdomDeltaBetweenTimes)");
    return output_odom;
  }

  // ROS_INFO_STREAM("Timestamps are: " << t1.toSec() << " and " << t2.toSec()
  //                                    << ". Difference is: "
  //                                    << t2.toSec() - t1.toSec());
  GtsamPosCov lidar_odom, visual_odom, wheel_odom;
  // ROS_INFO_STREAM("Lidar buffer size in GetFusedOdom is: "
  //                 << lidar_odometry_buffer_.size());
  FillGtsamPosCovOdom(lidar_odometry_buffer_, lidar_odom, t1, t2, LIDAR_ODOM_BUFFER_ID);
  FillGtsamPosCovOdom(visual_odometry_buffer_, visual_odom, t1, t2, VISUAL_ODOM_BUFFER_ID);
  FillGtsamPosCovOdom(wheel_odometry_buffer_, wheel_odom, t1, t2, WHEEL_ODOM_BUFFER_ID);
  if (lidar_odom.b_has_value == true) {
    // TODO: For the first implementation, pure lidar-based odometry is used.
    output_odom = lidar_odom;
  } else {
    ROS_ERROR("Failed to get odom from lidar");
    output_odom = lidar_odom;
  }
  if (visual_odom.b_has_value == true) {
    //
  }
  if (wheel_odom.b_has_value == true) {
    //
  }
  return output_odom;
}

double OdometryHandler::CalculatePoseDelta(const GtsamPosCov gtsam_pos_cov) const {
  auto pose = gtsam_pos_cov.pose;
  return pose.translation().norm();
}

void OdometryHandler::FillGtsamPosCovOdom(const OdomPoseBuffer& odom_buffer,
                                          GtsamPosCov& measurement,
                                          const ros::Time t1,
                                          const ros::Time t2, 
                                          const int odom_buffer_id) const {
  /*
  Receives odometric buffer, search within it a pair of poses for the given
  timestamp, Computes the relative transformation between the two poses and
  fills the GtsamPosCov struct
  */
  PoseCovStampedPair poses;
  if (GetPosesAtTimes(t1, t2, odom_buffer, poses, odom_buffer_id)) {
    measurement.b_has_value = true;
    measurement.pose = GetTransform(poses);
    measurement.covariance = GetCovariance(poses);
  } else {
    measurement.b_has_value = false;
  }
}

bool OdometryHandler::CheckOdomSize() {
  bool b_odom_has_data;
  b_odom_has_data = (lidar_odometry_buffer_.size() > 1);
  b_odom_has_data = b_odom_has_data || (visual_odometry_buffer_.size() > 1);
  b_odom_has_data = b_odom_has_data || (wheel_odometry_buffer_.size() > 1);

  return b_odom_has_data;
}

void OdometryHandler::ResetFactorData() {
  // TODO: ResetFactors() clear all fields of private class member
  factors_.b_has_data = false;
  factors_.type = "odom";
  factors_.factors.clear();
}

// Clear the buffer, keeping "min_buffer_size_" of data
void OdometryHandler::ClearOdometryBuffer(OdomPoseBuffer& odom_buffer, const unsigned int odom_buffer_id) {
  auto itr_begin = odom_buffer.begin();
  auto itr_last = std::next(odom_buffer.rbegin(), min_buffer_size_);
  odom_buffer.erase(itr_begin, itr_last.base());
  switch (odom_buffer_id) {
    case LIDAR_ODOM_BUFFER_ID:
      ROS_INFO("Clear Lidar odometry buffer"); 
      break;
    case VISUAL_ODOM_BUFFER_ID:
      ROS_INFO("Clear Visual odometry buffer");
      break;
    case WHEEL_ODOM_BUFFER_ID:
      ROS_INFO("Clear Wheel odometry buffer");
      break;
    default:
      ROS_ERROR("Invalid odometry buffer id in OdometryHandler::ClearOdometryBuffer");
      break;
  }
}

// Setters
// -----------------------------------------------------------------------------------------------

// Store individual odometric values in protected class members whenever a new key is created
void OdometryHandler::SetOdomValuesAtKey(const ros::Time query) {
  GetPoseAtTime(query, lidar_odometry_buffer_, lidar_odom_value_at_key_);
  GetPoseAtTime(query, visual_odometry_buffer_, visual_odom_value_at_key_);
  GetPoseAtTime(query, wheel_odometry_buffer_, wheel_odom_value_at_key_);
}

// Getters
// -----------------------------------------------------------------------------------------------

bool OdometryHandler::GetPoseAtTime(const ros::Time stamp,
                                    const OdomPoseBuffer& odom_buffer,
                                    PoseCovStamped& output) const {
  // If map is empty, return false to the caller
  if (odom_buffer.size() == 0) {
    return false;
  }

  // Given the input timestamp, search for lower bound (first entry that is not
  // less than the given timestamp)
  auto itrTime = odom_buffer.lower_bound(stamp.toSec());
  auto time2 = itrTime->first;
  double time_diff;

  // If this gives the start of the buffer, then take that PosCovStamped
  if (itrTime == odom_buffer.begin()) {
    output = itrTime->second;
    time_diff = itrTime->first - stamp.toSec();
    if (time_diff > ts_threshold_){
      ROS_WARN("Timestamp before the start of the odometry buffer beyond threshold [GetPoseAtTime]");
      ROS_INFO_STREAM("time diff is: " << time_diff << ". [GetPoseAtTime]");
    }
  } else if (itrTime == odom_buffer.end()) {
    // Check if it is past the end of the buffer - if so, then take the last
    // PosCovStamped
    itrTime--;
    output = itrTime->second;
    time_diff = stamp.toSec() - itrTime->first;
    if (time_diff > ts_threshold_){
      ROS_WARN("Timestamp past the end of the odometry buffer and beyond threshold [GetPoseAtTime]");
      ROS_INFO_STREAM("input time is " << stamp.toSec()
                                      << "s, and latest time is "
                                      << itrTime->first << " s"
                                      << " diff is " << time_diff << ". [GetPoseAtTime]");
    }
  } else {
    // Otherwise step back by 1 to get the time before the input time (time1,
    // stamp, time2)
    double time1 = std::prev(itrTime, 1)->first;

    // If closer to time2, then use that
    if (time2 - stamp.toSec() < stamp.toSec() - time1) {
      output = itrTime->second;
      time_diff = time2 - stamp.toSec();
    } else {
      // Otherwise use time1
      output = std::prev(itrTime, 1)->second;
      time_diff = stamp.toSec() - time1;
    }
  }

  // Check if the time difference is too large
  if (time_diff > ts_threshold_) {
    ROS_WARN("Time difference between request and latest PosCovStamped is too "
             "large, returning no PosC [GetPoseAtTime]");
    ROS_INFO_STREAM("Time difference is "
                    << time_diff << "s, threshold is: " << ts_threshold_ << ". [GetPoseAtTime]");
    return false;
  }

  return true;
}

bool OdometryHandler::GetPosesAtTimes(const ros::Time t1,
                                      const ros::Time t2,
                                      const OdomPoseBuffer& odom_buffer,
                                      PoseCovStampedPair& output_poses, 
                                      const int odom_buffer_id) const {
  PoseCovStamped first_pose, second_pose;

  // If unable to retrieve data of interest given query timestamp t1,
  // store in first_pose the value contained by the correspondant protected class member
  if (!GetPoseAtTime(t1, odom_buffer, first_pose)) {
    switch (odom_buffer_id) {
      case LIDAR_ODOM_BUFFER_ID:
        first_pose = lidar_odom_value_at_key_; 
        ROS_INFO("correct lidar odom buffer id");
        break;
      case VISUAL_ODOM_BUFFER_ID:
        first_pose = visual_odom_value_at_key_;      
        break;
      case WHEEL_ODOM_BUFFER_ID:
        first_pose = wheel_odom_value_at_key_; 
        break;
      default:
        ROS_ERROR("Invalid odometry buffer id in OdometryHandler::GetPosesAtTimes");
        return false;
        break; 
    }    
  }
   
  if (GetPoseAtTime(t2, odom_buffer, second_pose)) {
    output_poses = std::make_pair(first_pose, second_pose);
    return true;
  } else {
    return false;
  }

}

bool OdometryHandler::GetClosestLidarTime(const ros::Time stamp,
                                          ros::Time& closest_stamp) const {
  // Map based logic

  // If map is empty, return false to the caller
  if (lidar_odometry_buffer_.size() == 0) {
    return false;
  }

  // Given the input timestamp, search for lower bound (first entry that is not
  // less than the given timestamp)
  auto itrTime = lidar_odometry_buffer_.lower_bound(stamp.toSec());
  auto time2 = itrTime->first;

  // If this gives the start of the buffer, then take that PosCovStamped
  if (itrTime == lidar_odometry_buffer_.begin()) {
    closest_stamp.fromSec(itrTime->first);
    return true;
  }

  // Check if it is past the end of the buffer - if so, then take the last
  // PosCovStamped
  if (itrTime == lidar_odometry_buffer_.end()) {
    itrTime--;
    closest_stamp.fromSec(itrTime->first);
    if ((stamp - closest_stamp).toSec() > ts_threshold_){
      ROS_WARN("Timestamp past the end of the lidar odometry buffer [GetClosestLidarTime]");
      ROS_INFO_STREAM("input time is " << stamp.toSec()
                                      << "s, and latest time is "
                                      << itrTime->first << " s [GetClosestLidarTime]");
    }
    return true;
  }

  // Otherwise step back by 1 to get the time before the input time (time1,
  // stamp, time2)
  double time1 = std::prev(itrTime, 1)->first;
  double time_diff;

  // If closer to time2, then use that
  if (time2 - stamp.toSec() < stamp.toSec() - time1) {
    closest_stamp.fromSec(itrTime->first);
    time_diff = time2 - stamp.toSec();
  } else {
    // Otherwise use time1
    closest_stamp.fromSec(std::prev(itrTime, 1)->first);
    time_diff = stamp.toSec() - time1;
  }

  // Check if the time difference is too large
  if (time_diff > ts_threshold_) {
    ROS_WARN("Time difference between request and latest PosCovStamped is too "
             "large, returning no PosC");
    ROS_INFO_STREAM("Time difference is " << time_diff << "s");
    return false;
  }

  return true;
}

gtsam::Pose3 OdometryHandler::GetTransform(
    const PoseCovStampedPair pose_cov_stamped_pair) const {
  // Gets the transform between two pose stamped - the delta
  auto pose_first = gr::FromROS(pose_cov_stamped_pair.first.pose.pose);
  auto pose_end = gr::FromROS(pose_cov_stamped_pair.second.pose.pose);
  auto pose_delta = gu::PoseDelta(pose_first, pose_end);
  gtsam::Pose3 output = ToGtsam(pose_delta);
  return output;
}

gtsam::SharedNoiseModel OdometryHandler::GetCovariance(
    const PoseCovStampedPair pose_cov_stamped_pair) const {
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

// Converters
// -------------------------------------------------------------------------------------------

gtsam::Pose3 OdometryHandler::ToGtsam(const gu::Transform3& pose) const {
  gtsam::Vector3 t;

  t(0) = pose.translation(0);
  t(1) = pose.translation(1);
  t(2) = pose.translation(2);

  gtsam::Rot3 r(pose.rotation(0, 0),
                pose.rotation(0, 1),
                pose.rotation(0, 2),
                pose.rotation(1, 0),
                pose.rotation(1, 1),
                pose.rotation(1, 2),
                pose.rotation(2, 0),
                pose.rotation(2, 1),
                pose.rotation(2, 2));

  return gtsam::Pose3(r, t);
}

/*
DOCUMENTATION
Datatype Documentation

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
            std::vector<gtsam::Pose3> transforms; // The transform (for odom,
loop closures etc.) and pose for TS std::vector<Mat1212> covariances; //
Covariances for each transform std::vector<std::pair<ros::Time, ros::Time>>
time_stamps; // Time when the measurement as acquired std::vector<gtsam::Key>
artifact_key; // key for the artifacts
        };
*/