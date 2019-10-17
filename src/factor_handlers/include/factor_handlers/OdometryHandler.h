/*
 * Copyright Notes
 *
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
 *          Kamak Ebadi         (kamak.ebadi@jpl.nasa.gov)
 *          Nobuhiro Funabiki   (nobuhiro.funabiki@jpl.nasa.gov)
 */

// Define
#ifndef ODOMETRY_HANDLER_H
#define ODOMETRY_HANDLER_H

// Includes
#include <factor_handlers/LampDataHandlerBase.h>

// Typedefs
typedef geometry_msgs::PoseWithCovarianceStamped PoseCovStamped;
typedef nav_msgs::Odometry Odometry;
typedef std::pair<PoseCovStamped, PoseCovStamped> PoseCovStampedPair;
typedef std::map<double, PoseCovStamped> OdomPoseBuffer;
typedef std::pair<ros::Time, ros::Time> TimeStampedPair;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef struct {
  bool b_has_value;
  gtsam::Pose3 pose;
  gtsam::SharedNoiseModel covariance;
} GtsamPosCov;

typedef std::pair<GtsamPosCov, GtsamPosCov> GtsamPosCovPair;

// Class Definition
class OdometryHandler : public LampDataHandlerBase {
  friend class OdometryHandlerTest;

public:
  // Constructors and Destructors
  OdometryHandler();
  ~OdometryHandler();

  // Public methods
  bool Initialize(const ros::NodeHandle& n);
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // LAMP Interface
  FactorData GetData();
  bool GetOdomDelta(const ros::Time t_now, GtsamPosCov& delta_pose);
  bool GetOdomDeltaLatestTime(ros::Time& t_now, GtsamPosCov& delta_pose);
  bool GetKeyedScanAtTime(const ros::Time& stamp, PointCloud::Ptr& msg);
  GtsamPosCov GetFusedOdomDeltaBetweenTimes(const ros::Time t1,
                                            const ros::Time t2);

protected:
  // Odometry Subscribers
  ros::Subscriber lidar_odom_sub_;
  ros::Subscriber visual_odom_sub_;
  ros::Subscriber wheel_odom_sub_;

  // Pointcloud Subscribers
  ros::Subscriber point_cloud_sub_;

  // Odometry Callbacks
  void LidarOdometryCallback(const Odometry::ConstPtr& msg);
  void VisualOdometryCallback(const Odometry::ConstPtr& msg);
  void WheelOdometryCallback(const Odometry::ConstPtr& msg);

  // Pointcloud Callback
  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  // Map Odometry Storages
  OdomPoseBuffer lidar_odometry_buffer_;
  OdomPoseBuffer visual_odometry_buffer_;
  OdomPoseBuffer wheel_odometry_buffer_;

  // Point Cloud Storage (Time stamp and point cloud)
  std::map<double, PointCloud> point_cloud_buffer_;

  // Utilities

  // New map-based utilities
  // -------------------------------------------------------------------------------------------------------------------

  template <typename T1, typename T2>
  int CheckBufferSizeMap(const std::map<T1, T2>& x) {
    return x.size();
  }

  bool CheckOdomSize();

  bool InsertMsgInBufferMap(const Odometry::ConstPtr& odom_msg,
                            OdomPoseBuffer& buffer_map) {
    // TODO: Make it template
    auto initial_map_size = buffer_map.size();
    PoseCovStamped current_msg;
    current_msg.header = odom_msg->header;
    current_msg.pose = odom_msg->pose;
    auto current_time = odom_msg->header.stamp.toSec();
    buffer_map.insert({current_time, current_msg});
    auto final_map_size = buffer_map.size();
    if (final_map_size == (initial_map_size + 1)) {
      // Msg insertion was successful, return true to the caller
      return true;
    } else {
      return false;
    }
  }

  // End new map-based utilities
  // ---------------------------------------------------------------------------------------------------------------

  void FillGtsamPosCovOdom(const OdomPoseBuffer& odom_buffer,
                           GtsamPosCov& measurement,
                           const ros::Time t1,
                           const ros::Time t2) const;
  double CalculatePoseDelta(const GtsamPosCov gtsam_pos_cov) const;
  void ClearOdometryBuffers();
  void ResetFactorData();

  // Getters

  gtsam::Pose3
  GetTransform(const PoseCovStampedPair pose_cov_stamped_pair) const;
  gtsam::SharedNoiseModel
  GetCovariance(const PoseCovStampedPair pose_cov_stamped_pair) const;
  bool GetClosestLidarTime(const ros::Time time, ros::Time& closest_time) const;

  // Converters
  gtsam::Pose3 ToGtsam(const gu::Transform3& pose)
      const; // TODO: This function should be defined in the base class

  // The node's name.
  std::string name_;

  // Parameters
  double keyed_scan_time_diff_limit_;
  double pc_buffer_size_limit_;
  double translation_threshold_;

  // Fusion logic
  double ts_threshold_;
  ros::Time query_timestamp_first_;
  GtsamPosCov fused_odom_;

  // New methods to deal with maps
  bool GetPoseAtTime(const ros::Time stamp,
                     const OdomPoseBuffer& odom_buffer_map,
                     PoseCovStamped& output) const;
  bool GetPosesAtTimes(const ros::Time t1,
                       const ros::Time t2,
                       const OdomPoseBuffer& odom_buffer_map,
                       PoseCovStampedPair& output_poses) const;

  bool b_is_first_query_;

private:
};

#endif

/*
UNUSED
std::pair<ros::Time, ros::Time> GetTimeStamps(PoseCovStampedPair
pose_cov_stamped_pair); void CheckOdometryBuffer(OdomPoseBuffer& odom_buffer);
void PrepareFactor(OdomPoseBuffer& odom_buffer);
void MakeFactor(PoseCovStampedPair pose_cov_stamped_pair);
double CalculatePoseDelta(OdomPoseBuffer& odom_buffer);
PoseCovStamped GetDeltaBetweenPoses(const PoseCovStampedPair& input_poses);


template <typename T>
int CheckBufferSize(const std::vector<T>& x) {
     return x.size();
}

template <typename T1, typename T2>
bool InsertMsgInBuffer(const typename T1::ConstPtr& msg, std::vector<T2>&
buffer) {
    // TODO: This function should be defined in the base class
    auto prev_size = CheckBufferSize<T2>(buffer);
    T2 stored_msg;
    // TODO: The following two lines should be implemented in a function
    stored_msg.header = msg->header;
    stored_msg.pose = msg->pose;
    buffer.push_back(stored_msg);
    auto current_size = CheckBufferSize<T2>(buffer);
    if (current_size != (prev_size + 1)) return false;
    return true;
}
*/