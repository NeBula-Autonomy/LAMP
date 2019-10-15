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
class OdometryHandler : public LampDataHandlerBase{

    friend class OdometryHandlerTest;
    
    public:
        
        // Constructors and Destructors
        OdometryHandler();
        ~OdometryHandler();  

        // Public methods
        bool Initialize (const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

        // LAMP Interface 
        FactorData GetData();
        bool GetOdomDelta(const ros::Time t_now, GtsamPosCov& delta_pose);
        bool GetOdomDeltaLatestTime(ros::Time& t_now, GtsamPosCov& delta_pose);
        bool GetKeyedScanAtTime(const ros::Time& stamp, PointCloud::Ptr& msg);
        GtsamPosCov GetFusedOdomDeltaBetweenTimes(const ros::Time t1, const ros::Time t2);
        
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

        // Odometry Storages 
        OdomPoseBuffer lidar_odometry_buffer_;
        OdomPoseBuffer visual_odometry_buffer_;
        OdomPoseBuffer wheel_odometry_buffer_; 
                
        // Point Cloud Storage (Time stamp and point cloud)
        std::map<double, PointCloud> point_cloud_buffer_;

        //TODO: Check to see if we need this function? Not used anywhere in the code.
        // template <typename T1, typename T2> 
        // int CheckBufferSize(const std::map<T1, T2>& x) {
        //   return x.size();
        // }
        
        bool CheckOdomSize();
        bool InsertMsgInBuffer(const Odometry::ConstPtr& odom_msg, OdomPoseBuffer& buffer);
        void FillGtsamPosCovOdom(const OdomPoseBuffer& odom_buffer, GtsamPosCov& measurement, const ros::Time t1, const ros::Time t2) const;
        double CalculatePoseDelta(const GtsamPosCov gtsam_pos_cov) const;
        void ClearOdometryBuffers();
        void ResetFactorData();        

        // Getters
        bool GetPoseAtTime(const ros::Time stamp, const OdomPoseBuffer& odom_buffer, PoseCovStamped& output) const;
        bool GetPosesAtTimes(const ros::Time t1, const ros::Time t2, const OdomPoseBuffer& odom_buffer, PoseCovStampedPair& output_poses) const;
        gtsam::Pose3 GetTransform(const PoseCovStampedPair pose_cov_stamped_pair) const;        
        gtsam::SharedNoiseModel GetCovariance(const PoseCovStampedPair pose_cov_stamped_pair) const; 
        bool GetClosestLidarTime(const ros::Time time, ros::Time& closest_time) const;

        // Converters
        gtsam::Pose3 ToGtsam(const gu::Transform3& pose) const; // TODO: This function should be defined in the base class

        // The node's name.
        std::string name_;

        // Parameters
        double keyed_scan_time_diff_limit_;
        double pc_buffer_size_limit_;
        double translation_threshold_;

        // Fusion logic 
        bool b_is_first_query_;
        double ts_threshold_; 
        ros::Time query_timestamp_first_; 
        GtsamPosCov fused_odom_;

      private:
};

#endif