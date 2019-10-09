/*
 * Copyright Notes
 *
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
 *          Kamak Ebadi         (kamak.ebadi@jpl.nasa.gov)
 *          Nobuhiro Funabiki   (nobuhiro.funabiki@jpl.nasa.gov)
*/



#ifndef ODOMETRY_HANDLER_H
#define ODOMETRY_HANDLER_H



// Includes
#include <factor_handlers/LampDataHandlerBase.h>



// Typedefs - TODO make this common across packages somehow?
typedef geometry_msgs::PoseWithCovarianceStamped PoseCovStamped;
typedef nav_msgs::Odometry Odometry;
typedef std::pair<PoseCovStamped, PoseCovStamped> PoseCovStampedPair;
typedef std::vector<PoseCovStamped> OdomPoseBuffer;
typedef std::pair<ros::Time, ros::Time> TimeStampedPair;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef struct {
  bool b_has_value;
  gtsam::Pose3 pose; 
  gtsam::SharedNoiseModel covariance; 
} GtsamPosCov;

typedef struct  {
  GtsamPosCov lidar_odom; 
  GtsamPosCov visual_odom; 
  GtsamPosCov wheel_odom;
} GtsamOdom;

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

        // TODO: This function should be impletented as a template function in the base class
        // TODO: For example, template <typename TYPE> GetKeyedValueAtTime(ros::Time& stamp, TYPE& msg)
        bool GetKeyedScanAtTime(ros::Time& stamp, PointCloud::Ptr& msg);

        bool GetDeltaBetweenTimes(const ros::Time t1,
                                  const ros::Time t2,
                                  gtsam::Pose3& delta);

        FactorData GetData();

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

        // Protected methods
        // TODO: This function should be defined in the base class
        
 
        template <typename T>
        int CheckBufferSize(const std::vector<T>& x) {
            std::cout << x.size() << std::endl;
            return x.size();
        }

        template <typename T1, typename T2>
        bool InsertMsgInBuffer(const typename T1::ConstPtr& msg, std::vector<T2>& buffer) {
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

        GtsamPosCov FuseMultipleOdometry();
        void InsertGtsamOdometryInfo(const OdomPoseBuffer& odom_buffer, GtsamPosCov& measurement);
        double CalculatePoseDelta(OdomPoseBuffer& odom_buffer);
        double CalculatePoseDelta(GtsamPosCov gtsam_pos_cov);
        void ClearOdometryBuffers();
        

        // Getters 
        gtsam::Pose3 GetTransform(PoseCovStampedPair pose_cov_stamped_pair);        
        gtsam::SharedNoiseModel GetCovariance(PoseCovStampedPair pose_cov_stamped_pair); 
        std::pair<ros::Time, ros::Time> GetTimeStamps(PoseCovStampedPair pose_cov_stamped_pair);

        // Converters
        gtsam::Pose3 ToGtsam(const gu::Transform3& pose) const; // TODO: This function should be defined in the base class
 
        // LAMP Interface
        FactorData factors_;         

        // The node's name.
        std::string name_;

        // Parameters
        double keyed_scan_time_diff_limit_;
        double pc_buffer_size_limit_;
        double translation_threshold_;

        // Fusion logic 
        ros::Time query_timestamp_first_; 
        ros::Time query_timestamp_second_; 
       

        double ts_threshold_; 
        bool GetPoseAtTime(ros::Time t, const OdomPoseBuffer& odom_buffer, PoseCovStamped& output); 
        bool GetPosesAtTimes(ros::Time t1, ros::Time t2, const OdomPoseBuffer& odom_buffer, PoseCovStampedPair& output_poses);
        PoseCovStamped GetDeltaBetweenPoses(const PoseCovStampedPair& input_poses);
        // TODO: Unify GetDeltaBetweenPoses and CalculatePoseDelta into only one method

        // log
        // void CheckOdometryBuffer(OdomPoseBuffer& odom_buffer);
        // void PrepareFactor(OdomPoseBuffer& odom_buffer);        
        // void MakeFactor(PoseCovStampedPair pose_cov_stamped_pair);

      private:
};

#endif