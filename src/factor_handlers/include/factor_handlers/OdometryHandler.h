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




// Class Definition 
class OdometryHandler : public LampDataHandlerBase{



    friend class OdometryHandlerTest;
    


    public:
        
        // Constructors and Destructors
        OdometryHandler();
        ~OdometryHandler();  

        // Typedefs
        typedef geometry_msgs::PoseWithCovarianceStamped PoseCovStamped;
        typedef nav_msgs::Odometry Odometry;
        typedef std::pair<PoseCovStamped, PoseCovStamped> PoseCovStampedPair;
        typedef std::vector<PoseCovStamped> OdomPoseBuffer;
        typedef std::pair<ros::Time, ros::Time> TimeStampedPair;
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        // TODO make this common across packages somehow?

        // Public methods
        bool Initialize (const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

        // LAMP Interface
        // FactorData GetData();

        // TODO: This function should be impletented as a template function in the base class
        // TODO: For example, template <typename TYPE> GetKeyedValueAtTime(ros::Time& stamp, TYPE& msg)
        bool GetKeyedScanAtTime(ros::Time& stamp, PointCloud::Ptr& msg);

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
        template <typename T1, typename T2>
        bool InsertMsgInBuffer(const typename T1::ConstPtr& msg, std::vector<T2>& buffer);   
        template <typename T>
        int CheckBufferSize(const std::vector<T>& x) {
            std::cout << x.size() << std::endl;
            return x.size();
        }
        void CheckOdometryBuffer(OdomPoseBuffer& odom_buffer);
        double CalculatePoseDelta(OdomPoseBuffer& odom_buffer);
        void PrepareFactor(OdomPoseBuffer& odom_buffer);        
        void MakeFactor(PoseCovStampedPair pose_cov_stamped_pair);
        

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
        double ts_threshold_; 
        bool GetPoseAtTime(ros::Time t, const OdomPoseBuffer& odom_buffer, PoseCovStamped& output); 
        bool GetPosesAtTimes(ros::Time t1, ros::Time t2, const OdomPoseBuffer& odom_buffer, PoseCovStampedPair& output_poses);
        PoseCovStamped GetDeltaBetweenPoses(const PoseCovStampedPair& input_poses);
        // TODO: Unify GetDeltaBetweenPoses and CalculatePoseDelta in only one function

      private:
};

#endif