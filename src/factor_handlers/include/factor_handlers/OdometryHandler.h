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
        typedef std::pair<PoseCovStamped, PoseCovStamped> PoseCovStampedPair;
        typedef std::vector<PoseCovStamped> OdomPoseBuffer;
        typedef std::pair<ros::Time, ros::Time> TimeStampedPair;
        
        // Public methods
        bool Initialize (const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);



    protected: 

        // Odometry Subscribers 
        ros::Subscriber lidar_odom_sub_;
        ros::Subscriber visual_odom_sub_;
        ros::Subscriber wheel_odom_sub_;

        // Odometry Callbacks 
        void LidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg); 
        void VisualOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void WheelOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

        // Odometry Storages 
        OdomPoseBuffer lidar_odometry_buffer_; 
        OdomPoseBuffer visual_odometry_buffer_;
        OdomPoseBuffer wheel_odometry_buffer_;

        // Protected methods
        void CheckOdometryBuffer(OdomPoseBuffer& odom_buffer);
        template <typename TYPE>
        int CheckBufferSize(const std::vector<TYPE>& x);
        double CalculatePoseDelta(OdomPoseBuffer& odom_buffer);
        void PrepareFactor(OdomPoseBuffer& odom_buffer);        
        void MakeFactor(PoseCovStampedPair pose_cov_stamped_pair);

        // Getters 
        gtsam::Pose3 GetTransform(PoseCovStampedPair pose_cov_stamped_pair);        
        Mat1212 GetCovariance(PoseCovStampedPair pose_cov_stamped_pair); 
        std::pair<ros::Time, ros::Time> GetTimeStamps(PoseCovStampedPair pose_cov_stamped_pair);

        // Conversion utilities 
        gtsam::Pose3 ToGtsam(const gu::Transform3& pose) const; // TODO: This function should be defined in the base class
 
        // LAMP Interface
        FactorData factors_; 
        FactorData GetData();        

        // The node's name.
        std::string name_;
              

    private:    
    
};

#endif