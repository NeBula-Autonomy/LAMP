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
     
        OdometryHandler();
        ~OdometryHandler();  

        //Initialize(const ros::NodeHandle& n);  
        //RegisterOnlineCallbacks(const ros::NodeHandle& n);

        // Typedefs
        typedef geometry_msgs::PoseWithCovarianceStamped PoseCovStamped;
        typedef std::pair<PoseCovStamped, PoseCovStamped> PoseCovStampedPair;
        typedef std::vector<PoseCovStamped> OdomPoseBuffer;
        typedef std::pair<ros::Time, ros::Time> TimeStampedPair;

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

        // Utilities 
        void PrepareFactor(OdomPoseBuffer& odom_buffer);
        void CheckOdometryBuffer(OdomPoseBuffer& odom_buffer);
        int CheckMyBufferSize(const OdomPoseBuffer& x); 
        double CalculatePoseDelta(OdomPoseBuffer& odom_buffer);
        void MakeFactor(PoseCovStampedPair pose_cov_stamped_pair);

        // Getters 
        std::vector<gtsam::Pose3> GetTransform(PoseCovStampedPair pose_cov_stamped_pair);
        Mat1212 GetCovariance(PoseCovStampedPair pose_cov_stamped_pair); 
        TimeStampedPair GetTimeStamps(PoseCovStampedPair pose_cov_stamped_pair);

        FactorData GetData();        
        FactorData factors_;        
        
        // TODO: Get this templated method pass the unit test 
        template <typename TYPE>
        int CheckBufferSize(std::vector<TYPE> const& x);
        

    private:    
};

#endif