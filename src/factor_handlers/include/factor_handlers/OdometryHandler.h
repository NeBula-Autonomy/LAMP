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

        typedef std::vector<geometry_msgs::PoseWithCovarianceStamped> OdomPoseBuffer;

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
        std::vector<geometry_msgs::PoseWithCovarianceStamped> lidar_odometry_buffer_; 
        std::vector<geometry_msgs::PoseWithCovarianceStamped> visual_odometry_buffer_;
        std::vector<geometry_msgs::PoseWithCovarianceStamped> wheel_odometry_buffer_;

        // Odometry Storages Size Computer 
        int CheckMyBufferSize(const std::vector<geometry_msgs::PoseWithCovarianceStamped>& x);
        
        // Calculate Delta between two Poses 
        double CalculatePoseDelta(std::vector<geometry_msgs::PoseWithCovarianceStamped>& odom_buffer);
    
        // Utilities to check local buffer sizes
        template <typename TYPE>
        int CheckBufferSize(std::vector<TYPE> const& x);

    private:    
};

#endif