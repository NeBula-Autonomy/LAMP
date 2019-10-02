/*
 * Copyright Notes
 *
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
 *          Kamak Ebadi         (kamak.ebadi@jpl.nasa.gov)
 *          Nobuhiro Funabiki   (nobuhiro.funabiki@jpl.nasa.gov)
*/

#include <factor_handlers/OdometryHandler.h>


OdometryHandler::OdometryHandler() {
    std::cout<<"Odometry Handler Class Constructor"<<std::endl;
    }

OdometryHandler::~OdometryHandler() {
    std::cout<<"Odometry Handler Class Destructor"<<std::endl;
    }

// OdometryHandler::Initialize(const ros::NodeHandle& n){
//     ROS_INFO("Initialize Method of OdometryHandler Called"); 
//     // RegisterOnlineCallBacks(n);
// }

// void OdometryHandler::RegisterOnlineCallBacks(const ros::NodeHandler& n){

//     ROS_INFO("%s: Odometry Handler: Registering online callbacks.", name_.c_str());
    
//     // Create a local node handle to manage callback subscriptions.
//     ros::NodeHandle nl(n);
    
//     lidar_odom_sub_ =
//       nl.subscribe("LIDAR_ODOMETRY_TOPIC", 100, &OdometryHandler::LidarOdometryCallback, this);
// //     nav_msgs::Odometry::ConstPtr& msg) {
// //         ROS_INFO("LidarOdometryCallback")    
// //         geometry_msgs::Pose currentPose = msg.pose.pose 
// //         currentPose.header.stamp = msg.header.stamp; 
// //         lidar_odometry_deque_.push_back(currentPose); 
// //     }

// }



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
*/

void OdometryHandler::LidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {    
    ROS_INFO("LidarOdometryCallback");
    geometry_msgs::PoseStamped currentMsg;
    currentMsg.header = msg->header; 
    currentMsg.pose = msg->pose.pose;
    lidar_odometry_deque_.push_back(currentMsg); 
}

void OdometryHandler::VisualOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {    
    ROS_INFO("VisualOdometryCallback");
    geometry_msgs::PoseStamped currentMsg;
    currentMsg.header = msg->header; 
    currentMsg.pose = msg->pose.pose;
    visual_odometry_deque_.push_back(currentMsg); 
}

void OdometryHandler::WheelOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {    
    ROS_INFO("WheelOdometryCallback");
    geometry_msgs::PoseStamped currentMsg;
    currentMsg.header = msg->header; 
    currentMsg.pose = msg->pose.pose;
    wheel_odometry_deque_.push_back(currentMsg); 
}


// Header header
// string child_frame_id
// geometry_msgs/PoseWithCovariance pose
// geometry_msgs/TwistWithCovariance twist
