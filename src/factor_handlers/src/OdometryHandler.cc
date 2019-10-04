/*
 * Copyright Notes
 *
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
 *          Kamak Ebadi         (kamak.ebadi@jpl.nasa.gov)
 *          Nobuhiro Funabiki   (nobuhiro.funabiki@jpl.nasa.gov)
*/



// Includes
#include <factor_handlers/OdometryHandler.h>



// Constructor & Destructors 

OdometryHandler::OdometryHandler() {
    ROS_INFO("Odometry Handler Class Constructor");
    }

OdometryHandler::~OdometryHandler() {
    ROS_INFO("Odometry Handler Class Destructor");
    }


// Initialize 

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

bool LoadParameters(const ros::NodeHandle& n) {
    ROS_INFO("LoadParameters method called in OdometryHandler"); 
    // TODO: Load necessary parameters from yaml into local variables
    return true; 
}

bool OdometryHandler::RegisterCallbacks(const ros::NodeHandle& n) {
    ROS_INFO("%s: Registering online callbacks in OdometryHandler", name_.c_str());    
    ros::NodeHandle nl(n);    
    lidar_odom_sub_ = nl.subscribe("LIDAR_ODOMETRY_TOPIC", 1000, &OdometryHandler::LidarOdometryCallback, this); 
    visual_odom_sub_ = nl.subscribe("VISUAL ODOMETRY TOPIC", 1000, &OdometryHandler::VisualOdometryCallback, this); 
    wheel_odom_sub_ = nl.subscribe("WHEEL_ODOMETRY_TOPIC", 1000, &OdometryHandler::WheelOdometryCallback, this);
    return true;    
    }

// Callbacks 

void OdometryHandler::LidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {    
    ROS_INFO("LidarOdometryCallback");
    PoseCovStamped currentMsg;
    currentMsg.header = msg->header; 
    currentMsg.pose = msg->pose;
    lidar_odometry_buffer_.push_back(currentMsg); 
    CheckOdometryBuffer(lidar_odometry_buffer_);
}

void OdometryHandler::VisualOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {    
    ROS_INFO("VisualOdometryCallback");
    PoseCovStamped currentMsg;
    currentMsg.header = msg->header; 
    currentMsg.pose = msg->pose;
    visual_odometry_buffer_.push_back(currentMsg); 
    CheckOdometryBuffer(visual_odometry_buffer_);
}
void OdometryHandler::WheelOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {    
    ROS_INFO("WheelOdometryCallback");
    PoseCovStamped currentMsg;
    currentMsg.header = msg->header; 
    currentMsg.pose = msg->pose;
    wheel_odometry_buffer_.push_back(currentMsg); 
    CheckOdometryBuffer(wheel_odometry_buffer_);
}



// Utilities 

void OdometryHandler::CheckOdometryBuffer(OdomPoseBuffer& odom_buffer) {
    if (CheckBufferSize<PoseCovStamped>(odom_buffer) > 2) {
        if (CalculatePoseDelta(odom_buffer) > 1.0) {
            ROS_INFO("Moved more than 1 meter");
            std::cout<<"CheckOdometryBuffer Here"<<std::endl;
            PrepareFactor(odom_buffer);
        }
    }     
}

template <typename TYPE>
int OdometryHandler::CheckBufferSize(const std::vector<TYPE>& x) {
    return x.size();
}

double OdometryHandler::CalculatePoseDelta(OdomPoseBuffer& odom_buffer) {
    // TODO: Should be implemented in a cleaner way
    auto pose_first = gr::FromROS((*(odom_buffer.begin())).pose.pose);
    auto pose_end   = gr::FromROS((*(std::prev(odom_buffer.end()))).pose.pose);
    auto pose_delta = gu::PoseDelta(pose_first, pose_end);
    // auto pose_first_stamp = (*(odom_buffer.begin())).header.stamp;
    // auto pose_second_stamp = (*(std::prev(odom_buffer.end()))).header.stamp;    
    return pose_delta.translation.Norm();
}

void OdometryHandler::PrepareFactor(OdomPoseBuffer& odom_buffer) {
    auto first_odom_element = odom_buffer.begin();   
    auto last_odom_element = std::prev(odom_buffer.end());
    auto pose_cov_stamped_pair = std::make_pair(*first_odom_element, *last_odom_element);
    MakeFactor(pose_cov_stamped_pair);
    //ResetBuffer(last_odom_element); 
    //TODO: ResetBuffer clears the buffer and adds the last element of interest
}

void OdometryHandler::MakeFactor(PoseCovStampedPair pose_cov_stamped_pair) {
    //Makes a new factor by filling the fields of FactorData
    factors_.b_has_data = true;
    factors_.type = "odom";
    factors_.transforms.push_back(GetTransform(pose_cov_stamped_pair));
    // factors_.covariances.push_back(GetCovariance(pose_cov_stamped_pair));
    factors_.time_stamps.push_back(GetTimeStamps(pose_cov_stamped_pair));
}

gtsam::Pose3 OdometryHandler::GetTransform(PoseCovStampedPair pose_cov_stamped_pair) {
    //auto pose_delta = gu::PoseDelta(pose_cov_stamped_pair.first, pose_cov_stamped_pair.second);
    gtsam::Pose3 output;
    return output;
}

Mat1212 OdometryHandler::GetCovariance(PoseCovStampedPair pose_cov_stamped_pair) {
    std::cout<<"Needs to be implemented later" << std::endl;
    Mat1212 output; // TODO: Should be gtsam::SharedNoiseModel& bias_noise_model =  gtsam::noiseModel::Diagonal::Sigmas(biasSigmas);”
    return output;
}

std::pair<ros::Time, ros::Time> OdometryHandler::GetTimeStamps(PoseCovStampedPair pose_cov_stamped_pair) {
    std::cout<<"Needs to be implemented later" << std::endl;
    std::pair<ros::Time, ros::Time> output;
    return output;
}

FactorData OdometryHandler::GetData() {
    return factors_;
    // reset factors after this get called
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