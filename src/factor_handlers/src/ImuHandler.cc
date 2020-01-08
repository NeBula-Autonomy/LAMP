/*
 * Copyright Notes
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
 */

// Includes
#include <factor_handlers/ImuHandler.h>

namespace pu = parameter_utils;

// Constructor and Destructor -------------------------------------------------------------

ImuHandler::ImuHandler() {
    ROS_INFO("ImuHandler Class Constructor");
}

ImuHandler::~ImuHandler() {
    ROS_INFO("ImuHandler Class Destructor");
}

// Initialization -------------------------------------------------------------------------

bool ImuHandler::Initialize(const ros::NodeHandle& n) {
    ROS_INFO("ImuHandler - Initialize");    
    name_ = ros::names::append(n.getNamespace(), "ImuHandler");
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

bool ImuHandler::LoadParameters(const ros::NodeHandle& n) {    
    ROS_INFO("ImuHandler - LoadParameters");    
    if (!pu::Get("imu/buffer_size_limit", buffer_size_limit_))
        return false;
    if (!pu::Get("imu/ts_threshold", ts_threshold_))
        return false;
    if (!pu::Get("imu/base_frame_id", base_frame_id_))
        return false;
    if (!pu::Get("imu/imu_frame_id", imu_frame_id_)) 
        return false;
    if (!pu::Get("imu/b_convert_imu_frame", b_convert_imu_frame_)) 
        return false;
    if (!pu::Get("imu/b_verbosity", b_verbosity_))
        return false;
    if (!pu::Get("noise_sigma_imu", noise_sigma_imu_)) 
        return false;    
    LoadCalibrationFromTfTree();
    return true;
}

bool ImuHandler::RegisterCallbacks(const ros::NodeHandle& n) {    
    ROS_INFO("%s: Registering online callbacks in ImuHandler", name_.c_str());  
    ros::NodeHandle nl(n);
    imu_sub_ = nl.subscribe("imu_topic", 1000, &ImuHandler::ImuCallback, this);     
    return true;
}

// Callback -------------------------------------------------------------------------------

void ImuHandler::ImuCallback(const ImuMessage::ConstPtr& msg) {    
    if (b_verbosity_) ROS_INFO("ImuHandler - ImuCallback"); 
    if (CheckBufferSize() > buffer_size_limit_){
        imu_buffer_.erase(imu_buffer_.begin());
    }   
    if (!InsertMsgInBuffer(msg)){
        ROS_WARN("ImuHandler - ImuCallback - Unable to store message in buffer");
    }
}

// LAMP Interface -------------------------------------------------------------------------

std::shared_ptr<FactorData> ImuHandler::GetData() {
    if (b_verbosity_) ROS_INFO("ImuHandler - GetData"); 
    std::shared_ptr<ImuData> factors_output = std::make_shared<ImuData>(factors_);    
    factors_output->b_has_data = false; 
    if (CheckBufferSize()==0) {
        ROS_WARN("Buffers are empty, returning no data");
        return factors_output;
    }
    ImuQuaternion imu_quaternion;
    ros::Time query_stamp_ros;
    query_stamp_ros.fromSec(query_stamp_);
    if (GetQuaternionAtTime(query_stamp_ros, imu_quaternion)==true){        
        if (b_verbosity_) ROS_INFO("Successfully extracted data from buffer");
        
        geometry_msgs::Quaternion imu_quaternion_msg; 
        tf::quaternionEigenToMsg(imu_quaternion, imu_quaternion_msg);
        auto imu_ypr = QuaternionToYpr(imu_quaternion_msg);        
        
        ImuFactor new_factor(CreateAttitudeFactor(imu_ypr));
        factors_output->b_has_data = true; 
        factors_output->type = "imu";        
        factors_output->factors.push_back(new_factor);
        ResetFactorData();
    }
    else {
        factors_output->b_has_data = false; 
    }
    return factors_output; 
}

// Buffers --------------------------------------------------------------------------------

bool ImuHandler::InsertMsgInBuffer(const ImuMessage::ConstPtr& msg) {  
    if (b_verbosity_) ROS_INFO("ImuHandler - InsertMsgInBuffer");  
    auto initial_size = imu_buffer_.size();    
    double current_time = msg->header.stamp.toSec();    
    ImuQuaternion current_quaternion;
    tf::quaternionMsgToEigen(msg->orientation, current_quaternion);
    if (b_convert_imu_frame_) {
        current_quaternion = I_T_B_q_*current_quaternion*I_T_B_q_.inverse(); 
    }
    imu_buffer_.insert({current_time, current_quaternion});
    auto final_size = imu_buffer_.size();    
    if (final_size == (initial_size+1)) {
        return true;
    }
    else {
        return false; 
    } 
}

int ImuHandler::CheckBufferSize() const {    
    if (b_verbosity_) ROS_INFO("ImuHandler - ChechBufferSize");    
    return imu_buffer_.size();
}

bool ImuHandler::ClearBuffer() {
    if (b_verbosity_) ROS_INFO("ImuHandler - ClearBuffer");    
    imu_buffer_.clear();    
    if (CheckBufferSize()==0) {
        if (b_verbosity_) ROS_INFO("Successfully cleared buffer");
        return true;
    }
    else {
        ROS_WARN("Could not clear buffer");
        return false; 
    }
}

// Quaternions ----------------------------------------------------------------------------

bool ImuHandler::GetQuaternionAtTime(const ros::Time& stamp, ImuQuaternion& imu_quaternion) const {
    // TODO: Implement GetValueAtTime in base class as common functionality needed by all handlers
    // TODO: ClearPreviousImuMsgsInBuffer where needed 
    if (b_verbosity_) ROS_INFO("ImuHandler - GetQuaternionAtTime"); 
    if (imu_buffer_.size() == 0){
        return false;
    }
    auto itrTime = imu_buffer_.lower_bound(stamp.toSec());
    auto time2 = itrTime->first;
    double time_diff;    
    if (itrTime == imu_buffer_.begin()) {
        imu_quaternion = itrTime->second;
        time_diff = itrTime->first - stamp.toSec();
        ROS_WARN("itrTime points to imu_buffer_ begin");
    }
    else if (itrTime == imu_buffer_.end()) {
        itrTime--;
        imu_quaternion = itrTime->second;
        time_diff = stamp.toSec() - itrTime->first;
        ROS_WARN("itrTime points to imu_buffer_ end");
    }
    else {
        double time1 = std::prev(itrTime, 1)->first;
        if (time2 - stamp.toSec() < stamp.toSec() - time1) {
            imu_quaternion = itrTime->second;
            time_diff = time2 - stamp.toSec();
        } 
        else {
            imu_quaternion = std::prev(itrTime, 1)->second;
            time_diff = stamp.toSec() - time1;
        }
    }
    if (fabs(time_diff) > ts_threshold_) { 
        ROS_WARN_STREAM("Time difference is "
                        << time_diff << "s, threshold is: " << ts_threshold_ << ", returning false");
        return false;
    }     
  return true; 
}

Eigen::Vector3d ImuHandler::QuaternionToYpr(const geometry_msgs::Quaternion& imu_quaternion) const {
    if (b_verbosity_) ROS_INFO("ImuHandler - QuaternionToYpr");
    tf::Quaternion q(imu_quaternion.x, imu_quaternion.y, imu_quaternion.z, imu_quaternion.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    Eigen::Vector3d imu_ypr(yaw, pitch, roll);
    return imu_ypr; // radians
}

// Factors --------------------------------------------------------------------------------

Pose3AttitudeFactor ImuHandler::CreateAttitudeFactor(const Eigen::Vector3d& imu_ypr) const {
    // TODO: Make sure Rot3::Ypr works with input imu_ypr data expressed in radians
    if (b_verbosity_) ROS_INFO("ImuHandler - CreateAttitudeFactor");
    Unit3 ref(0, 0, -1); 
    SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, noise_sigma_imu_);    
    Rot3 R_imu = Rot3::Ypr(0, double(imu_ypr[1]), double(imu_ypr[2])); // Yaw can be set to 0
    Unit3 meas = Unit3(Rot3(R_imu.transpose()).operator*(ref));
    Pose3AttitudeFactor factor(query_key_, meas, model, ref);
    return factor;
}

void ImuHandler::ResetFactorData() {
    if (b_verbosity_) ROS_INFO("ImuHandler - ResetFactorData");
    factors_.b_has_data = false;
    factors_.type = "imu";
    factors_.factors.clear();
}

bool ImuHandler::SetTimeForImuAttitude(const ros::Time& stamp) {        
    if (b_verbosity_) ROS_INFO("ImuHandler - SetTimeForImuAttitude");    
    query_stamp_ = stamp.toSec();    
    if (query_stamp_ == stamp.toSec()) {
        return true;
    }
    else {
        ROS_WARN("Could not store received stamp in protected class member");
        return false;
    }
}

bool ImuHandler::SetKeyForImuAttitude(const Symbol& key) {        
    if (b_verbosity_) ROS_INFO("ImuHandler - SetKeyForImuAttitude");    
    query_key_ = key;    
    if (query_key_ == key) {
        return true;
    }
    else {
        ROS_WARN("Could not store received symbol into protected class member");
        return false;
    }
}

// Transformations ------------------------------------------------------------------------

bool ImuHandler::LoadCalibrationFromTfTree() {
    ROS_WARN_DELAYED_THROTTLE(2.0, 
                            "Waiting for \'%s\' and \'%s\' to appear in tf_tree...",
                            imu_frame_id_,
                            base_frame_id_);
    tf::StampedTransform imu_T_base_transform;
    try {   
        imu_T_base_listener_.waitForTransform(
            imu_frame_id_,
            base_frame_id_,
            ros::Time(0),
            ros::Duration(2.0));
      
        imu_T_base_listener_.lookupTransform(
            imu_frame_id_,
            base_frame_id_,
            ros::Time(0),
            imu_T_base_transform);

        geometry_msgs::TransformStamped imu_T_base_tmp_msg;
        tf::transformStampedTFToMsg(imu_T_base_transform, imu_T_base_tmp_msg);        
        tf::transformMsgToEigen(imu_T_base_tmp_msg.transform, I_T_B_);
        B_T_I_ = I_T_B_.inverse();
        ROS_INFO_STREAM("Loaded pose_sensor to imu calibration B_T_L:");
        std::cout << I_T_B_.translation() << std::endl;
        std::cout << I_T_B_.rotation() << std::endl;        
        I_T_B_q_ = Eigen::Quaterniond(I_T_B_.rotation());
        ROS_INFO("q: x: %.3f, y: %.3f, z: %.3f, w: %.3f", I_T_B_q_.x(), I_T_B_q_.y(), I_T_B_q_.z(), I_T_B_q_.w());
        return true; 
    }     
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        I_T_B_ = Eigen::Affine3d::Identity();
        B_T_I_ = Eigen::Affine3d::Identity();
        return false;
    }
}