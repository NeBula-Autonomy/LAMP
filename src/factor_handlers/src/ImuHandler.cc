/*
 * Copyright Notes
 *
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
*/

// Includes
#include <factor_handlers/ImuHandler.h>

namespace pu = parameter_utils;

// Constructor and Destructor ---------------------------------------------------------------------------

ImuHandler::ImuHandler() 
  : ts_threshold_(0.0) {
    ROS_INFO("ImuHandler Class Constructor");
}

ImuHandler::~ImuHandler() {
    ROS_INFO("ImuHandler Class Destructor");
}

// Initialize -------------------------------------------------------------------------------------------

bool ImuHandler::Initialize(const ros::NodeHandle& n){

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

    // Parameter used in GetOrientationAtTime method
    if (!pu::Get("ts_threshold_", ts_threshold_))
        return false;

    return true;
}

bool ImuHandler::RegisterCallbacks(const ros::NodeHandle& n) {
    
    ROS_INFO("%s: Registering online callbacks in ImuHandler", name_.c_str());
  
    ros::NodeHandle nl(n);

    imu_sub_ = nl.subscribe("imu_topic", 1000, &ImuHandler::ImuCallback, this); 
    
    return true;
}

void ImuHandler::ImuCallback(const ImuMessage::ConstPtr& msg) {
    
    ROS_INFO("ImuHandler - ImuCallback");
    
    if (!InsertMsgInBuffer(msg)){
        ROS_WARN("ImuHandler - ImuCallback - Unable to store message in buffer");
    }     

}

bool ImuHandler::InsertMsgInBuffer(const ImuMessage::ConstPtr& msg) {    
    
    auto initial_size = imu_buffer_.size();
    
    auto current_time = msg->header.stamp.toSec();    
    ImuOrientation current_orientation = msg->orientation;  

    // TODO: We maybe should be storing also linear_acceleration and linear_acceleration_covariance  
    
    imu_buffer_.insert({current_time, current_orientation});     
    
    auto final_size = imu_buffer_.size();    
    if (final_size == (initial_size+1)) {
        // Msg insertion was successfull, return true to the caller
        return true;
    }
    else {
        return false; 
    } 

}

int ImuHandler::CheckImuBufferSize() const {
    
    ROS_INFO("ImuCallback - ChechImuBufferSize");
    
    return imu_buffer_.size();

}

bool ImuHandler::ClearImuBuffer() {

    ROS_INFO("ImuHandler - ClearImuBuffer");
    
    imu_buffer_.clear();
    
    if (CheckImuBufferSize()==0) {
        ROS_INFO("Successfully cleared Imu Buffer");
        return true;
    }
    else {
        ROS_WARN("Could not clear the Imu Buffer");
        return false; 
    }

}

bool ImuHandler::GetOrientationAtTime(const ros::Time stamp, ImuOrientation& imu_orientation) const {

    // TODO: Implement generic GetValueAtTime in base class as it is a common need by all handlers

    ROS_INFO("ImuHandler - GetOrientationAtTime"); 

    // If map is empty, return false to the caller 
    if (imu_buffer_.size() == 0){
        return false;
    }

    // Given the input timestamp, search for lower bound (first entry that is not less than the given timestamp)
    auto itrTime = imu_buffer_.lower_bound(stamp.toSec());
    auto time2 = itrTime->first;
    double time_diff;

    // If this gives the start of the buffer, then take that ImuOrientation 
    if (itrTime == imu_buffer_.begin()) {
        imu_orientation = itrTime->second;
        time_diff = itrTime->first - stamp.toSec();
        ROS_WARN("Timestamp before the start of the imu buffer");
        ROS_INFO_STREAM("time diff is: " << time_diff);
    } 
    else if (itrTime == imu_buffer_.end()) {
        // Check if it is past the end of the buffer - if so, then take the last ImuOrientation 
        ROS_WARN("Timestamp past the end of the imu buffer");
        itrTime--;
        imu_orientation = itrTime->second;
        time_diff = stamp.toSec() - itrTime->first;
        ROS_INFO_STREAM("input time is " << stamp.toSec()
                                        << "s, and latest time is "
                                        << itrTime->first << " s"
                                        << " diff is " << time_diff);
    } 
    else {
        // Otherwise step back by 1 to get the time before the input time (time1, stamp, time2)
        double time1 = std::prev(itrTime, 1)->first;

        // If closer to time2, then use that
        if (time2 - stamp.toSec() < stamp.toSec() - time1) {
            imu_orientation = itrTime->second;
            time_diff = time2 - stamp.toSec();
        } 
        else {
            // Otherwise use time1
            imu_orientation = std::prev(itrTime, 1)->second;
            time_diff = stamp.toSec() - time1;
        }
    }

    // Check if the time difference is too large
    if (time_diff > ts_threshold_) { 
        ROS_WARN("Time difference between request and latest ImuOrientation is too large, returning no ImuOrientation");
        ROS_INFO_STREAM("Time difference is "
                        << time_diff << "s, threshold is: " << ts_threshold_);
        return false;
    } 
    
  return true; 

}

bool ImuHandler::SetTimeForImuAttitude(const ros::Time& stamp) {
    
    // TODO: Could return void and throw an exception if insertion is unsuccessful
    
    ROS_INFO("ImuHandler - SetTimeForImuAttitude");
    
    query_stamp_ = stamp.toSec();
    
    if (query_stamp_ == stamp.toSec()) {
        return true;
    }
    else {
        ROS_WARN("Could not store received stamp into private class member");
        return false;
    }

}




/*
Datatype documentation 

sensor_msgs::Imu
    std_msgs/Header header
    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance
    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance
    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance

NOTE: With ImuOrientation we refer to ImuQuaternion 

geometry_msgs::Quaternion
    float64 x
    float64 y
    float64 z
    float64 w

At the current point, I: 
    - Subscribe to imu_topic 
    - Store all imu_messages into a map buffer 
    - Get the value of interest by timestamp based search 
    - Once the value of interest has been retrieved, fill factor data for LAMP and return 
    - lamp

Benjamin 

    For IMU the idea is that the struct would contain the attitude information in the transform field (only using the orientation part) 
    Then the ProcessIMUData() in lamp would convert that to a factor
    But we would need to call something before this to set the time:
        imu_handler_.SetTimeForIMUAttitude(stamp)
        Imu_handler_.GetData()

*/