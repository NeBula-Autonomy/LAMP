/*
 * Copyright Notes
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
*/

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

// Includes 
#include <factor_handlers/LampDataHandlerBase.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_msg.h>

// Typedefs 
typedef sensor_msgs::Imu ImuMessage;
typedef geometry_msgs::Quaternion ImuQuaternion; 
typedef std::map<double, ImuQuaternion> ImuBuffer;

using namespace gtsam;

class ImuHandler : public LampDataHandlerBase {
    
    friend class ImuHandlerTest;

    public:

        // Constructor & Destructor
        ImuHandler();
        ~ImuHandler();

        // Initialization 
        bool Initialize (const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

        // LAMP Interface 
        FactorData GetData(); 

    protected: 

        // Subscribers, callbacks and buffers
        ros::Subscriber imu_sub_;
        void ImuCallback(const ImuMessage::ConstPtr& msg);
        ImuBuffer imu_buffer_;

        // Utilites 
        int CheckBufferSize() const; 
        bool InsertMsgInBuffer(const ImuMessage::ConstPtr& msg) ;
        bool ClearBuffer();
        void ResetFactorData();
        Pose3AttitudeFactor CreateAttitudeFactor(const Eigen::Vector3d& imu_rpy) const;

        // Setters
        bool SetTimeForImuAttitude(const ros::Time& stamp);
        bool SetKeyForImuAttitude(const Symbol& key);

        // Getters 
        bool GetQuaternionAtTime(const ros::Time& stamp, ImuQuaternion& imu_quaternion) const;

        // Parameters 
        std::string name_;     
        double ts_threshold_;  
        double query_stamp_;
        Symbol query_key_;

        // Converters          
        Eigen::Vector3d QuaternionToYpr(const ImuQuaternion& imu_quaternion) const;
        // imu_frame to base_frame conversion 
        std::string base_frame_id_; 
        std::string imu_frame_id_;          
        tf::TransformListener imu_T_laser_listener_;
        Eigen::Affine3d I_T_B_;    
        Eigen::Affine3d B_T_I_; 
        Eigen::Quaterniond I_T_B_q_;   
        bool LoadCalibrationFromTfTree();
    
    private:
        
};

#endif