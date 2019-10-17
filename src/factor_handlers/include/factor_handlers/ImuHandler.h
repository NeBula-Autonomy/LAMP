/*
 * Copyright Notes
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
*/

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <factor_handlers/LampDataHandlerBase.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>

typedef sensor_msgs::Imu ImuMessage;
typedef Eigen::Quaterniond ImuQuaternion;
typedef std::map<double, ImuQuaternion> ImuBuffer;
typedef Eigen::Vector3d Vector3d;

using namespace gtsam;

class ImuHandler : public LampDataHandlerBase {
    
    friend class ImuHandlerTest;

    public:

        ImuHandler();
        ~ImuHandler();

        bool Initialize (const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

        FactorData GetData(); 

    protected: 

        std::string name_;  

        ros::Subscriber imu_sub_;
        void ImuCallback(const ImuMessage::ConstPtr& msg);

        ImuBuffer imu_buffer_;
        bool InsertMsgInBuffer(const ImuMessage::ConstPtr& msg) ;
        int CheckBufferSize() const; 
        bool ClearBuffer();          

        bool GetQuaternionAtTime(const ros::Time& stamp, ImuQuaternion& imu_quaternion) const; 
        double ts_threshold_;

        Vector3d QuaternionToYpr(const ImuQuaternion& imu_quaternion) const;

        Pose3AttitudeFactor CreateAttitudeFactor(const Vector3d& imu_rpy) const;              
        void ResetFactorData();                     

        bool SetTimeForImuAttitude(const ros::Time& stamp);  
        double query_stamp_;

        bool SetKeyForImuAttitude(const Symbol& key);
        Symbol query_key_;

        bool LoadCalibrationFromTfTree();
        tf::TransformListener imu_T_base_listener_;
        std::string base_frame_id_; 
        std::string imu_frame_id_;          
        Eigen::Affine3d I_T_B_;    
        Eigen::Affine3d B_T_I_; 
        Eigen::Quaterniond I_T_B_q_;  
    
    private:
        
};

#endif