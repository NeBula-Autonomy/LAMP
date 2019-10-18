/*
 * Copyright Notes
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
*/

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

// Includes
#include <factor_handlers/LampDataHandlerBase.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>

// Typedefs
typedef sensor_msgs::Imu ImuMessage;
typedef Eigen::Quaterniond ImuQuaternion;
typedef std::map<double, ImuQuaternion> ImuBuffer;
typedef Eigen::Vector3d Vector3d;

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
  FactorData* GetData(); 

protected: 

  std::string name_;  

  // Subscriber & Callback 
  ros::Subscriber imu_sub_;
  void ImuCallback(const ImuMessage::ConstPtr& msg);

  // Buffers
  ImuBuffer imu_buffer_;
  int buffer_size_limit_;
  bool InsertMsgInBuffer(const ImuMessage::ConstPtr& msg) ;
  int CheckBufferSize() const; 
  bool ClearBuffer();          

  // Quaternions
  double ts_threshold_;
  bool GetQuaternionAtTime(const ros::Time& stamp, ImuQuaternion& imu_quaternion) const; 
  Vector3d QuaternionToYpr(const ImuQuaternion& imu_quaternion) const;

  // Factors
  Pose3AttitudeFactor CreateAttitudeFactor(const Vector3d& imu_rpy) const;              
  void ResetFactorData();  
  bool SetTimeForImuAttitude(const ros::Time& stamp);  
  double query_stamp_;
  bool SetKeyForImuAttitude(const Symbol& key);
  Symbol query_key_;

  // Transformations
  bool LoadCalibrationFromTfTree();
  tf::TransformListener imu_T_base_listener_;
  std::string base_frame_id_; 
  std::string imu_frame_id_;          
  Eigen::Affine3d I_T_B_;    
  Eigen::Affine3d B_T_I_; 
  Eigen::Quaterniond I_T_B_q_;  

  // IMU output data
  ImuData factors_;

        
};

#endif