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

using namespace gtsam;

class ImuHandler : public LampDataHandlerBase {
    
  friend class ImuHandlerTest;

public:

  ImuHandler();
  ~ImuHandler();

  bool Initialize (const ros::NodeHandle& n);
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // LAMP Interface
  virtual std::shared_ptr<FactorData> GetData(); 
  bool SetTimeForImuAttitude(const ros::Time& stamp);
  bool SetKeyForImuAttitude(const Symbol& key);
  
protected: 

  std::string name_;  
  bool b_verbosity_;

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
  Eigen::Vector3d QuaternionToYpr(const geometry_msgs::Quaternion& imu_quaternion) const;
  
  // Factors
  Pose3AttitudeFactor CreateAttitudeFactor(const Eigen::Vector3d& imu_ypr) const;              
  void ResetFactorData();  
  double query_stamp_;
  Symbol query_key_;
  ImuData factors_;
  double noise_sigma_imu_;

  // Transformations
  bool b_convert_imu_frame_;
  bool LoadCalibrationFromTfTree();
  tf::TransformListener imu_T_base_listener_;
  std::string base_frame_id_; 
  std::string imu_frame_id_;          
  Eigen::Affine3d I_T_B_;    
  Eigen::Affine3d B_T_I_; 
  Eigen::Quaterniond I_T_B_q_;  
  
  // Check IMU for NANS
  bool CheckNans(const ImuMessage &msg);
};

#endif