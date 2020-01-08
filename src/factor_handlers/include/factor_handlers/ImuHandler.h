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
typedef Eigen::Quaterniond ImuQuaternion; // w,x,y,z
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
  virtual std::shared_ptr<FactorData> GetData(); 
  bool SetTimeForImuAttitude(const ros::Time& stamp);
  bool SetKeyForImuAttitude(const Symbol& key);
  
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
  double query_stamp_;
  Symbol query_key_;
  ImuData factors_;

  // Transformations
  bool LoadCalibrationFromTfTree();
  tf::TransformListener imu_T_base_listener_;
  std::string base_frame_id_; 
  std::string imu_frame_id_;          
  Eigen::Affine3d I_T_B_;    
  Eigen::Affine3d B_T_I_; 
  Eigen::Quaterniond I_T_B_q_;  

  bool b_convert_imu_frame_;
  bool b_verbosity_;
  double noise_sigma_imu_;

  // New TF based quaternion to angle conversion 
  std::vector<double> QuaternionToYprNew(const geometry_msgs::Quaternion& imu_quaternion) const;
  Pose3AttitudeFactor CreateAttitudeFactorNew(const std::vector<double>& imu_ypr) const;
        
};

#endif