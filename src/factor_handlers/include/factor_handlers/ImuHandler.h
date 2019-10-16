/*
 * Copyright Notes
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
*/

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

// Includes 
#include <factor_handlers/LampDataHandlerBase.h>
#include <sensor_msgs/Imu.h>
#include <gtsam/navigation/AttitudeFactor.h>

// Typedefs 
typedef sensor_msgs::Imu ImuMessage;
typedef geometry_msgs::Quaternion ImuOrientation; 
typedef std::map<double, ImuOrientation> ImuBuffer;
typedef gtsam::Unit3 Unit3;
typedef gtsam::AttitudeFactor AttitudeFactor;
typedef gtsam::Pose3 GtsamPose3;

class ImuHandler : public LampDataHandlerBase {
    
    friend class ImuHandlerTest;

    public:

        ImuHandler();
        ~ImuHandler();

        // Public methods
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
        int CheckImuBufferSize() const; 
        bool InsertMsgInBuffer(const ImuMessage::ConstPtr& msg) ;
        bool ClearImuBuffer();
        bool SetTimeForImuAttitude(const ros::Time& stamp);
        void ResetFactorData();
        
        // Getters 
        bool GetOrientationAtTime(const ros::Time stamp, ImuOrientation& imu_orientation) const;
 
        // Converters 
        // TODO: We should create AttitudeFactor, but LAMP expects Pose3

        // Parameters 
        std::string name_;     
        double ts_threshold_;  
        double query_stamp_;

    
    private:
        
};

#endif