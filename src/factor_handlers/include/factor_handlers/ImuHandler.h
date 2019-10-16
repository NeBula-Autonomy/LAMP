/*
 * Copyright Notes
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
*/

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

// Includes 
#include <factor_handlers/LampDataHandlerBase.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <sensor_msgs/Imu.h>

// Typedefs 
typedef sensor_msgs::Imu ImuMessage;
typedef geometry_msgs::Quaternion ImuOrientation; 
typedef std::map<double, ImuOrientation> ImuBuffer;
typedef gtsam::Symbol Symbol;
typedef gtsam::Unit3 Unit3;
typedef gtsam::Pose3AttitudeFactor Pose3AttitudeFactor; 

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
        Pose3AttitudeFactor CreateAttitudeFactor(const ImuOrientation& imu_orientation) const;

        

        // Setters
        bool SetTimeForImuAttitude(const ros::Time& stamp);
        bool SetKeyForImuAttitude(const Symbol& key);

        // Getters 
        bool GetOrientationAtTime(const ros::Time& stamp, ImuOrientation& imu_orientation) const;

        // Parameters 
        std::string name_;     
        double ts_threshold_;  
        double query_stamp_;
        Symbol query_key_;
    
    private:
        
};

#endif