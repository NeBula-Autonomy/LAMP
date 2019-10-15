/*
 * Copyright Notes
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
*/

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

// Includes 
#include <factor_handlers/LampDataHandlerBase.h>
#include <sensor_msgs/Imu.h>

// Typedefs 
typedef sensor_msgs::Imu ImuMessage;
typedef geometry_msgs::Quaternion ImuOrientation; 
typedef std::map<double, ImuOrientation> ImuBuffer;

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
        // FactorData GetData(); // TODO: Give LAMP gtsam::AttitudeFactor

    protected: 

        // Subscribers, callbacks and buffers
        ros::Subscriber imu_sub_;
        void ImuCallback(const ImuMessage::ConstPtr& msg);
        ImuBuffer imu_buffer_;

        // Utilites 
        int CheckImuBufferSize() const; 
        bool InsertMsgInBuffer(const ImuMessage::ConstPtr& msg) ;
        bool ClearImuBuffer();
        
        // Getters 
        bool GetOrientationAtTime(const ros::Time stamp, ImuOrientation& imu_orientation) const;
 
        // Converters 

        // Parameters 
        std::string name_;     
        double ts_threshold_;   
    
    private:
        
};

#endif