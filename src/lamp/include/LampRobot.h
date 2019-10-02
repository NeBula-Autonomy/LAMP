/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_ROBOT_H
#define LAMP_ROBOT_H

// Includes 
#include <lamp/LampBase.h>
#include <factor_handlers/OdometryHandler.h>

// Services

// Class Definition
class LampRobot : public LampBase {
  public:
    // typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    // Constructor
    LampRobot();

    // Destructor
    ~LampRobot();    


    // Override base class functions where needed 
    // bool Initialize();

  protected:

    // instantiate all handlers that are being used in the derived classes
    virtual bool InitializeHandlers(); 

    // load parameters from yaml files
    virtual bool LoadParameters(const ros::NodeHandle& n);

    // retrieve data from all handlers
    virtual bool CheckHandlers(); // - inside timed callback
    // TODO consider checking handlers at different frequencies

    virtual bool RegisterOnlineCallbacks(const ros::NodeHandle& n);

    virtual bool CreatePublishers(const ros::NoseHandle& n);

    void ProcessTimerCallback(const ros::TimerEvent& ev);
    // TODO - move to base class?
    ros::Timer update_rate_;


  private:
    // Overwrite base classs functions where needed


    // Factor Hanlder Wrappers
    bool ProcessOdomData(FactorData data);
    void ProcessArtifactData(FactorData data);
    void ProcessAprilData(FactorData data);
    // void ProcessUWBData(FactorData data);
    // Example use:
    // ProcessArtifactData(artifact_handler_.GetData());


    // Data Handler classes
    OdometryHandler odometry_handler_; 
    ArtifactHandler artifact_handler_;
    AprilHandler april_handler_;
    // Manual LC
    // IMU
    // TS
    // LoopClosure



    // Add new functions as needed


    // Add new variables as needed




};


#endif
