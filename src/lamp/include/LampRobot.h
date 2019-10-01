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


  private:
    // Overwrite base classs functions where needed

    virtual bool InitializeHandlers();


    // Factor Hanlder Wrappers
    void ProcessOdomData(FactorData data);
    void ProcessArtifactData(FactorData data);
    void ProcessAprilData(FactorData data);
    void ProcessUWBData(FactorData data);
    // Example use:
    // ProcessArtifactData(artifact_handler_.GetData());


    // Data Handler classes
    // OdometryHandler odometry_handler_; 
    // ArtifactHandler artifact_handler_;



    // Add new functions as needed


    // Add new variables as needed




};


#endif
