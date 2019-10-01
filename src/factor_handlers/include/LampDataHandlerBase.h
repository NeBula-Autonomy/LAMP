/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_DATA_HANDLER_BASE_H
#define LAMP_DATA_HANDLER_BASE_H

// Includes 
#include <ros/ros.h>

#include <utils/CommonStructs.h>


class LampDataHandlerBase {
  public:

    LampDataHandlerBase();
    ~LampDataHandlerBase();

    bool Initialize(const ros::NodeHandle& n);

    FactorData GetData(); 
    // TODO
    // List all the outputs and maybe create custom struct?
    // Vector maybe - for multiple factors

    // Return timestamps (for keys) and transforms
    // Odom - transform (and covariance) and two timestamps
    // Loop closure - transform (and covar) and two timestamps
    // Artifact - transform and one timestamp and artifact_key
    // April - the same
    // TS - Position and one timestamp 
    // IMU - attitude (roll and pitch) and return time stamp with input timestamp

    // [Pose, [timestamp, timestamp]]

  protected:

    // Node initialization.
    bool LoadParameters(const ros::NodeHandle& n);
    bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
    bool RegisterLogCallbacks(const ros::NodeHandle& n);
    bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
    bool CreatePublishers(const ros::NodeHandle& n);

    // Callback functions 
    // void DataCallback();

    

  private:

};