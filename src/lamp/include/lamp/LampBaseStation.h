/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_BASE_STATION_H
#define LAMP_BASE_STATION_H

#include <lamp/LampBase.h>

#include <factor_handlers/ManualLoopClosureHandler.h>

// Services

// Class Definition
class LampBaseStation : public LampBase {
  public:
    // Constructor
    LampBaseStation();

    // Destructor
    ~LampBaseStation();    

    // Override base class functions where needed 
    virtual bool Initialize(const ros::NodeHandle& n, bool from_log);

  protected: 
    
    // instantiate all handlers that are being used in the derived classes
    virtual bool InitializeHandlers(const ros::NodeHandle& n); 

    // load parameters from yaml files
    virtual bool LoadParameters(const ros::NodeHandle& n);

    // retrieve data from all handlers
    virtual bool CheckHandlers(); // - inside timed callback
    // TODO consider checking handlers at different frequencies

    virtual bool RegisterCallbacks(const ros::NodeHandle& n);

    virtual bool CreatePublishers(const ros::NodeHandle& n);

    // Main update timer callback
    virtual void ProcessTimerCallback(const ros::TimerEvent& ev);

    PointCloudMapper mapper_;

  private:
    // Overwrite base classs functions where needed


    // Data Handler classes
    ManualLoopClosureHandler manual_loop_closure_handler_; 

};


#endif
