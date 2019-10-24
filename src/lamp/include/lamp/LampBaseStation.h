/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_BASE_STATION_H
#define LAMP_BASE_STATION_H

#include <lamp/LampBase.h>

#include <factor_handlers/ManualLoopClosureHandler.h>
#include <factor_handlers/PoseGraphHandler.h>


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
    
    // Instantiate all handlers that are being used in the derived classes
    virtual bool InitializeHandlers(const ros::NodeHandle& n); 

    // Load parameters from yaml files
    virtual bool LoadParameters(const ros::NodeHandle& n);

    // Create subscribers
    virtual bool RegisterCallbacks(const ros::NodeHandle& n);

    // Create publishers
    virtual bool CreatePublishers(const ros::NodeHandle& n);

    // retrieve data from all handlers
    virtual bool CheckHandlers(); // - inside timed callback

    // Initialize the base station pose graph
    virtual bool InitializeGraph();

    // Main update timer callback
    virtual void ProcessTimerCallback(const ros::TimerEvent& ev);

    // Robots that the base station subscribes to
    std::vector<std::string> robot_names_;

    // Factor handler wrappers
    bool ProcessPoseGraphData(FactorData* data);

    // Data handler classes
    PoseGraphHandler pose_graph_handler_;


  private:
    // Overwrite base classs functions where needed

    double zero_noise_;


    // Data Handler classes
    ManualLoopClosureHandler manual_loop_closure_handler_; 

};


#endif
