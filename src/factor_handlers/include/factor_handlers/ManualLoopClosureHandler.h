/*
 * Copyright Notes
 *
 * Authors: Alex Stephens    (alex.stephens@jpl.nasa.gov)
 */

#ifndef MANUAL_LOOP_CLOSURE_HANDLER_H
#define MANUAL_LOOP_CLOSURE_HANDLER_H

// Includes 
#include <factor_handlers/LampDataHandlerBase.h>

namespace gu = geometry_utils;
namespace gr = geometry_utils::ros;
namespace pu = parameter_utils;

class ManualLoopClosureHandler : public LampDataHandlerBase {

  public:

    ManualLoopClosureHandler();
    virtual ~ManualLoopClosureHandler();

    virtual bool Initialize(const ros::NodeHandle& n);
    virtual FactorData* GetData();

  protected:

    // Node initialization.
    bool LoadParameters(const ros::NodeHandle& n);
    bool RegisterCallbacks(const ros::NodeHandle& n);
    bool CreatePublishers(const ros::NodeHandle& n);

    // Reset Factor data
    void ResetFactorData();

  protected:

    // The node's name.
    std::string name_;

    // Main subscriber 
    ros::Subscriber manual_loop_closure_sub_;

    // Main callback 
    void ManualLoopClosureCallback(const pose_graph_msgs::PoseGraph::ConstPtr& msg);

    // Factor data
    LoopClosureData factors_; 

    // Precisions
    double manual_lc_rot_precision_;
    double manual_lc_trans_precision_;
    
    // Manual LC noise
    gtsam::SharedNoiseModel noise_;



  private:

};

#endif
