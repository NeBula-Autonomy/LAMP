// Includes
#include "factor_handlers/ManualLoopClosureHandler.h"

// Constructor
ManualLoopClosureHandler::ManualLoopClosureHandler() { }

// Destructor
ManualLoopClosureHandler::~ManualLoopClosureHandler() { }


/*! \brief Initialize parameters and callbacks. 
 * n - Nodehandle
 * Returns bool
 */
bool ManualLoopClosureHandler::Initialize(const ros::NodeHandle& n){
  name_ = ros::names::append(n.getNamespace(), "ManualLoopClosureHandler");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load manual loop closure parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register manual loop closure callback.", name_.c_str());
    return false;
  }

  return true;
}

bool ManualLoopClosureHandler::LoadParameters(const ros::NodeHandle& n) {

  return true; 
}

bool ManualLoopClosureHandler::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  manual_loop_closure_sub_ = nl.subscribe(
    "manual_lc", 1000, &ManualLoopClosureHandler::ManualLoopClosureCallback, this);

  return true;
}

void ManualLoopClosureHandler::ManualLoopClosureCallback(const pose_graph_msgs::PoseGraph::ConstPtr& msg) {

  // Convert the message to factor data
  for (const pose_graph_msgs::PoseGraphEdge& edge : msg->edges) {

    // Create the new factor
    LoopClosureFactor new_factor;
    new_factor.transform = utils::EdgeMessageToPose(edge);
    new_factor.key_from = edge.key_from;
    new_factor.key_to = edge.key_to;
    new_factor.stamp = msg->header.stamp;

    // TODO handle covariances

    // Add the new factor
    factors_.factors.push_back(new_factor);
  }

}

void ManualLoopClosureHandler::ResetFactorData() {
  factors_.b_has_data = false;
  factors_.type = "manualloopclosure";
  factors_.factors.clear();
}


FactorData* ManualLoopClosureHandler::GetData() {

  LoopClosureData* output_data = new LoopClosureData(factors_);

  ResetFactorData();

  return output_data;
}