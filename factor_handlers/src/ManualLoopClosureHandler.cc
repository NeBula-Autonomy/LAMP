// Includes
#include "factor_handlers/ManualLoopClosureHandler.h"

// Constructor
ManualLoopClosureHandler::ManualLoopClosureHandler() {

  // Initialise the noise model
  gtsam::Vector6 zero;
  zero << 0, 0, 0, 0, 0, 0;
  noise_ = gtsam::noiseModel::Diagonal::Sigmas(zero);  
 }

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

  // Manual LC precisions
  if (!pu::Get("manual_lc_rot_precision", manual_lc_rot_precision_))
    return false;
  if (!pu::Get("manual_lc_trans_precision", manual_lc_trans_precision_))
    return false;

  // Load the manual LC noise model
  gtsam::Vector6 noise_vec;
  noise_vec.head<3>().setConstant(manual_lc_rot_precision_);
  noise_vec.tail<3>().setConstant(manual_lc_trans_precision_);
  noise_ = gtsam::noiseModel::Diagonal::Precisions(noise_vec);

  return true; 
}

bool ManualLoopClosureHandler::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  manual_loop_closure_sub_ = nl.subscribe(
    "manual_lc", 1000, &ManualLoopClosureHandler::ManualLoopClosureCallback, this);
  suggest_loop_closure_sub_ =
      nl.subscribe("manual_lc_suggestion",
                   1000,
                   &ManualLoopClosureHandler::SuggestLoopClosureCallback,
                   this);

  suggest_loop_closure_pub_ = nl.advertise<pose_graph_msgs::PoseGraph>(
      "suggest_loop_closures", 10, false);

  return true;
}

void ManualLoopClosureHandler::ManualLoopClosureCallback(const pose_graph_msgs::PoseGraph::ConstPtr& msg) {

  // Exit if no new data
  if (!msg->edges.size()) {
    return;
  }

  // Convert the message to factor data
  for (const pose_graph_msgs::PoseGraphEdge& edge : msg->edges) {

    // Create the new factor
    LoopClosureFactor new_factor;
    new_factor.transform = lamp_utils::MessageToPose(edge);
    new_factor.key_from = edge.key_from;
    new_factor.key_to = edge.key_to;
    new_factor.stamp = msg->header.stamp;

    // TODO handle covariances
    new_factor.covariance = noise_;

    // Add the new factor
    factors_.factors.push_back(new_factor);
  }

  // Record that new data was stored
  factors_.b_has_data = true;
}

void ManualLoopClosureHandler::SuggestLoopClosureCallback(const pose_graph_msgs::PoseGraph::ConstPtr& msg) {
  suggest_loop_closure_pub_.publish(*msg);
}

void ManualLoopClosureHandler::ResetFactorData() {
  factors_.b_has_data = false;
  factors_.type = "manualloopclosure";
  factors_.factors.clear();
}


std::shared_ptr<FactorData> ManualLoopClosureHandler::GetData() {

  std::shared_ptr<LoopClosureData> output_data = std::make_shared<LoopClosureData>(factors_);
  ResetFactorData();

  return output_data;
}
