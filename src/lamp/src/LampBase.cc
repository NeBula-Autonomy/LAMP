/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */


// Includes
#include <lamp/LampBase.h>

// #include <math.h>
// #include <ctime>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

// Constructor
LampBase::LampBase(){
     // any other things on construction 
    }

// Destructor
LampBase::~LampBase() {}

// Initialization
bool LampBase::Initialize(const ros::NodeHandle& n) {

  LoadParameters(n);
  CreatePublishers(n);
  // InitializeHandlers(n);

}

// Load Parameters
bool LampBase::LoadParameters(const ros::NodeHandle& n) {

}

// Create Publishers
bool LampBase::CreatePublishers(const ros::NodeHandle& n) {

}

// TODO might be common - check
bool LampBase::PublishPoseGraph(const ros::NodeHandle& n){
  return false;
}


bool LampBase::RegisterOnlineCallbacks(const ros::NodeHandle& n){
  ros::NodeHandle nl(n);

  slow_graph_sub_ = nl.subscribe("lamp/pose_graph_lc", 1, &LampBase::OptimizerUpdateCallback, this);

  return false;
}

bool LampBase::InitializeHandlers(const ros::NodeHandle& n){
  return false;
}

bool LampBase::CheckHandlers(){
  return false;
}

gtsam::Key LampBase::getKeyAtTime(const ros::Time& stamp) const {

  auto iterTime = stamps_keyed_.lower_bound(stamp.toSec()); // First key that is not less than timestamp 

  // TODO - interpolate - currently just take one
  double t2 = iterTime->first;

  if (iterTime == stamps_keyed_.begin()){
    ROS_WARN("Only one value in the graph - using that");
    return iterTime->second;
  }
  double t1 = std::prev(iterTime,1)->first; 

  // std::cout << "Time 1 is: " << t1 << ", Time 2 is: " << t2 << std::endl;

  gtsam::Symbol key;

  // if (t2-stamp.toSec() < stamp.toSec() - t1) {
  //   // t2 is closer - use that key
  //   // std::cout << "Selecting later time: " << t2 << std::endl;
  //   key = iterTime->second;
  // } else {
  //   // t1 is closer - use that key
  //   // std::cout << "Selecting earlier time: " << t1 << std::endl;
  //   key = std::prev(iterTime,1)->second;
  //   iterTime--;
  // }
  // // std::cout << "Key is: " << key << std::endl;
  // if (iterTime == std::prev(stamps_keyed_.begin())){
  //   ROS_WARN("Invalid time for graph (before start of graph range). Choosing next value");
  //   iterTime++;
  //   // iterTime = stamps_keyed_.begin();
  //   key = iterTime->second;
  // } else if(iterTime == stamps_keyed_.end()) {
  //   ROS_WARN("Invalid time for graph (past end of graph range). take latest pose");
  //   key = key_ -1;
  // }

  return key; 

}

void LampBase::OptimizerUpdateCallback(const pose_graph_msgs::PoseGraphConstPtr &msg) {
  
  // Process the slow graph update
  merger_.on_slow_graph_msg(msg);

  gtsam::Values new_values; 
  gtsam::Key key;

  // Update the internal LAMP graph using the one stored by the merger
  pose_graph_msgs::PoseGraph current_graph = merger_.GetCurrentGraph();

  // update the LAMP internal values_
  for (const pose_graph_msgs::PoseGraphNode &msg_node : msg->nodes) {
    // Get key 
    key = gtsam::Symbol(msg_node.key);

    gtsam::Pose3 full_pose;

    gtsam::Point3 pose_translation(msg_node.pose.position.x,
                                  msg_node.pose.position.y,
                                  msg_node.pose.position.z);
    gtsam::Rot3 pose_orientation(gtsam::Rot3::quaternion(msg_node.pose.orientation.w,
                                                  msg_node.pose.orientation.x,
                                                  msg_node.pose.orientation.y,
                                                  msg_node.pose.orientation.z));
    full_pose = gtsam::Pose3(pose_orientation, pose_translation);

    new_values.insert(key, full_pose);
  }

  // Update internal pose graph values
  values_ = new_values;
}
    