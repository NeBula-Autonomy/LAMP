/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */


// Includes
#include <lamp/LampBaseStation.h>

// #include <math.h>
// #include <ctime>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

// Constructor (if there is override)
LampBaseStation::LampBaseStation() {}

//Destructor
LampBaseStation::~LampBaseStation() {}

// Initialization - override for Base Station Setup
bool LampBaseStation::Initialize(const ros::NodeHandle& n, bool from_log) {

  // Get the name of the process
  name_ = ros::names::append(n.getNamespace(), "LampBaseStation");

  if (!mapper_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize mapper.", name_.c_str());
    return false;
  }

  // Add load params etc
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  // Register Callbacks
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Publishers
  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to create publishers.", name_.c_str());
    return false;
  }

  // Init Handlers
  if (!InitializeHandlers(n)){
    ROS_ERROR("%s: Failed to initialize handlers.", name_.c_str());
    return false;
  }

}

bool LampBaseStation::LoadParameters(const ros::NodeHandle& n) {

  // Names of all robots for base station to subscribe to
  if (!pu::Get("robot_names", robot_names_)) {
  ROS_ERROR("%s: No robot names provided to base station.", name_.c_str());
    return false;
  }
  else {
    for (auto s : robot_names_) {
      ROS_INFO_STREAM("Registered new robot: " << s);
    }
  }

  // TODO : separate rate for base and robot
  if (!pu::Get("rate/update_rate", update_rate_))
    return false;

  // Initialize frame IDs
  pose_graph_.fixed_frame_id = "world";


  return true;
}

bool LampBaseStation::RegisterCallbacks(const ros::NodeHandle& n) {

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Create the timed callback
  update_timer_ =
      nl.createTimer(update_rate_, &LampBaseStation::ProcessTimerCallback, this);

  return true; 
}

bool LampBaseStation::CreatePublishers(const ros::NodeHandle& n) {

  // Creates pose graph publishers in base class
  LampBase::CreatePublishers(n);

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Base station publishers
  pose_graph_to_optimize_pub_ = nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph_to_optimize", 10, false);
  keyed_scan_pub_ = nl.advertise<pose_graph_msgs::KeyedScan>("keyed_scans", 10, false);

  return true; 
}

bool LampBaseStation::InitializeHandlers(const ros::NodeHandle& n){
  
  // Manual loop closure handler
  if (!manual_loop_closure_handler_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize the manual loop closure handler.", name_.c_str());
    return false;
  }

  // Pose graph handler -- requires robot_names parameter loaded
  if (robot_names_.size() == 0) {
    ROS_ERROR("%s: No robots for base station to subscribe to.", name_.c_str());
    return false;
  }
  else if (!pose_graph_handler_.Initialize(n, robot_names_)) {
    ROS_ERROR("%s: Failed to initialize the pose graph handler.", name_.c_str());
    return false;
  }
  
  return true; 
}

bool LampBaseStation::InitializeGraph() {

  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3(1,0,0,0), 
                                   gtsam::Point3(0,0,0));
  gtsam::Vector6 zero;
  zero << 0, 0, 0, 0, 0, 0;
  gtsam::noiseModel::Diagonal::shared_ptr covariance =        
      gtsam::noiseModel::Diagonal::Sigmas(zero);

  pose_graph_.Initialize(initial_key_, pose, covariance);

  return true;
}


void LampBaseStation::ProcessTimerCallback(const ros::TimerEvent& ev) {

  // Check the handlers
  CheckHandlers();

  // Publish the pose graph
  if (b_has_new_factor_) {
    ROS_INFO_STREAM("Publishing pose graph and map");
    PublishPoseGraph();

    // Update and publish the map
    // GenerateMapPointCloud();
    mapper_.PublishMap();

    b_has_new_factor_ = false;
  }

  // Start optimize, if needed
  if (b_run_optimization_) {
      ROS_INFO_STREAM("Publishing pose graph to optimizer");
      PublishPoseGraphForOptimizer();

      b_run_optimization_ = false; 
  }

  // Publish anything that is needed 
}

bool LampBaseStation::ProcessPoseGraphData(FactorData* data) {
  // ROS_INFO_STREAM("In ProcessPoseGraphData");

  // Extract pose graph data
  PoseGraphData* pose_graph_data = dynamic_cast<PoseGraphData*>(data);

  // Check if there are new pose graphs
  if (!pose_graph_data->b_has_data) {
    return false; 
  }

  ROS_INFO_STREAM("New data received at base: " << pose_graph_data->graphs.size() <<
   " graphs, " << pose_graph_data->scans.size() << " scans ");
  b_has_new_factor_ = true;


  pose_graph_msgs::PoseGraph::Ptr graph_ptr; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // PointCloud::Ptr new_scan(new PointCloud);

  gtsam::Values new_values;

  for (auto g : pose_graph_data->graphs) {

    // Track nodes
    for (pose_graph_msgs::PoseGraphNode n : g->nodes) {
      pose_graph_.InsertKeyedStamp(n.key, n.header.stamp);

      // Values to be added to the graph
      if (!pose_graph_.values.exists(n.key)) {
        new_values.insert(n.key, utils::ToGtsam(n.pose));

        // Add the new values to the graph
        pose_graph_.AddNewValues(new_values);
        new_values.clear();
      }
      else {
        ROS_WARN("Tried adding value to pose graph that already exists");
      }
    }

    // Track edges
    for (pose_graph_msgs::PoseGraphEdge e : g->edges) {
      pose_graph_.TrackFactor(e.key_from, 
                              e.key_to, 
                              e.type, 
                              utils::ToGtsam(e.pose), 
                              utils::ToGtsam(e.covariance));

    }

    ROS_INFO_STREAM("Added new pose graph");
  }



  ROS_INFO_STREAM("Keyed stamps: " << pose_graph_.keyed_stamps.size());

  // Update from stored keyed scans 
  for (auto s : pose_graph_data->scans) {
    pcl::fromROSMsg(s->scan, *scan_ptr);
    pose_graph_.InsertKeyedScan(s->key, scan_ptr); // TODO: add overloaded function
    AddTransformedPointCloudToMap(s->key);

    ROS_INFO_STREAM("Added new point cloud to map, " << scan_ptr->points.size() << " points");
  }
}


// Check for data from all of the handlers
bool LampBaseStation::CheckHandlers() {

  // Check for pose graphs from the robots
  ProcessPoseGraphData(pose_graph_handler_.GetData());

  return true;
}