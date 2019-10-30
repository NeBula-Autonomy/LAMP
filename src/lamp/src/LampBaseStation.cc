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
LampBaseStation::LampBaseStation(): 
        zero_noise_(0.0001),
        b_published_initial_node_(false) {}

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

  // Init graph
  InitializeGraph();
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

  // Initialize booleans
  b_run_optimization_ = false;
  b_has_new_factor_ = false;
  b_has_new_scan_ = false;


  return true;
}

bool LampBaseStation::RegisterCallbacks(const ros::NodeHandle& n) {

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Create the timed callback
  update_timer_ =
      nl.createTimer(update_rate_, &LampBaseStation::ProcessTimerCallback, this);

  back_end_pose_graph_sub_ = nl.subscribe("optimized_values",
                                          1,
                                          &LampBaseStation::OptimizerUpdateCallback,
                                          dynamic_cast<LampBase*>(this));

  laser_loop_closure_sub_ = nl.subscribe("laser_loop_closures",
                                         1,
                                         &LampBaseStation::LaserLoopClosureCallback,
                                         dynamic_cast<LampBase*>(this));

  debug_sub_ = nl.subscribe("debug",
                            1,
                            &LampBaseStation::DebugCallback,
                            this);

  return true; 
}

bool LampBaseStation::CreatePublishers(const ros::NodeHandle& n) {

  // Creates pose graph publishers in base class
  LampBase::CreatePublishers(n);

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Base station publishers
  pose_graph_to_optimize_pub_ = nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph_to_optimize", 10, false);

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

  gtsam::Vector6 noise;
  noise << zero_noise_, zero_noise_, zero_noise_, zero_noise_, zero_noise_, zero_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr covariance(
      gtsam::noiseModel::Diagonal::Sigmas(noise));

  pose_graph_.Initialize(utils::LAMP_BASE_INITIAL_KEY, pose, covariance);

  return true;
}


void LampBaseStation::ProcessTimerCallback(const ros::TimerEvent& ev) {

  // Check the handlers
  CheckHandlers();

  // Send data to optimizer - pose graph and map publishing happens in 
  // callback when data is received back from optimizer
  if (b_run_optimization_) {
    ROS_INFO_STREAM("Publishing pose graph to optimizer");
    PublishPoseGraphForOptimizer();

    b_run_optimization_ = false; 
  }

  if (b_has_new_factor_) {
    PublishPoseGraph();

    b_has_new_factor_ = false;
  }

  if (b_has_new_scan_) {
    mapper_.PublishMap();

    b_has_new_scan_ = false;
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

  gtsam::Values new_values;

  // First check for initial nodes
  for (auto g : pose_graph_data->graphs) {
    for (pose_graph_msgs::PoseGraphNode n : g->nodes) {

      // First node from this robot - add factor connecting to origin
      if (utils::IsRobotPrefix(gtsam::Symbol(n.key).chr()) &&!pose_graph_.values.exists(n.key) && gtsam::Symbol(n.key).index() == 0) {

        ProcessFirstRobotNode(n);
        continue; // Each robot pose graph can only have one initial node
      }
    }
  }

  for (auto g : pose_graph_data->graphs) {

    // Register new data - this will cause pose graph to publish
    b_has_new_factor_ = true;

    for (pose_graph_msgs::PoseGraphNode n : g->nodes) {
      ROS_INFO_STREAM("Received node with key " << gtsam::Symbol(n.key).chr() << gtsam::Symbol(n.key).index());
      pose_graph_.InsertKeyedStamp(n.key, n.header.stamp);

      // Add new value to graph
      if (!pose_graph_.values.exists(n.key)) {
        // Add the new values to the graph
        new_values.insert(n.key, utils::ToGtsam(n.pose));

      }

      // Update existing value in graph
      else {
        if (new_values.exists(n.key)) {
          new_values.update(n.key, utils::ToGtsam(n.pose));
        }
        else {
          new_values.insert(n.key, utils::ToGtsam(n.pose));
        }
      }
    }

    pose_graph_.AddNewValues(new_values);
    new_values.clear();

    // Track edges
    for (pose_graph_msgs::PoseGraphEdge e : g->edges) {
      pose_graph_.TrackFactor(e.key_from, 
                              e.key_to, 
                              e.type, 
                              utils::ToGtsam(e.pose), 
                              utils::ToGtsam(e.covariance));

      // Check for new loop closure edges
      if (e.type == pose_graph_msgs::PoseGraphEdge::LOOPCLOSE) {
      
        // Run optimization to update the base station graph afterwards
        b_run_optimization_ = true;
      }

    }

    // Track priors
    for (pose_graph_msgs::PoseGraphNode p : g->priors) {

      // Ignore the prior attached to the first node
      if (gtsam::Symbol(p.key).index() == 0) {
        continue;
      }

      pose_graph_.TrackNode(p.header.stamp, 
                            p.key, 
                            utils::ToGtsam(p.pose), 
                            utils::ToGtsam(p.covariance));
    }

    ROS_INFO_STREAM("Added new pose graph");
  }

  ROS_INFO_STREAM("Keyed stamps: " << pose_graph_.keyed_stamps.size());

  // Update from stored keyed scans 
  for (auto s : pose_graph_data->scans) {

    // Register new data - this will cause map to publish
    b_has_new_scan_ = true;

    pcl::fromROSMsg(s->scan, *scan_ptr);
    pose_graph_.InsertKeyedScan(s->key, scan_ptr); // TODO: add overloaded function
    AddTransformedPointCloudToMap(s->key);

    ROS_INFO_STREAM("Added new point cloud to map, " << scan_ptr->points.size() << " points");
  }

  return true; 
}

void LampBaseStation::ProcessFirstRobotNode(pose_graph_msgs::PoseGraphNode n) {

  // If this is the first node from ANY robot, send the z0 node to the optimizer first
  if (!b_published_initial_node_) {
    PublishPoseGraphForOptimizer();
    b_published_initial_node_ = true;
  }

  pose_graph_.InsertKeyedStamp(n.key, n.header.stamp);

  gtsam::Values new_values;
  gtsam::Vector6 noise;
  noise << zero_noise_, zero_noise_, zero_noise_, zero_noise_, zero_noise_, zero_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr covariance(
      gtsam::noiseModel::Diagonal::Sigmas(noise));

  ROS_INFO_STREAM("Tracking initial factor-----------------------");
  pose_graph_.TrackFactor(utils::LAMP_BASE_INITIAL_KEY, 
                  n.key, 
                  pose_graph_msgs::PoseGraphEdge::ODOM, 
                  utils::ToGtsam(n.pose), 
                  covariance);

  new_values.insert(n.key, utils::ToGtsam(n.pose));
  pose_graph_.AddNewValues(new_values);
  PublishPoseGraphForOptimizer();

  PublishPoseGraph();
}

bool LampBaseStation::ProcessManualLoopClosureData(FactorData* data) {

  // Extract loop closure data
  LoopClosureData* manual_loop_closure_data = dynamic_cast<LoopClosureData*>(data);

  if (!manual_loop_closure_data->b_has_data) {
    return false;
  }

  ROS_INFO_STREAM("Received new manual loop closure data");

  for (auto factor : manual_loop_closure_data->factors) {
    pose_graph_.TrackFactor(factor.key_from, 
                            factor.key_to,
                            pose_graph_msgs::PoseGraphEdge::LOOPCLOSE, 
                            factor.transform, 
                            factor.covariance);

    b_run_optimization_ = true;
  }

  return true;
}


// Check for data from all of the handlers
bool LampBaseStation::CheckHandlers() {

  // Check for pose graphs from the robots
  ProcessPoseGraphData(pose_graph_handler_.GetData());

  // Check for manual loop closures
  ProcessManualLoopClosureData(manual_loop_closure_handler_.GetData());



  return true;
}

void LampBaseStation::DebugCallback(const std_msgs::String msg) {
  ROS_INFO_STREAM("Debug message received");
  ROS_INFO_STREAM(msg.data);

}
