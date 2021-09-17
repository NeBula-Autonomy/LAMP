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
LampBaseStation::LampBaseStation() : b_published_initial_node_(false) {
  // On base station LAMP, republish values after optimization
  b_repub_values_after_optimization_ = true;
  keyed_scan_candidates_.clear();
  mapper_ = std::make_shared<SimplePointCloudMapper>();
}

// Destructor
LampBaseStation::~LampBaseStation() {}

// Initialization - override for Base Station Setup
bool LampBaseStation::Initialize(const ros::NodeHandle& n) {
  // Get the name of the process
  name_ = ros::names::append(n.getNamespace(), "LampBaseStation");

  if (!mapper_->Initialize(n)) {
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
  if (!InitializeHandlers(n)) {
    ROS_ERROR("%s: Failed to initialize handlers.", name_.c_str());
    return false;
  }
  return true;
}

bool LampBaseStation::LoadParameters(const ros::NodeHandle& n) {
  if (!pu::Get("base/b_optimize_on_artifacts", b_optimize_on_artifacts_))
    return false;
  if (!pu::Get("b_use_uwb", b_use_uwb_))
    return false;

  // Names of all robots for base station to subscribe to
  if (!pu::Get("robot_names", robot_names_)) {
    ROS_ERROR("%s: No robot names provided to base station.", name_.c_str());
    return false;
  } else {
    for (auto s : robot_names_) {
      ROS_INFO_STREAM("Registered new robot: " << s);
    }
  }

  // TODO : separate rate for base and robot
  if (!pu::Get("rate/update_rate", update_rate_))
    return false;

  // Fixed precisions
  // TODO - eventually remove the need to use this
  if (!SetFactorPrecisions()) {
    ROS_ERROR("SetFactorPrecisions failed");
    return false;
  }

  // Initialize frame IDs
  pose_graph_.fixed_frame_id = "world";

  // Initialize booleans
  b_run_optimization_ = false;
  b_has_new_factor_ = false;
  b_has_new_scan_ = false;
  b_have_received_first_pg_ = true; // True for base station

  return true;
}

bool LampBaseStation::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Create the timed callback
  update_timer_ = nl.createTimer(
      update_rate_, &LampBaseStation::ProcessTimerCallback, this);

  back_end_pose_graph_sub_ =
      nl.subscribe("optimized_values",
                   1,
                   &LampBaseStation::OptimizerUpdateCallback,
                   dynamic_cast<LampBase*>(this));

  laser_loop_closure_sub_ =
      nl.subscribe("laser_loop_closures",
                   1,
                   &LampBaseStation::LaserLoopClosureCallback,
                   dynamic_cast<LampBase*>(this));

  remove_robot_sub_ = nl.subscribe("remove_robot_from_graph",
                                   1,
                                   &LampBaseStation::RemoveRobotCallback,
                                   this);

  // Uncomment when needed for debugging
  debug_sub_ = nl.subscribe("debug", 1, &LampBaseStation::DebugCallback, this);

  return true;
}

bool LampBaseStation::CreatePublishers(const ros::NodeHandle& n) {
  // Creates pose graph publishers in base class
  LampBase::CreatePublishers(n);

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Base station publishers
  pose_graph_to_optimize_pub_ = nl.advertise<pose_graph_msgs::PoseGraph>(
      "pose_graph_to_optimize", 10, true);
  lamp_pgo_reset_pub_ = nl.advertise<std_msgs::Bool>("reset_pgo", 10, true);

  // Robot pose publishers
  ros::Publisher pose_pub_;
  for (auto robot : robot_names_) {
    pose_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
        "/" + robot + "/lamp/pose_base", 10, false);
    publishers_pose_[utils::GetRobotPrefix(robot)] = pose_pub_;
  }

  return true;
}

bool LampBaseStation::InitializeHandlers(const ros::NodeHandle& n) {
  // Manual loop closure handler
  if (!manual_loop_closure_handler_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize the manual loop closure handler.",
              name_.c_str());
    return false;
  }

  // Pose graph handler -- requires robot_names parameter loaded
  if (robot_names_.size() == 0) {
    ROS_ERROR("%s: No robots for base station to subscribe to.", name_.c_str());
    return false;
  } else if (!pose_graph_handler_.Initialize(n, robot_names_)) {
    ROS_ERROR("%s: Failed to initialize the pose graph handler.",
              name_.c_str());
    return false;
  } else if (!robot_pose_handler_.Initialize(n, robot_names_)) {
    ROS_ERROR("%s: Failed to initialize the robot pose handler.",
              name_.c_str());
    return false;
  }

  return true;
}

void LampBaseStation::ProcessTimerCallback(const ros::TimerEvent& ev) {
  // Check the handlers
  CheckHandlers();

  if (!pose_graph_.CheckGraphValid()) {
    ROS_WARN("Invalid pose graph on base. Not publishing and updating. ");
  }
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
    mapper_->PublishMapInfo();
    mapper_->PublishMap();

    b_has_new_scan_ = false;
  }

  // Publish anything that is needed
}

bool LampBaseStation::ProcessPoseGraphData(std::shared_ptr<FactorData> data) {
  // ROS_INFO_STREAM("In ProcessPoseGraphData");

  // Extract pose graph data
  std::shared_ptr<PoseGraphData> pose_graph_data =
      std::dynamic_pointer_cast<PoseGraphData>(data);

  // Check if there are new pose graphs
  if (!pose_graph_data->b_has_data) {
    return false;
  }

  ROS_DEBUG_STREAM("New data received at base: "
                  << pose_graph_data->graphs.size() << " graphs, "
                  << pose_graph_data->scans.size() << " scans ");
  b_has_new_factor_ = true;

  pose_graph_msgs::PoseGraph::Ptr graph_ptr;

  gtsam::Values new_values;

  for (auto g : pose_graph_data->graphs) {
    ROS_DEBUG_STREAM("LampBase new graph with "
                     << g->nodes.size() << " nodes and " << g->edges.size()
                     << " edges");

    // Register new data - this will cause pose graph to publish
    b_has_new_factor_ = true;

    // Merge the current (slow) graph before processing incoming graphs
    merger_.OnSlowGraphMsg(pose_graph_.ToMsg());
    merger_.OnFastGraphMsg(g);

    // Update the internal pose graph using the merger output
    pose_graph_msgs::PoseGraphConstPtr fused_graph(
        new pose_graph_msgs::PoseGraph(merger_.GetCurrentGraph()));
    // ROS_INFO_STREAM("Fused Graph: UpdateFromMsg");
    pose_graph_.UpdateFromMsg(fused_graph);

    // Check for new loop closure edges
    for (pose_graph_msgs::PoseGraphEdge e : g->edges) {
      // Optimize on loop closures, IMU factors and artifact loop closures
      if (e.type == pose_graph_msgs::PoseGraphEdge::LOOPCLOSE ||
          (b_optimize_on_artifacts_ &&
           e.type == pose_graph_msgs::PoseGraphEdge::ARTIFACT) ||
          (b_use_uwb_ && e.type == pose_graph_msgs::PoseGraphEdge::UWB_RANGE) ||
          (b_use_uwb_ &&
           e.type == pose_graph_msgs::PoseGraphEdge::UWB_BETWEEN)) {
        // Run optimization to update the base station graph afterwards
        b_run_optimization_ = true;
      }

      if (e.type == pose_graph_msgs::PoseGraphEdge::IMU) {
        b_run_optimization_ = true;
      }
    }

    // Store the pose at the most recent node for each robot
    for (pose_graph_msgs::PoseGraphNode n : g->nodes) {
      char prefix = gtsam::Symbol(n.key).chr();
      if (!utils::IsRobotPrefix(prefix))
        continue;

      // First pose from this robot
      if (!latest_node_pose_.count(prefix)) {
        latest_node_pose_[prefix] =
            std::make_pair(n.key, utils::ToGtsam(n.pose));
      } else if (latest_node_pose_[prefix].first <= n.key) {
        ROS_DEBUG_STREAM("Updated pose for robot " << prefix);
        latest_node_pose_[prefix] =
            std::make_pair(n.key, utils::ToGtsam(n.pose));
      }
    }

    ROS_DEBUG_STREAM("Added new pose graph");
  }

  ROS_DEBUG_STREAM("Keyed stamps: " << pose_graph_.keyed_stamps.size());

  // Update from stored keyed scans
  for (auto s : pose_graph_data->scans) {
    // Register new data - this will cause map to publish
    b_has_new_scan_ = true;

    // Create new PCL pointer
    PointCloud::Ptr scan_ptr(new PointCloud);

    // Copy from ROS to PCL
    pcl::fromROSMsg(s->scan, *scan_ptr);

    pose_graph_.InsertKeyedScan(s->key,
                                scan_ptr); // TODO: add overloaded function

    // Add key to the list of scan candidates to add to the map
    keyed_scan_candidates_.push_back(s->key);

    ROS_DEBUG_STREAM("Added new point cloud to map, " << scan_ptr->points.size()
                                                     << " points");
  }

  // Go through the candidates to add to the map'
  AddKeyedScanCandidatesToMap();

  return true;
}

void LampBaseStation::AddKeyedScanCandidatesToMap() {
  // Loop through list of candidates
  auto key_it = keyed_scan_candidates_.begin();
  int scan_count = 0;

  while (key_it != keyed_scan_candidates_.end()) {
    // Add to map if the key exists in the nodes in the pose-graph
    if (pose_graph_.HasKey(*key_it)) {
      // Add keyed scan to map
      AddTransformedPointCloudToMap(*key_it);

      // Erase this from the list and increment the iterator
      key_it = keyed_scan_candidates_.erase(key_it);

      scan_count++;
    } else {
      // Increment key_it
      ++key_it;
    }
  }
  ROS_DEBUG_STREAM("Added " << scan_count << " scans to the map.");
}

bool LampBaseStation::ProcessRobotPoseData(std::shared_ptr<FactorData> data) {
  // Extract pose graph data
  std::shared_ptr<RobotPoseData> pose_data =
      std::dynamic_pointer_cast<RobotPoseData>(data);

  // Check if there are new pose graphs
  if (!pose_data->b_has_data) {
    return false;
  }

  for (auto pair : pose_data->poses) {
    char robot = utils::GetRobotPrefix(pair.first);
    gtsam::Pose3 pose = pair.second.pose;

    if (latest_node_pose_.count(robot)) {
      gtsam::Pose3 last_pose_base = pose_graph_.LastPose(robot);
      gtsam::Pose3 last_pose_robot = latest_node_pose_[robot].second;

      gtsam::Pose3 delta = last_pose_robot.between(pose);
      gtsam::Pose3 new_pose = last_pose_base.compose(delta);

      // Convert to ROS to publish
      geometry_msgs::PoseStamped msg;
      msg.pose = utils::GtsamToRosMsg(new_pose);
      msg.header.frame_id = pose_graph_.fixed_frame_id;
      msg.header.stamp = pair.second.stamp;

      publishers_pose_[robot].publish(msg);
    }
  }
  return true;
}

bool LampBaseStation::ProcessManualLoopClosureData(
    std::shared_ptr<FactorData> data) {
  // Extract loop closure data
  std::shared_ptr<LoopClosureData> manual_loop_closure_data =
      std::dynamic_pointer_cast<LoopClosureData>(data);

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

  // Check for poses
  ProcessRobotPoseData(robot_pose_handler_.GetData());

  return true;
}

bool LampBaseStation::ProcessArtifactGT() {
  // Read from file
  if (!pu::Get("artifacts_GT", artifact_GT_strings_)) {
    ROS_ERROR("%s: No artifact ground truth data provided.", name_.c_str());
    return false;
  }

  for (auto s : artifact_GT_strings_) {
    ROS_INFO_STREAM("New artifact ground truth");
    artifact_GT_.push_back(ArtifactGroundTruth(s));

    ROS_INFO_STREAM(
        "\t" << gtsam::DefaultKeyFormatter(artifact_GT_.back().key));
    ROS_INFO_STREAM("\t" << artifact_GT_.back().type);
    ROS_INFO_STREAM("\t" << artifact_GT_.back().position.x() << ", "
                         << artifact_GT_.back().position.y() << ", "
                         << artifact_GT_.back().position.z());
  }

  for (auto a : artifact_GT_) {
    if (!pose_graph_.HasKey(a.key)) {
      ROS_WARN_STREAM("Unable to add artifact ground truth for key "
                      << gtsam::DefaultKeyFormatter(a.key));
      continue;
    }

    // Add the prior
    pose_graph_.TrackPrior(a.key,
                           gtsam::Pose3(gtsam::Rot3(), a.position),
                           SetFixedNoiseModels("artifact_gt"));

    // Trigger optimisation
    b_run_optimization_ = true;
  }

  return true;
}

void LampBaseStation::RemoveRobotCallback(const std_msgs::String msg) {
  ROS_INFO_STREAM("Recieved remove robot message for robot " << msg.data);

  // Remove the pose graph
  pose_graph_.RemoveRobotFromGraph(msg.data);

  // Erase latest_node_pose_
  latest_node_pose_.erase(utils::GetRobotPrefix(msg.data));

  // Send reset to lamp_pgo
  std_msgs::Bool signal;
  signal.data = true;
  lamp_pgo_reset_pub_.publish(signal);

  // Publish graph to optimize
  ROS_INFO_STREAM("Sending pose graph to optimizer");
  PublishPoseGraphForOptimizer();
}

void LampBaseStation::DebugCallback(const std_msgs::String msg) {
  ROS_INFO_STREAM("Debug message received: " << msg.data);

  // Split message data into a vector of space-separated strings
  std::vector<std::string> data;
  boost::split(data, msg.data, [](char c) { return c == ' '; });

  if (data.size() == 0) {
    ROS_INFO_STREAM("Invalid debug message data");
  }
  std::string cmd = data[0];

  // Freeze the current point cloud map on the visualizer
  if (cmd == "freeze") {
    ROS_INFO_STREAM("Publishing frozen map");
    mapper_->PublishMapFrozen();
  }

  // Read in artifact ground truth data
  else if (cmd == "artifact_gt") {
    ROS_INFO_STREAM("Processing artifact ground truth data");
    ProcessArtifactGT();
  }

  // Save the pose graph
  else if (cmd == "save") {
    ROS_INFO_STREAM("Saving the pose graph");

    // Use filename if provided
    if (data.size() >= 2) {
      pose_graph_.Save(data[1]);
    } else {
      pose_graph_.Save("saved_pose_graph.zip");
    }
  }

  // Load pose graph from file
  else if (cmd == "load") {
    ROS_INFO_STREAM("Loading pose graph and keyed scans");

    // Use filename if provided
    if (data.size() >= 2) {
      pose_graph_.Load(data[1]);
    } else {
      pose_graph_.Load("saved_pose_graph.zip");
    }

    PublishPoseGraph();
    ROS_INFO_STREAM("Done Loading pose graph");
    ReGenerateMapPointCloud();
    ROS_INFO_STREAM("Done regenerating Map Pointcloud");

    PublishAllKeyedScans(); // So the loop closure module has all the keyed
                            // scans
    ROS_INFO_STREAM("Done publishing keyed scans");
  }

  // Read in artifact ground truth data
  else if (msg.data == "optimize") {
    ROS_INFO_STREAM("Sending pose graph to optimizer");
    PublishPoseGraphForOptimizer();
  }

  else {
    ROS_WARN_STREAM("Debug message not recognized");
  }
}
