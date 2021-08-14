#include <loop_closure/RssiLoopClosure.h>

namespace lamp_loop_closure {
RssiLoopClosure::RssiLoopClosure() {}

RssiLoopClosure::~RssiLoopClosure() {}

bool RssiLoopClosure::Initialize(const ros::NodeHandle& n) {
  node_name_ = ros::names::append(n.getNamespace(), "OdometryHandler");
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", node_name_.c_str());
    return false;
  }
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", node_name_.c_str());
    return false;
  }

  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", node_name_.c_str());
    return false;
  }

  return true;
}
bool RssiLoopClosure::LoadParameters(const ros::NodeHandle& n) {
  if (!LoadRssiParameters(n)) {
    ROS_ERROR("Failed to load RSSI parameters from the configuration file ");
    return false;
  }

  if (!LoadRobotsList(n)) {
    ROS_ERROR("Failed to load RSSI parameters from the configuration file ");
    return false;
  }

  ROS_INFO_STREAM("Showing Rssis that have been dropped.");
  ShowRssiList();
  ROS_INFO_STREAM("Showing robots to consider loop closure.");
  ShowRobotList();
  return true;
}

bool RssiLoopClosure::LoadRobotsList(const ros::NodeHandle& n) {
  std::vector<std::string> robots_loop_closure;
  if (!n.getParam("robots_loop_closure", robots_loop_closure))
    ROS_ERROR("Failed to get robot_loop_closure parameter from server.");

  for (const auto& robot_loop_closure : robots_loop_closure) {
    ROS_INFO_STREAM("Loading: " << robot_loop_closure);
    rssi_scom_robot_list_.insert(
        {"scom-" + robot_loop_closure, silvus_msgs::SilvusStreamscapeNode()});
  }
  return true;
}
bool RssiLoopClosure::LoadRssiParameters(const ros::NodeHandle& n) {
  if (!pu::Get("update_rate", update_rate_))
    return false;
  ROS_INFO_STREAM("Update rate: " << update_rate_);
  if (!pu::Get("acceptable_shortest_rssi_distance",
               acceptable_shortest_rssi_distance_))
    return false;
  ROS_INFO_STREAM("acceptable_shortest_rssi_distance: "
                  << acceptable_shortest_rssi_distance_);

  if (!pu::Get("check_for_loop_closures", b_check_for_loop_closures_))
    return false;
  ROS_INFO_STREAM("b_check_for_loop_closures_: " << b_check_for_loop_closures_);

  return true;
}

bool RssiLoopClosure::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  // tracking db strength
  db_tracking = nl.advertise<std_msgs::Float64>("db_tracking", 10, false);
  visualize_rssi_placement =
      nl.advertise<visualization_msgs::Marker>("rssi", 1000, false);
  return true;
}
bool RssiLoopClosure::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  pose_graph_sub_ =
      nl.subscribe("pose_graph", 10, &RssiLoopClosure::KeyedPoseCallback, this);

  comm_node_aggregated_status_sub_ =
      nl.subscribe("rssi_aggregated_drop_status",
                   10,
                   &RssiLoopClosure::CommNodeAggregatedStatusCallback,
                   this);

  comm_node_raw_ = nl.subscribe(
      "silvus_raw", 10, &RssiLoopClosure::CommNodeRawCallback, this);

  // Timers
  uwb_update_timer_ =
      nl.createTimer(update_rate_, &RssiLoopClosure::RssiTimerCallback, this);
  return true;
}

void RssiLoopClosure::KeyedPoseCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
  raw_mutex_.lock();

  pose_graph_msgs::PoseGraphNode node_msg;

  for (auto& robot_trajectory : robots_trajectory_) {
    robot_trajectory.second.clear();
  }

  for (const auto& node_msg : graph_msg->nodes) {
    gtsam::Symbol new_key = gtsam::Symbol(node_msg.key); // extract
    if (!utils::IsRobotPrefix(new_key.chr()))
      continue;

    robots_trajectory_[new_key.chr()].insert(
        {node_msg.header.stamp.toSec(), node_msg});

    //    GenerateLoops(new_key);
  }
  raw_mutex_.unlock();
  //  if (loop_candidate_pub_.getNumSubscribers() > 0 && candidates_.size() > 0)
  //  {
  //    PublishLoops();
  //    ClearLoops();
  //  }
  return;
}

void RssiLoopClosure::GenerateLoops(const gtsam::Key& new_key) {
  // Loop closure off. No candidates generated
  if (!b_check_for_loop_closures_)
    return;

  //  const gtsam::Symbol key = gtsam::Symbol(new_key);
  //  std::vector<pose_graph_msgs::LoopCandidate> potential_candidates;
  //  for (auto it = keyed_poses_.begin(); it != keyed_poses_.end(); ++it) {
  //    const gtsam::Symbol other_key = it->first;

  //    // Don't self-check.
  //    if (key == other_key)
  //      continue;

  //    // Don't compare against poses that were recently collected.
  //    if (utils::IsKeyFromSameRobot(key, other_key) &&
  //        std::llabs(key.index() - other_key.index()) < skip_recent_poses_)
  //      continue;

  //    double distance = DistanceBetweenKeys(key, other_key);

  //    if (distance > proximity_threshold_) {
  //      continue;
  //    }

  //    // If all the checks pass create candidate and add to queue
  //    pose_graph_msgs::LoopCandidate candidate;
  //    candidate.header.stamp = ros::Time::now();
  //    candidate.key_from = new_key;
  //    candidate.key_to = other_key;
  //    candidate.pose_from = utils::GtsamToRosMsg(keyed_poses_[new_key]);
  //    candidate.pose_to = utils::GtsamToRosMsg(keyed_poses_[other_key]);
  //    candidate.type = pose_graph_msgs::LoopCandidate::PROXIMITY;
  //    candidate.value = distance;

  //    potential_candidates.push_back(candidate);
  //  }
  //  if (potential_candidates.size() < n_closest_) {
  //    candidates_.insert(candidates_.end(),
  //                       potential_candidates.begin(),
  //                       potential_candidates.end());
  //  } else {
  //    sort(potential_candidates.begin(),
  //         potential_candidates.end(),
  //         [](const pose_graph_msgs::LoopCandidate& lhs,
  //            const pose_graph_msgs::LoopCandidate& rhs) {
  //           return lhs.value < rhs.value;
  //         });
  //    candidates_.insert(candidates_.end(),
  //                       potential_candidates.begin(),
  //                       potential_candidates.begin() + n_closest_);
  //  }
  return;
}

bool is_robot_radio(const std::string& hostname) {
  if (hostname.find("scom-") != std::string::npos) {
    return true;
  }
  return false;
}

pose_graph_msgs::PoseGraphNode GetClosestPoseAtTime(
    const std::map<double, pose_graph_msgs::PoseGraphNode>& robot_trajectory,
    const ros::Time& stamp,
    double time_threshold = 2.0,
    bool check_threshold = false) {
  // If there are no keys, throw an error
  if (robot_trajectory.empty()) {
    ROS_ERROR("Cannot get closest key - no keys are stored");
    return pose_graph_msgs::PoseGraphNode();
  }

  // Output key
  pose_graph_msgs::PoseGraphNode pose_out;

  // Iterators pointing immediately before and after the target time
  auto iterAfter = robot_trajectory.lower_bound(stamp.toSec());
  auto iterBefore = std::prev(iterAfter);
  double t1 = iterBefore->first;
  double t2 = iterAfter->first;
  double t_closest;

  bool b_is_end_case = false;

  // If time is before the start or after the end, return first/last key
  if (iterAfter == robot_trajectory.begin()) {
    ROS_ERROR("Time stamp before start of range (GetClosestKeyAtTime)");
    pose_out = iterAfter->second;
    t_closest = t2;
    b_is_end_case = true;
  } else if (iterAfter == robot_trajectory.end()) {
    ROS_ERROR("Time past end of the range (GetClosestKeyAtTime)");
    pose_out = iterBefore->second;
    t_closest = t1;
    b_is_end_case = true;
  }

  if (!b_is_end_case) {
    // Otherwise return the closer key
    if (stamp.toSec() - t1 < t2 - stamp.toSec()) {
      pose_out = iterBefore->second;
      t_closest = t1;
    } else {
      pose_out = iterAfter->second;
      t_closest = t2;
    }
  }

  // Check threshold
  if (check_threshold && std::abs(t_closest - stamp.toSec()) > time_threshold) {
    ROS_ERROR("Delta between queried time and closest time in graph too large");
    ROS_INFO_STREAM("Time queried is: " << stamp.toSec()
                                        << ", closest time is: " << t_closest);
    ROS_INFO_STREAM("Difference is "
                    << std::abs(stamp.toSec() - t_closest)
                    << ", allowable max is: " << time_threshold);
    pose_out = pose_graph_msgs::PoseGraphNode();
  } else if (std::abs(t_closest - stamp.toSec()) > time_threshold) {
    ROS_WARN_STREAM(
        "Delta between queried time and closest time in graph too large\n"
        << "Time queried is: " << stamp.toSec()
        << ", closest time is: " << t_closest << "\n Difference is "
        << std::abs(stamp.toSec() - t_closest)
        << ", allowable max is: " << time_threshold);
  }

  return pose_out;
}

void RssiLoopClosure::CommNodeAggregatedStatusCallback(
    const core_msgs::CommNodeStatus::ConstPtr& msg) {
  auto rssi_list_dropped = msg->dropped;
  auto time_stamp = msg->header.stamp;

  //  ROS_INFO_STREAM(
  //      "Get a new drop status from comm_node_manager on the base station ");
  raw_mutex_.lock();
  for (const auto& dropped : rssi_list_dropped) {
    if (is_robot_radio(dropped.hostname))
      continue;
    if (rssi_node_dropped_list_.find(dropped.hostname) !=
        rssi_node_dropped_list_.end()) {
      continue;
    } else {
      if (rssi_node_dropped_time_stamp_.find(dropped.hostname) ==
          rssi_node_dropped_time_stamp_.end()) {
        ROS_INFO_STREAM("Time stamp for node arrival: "
                        << dropped.hostname << " time stamp: " << time_stamp);
        rssi_node_dropped_time_stamp_[dropped.hostname] = time_stamp;
      }
      ROS_INFO_STREAM(
          "NOT KEY FOUND"
          << dropped.robot_name << " trajectory size: "
          << robots_trajectory_[utils::GetRobotPrefix(dropped.robot_name)]
                 .size());

      auto the_closest_pose = GetClosestPoseAtTime(
          robots_trajectory_[utils::GetRobotPrefix(dropped.robot_name)],
          rssi_node_dropped_time_stamp_[dropped.hostname]);

      if (the_closest_pose.key == 0) {
        continue;
      }
      rssi_node_dropped_list_[dropped.hostname].node_info = dropped;
      rssi_node_dropped_list_[dropped.hostname].b_dropped = true;
      rssi_node_dropped_list_[dropped.hostname].graph_node = the_closest_pose;

      ROS_INFO_STREAM(
          "RSSI ID:"
          << dropped.uwb_id << " was dropped by " << dropped.robot_name
          << " label " << dropped.hostname << " key name: "
          << utils::GetRobotPrefix(dropped.robot_name) << " pose: "
          << rssi_node_dropped_list_[dropped.hostname].graph_node.pose
          << " I D : "
          << rssi_node_dropped_list_[dropped.hostname].graph_node.ID << " KEY: "
          << rssi_node_dropped_list_[dropped.hostname].graph_node.key);
    }
  }

  raw_mutex_.unlock();
  VisualizeRssi();
  //  ShowRssiList();
} // namespace lamp_loop_closure

void RssiLoopClosure::RssiTimerCallback(const ros::TimerEvent& event) {
  raw_mutex_.lock();
  for (const auto& robot_radio : rssi_scom_robot_list_) {
    // iterate over radios that are connected to the robot radio
    for (const auto& neighbour : robot_radio.second.neighbors) {
      // if radio conntected to the robot radio was dropped (so it must exist in
      // the rssi_node_dropped_list) -> true since haven't reached end
      if (rssi_node_dropped_list_.find(neighbour.neighbor_node_label) !=
          rssi_node_dropped_list_.end()) {
        // calculate path loss in db and compare with the value from config
        if (CalculatePathLossForNeighbor(neighbour) <
            acceptable_shortest_rssi_distance_) {
          ROS_INFO_STREAM(
              "Robot : "
              << robot_radio.first << " ~ " << neighbour.neighbor_node_label
              << " distance " << CalculatePathLossForNeighbor(neighbour)
              << " pose: "
              << rssi_node_dropped_list_[neighbour.neighbor_node_label]
                     .graph_node.pose.position.x
              << " "
              << rssi_node_dropped_list_[neighbour.neighbor_node_label]
                     .graph_node.pose.position.y
              << " "
              << rssi_node_dropped_list_[neighbour.neighbor_node_label]
                     .graph_node.pose.position.z);

          LoopCandidateToPrepare loop_candidate_to_prepare_;

          loop_candidate_to_prepare_.time_stamp_ =
              rssi_scom_robot_list_updated_time_stamp_[robot_radio.first];
          loop_candidate_to_prepare_.robot_name_ =
              robot_radio.second.robot_name;
          loop_candidates_to_prepare_.emplace_back(loop_candidate_to_prepare_);
        }
      }
    }
  }

  raw_mutex_.unlock();

  std::vector<LoopCandidateToPrepare> loop_closures_for_further_processing;
  for (auto& loop_candidate_to_prepare : loop_candidates_to_prepare_) {
    ROS_INFO_STREAM(
        "loop_candidate_to_prepare.robot_name_: "
        << loop_candidate_to_prepare.robot_name_ << " trajectory size: "
        << robots_trajectory_[utils::GetRobotPrefix(
                                  loop_candidate_to_prepare.robot_name_)]
               .size());
    auto pose_associated_for_given_robot =
        GetClosestPoseAtTime(robots_trajectory_[utils::GetRobotPrefix(
                                 loop_candidate_to_prepare.robot_name_)],
                             loop_candidate_to_prepare.time_stamp_);
    if (pose_associated_for_given_robot.key != 0) {
      VisualizeRobotNodesWithPotentialLoopClosure(
          pose_associated_for_given_robot);
      //      pose_graph_msgs::LoopCandidate candidate;
      //      candidate.header.stamp = ros::Time::now();
      //      candidate.key_from = new_key;
      //      candidate.key_to = other_key;
      //      candidate.pose_from =
      //  utils::GtsamToRosMsg(keyed_poses_[new_key]);
      //      candidate.pose_to =
      // utils::GtsamToRosMsg(keyed_poses_[other_key]);
      //      candidate.type = pose_graph_msgs::LoopCandidate::PROXIMITY;
      //      candidate.value = distance;
    } else {
      loop_closures_for_further_processing.push_back(loop_candidate_to_prepare);
    }
  }
  std::swap(loop_candidates_to_prepare_, loop_closures_for_further_processing);
}

void RssiLoopClosure::CommNodeRawCallback(
    const silvus_msgs::SilvusStreamscape::ConstPtr& msg) {
  auto nodes_in_range = msg->nodes;
  raw_mutex_.lock();
  // for every radio node that was sent to the base and is a kind of range
  for (const auto& node : nodes_in_range) {
    // if it's a robot radio (so it's scom-huskyx, scom-spotx, etc)
    if (!node.robot_name.empty()) {
      // if the loop closure from this robot we are interested in -> update (in
      // config you can specify)
      if (rssi_scom_robot_list_.count(node.node_label)) {
        // update scom-robot information regarding neighbours, etc...
        rssi_scom_robot_list_[node.node_label] = node;
        // keep timestamp when it happened because then you need to get
        // association from the pose_graph
        rssi_scom_robot_list_updated_time_stamp_[node.node_label] =
            msg->header.stamp;
      }
    }
  }
  raw_mutex_.unlock();
}

// math calculation
float RssiLoopClosure::CalculatePathLossForNeighbor(
    const silvus_msgs::SilvusStreamscapeNeighbor& neighbor) {
  float signal_power_sum = neighbor.my_txpw_actual_dBm -
      static_cast<float>(neighbor.received_signal_power_dBm[0]) +
      neighbor.my_txpw_actual_dBm -
      static_cast<float>(neighbor.received_signal_power_dBm[1]);
  float path_loss_db =
      signal_power_sum / static_cast<float>(ANTENAS_NUMBER_IN_ROBOT);
  return path_loss_db;
}

// visualization and printing

void RssiLoopClosure::ShowRssiList() const {
  for (const auto& [rssi_id, info] : rssi_node_dropped_list_) {
    ROS_INFO_STREAM("RSSI ID: " << info.node_info.uwb_id
                                << "\t Holder: " << info.node_info.robot_name
                                << "\t Hostname: " << info.node_info.hostname
                                << "\t Drop Status: " << info.b_dropped);
  }
}

void RssiLoopClosure::ShowDroppedRssiList() const {
  for (const auto& [rssi_id, info] : rssi_node_dropped_list_) {
    ROS_INFO_STREAM("There are nodes: " << rssi_node_dropped_list_.size());
    if (info.b_dropped) {
      ROS_INFO_STREAM("RSSI ID: " << info.node_info.uwb_id
                                  << "\t Holder: " << info.node_info.robot_name
                                  << "\t Hostname: " << info.node_info.hostname
                                  << "\t Drop Status: " << info.b_dropped);
    }
  }
}

void RssiLoopClosure::ShowRobotList() const {
  for (const auto& [robot_label, info] : rssi_scom_robot_list_) {
    ROS_INFO_STREAM("Robot label: " << robot_label
                                    << "\t node_label: " << info.node_label
                                    << "\t robot_name: " << info.robot_name
                                    << "\t node_id " << info.node_id);
  }
}

void RssiLoopClosure::VisualizeRssi() {
  int idx = 0;
  for (const auto& node : rssi_node_dropped_list_) {
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    // m.ns = pose_graph_.fixed_frame_id + "node" + std::to_string(key);
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.id = idx++;
    m.pose.position = node.second.graph_node.pose.position;
    visualize_rssi_placement.publish(m);
  }
}

void RssiLoopClosure::VisualizeRobotNodesWithPotentialLoopClosure(
    const pose_graph_msgs::PoseGraphNode& node_pose,
    float red,
    float green,
    float blue) {
  visualization_msgs::Marker m;
  m.header.frame_id = "world";
  // m.ns = pose_graph_.fixed_frame_id + "node" + std::to_string(key);
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::SPHERE;
  m.color.r = 0.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.id = idx2++;
  m.pose.position = node_pose.pose.position;
  visualize_rssi_placement.publish(m);
}
} // namespace lamp_loop_closure
