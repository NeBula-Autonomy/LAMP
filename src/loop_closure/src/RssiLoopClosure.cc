#include <loop_closure/RssiLoopClosure.h>

namespace lamp_loop_closure {
RssiLoopClosure::RssiLoopClosure() {}

RssiLoopClosure::~RssiLoopClosure() {}

bool RssiLoopClosure::Initialize(const ros::NodeHandle& n) {
  node_name_ = ros::names::append(n.getNamespace(), "RssiLoopClosure");
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
  if (!LoopGeneration::LoadParameters(n))
    return false;
  ROS_INFO_STREAM("b_check_for_loop_closures_: " << b_check_for_loop_closures_);
  if (!LoadRssiParameters(n)) {
    ROS_ERROR("Failed to load RSSI parameters from the configuration file ");
    return false;
  }

  if (!LoadRobotsList(n)) {
    ROS_ERROR("Failed to load RSSI parameters from the configuration file ");
    return false;
  }

  ShowRobotList();
  return true;
}

bool RssiLoopClosure::LoadRobotsList(const ros::NodeHandle& n) {
  std::vector<std::string> robots_name;
  if (!n.getParam("robots_loop_closure", robots_name))
    ROS_ERROR("Failed to get robot_loop_closure parameter from server.");

  for (const auto& robot_name : robots_name) {
    ROS_INFO_STREAM("Loading: " << robot_name);
    rssi_scom_robot_list_.insert(
        {"scom-" + robot_name, silvus_msgs::SilvusStreamscapeNode()});
  }
  return true;
}
bool RssiLoopClosure::LoadRssiParameters(const ros::NodeHandle& n) {
  if (!pu::Get("update_rate", update_rate_))
    return false;
  ROS_INFO_STREAM("Update rate: " << update_rate_);
  if (!pu::Get("acceptable_shortest_rssi_distance", measured_path_loss_dB_))
    return false;
  ROS_INFO_STREAM(
      "acceptable_shortest_rssi_distance: " << measured_path_loss_dB_);

  //  if (!pu::Get("check_for_loop_closures", b_check_for_loop_closures_))
  //    return false;

  if (!pu::Get("close_keys_threshold", close_keys_threshold_))
    return false;
  ROS_INFO_STREAM("close_keys_threshold: " << close_keys_threshold_);

  if (!pu::Get("radio_loop_closure_method", radio_loop_closure_method_))
    return false;
  ROS_INFO_STREAM("radio_loop_closure_method: " << radio_loop_closure_method_);

  return true;
}

bool RssiLoopClosure::CreatePublishers(const ros::NodeHandle& n) {
  if (!LoopGeneration::CreatePublishers(n))
    return false;

  ros::NodeHandle nl(n);
  db_tracking = nl.advertise<std_msgs::Float64>("db_tracking", 10, false);
  visualize_rssi_placement =
      nl.advertise<visualization_msgs::Marker>("rssi", 1000, false);
  highlight_pub_ =
      nl.advertise<visualization_msgs::Marker>("potential_rssi", 10, false);
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

//*************CALLBACKS*******************/

void RssiLoopClosure::KeyedPoseCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
  raw_mutex_.lock(); // raw mutes for robots_trajectory

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
  }
  raw_mutex_.unlock();
  return;
}

bool RssiLoopClosure::is_robot_radio(const std::string& hostname) const {
  if (hostname.find("scom-") != std::string::npos) {
    return true;
  }
  return false;
}

// callback that monitors the dropping status of the node and stores time stamp
// of dropping topic: comm_node_manager/status_agg
void RssiLoopClosure::CommNodeAggregatedStatusCallback(
    const core_msgs::CommNodeStatus::ConstPtr& msg) {
  auto rssi_list_dropped = msg->dropped;
  auto time_stamp = msg->header.stamp;
  raw_mutex_.lock();
  // for all dropped radios
  for (const auto& rssi_comm_dropped : rssi_list_dropped) {
    // if radio is on the robot (and for some weird reason has status dropped)
    // -> don't take any action
    if (is_robot_radio(rssi_comm_dropped.hostname))
      continue;
    // if the radio exists in the list it means it was added before => continue
    if (rssi_scom_dropped_list_.find(rssi_comm_dropped.hostname) !=
        rssi_scom_dropped_list_.end()) {
      continue;
    } else {
      // if the radio doesn't exist bookeep timestamp of the dropping
      if (rssi_scom_dropped_time_stamp_.find(rssi_comm_dropped.hostname) ==
          rssi_scom_dropped_time_stamp_.end()) {
        ROS_INFO_STREAM("Time stamp for node arrival: "
                        << rssi_comm_dropped.hostname
                        << " time stamp: " << time_stamp);
        rssi_scom_dropped_time_stamp_[rssi_comm_dropped.hostname] = time_stamp;
      }
      // Looking for the closest node in the trajectory of the robot that was
      // dropping the radio
      auto the_closest_pose = GetClosestPoseAtTime(
          robots_trajectory_[utils::GetRobotPrefix(
              rssi_comm_dropped.robot_name)],
          rssi_scom_dropped_time_stamp_[rssi_comm_dropped.hostname]);

      // if key is 0 it means the pose hasn't been found therefore we try to add
      // the radio to rssi_node_drooped_list_ in next callback
      if (the_closest_pose.key == 0) {
        ROS_WARN_STREAM(
            "KEY 0, poses couldn't be assigned yet! Trajectory size: "
            << robots_trajectory_[utils::GetRobotPrefix(
                                      rssi_comm_dropped.robot_name)]
                   .size());
        continue;
      }

      rssi_scom_dropped_list_[rssi_comm_dropped.hostname].comm_node_info =
          rssi_comm_dropped;
      rssi_scom_dropped_list_[rssi_comm_dropped.hostname].b_dropped = true;
      rssi_scom_dropped_list_[rssi_comm_dropped.hostname].pose_graph_node =
          the_closest_pose;
      rssi_scom_dropped_list_[rssi_comm_dropped.hostname].time_stamp =
          rssi_scom_dropped_time_stamp_[rssi_comm_dropped.hostname];

      PrintDropStatus(rssi_comm_dropped);
    }
  }

  raw_mutex_.unlock();
  VisualizeRssi();
}

void RssiLoopClosure::RssiTimerCallback(const ros::TimerEvent& event) {
  raw_mutex_.lock();
  for (auto& scom_robot : rssi_scom_robot_list_) {
    // iterate over radios that are connected to the scom robot radio
    for (const auto& neighbour : scom_robot.second.neighbors) {
      // if radio conntected to the robot radio was dropped (so it must exist in
      // the rssi_node_dropped_list) -> true since haven't reached end
      if (rssi_scom_dropped_list_.find(neighbour.neighbor_node_label) !=
          rssi_scom_dropped_list_.end()) {
        // calculate path loss in db and compare with the threshikd value from
        // config
        float path_loss = CalculatePathLossForNeighbor(neighbour);

        if (path_loss < measured_path_loss_dB_) {
          // get the pose from robot trajectory that was the closest at time
          // stamp

          auto scom_pose_associated_for_scom_robot = GetClosestPoseAtTime(
              robots_trajectory_[utils::GetRobotPrefix(
                  scom_robot.second.robot_name)],
              rssi_scom_robot_list_updated_time_stamp_[scom_robot.first]);

          bool appended = rssi_scom_dropped_list_[neighbour.neighbor_node_label]
                              .append_node(scom_pose_associated_for_scom_robot);

          if (appended) {
            auto color = getColorByIndex(
                rssi_scom_dropped_list_[neighbour.neighbor_node_label]
                    .flyby_number);

            VisualizeRobotNodesWithPotentialLoopClosure(
                scom_pose_associated_for_scom_robot,
                color.r,
                color.g,
                color.b,
                0.5,
                std::to_string(path_loss));
          }
        }
      }
    }
  }

  raw_mutex_.unlock();
  GenerateLoops();
  if (loop_candidate_pub_.getNumSubscribers() > 0 && candidates_.size() > 0) {
    PublishLoops();
    ClearLoops();
  }
}

// This callback reads raw msgs from silvus and updates radio information for
// robots that we want them to do loop closures (in config you set it up) -
// (rssi_scom_robot_list_).
// Moreover it stores information when the radio information was updated
// (rssi_scom_robot_list_updated_time_stamp_), basically update is needed for
// gathering information about strength of the signal to calculate the
// pseudorange
// topic to listen : comm/silvus/raw
void RssiLoopClosure::CommNodeRawCallback(
    const silvus_msgs::SilvusStreamscape::ConstPtr& msg) {
  raw_mutex_.lock();
  // for every radio node that was sent to the base and it's in kind of range
  for (const auto& node : msg->nodes) {
    // if it's a robot radio (so it's scom-huskyx, scom-spotx, etc)
    if (!node.robot_name.empty()) {
      // if the loop closure from this robot we are interested in -> update
      // (in config you can specify)
      if (rssi_scom_robot_list_.count(node.node_label)) {
        // update scom-robot information regarding neighbours, etc...
        //        ROS_INFO_STREAM("Update communication msgs with" <<
        //        node.node_label);
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
//*************HELPER FUNCTIONS*******************/

pose_graph_msgs::PoseGraphNode RssiLoopClosure::GetClosestPoseAtTime(
    const std::map<double, pose_graph_msgs::PoseGraphNode>& robot_trajectory,
    const ros::Time& stamp,
    double time_threshold,
    bool check_threshold) {
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

void RssiLoopClosure::GenerateLoops() {
  // Loop closure off. No candidates generated
  if (!b_check_for_loop_closures_)
    return;

  if (radio_loop_closure_method_ == "radio_to_nodes") {
    RadioToNodesLoopClosure();
  } else if (radio_loop_closure_method_ == "nodes_to_nodes") {
    NodesToNodesLoopClosures();
  } else {
    ROS_ERROR_STREAM("Something is wrong in RSSI LOOP CLOSURE SINCE THE METHOD "
                     "DOESN't EXIST!!!!");
    EXIT_FAILURE;
  }

  ROS_INFO_STREAM("Sending loop closures: " << candidates_.size());

  if (loop_candidate_pub_.getNumSubscribers() > 0 && candidates_.size() > 0) {
    PublishLoops();
    ClearLoops();
  }
  return;
}

void RssiLoopClosure::RadioToNodesLoopClosure() {
  for (auto& rssi_node_dropped : rssi_scom_dropped_list_) {
    for (size_t i = 0;
         i < rssi_node_dropped.second.all_nodes_around_comm.size();
         i++) {
      if (i == 0)
        continue;
      for (auto& flyby_node :
           rssi_node_dropped.second.all_nodes_around_comm[i]) {
        if (flyby_node.was_sent == true) {
          continue;
        }
        pose_graph_msgs::LoopCandidate candidate;
        candidate.header.stamp = ros::Time::now();
        candidate.key_from = rssi_node_dropped.second.pose_graph_node.key;
        candidate.key_to = flyby_node.candidate_pose.key;
        candidate.pose_from = rssi_node_dropped.second.pose_graph_node.pose;
        candidate.pose_to = flyby_node.candidate_pose.pose;
        candidate.type = pose_graph_msgs::LoopCandidate::PROXIMITY;
        candidate.value = 0.0;
        candidates_.push_back(candidate);
        flyby_node.was_sent = true;

        VisualizeEdgesForPotentialLoopClosure(
            rssi_node_dropped.second.pose_graph_node,
            flyby_node.candidate_pose);
      }
    }
  }
}
void RssiLoopClosure::NodesToNodesLoopClosures() {
  for (auto& rssi_node_dropped : rssi_scom_dropped_list_) {
    for (size_t flyby_i = 0;
         flyby_i < rssi_node_dropped.second.all_nodes_around_comm.size();
         flyby_i++) {
      if (flyby_i == 0)
        continue;
      for (auto& flyby_node :
           rssi_node_dropped.second.all_nodes_around_comm[flyby_i]) {
        if (flyby_node.was_sent == true) {
          continue;
        }
        pose_graph_msgs::LoopCandidate candidate_to_comm;
        candidate_to_comm.header.stamp = ros::Time::now();
        candidate_to_comm.key_from =
            rssi_node_dropped.second.pose_graph_node.key;
        candidate_to_comm.key_to = flyby_node.candidate_pose.key;
        candidate_to_comm.pose_from =
            rssi_node_dropped.second.pose_graph_node.pose;
        candidate_to_comm.pose_to = flyby_node.candidate_pose.pose;
        candidate_to_comm.type = pose_graph_msgs::LoopCandidate::PROXIMITY;
        candidate_to_comm.value = 0.0;
        candidates_.push_back(candidate_to_comm);
        flyby_node.was_sent = true;

        VisualizeEdgesForPotentialLoopClosure(
            rssi_node_dropped.second.pose_graph_node,
            flyby_node.candidate_pose);

        for (size_t flyby_j = 0; flyby_j < flyby_i; flyby_j++) {
          for (const auto& flyby_j_node :
               rssi_node_dropped.second.all_nodes_around_comm[flyby_j]) {
            pose_graph_msgs::LoopCandidate candidate;
            candidate.header.stamp = ros::Time::now();
            candidate.key_from = flyby_j_node.candidate_pose.key;
            candidate.key_to = flyby_node.candidate_pose.key;
            candidate.pose_from = flyby_j_node.candidate_pose.pose;
            candidate.pose_to = flyby_node.candidate_pose.pose;
            candidate.type = pose_graph_msgs::LoopCandidate::PROXIMITY;
            candidate.value = 0.0;
            candidates_.push_back(candidate);
            //            flyby_node.was_sent = true;

            VisualizeEdgesForPotentialLoopClosure(flyby_j_node.candidate_pose,
                                                  flyby_node.candidate_pose);
          }
        }
      }
    }
  }
}
//*************MATH CALCULATION*******************/
// Calculating pass loss for the given node
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

//*************Visualization and Printing*******************/
void RssiLoopClosure::PrintDropStatus(
    const core_msgs::CommNodeInfo& node_info) const {
  ROS_INFO_STREAM(
      "RSSI ID:"
      << node_info.uwb_id << " was dropped by " << node_info.robot_name
      << " label " << node_info.hostname << " key name: "
      << utils::GetRobotPrefix(node_info.robot_name) << " pose: "
      << rssi_scom_dropped_list_.at(node_info.hostname).pose_graph_node.pose
      << " I D : "
      << rssi_scom_dropped_list_.at(node_info.hostname).pose_graph_node.ID
      << " KEY: "
      << rssi_scom_dropped_list_.at(node_info.hostname).pose_graph_node.key);
}

void RssiLoopClosure::ShowRssiList() const {
  for (const auto& [rssi_id, info] : rssi_scom_dropped_list_) {
    ROS_INFO_STREAM("RSSI ID: "
                    << info.comm_node_info.uwb_id
                    << "\t Holder: " << info.comm_node_info.robot_name
                    << "\t Hostname: " << info.comm_node_info.hostname
                    << "\t Drop Status: " << info.b_dropped);
  }
}

void RssiLoopClosure::ShowDroppedRssiList() const {
  for (const auto& [rssi_id, info] : rssi_scom_dropped_list_) {
    ROS_INFO_STREAM("There are nodes: " << rssi_scom_dropped_list_.size());
    if (info.b_dropped) {
      ROS_INFO_STREAM("RSSI ID: "
                      << info.comm_node_info.uwb_id
                      << "\t Holder: " << info.comm_node_info.robot_name
                      << "\t Hostname: " << info.comm_node_info.hostname
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
  int indx = 56000;
  for (const auto& node : rssi_scom_dropped_list_) {
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    // m.ns = pose_graph_.fixed_frame_id + "node" + std::to_string(key);
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.1;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.text = node.first;
    m.id = indx;
    m.pose.position = node.second.pose_graph_node.pose.position;
    visualize_rssi_placement.publish(m);
    indx++;
    m.scale.z = 0.5;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    m.id = indx;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    visualize_rssi_placement.publish(m);
    indx++;
  }
}

void RssiLoopClosure::VisualizeRobotNodesWithPotentialLoopClosure(
    const pose_graph_msgs::PoseGraphNode& node_pose,
    float red,
    float green,
    float blue,
    float scale,
    std::string name) {
  visualization_msgs::Marker m;
  m.header.frame_id = "world";
  // m.ns = pose_graph_.fixed_frame_id + "node" + std::to_string(key);
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::SPHERE;
  m.color.r = red;
  m.color.g = green;
  m.color.b = blue;
  m.color.a = 0.5;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.id = idx2;
  m.text = name;
  idx2++;
  m.pose.position = node_pose.pose.position;
  visualize_rssi_placement.publish(m);
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 1.0;
  m.color.a = 1.0;
  m.id = idx2++;
  m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  visualize_rssi_placement.publish(m);
}

void RssiLoopClosure::VisualizeText(
    const pose_graph_msgs::PoseGraphNode& node_pose, std::string name) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  // marker.header.stamp = ros::Time();
  marker.ns = "";
  marker.id = idx2;
  marker.pose.position = node_pose.pose.position;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.z = 1.05;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.text = "XD";

  visualize_rssi_placement.publish(marker);
  idx2++;
}

bool RssiLoopClosure::VisualizeEdgesForPotentialLoopClosure(
    const pose_graph_msgs::PoseGraphNode& node1,
    const pose_graph_msgs::PoseGraphNode& node2) {
  ROS_INFO("Highlighting factor between %lu and %lu.",
           node1.pose.position,
           node2.pose.position);

  visualization_msgs::Marker m;
  m.header.frame_id = "world";

  m.id = idx2;
  idx2++;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.color.r = 0.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.color.a = 0.5;
  m.scale.x = 0.05;
  // m.scale.y = 1.0;
  // m.scale.z = 1.0;

  m.points.push_back(node1.pose.position);
  m.points.push_back(node2.pose.position);
  highlight_pub_.publish(m);

  //  VisualizeRobotNodesWithPotentialLoopClosure(node1, 0.0, 1.0, 0.0, 1.0);
  // VisualizeRobotNodesWithPotentialLoopClosure(node2, 0.0, 1.0, 0.0, 0.6);

  return true;
}
} // namespace lamp_loop_closure
