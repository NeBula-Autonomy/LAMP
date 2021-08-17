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
  for (const auto& scom_robot : rssi_scom_robot_list_) {
    // iterate over radios that are connected to the scom robot radio
    for (const auto& neighbour : scom_robot.second.neighbors) {
      // if radio conntected to the robot radio was dropped (so it must exist in
      // the rssi_node_dropped_list) -> true since haven't reached end
      if (rssi_scom_dropped_list_.find(neighbour.neighbor_node_label) !=
          rssi_scom_dropped_list_.end()) {
        // calculate path loss in db and compare with the threshikd value from
        // config

        if (CalculatePathLossForNeighbor(neighbour) < measured_path_loss_dB_) {
          // get the pose from robot trajectory that was the closest at time
          // stamp

          auto scom_pose_associated_for_scom_robot = GetClosestPoseAtTime(
              robots_trajectory_[utils::GetRobotPrefix(
                  scom_robot.second.robot_name)],
              rssi_scom_robot_list_updated_time_stamp_[scom_robot.first]);

          // if pose has key 0 it means that node haven't been found; if keys
          // are the same means that the robot doesn't move and is in the same
          // place what the comm node has a pose, and check if the same pose was
          // included before - > if 3x yes then accept the pose for the signal
          bool has_scom_node_associated_been_found_in_robot_trajectory =
              scom_pose_associated_for_scom_robot.key != 0;
          if (!has_scom_node_associated_been_found_in_robot_trajectory)
            ROS_INFO_STREAM("Key 0!");
          bool is_scom_node_associated_the_same_as_dropped_nodee =
              scom_pose_associated_for_scom_robot.key !=
              rssi_scom_dropped_list_[neighbour.neighbor_node_label]
                  .pose_graph_node.key;
          if (!is_scom_node_associated_the_same_as_dropped_nodee)
            ROS_INFO_STREAM("Pose has the same node as comm");
          bool is_this_new_graph_node =
              rssi_scom_dropped_list_[neighbour.neighbor_node_label]
                  .nodes_around_comm.count(
                      scom_pose_associated_for_scom_robot.key) == 0;
          if (!is_this_new_graph_node)
            ROS_INFO_STREAM("This node was associated to the comm");

          if (has_scom_node_associated_been_found_in_robot_trajectory and
              is_scom_node_associated_the_same_as_dropped_nodee and
              is_this_new_graph_node) {
            ROS_INFO_STREAM(
                "Accepted: New node associated with the comm "
                << rssi_scom_robot_list_updated_time_stamp_[scom_robot.first]
                << " is " << scom_pose_associated_for_scom_robot.pose.position.x
                << " " << scom_pose_associated_for_scom_robot.pose.position.x
                << " " << scom_pose_associated_for_scom_robot.pose.position.z);
            // insert to the history associaated with the poses
            rssi_scom_dropped_list_[neighbour.neighbor_node_label]
                .nodes_around_comm.insert(
                    {scom_pose_associated_for_scom_robot.key,
                     PoseGraphNodeLoopClosureStatus(
                         false, scom_pose_associated_for_scom_robot)});
            ROS_INFO_STREAM(
                "Nodes around "
                << neighbour.neighbor_node_label << " "
                << rssi_scom_dropped_list_[neighbour.neighbor_node_label]
                       .nodes_around_comm.size());

            VisualizeRobotNodesWithPotentialLoopClosure(
                scom_pose_associated_for_scom_robot,
                255.0 / 255.0,
                165.0 / 255.0,
                0.0);
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
  } else {
    ROS_ERROR_STREAM("Something is wrong in RSSI LOOP CLOSURE SINCE THE METHOD "
                     "DOESN't EXIST!!!!");
    EXIT_FAILURE;
  }
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
  ROS_INFO_STREAM("CANDIDATES FOR LOOP CLOSURE!!!!");
  ROS_INFO_STREAM("Sending loop closures: " << candidates_.size());

  for (const auto& candidate : candidates_) {
    ROS_INFO_STREAM("key from: " << candidate.key_from << " to "
                                 << candidate.key_to << " pose from "
                                 << candidate.pose_from << " to "
                                 << candidate.pose_to);
  }
  if (loop_candidate_pub_.getNumSubscribers() > 0 && candidates_.size() > 0) {
    PublishLoops();
    ClearLoops();
  }
  return;
}

void RssiLoopClosure::RadioToNodesLoopClosure() {
  ROS_INFO_STREAM(
      "<<<<<<<<<<<<---------------------------------------->>>>>>>>>>");

  for (auto& rssi_node_dropped : rssi_scom_dropped_list_) {
    ROS_INFO_STREAM("Looking for loop closures from nodes nearby the comm: "
                    << rssi_node_dropped.first << " number of candidates: "
                    << rssi_node_dropped.second.nodes_around_comm.size());
    for (auto& node_around_comm : rssi_node_dropped.second.nodes_around_comm) {
      if (node_around_comm.second.first == true) {
        continue;
      }
      auto node_a =
          gtsam::Symbol(rssi_node_dropped.second.pose_graph_node.key).index();
      auto node_b = gtsam::Symbol(node_around_comm.second.second.key).index();
      std::uint64_t diff_index;
      if (node_a > node_b) {
        diff_index = node_a - node_b;
      } else {
        diff_index = node_b - node_a;
      }

      ROS_INFO_STREAM("diff: " << gtsam::Symbol(diff_index).index());
      bool logic = diff_index > close_keys_threshold_;
      ROS_INFO_STREAM("LOGIC indexes: " << logic);
      bool logic2 =
          gtsam::Symbol(rssi_node_dropped.second.pose_graph_node.key).chr() !=
          gtsam::Symbol(node_around_comm.second.second.key).chr();
      ROS_INFO_STREAM("LOGIC robot: " << logic2);
      if (logic or logic2) {
        pose_graph_msgs::LoopCandidate candidate;
        candidate.header.stamp = ros::Time::now();
        candidate.key_from = rssi_node_dropped.second.pose_graph_node.key;
        candidate.key_to = node_around_comm.second.second.key;
        candidate.pose_from = rssi_node_dropped.second.pose_graph_node.pose;
        candidate.pose_to = node_around_comm.second.second.pose;
        candidate.type = pose_graph_msgs::LoopCandidate::PROXIMITY;
        candidate.value = 0.0;
        candidates_.push_back(candidate);
        VisualizeEdgesForPotentialLoopClosure(
            rssi_node_dropped.second.pose_graph_node,
            node_around_comm.second.second);
        node_around_comm.second.first = true;
      } else {
        node_around_comm.second.first = true;
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
    m.color.a = 0.5;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.id = indx;
    m.pose.position = node.second.pose_graph_node.pose.position;
    visualize_rssi_placement.publish(m);
    indx++;
  }
}

void RssiLoopClosure::VisualizeRobotNodesWithPotentialLoopClosure(
    const pose_graph_msgs::PoseGraphNode& node_pose,
    float red,
    float green,
    float blue,
    float scale) {
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
  m.id = idx2++;
  m.pose.position = node_pose.pose.position;
  visualize_rssi_placement.publish(m);
}

bool RssiLoopClosure::VisualizeEdgesForPotentialLoopClosure(
    const pose_graph_msgs::PoseGraphNode& node1,
    const pose_graph_msgs::PoseGraphNode& node2) {
  ROS_INFO("Highlighting factor between %lu and %lu.",
           node1.pose.position,
           node2.pose.position);

  visualization_msgs::Marker m;
  m.header.frame_id = "world";

  m.id = idx2++;
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
  VisualizeRobotNodesWithPotentialLoopClosure(node2, 0.0, 1.0, 0.0, 1.0);

  return true;
}
} // namespace lamp_loop_closure
