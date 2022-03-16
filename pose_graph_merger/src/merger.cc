#include <pose_graph_merger/merger.h>

namespace gu = geometry_utils;

Merger::Merger()
  : b_received_first_fast_pose_(false),
    b_received_first_slow_pose_(false),
    b_block_slow_pose_update(false),
    lastSlow(nullptr) {}

void Merger::InsertNewEdges(const pose_graph_msgs::PoseGraphConstPtr& msg) {

  if (msg->edges.size() == 0){
    // No edges - nothing to add
    return;
  }

  // New edges that we want to update with
  std::map<std::tuple<gtsam::Key, gtsam::Key, int>,
           pose_graph_msgs::PoseGraphEdge>
      updated_artifact_edges;

  // Add new edges and skip existing edges
  for (const GraphEdge& edge : msg->edges) {
    std::tuple<gtsam::Key, gtsam::Key, int> id =
        std::make_tuple(edge.key_from, edge.key_to, edge.type);
    if (IsEdgeNew(edge)){
      // Add to the merged graph
      merged_graph_.edges.push_back(edge);

      // Add to stored set of edges
      unique_edges_.insert(id);
    } else if (edge.type == pose_graph_msgs::PoseGraphEdge::ARTIFACT) {
      ROS_DEBUG_STREAM("\nMerger: Repeated artifact edge with key to "
                       << gtsam::DefaultKeyFormatter(edge.key_to));
      updated_artifact_edges[id] = edge;
    }
  }

  // Replace the existing artifact edges with updated_artifact_edges
  pose_graph_msgs::PoseGraph new_graph;

  auto e = merged_graph_.edges.begin();
  while (e != merged_graph_.edges.end()) {
    auto edge_itr = *e;
    if (edge_itr.type == pose_graph_msgs::PoseGraphEdge::ARTIFACT) {
      std::tuple<gtsam::Key, gtsam::Key, int> id =
          std::make_tuple(edge_itr.key_from, edge_itr.key_to, edge_itr.type);
      if (updated_artifact_edges.find(id) != updated_artifact_edges.end()) {
        e++;
        continue;
      }
    }
    new_graph.edges.push_back(edge_itr);
    e++;
  }

  auto itr = updated_artifact_edges.begin();
  while (itr != updated_artifact_edges.end()) {
    new_graph.edges.push_back(itr->second);
    itr++;
  }
  merged_graph_.edges = new_graph.edges;
}

void Merger::InsertNode(const pose_graph_msgs::PoseGraphNode& node) {
  // Just update the node if it is already exist
  if (merged_graph_KeyToIndex_.find(node.key) !=
      merged_graph_KeyToIndex_.end()) {
    merged_graph_.nodes[merged_graph_KeyToIndex_[node.key]] = node;
    ROS_DEBUG_STREAM(
        "\n[Insert Node] key to index mapping already exists, with key: "
        << gtsam::DefaultKeyFormatter(node.key) << " and index "
        << merged_graph_KeyToIndex_[node.key]);
    return;
  }

  // Track the index at which the node was inserted
  ROS_DEBUG_STREAM("\nAdding new key to index mapping, with key: "
                   << gtsam::DefaultKeyFormatter(node.key) << " and index "
                   << merged_graph_.nodes.size());
  merged_graph_KeyToIndex_[node.key] = merged_graph_.nodes.size();

  // Add the node to the graph
  merged_graph_.nodes.push_back(node);

  // If the node is from a new robot, add it to the set of robots in the merged graph
  auto prefix = gtsam::Symbol(node.key).chr();
  if (robots_.count(prefix) == 0) {
    robots_.insert(prefix);
  }
}

void Merger::ClearNodes() {
  merged_graph_.nodes.clear();
  merged_graph_KeyToIndex_.clear();
}

bool Merger::IsEdgeNew(const pose_graph_msgs::PoseGraphEdge& msg) {
  // Checks to see if an edge is new
  std::tuple<gtsam::Key, gtsam::Key, int> id = std::make_tuple(msg.key_from, msg.key_to, msg.type);
  return unique_edges_.count(id) == 0;
}

std::set<char> Merger::GetNewRobots(const pose_graph_msgs::PoseGraphConstPtr& msg) {

  std::set<char> new_robots;

  for (const GraphNode& node : msg->nodes) {
  auto prefix = gtsam::Symbol(node.key).chr();
    if (!utils::IsRobotPrefix(prefix)) {
      continue;
    }

    // This node is a robot that is not currently in the merged graph
    if (robots_.count(prefix) == 0) {
      new_robots.insert(prefix);
    }
  }

  return new_robots;
}

void Merger::OnSlowGraphMsg(const pose_graph_msgs::PoseGraphConstPtr& msg) {
  ROS_DEBUG_STREAM("Received slow graph, size " << msg->nodes.size());

  // Clear existing nodes
  ClearNodes();

  // Insert all Nodes - slow graph should be the most accurate and up to date
  for (const GraphNode& node : msg->nodes) {
    InsertNode(node);
  }

  InsertNewEdges(msg);
}

void Merger::OnFastGraphMsg(const pose_graph_msgs::PoseGraphConstPtr& msg) {
  ROS_DEBUG_STREAM("Received fast graph, size " << msg->nodes.size());

  // NOTE: Assuming these messages are only from one robot

  std::set<char> new_robots = GetNewRobots(msg);

  // If no slow graph (or an empty slow graph only) has been received, merged
  // graph is the fast graph only
  if (merged_graph_.nodes.size() == 0 || new_robots.size() > 0) {
    ROS_DEBUG_STREAM("Fast graph callback without any slow graphs");

    for (const GraphNode& node : msg->nodes) {
      InsertNode(node);
    }

    InsertNewEdges(msg);
    return;
  }

  // Add new edges
  InsertNewEdges(msg);

  // Get header from the fastGraph - most recent graph
  merged_graph_.header = msg->header;

  std::map<long unsigned int, std::set<const GraphEdge*>> fastOutAdjList;
  std::map<long unsigned int, std::set<const GraphEdge*>> fastInAdjList;
  for (const GraphEdge& edge : msg->edges) {
    fastOutAdjList[edge.key_from].insert(&edge);
    fastInAdjList[edge.key_to].insert(&edge);
  }

  // use map to order the new fast nodes by the order they were created in
  // std::map<unsigned int, const GraphNode*> newFastNodes;
  std::map<long unsigned int, const GraphNode*> fastKeyToNode;

  for (const GraphNode& node : msg->nodes) {
    if (merged_graph_KeyToIndex_.count(node.key) != 0) {
      // Replace the stamp with the fast graph stamp (most correct)
      merged_graph_.nodes[merged_graph_KeyToIndex_[node.key]].header =
          node.header;
      // TODO 1: we want to add the artifact anyway
      // if artifact node -> add that

      if (fastInAdjList.find(node.key) != fastInAdjList.end()) {
        const GraphEdge* edge_to_check = *fastInAdjList[node.key].begin();
        if (edge_to_check->type == pose_graph_msgs::PoseGraphEdge::ARTIFACT) {
          ROS_DEBUG_STREAM(
              "\nDebug Merger: Adding the reobserved artifact to newfastnode "
              << gtsam::DefaultKeyFormatter(node.key));
          // newFastNodes[node.header.seq] = &node;
          fastKeyToNode[node.key] = &node;
        }
      }
      //
      continue; // Then skip
    }

    // newFastNodes[node.header.seq] = &node;
    fastKeyToNode[node.key] = &node;
    // ROS_INFO_STREAM("Added new fast node, key " << node.key);
  }

  // for each node in the fast graph which is not in the graph
  for (auto kv : fastKeyToNode) {
    // ROS_INFO_STREAM("Adding new node");
    // the fast node to add to the merged_graph_
    const GraphNode* fastNode = kv.second;
    // TODO 2: we skip the artifact edge if there is no edge. Find == end

    // edge in the fast graph to this fast node
    const GraphEdge* edgeToFastNode = *fastInAdjList[fastNode->key].begin();

    // create a copy of the fast node and edge to add to the merged_graph_
    GraphNode new_merged_graph_node = *fastNode;
    GraphEdge new_merged_graph_edge = *edgeToFastNode;

    // find the node in the merged_graph_ corresponding to previous node in fast
    // graph
    long unsigned int prevFastKey = edgeToFastNode->key_from;
    // Check if the prior node exists
    if (merged_graph_KeyToIndex_.count(prevFastKey) == 0) {
      // Prior node doesn't exist - don't adjust
      ROS_WARN_STREAM("[FastGraph] Have missing node with an edge-from. Key: "
                      << gtsam::DefaultKeyFormatter(prevFastKey)
                      << ", edge to: "
                      << gtsam::DefaultKeyFormatter(fastNode->key)
                      << ". Using current robot-graph value.");
      // Just use the existing pose value
      ROS_DEBUG_STREAM("\n[Fast Graph Add] Adding new node with key "
                       << gtsam::DefaultKeyFormatter(new_merged_graph_node.key)
                       << ", with edge from "
                       << gtsam::DefaultKeyFormatter(prevFastKey));
      InsertNode(new_merged_graph_node);
      continue;
    }

    const GraphNode* merged_graph_PrevNode =
        &merged_graph_.nodes[merged_graph_KeyToIndex_[prevFastKey]];

    // calculate the pose of the new merged graph node by applying the edge
    // transformation to the previous node
    Eigen::Affine3d new_merged_graph_edge_tf;
    tf::poseMsgToEigen(new_merged_graph_edge.pose, new_merged_graph_edge_tf);
    // ROS_INFO_STREAM("edge tf is " << new_merged_graph_edge_tf.matrix());

    Eigen::Affine3d merged_graph_PrevNodeTf;
    tf::poseMsgToEigen(merged_graph_PrevNode->pose, merged_graph_PrevNodeTf);
    // ROS_INFO_STREAM("prev node tf is " << merged_graph_PrevNodeTf.matrix());

    Eigen::Affine3d currGraphNodeTf = merged_graph_PrevNodeTf * new_merged_graph_edge_tf;
    // ROS_INFO_STREAM("Resulting tf is " << currGraphNodeTf.matrix());
    tf::poseEigenToMsg(currGraphNodeTf, new_merged_graph_node.pose);

    // normalise pose rotation
    NormalizeNodeOrientation(new_merged_graph_node);

    // Add nodes to the graph
    ROS_DEBUG_STREAM("\n[Fast Graph Add] Adding new node with key "
                     << gtsam::DefaultKeyFormatter(new_merged_graph_node.key)
                     << ", with edge from "
                     << gtsam::DefaultKeyFormatter(prevFastKey));
    InsertNode(new_merged_graph_node);
  }

  ROS_DEBUG_STREAM("Finished merging graph, size "
                  << merged_graph_.nodes.size());
}

void Merger::NormalizeNodeOrientation(pose_graph_msgs::PoseGraphNode & msg){
    double norm = pow(msg.pose.orientation.w*msg.pose.orientation.w + msg.pose.orientation.x*msg.pose.orientation.x + msg.pose.orientation.y*msg.pose.orientation.y + msg.pose.orientation.z*msg.pose.orientation.z, 0.5);
    msg.pose.orientation.w = msg.pose.orientation.w/norm;
    msg.pose.orientation.x = msg.pose.orientation.x/norm;
    msg.pose.orientation.y = msg.pose.orientation.y/norm;
    msg.pose.orientation.z = msg.pose.orientation.z/norm;
}

void Merger::OnSlowPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // if (!b_block_slow_pose_update){ // blocked if updated in fast_pose callback
  this->last_slow_pose_ = gu::ros::FromROS(msg->pose);

  // Store the fast pose when we get the slow pose
  if (b_received_first_fast_pose_) {
    fast_pose_at_slow_ = GetPoseAtTime(msg->header.stamp);
    // Assume that there will be many fast poses before the first slow pose
    CleanUpMap(msg->header.stamp);
  }

  b_received_first_slow_pose_ = true;
}

void Merger::OnFastPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // Update current fast pose
  current_fast_pose_ = gu::ros::FromROS(msg->pose);

  // Add to keyed buffer
  timestamped_poses_[msg->header.stamp.toSec()] = current_fast_pose_;

  if (!b_received_first_slow_pose_) {
    // Publish the fast message directly.
    geometry_msgs::PoseStamped out_msg = *msg;
    mergedPosePub.publish(out_msg);
  }

  if (!b_received_first_fast_pose_) {
    // Initialize Need first pose to get deltas]
    fast_pose_at_slow_ = current_fast_pose_;
    b_received_first_fast_pose_ = true;
    return;
  }

  // Update pose from the fast pose at the time of the last slow pose update
  geometry_utils::Transform3 delta =
      gu::PoseDelta(this->fast_pose_at_slow_, this->current_fast_pose_);
  b_block_slow_pose_update = true;
  this->current_pose_est_ = gu::PoseUpdate(this->last_slow_pose_, delta);

  // Fill message
  geometry_msgs::PoseStamped out_msg = *msg;
  out_msg.pose = gu::ros::ToRosPose(this->current_pose_est_);

  // Publish
  mergedPosePub.publish(out_msg);

  b_block_slow_pose_update = false;
}

// Get the pose at a given time
geometry_utils::Transform3 Merger::GetPoseAtTime(const ros::Time& stamp) {
  geometry_utils::Transform3 pose;

  if (timestamped_poses_.size() == 0) {
    ROS_WARN("No poses in map..., returning identity");
    return pose;
  }

  if (timestamped_poses_.size() == 1) {
    ROS_DEBUG("Only one fast pose in map, using it");
    return std::prev(timestamped_poses_.end(), 1)->second;
  }

  auto iter = timestamped_poses_.lower_bound(
      stamp.toSec()); // First key that is not less than timestamp
  ROS_DEBUG_STREAM("slow timestamp is " << stamp.toSec());

  double t2 = iter->first;
  double t1 = std::prev(iter, 1)->first;

  if (t2 - stamp.toSec() < stamp.toSec() - t1) {
    // t2 is closer - use that poes
    pose = iter->second;
  } else {
    // t1 is closer - use that key
    pose = std::prev(iter, 1)->second;
    iter--;
  }
  if (iter == std::prev(timestamped_poses_.begin())) {
    iter++;
    pose = iter->second;
  } else if (iter == timestamped_poses_.end()) {
    ROS_WARN(
        "Invalid time for graph (past end of graph range). take latest pose");
    pose = std::prev(timestamped_poses_.end(), 1)->second;
    iter--;
  }

  ROS_DEBUG_STREAM("Corresponding fast timestamp is " << iter->first);

  return pose;
}

void Merger::CleanUpMap(const ros::Time& stamp) {
  auto iter = timestamped_poses_.lower_bound(
      stamp.toSec()); // First key that is not less than timestamp

  // Erase from the start to the last time below the current.
  ROS_DEBUG_STREAM("Size of timestamped poses before erase is: "
                  << timestamped_poses_.size());
  timestamped_poses_.erase(timestamped_poses_.begin(), std::prev(iter, 1));
  ROS_DEBUG_STREAM("Size of timestamped poses after erase is: "
                  << timestamped_poses_.size());
}

pose_graph_msgs::PoseGraph Merger::GetCurrentGraph() {
  return merged_graph_;
}
