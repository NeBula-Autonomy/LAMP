#include <pose_graph_merger/merger.h>

namespace gu = geometry_utils;

Merger::Merger()
  : b_received_first_fast_pose_(false),
    b_received_first_slow_pose_(false),
    b_block_slow_pose_update(false),
    lastSlow(nullptr) {}

void Merger::OnSlowGraphMsg(const pose_graph_msgs::PoseGraphConstPtr& msg) {
  ROS_INFO_STREAM("Received slow graph, size " << msg->nodes.size());
  this->lastSlow = msg;
}

void Merger::OnFastGraphMsg(const pose_graph_msgs::PoseGraphConstPtr& msg) {
  ROS_INFO_STREAM("Received fast graph, size " << msg->nodes.size());

  // If no slow graph (or an empty slow graph only) has been received, merged
  // graph is the fast graph only
  if (lastSlow == nullptr || lastSlow->nodes.size() == 0) {
    for (const GraphNode& node : msg->nodes) {
      merged_graph_.nodes.push_back(node);
    }
    for (const GraphEdge& edge : msg->edges) {
      merged_graph_.edges.push_back(edge);
    }
    return;
  }

  // Clear the existing merged graph
  merged_graph_ = pose_graph_msgs::PoseGraph();

  // Get header from the fastGraph - most recent graph
  merged_graph_.header = msg->header;

  std::map<long unsigned int, int> merged_graph_KeyToIndex;

  // initialise merged graph as a copy of the most recent slow graph
  // Add slow graph nodes
  for (const GraphNode& node : this->lastSlow->nodes) {
    merged_graph_KeyToIndex[node.key] = merged_graph_.nodes.size();
    merged_graph_.nodes.push_back(node);
  }

  // Add edges from the fast graph that connect nodes in the slow graph
  for (const GraphEdge& edge : msg->edges) {
    // Only add edges that connect two nodes that are in the slow graph
    // These are not new edges for the optimized graph - others are
    if (merged_graph_KeyToIndex.count(edge.key_from) != 0 &&
        merged_graph_KeyToIndex.count(edge.key_to) != 0) {
      merged_graph_.edges.push_back(edge);
    }
  }

  // use map to order the new fast nodes by the order they were created in
  std::map<unsigned int, const GraphNode*> newFastNodes;
  std::map<long unsigned int, const GraphNode*> fastKeyToNode;

  for (const GraphNode& node : msg->nodes) {
    if (merged_graph_KeyToIndex.count(node.key) != 0)
      continue; // skip if this node is in the slow graph

    newFastNodes[node.header.seq] = &node;
    fastKeyToNode[node.key] = &node;
    ROS_INFO_STREAM("Added new fast node, key " << node.key);
  }

  std::map<long unsigned int, std::set<const GraphEdge*>> fastOutAdjList;
  std::map<long unsigned int, std::set<const GraphEdge*>> fastInAdjList;
  for (const GraphEdge& edge : msg->edges) {
    fastOutAdjList[edge.key_from].insert(&edge);
    fastInAdjList[edge.key_to].insert(&edge);
  }

  // for each node in the fast graph which is not in the graph
  for (auto kv : fastKeyToNode) {
    ROS_INFO_STREAM("Adding new node");
    // the fast node to add to the merged_graph_
    const GraphNode* fastNode = kv.second;

    // edge in the fast graph to this fast node
    const GraphEdge* edgeToFastNode = *fastInAdjList[fastNode->key].begin();

    // create a copy of the fast node and edge to add to the merged_graph_
    GraphNode new_merged_graph_node = *fastNode;
    GraphEdge new_merged_graph_edge = *edgeToFastNode;

    // find the node in the merged_graph_ corresponding to previous node in fast
    // graph
    long unsigned int prevFastKey = edgeToFastNode->key_from;
    const GraphNode* merged_graph_PrevNode =
        &merged_graph_.nodes[merged_graph_KeyToIndex[prevFastKey]];

    // calculate the pose of the new merged graph node by applying the edge
    // transformation to the previous node
    Eigen::Affine3d new_merged_graph_edge_tf;
    tf::poseMsgToEigen(new_merged_graph_edge.pose, new_merged_graph_edge_tf);
    // ROS_INFO_STREAM("edge tf is " << new_merged_graph_edge_tf.matrix());

    Eigen::Affine3d merged_graph_PrevNodeTf;
    tf::poseMsgToEigen(merged_graph_PrevNode->pose, merged_graph_PrevNodeTf);
    // ROS_INFO_STREAM("prev node tf is " << merged_graph_PrevNodeTf.matrix());

    Eigen::Affine3d currGraphNodeTf;
    currGraphNodeTf = merged_graph_PrevNodeTf * new_merged_graph_edge_tf;
    // ROS_INFO_STREAM("Resulting tf is " << currGraphNodeTf.matrix());
    tf::poseEigenToMsg(currGraphNodeTf, new_merged_graph_node.pose);

    merged_graph_KeyToIndex[new_merged_graph_node.key] =
        merged_graph_.nodes.size();
    merged_graph_.nodes.push_back(new_merged_graph_node);
    merged_graph_.edges.push_back(new_merged_graph_edge);
  }

  ROS_INFO_STREAM("Finished merging graph, size "
                  << merged_graph_.nodes.size());
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
    ROS_INFO("Only one fast pose in map, using it");
    return std::prev(timestamped_poses_.end(), 1)->second;
  }

  auto iter = timestamped_poses_.lower_bound(
      stamp.toSec()); // First key that is not less than timestamp
  ROS_INFO_STREAM("slow timestamp is " << stamp.toSec());

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

  ROS_INFO_STREAM("Corresponding fast timestamp is " << iter->first);

  return pose;
}

void Merger::CleanUpMap(const ros::Time& stamp) {
  auto iter = timestamped_poses_.lower_bound(
      stamp.toSec()); // First key that is not less than timestamp

  // Erase from the start to the last time below the current.
  ROS_INFO_STREAM("Size of timestamped poses before erase is: "
                  << timestamped_poses_.size());
  timestamped_poses_.erase(timestamped_poses_.begin(), std::prev(iter, 1));
  ROS_INFO_STREAM("Size of timestamped poses after erase is: "
                  << timestamped_poses_.size());
}

pose_graph_msgs::PoseGraph Merger::GetCurrentGraph() {
  return merged_graph_;
}