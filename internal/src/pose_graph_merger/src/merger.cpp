#include <pose_graph_merger/merger.hpp>

namespace gu = geometry_utils;

Merger::Merger(ros::NodeHandle nh, ros::NodeHandle pnh) :
    nodeHandle(nh),
    privNodeHandle(pnh),
    b_received_first_fast_pose_(false),
    b_received_first_slow_pose_(false),
    b_block_slow_pose_update(false),
    lastSlow(nullptr)
{

    this->fastGraphSub = this->nodeHandle.subscribe("blam_slam_fe/pose_graph", 1, &Merger::on_fast_graph_msg, this);

    this->slowGraphSub = this->nodeHandle.subscribe("blam_slam/pose_graph_lc", 1, &Merger::on_slow_graph_msg, this);

    this->mergedGraphPub = this->nodeHandle.advertise<pose_graph_msgs::PoseGraph>
            ("blam_slam/pose_graph", 1, true);

    this->fastPoseSub = this->nodeHandle.subscribe("blam_slam_fe/localization_integrated_estimate", 1, &Merger::on_fast_pose_msg, this);

    this->slowPoseSub = this->nodeHandle.subscribe("blam_slam/localization_integrated_estimate_lc", 1, &Merger::on_slow_pose_msg, this);

    this->mergedPosePub = this->nodeHandle.advertise<geometry_msgs::PoseStamped>
            ("blam_slam/localization_integrated_estimate", 1, true);            
}

void Merger::on_slow_graph_msg(const pose_graph_msgs::PoseGraphConstPtr &msg) {
    ROS_INFO_STREAM("Recevived slow graph");
    this->lastSlow = msg;
}

void Merger::on_fast_graph_msg(const pose_graph_msgs::PoseGraphConstPtr &msg) {
    ROS_INFO_STREAM("Recevived fast graph");
    if (lastSlow == nullptr) {
        ROS_WARN("Fast graph recieved but no slow pose graph recieved");
        return;
    }

    //initialise merged graph as a copy of the most recent slow graph
    pose_graph_msgs::PoseGraph mergedGraph;
    mergedGraph.header = msg->header;

    std::map<long unsigned int, int> mergedGraphKeyToIndex;

    for (const Node& node : this->lastSlow->nodes) {
        mergedGraphKeyToIndex[node.key] = mergedGraph.nodes.size();
        mergedGraph.nodes.push_back(node);
    }
    for (const Edge& edge: this->lastSlow->edges) {
        mergedGraph.edges.push_back(edge);
    }


    //use map to order the new fast nodes by the order they were created in
    std::map<unsigned int, const Node*> newFastNodes;
    std::map<long unsigned int, const Node*> fastKeyToNode;

    for (const Node& node : msg->nodes) {
        if (mergedGraphKeyToIndex.count(node.key) != 0) continue; //skip if this node is in the slow graph

        newFastNodes[node.header.seq] = &node;
        fastKeyToNode[node.key] = &node;
    }

    std::map<long unsigned int, std::set<const Edge*>> fastOutAdjList;
    std::map<long unsigned int, std::set<const Edge*>> fastInAdjList;
    for (const Edge& edge : msg->edges) {
        fastOutAdjList[edge.key_from].insert(&edge);
        fastInAdjList[edge.key_to].insert(&edge);
    }

    //for each node in the fast graph which is not in the graph
    for (auto kv : fastKeyToNode) {

        //the fast node to add to the mergedGraph
        const Node* fastNode = kv.second;

        //edge in the fast graph to this fast node
        const Edge* edgeToFastNode = *fastInAdjList[fastNode->key].begin();

        //create a copy of the fast node and edge to add to the mergedGraph
        Node newMergedGraphNode = *fastNode;
        Edge newMergedGraphEdge = *edgeToFastNode;

        //find the node in the mergedGraph corresponding to previous node in fast graph
        long unsigned int prevFastKey = edgeToFastNode->key_from;
        const Node* mergedGraphPrevNode = &mergedGraph.nodes[mergedGraphKeyToIndex[prevFastKey]];

        //calculate the pose of the new merged graph node by applying the edge transformation to the previous node
        Eigen::Affine3d newMergedGraphEdgeTf;
        tf::poseMsgToEigen(newMergedGraphEdge.pose, newMergedGraphEdgeTf);
        // ROS_INFO_STREAM("edge tf is " << newMergedGraphEdgeTf.matrix());

        Eigen::Affine3d mergedGraphPrevNodeTf;
        tf::poseMsgToEigen(mergedGraphPrevNode->pose, mergedGraphPrevNodeTf);
        // ROS_INFO_STREAM("prev node tf is " << mergedGraphPrevNodeTf.matrix());

        Eigen::Affine3d currGraphNodeTf;
        currGraphNodeTf = mergedGraphPrevNodeTf * newMergedGraphEdgeTf;
        // ROS_INFO_STREAM("Resulting tf is " << currGraphNodeTf.matrix());
        tf::poseEigenToMsg(currGraphNodeTf, newMergedGraphNode.pose);

        mergedGraphKeyToIndex[newMergedGraphNode.key] = mergedGraph.nodes.size();
        mergedGraph.nodes.push_back(newMergedGraphNode);
        mergedGraph.edges.push_back(newMergedGraphEdge);
    }

    this->mergedGraphPub.publish(mergedGraph);
}

void Merger::on_slow_pose_msg(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO_STREAM("Recevived slow pose");
    ROS_INFO_STREAM("Slow timestamp is " << msg->header.stamp.toSec());
    
    if (!b_block_slow_pose_update){ // blocked if updated in fast_pose callback
        this->last_slow_pose_ = gu::ros::FromROS(msg->pose);

        // Store the fast pose when we get the slow pose
        if (b_received_first_fast_pose_){
            fast_pose_at_slow_ = get_pose_at_time(msg->header.stamp);
            // Assume that there will be many fast poses before the first slow pose
            clean_up_map(msg->header.stamp);
        }

        b_received_first_slow_pose_ = true;
    }
    
}

void Merger::on_fast_pose_msg(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // ROS_INFO_STREAM("Recevived fast pose");
    // ROS_INFO_STREAM("Fast timestamp is " << msg->header.stamp.toSec());

    // Update current fast pose
    current_fast_pose_ = gu::ros::FromROS(msg->pose);

    // Add to keyed buffer
    timestamped_poses_[msg->header.stamp.toSec()] = current_fast_pose_;

    if (!b_received_first_slow_pose_){
        // Publish the fast message directly.
        geometry_msgs::PoseStamped out_msg = *msg;
        mergedPosePub.publish(out_msg);
    }

    if (!b_received_first_fast_pose_){
        // Initialize Need first pose to get deltas]
        fast_pose_at_slow_ = current_fast_pose_;
        b_received_first_fast_pose_ = true;
        return;
    }

    // Update pose from the fast pose at the time of the last slow pose update
    geometry_utils::Transform3 delta = gu::PoseDelta(this->fast_pose_at_slow_, this->current_fast_pose_);
    b_block_slow_pose_update = true;
    this->current_pose_est_ = gu::PoseUpdate(this->last_slow_pose_, delta);

    // Fill message
    geometry_msgs::PoseStamped out_msg = *msg;
    out_msg.pose =  gu::ros::ToRosPose(this->current_pose_est_);

    // Publish
    mergedPosePub.publish(out_msg);

    b_block_slow_pose_update = false;
}

// Get the pose at a given time
geometry_utils::Transform3 Merger::get_pose_at_time(const ros::Time &stamp){

  geometry_utils::Transform3 pose;

  if (timestamped_poses_.size() == 0){
    ROS_WARN("No poses in map..., returning identity");
    return pose;
  }

  if (timestamped_poses_.size() == 1){
      ROS_INFO("Only one fast pose in map, using it");
      return std::prev(timestamped_poses_.end(),1)->second;
  }

  auto iter = timestamped_poses_.lower_bound(stamp.toSec()); // First key that is not less than timestamp 
  ROS_INFO_STREAM("slow timestamp is " << stamp.toSec());

  double t2 = iter->first;
  double t1 = std::prev(iter,1)->first; 

  if (t2-stamp.toSec() < stamp.toSec() - t1) {
    // t2 is closer - use that poes
    pose = iter->second;
  } else {
    // t1 is closer - use that key
    pose = std::prev(iter,1)->second;
    iter--;
  }
  if (iter == std::prev(timestamped_poses_.begin())){
    iter++;
    pose = iter->second;
  } else if(iter == timestamped_poses_.end()) {
    ROS_WARN("Invalid time for graph (past end of graph range). take latest pose");
    pose = std::prev(timestamped_poses_.end(),1)->second;
    iter--;
  }

  ROS_INFO_STREAM("Corresponding fast timestamp is " << iter->first);

  return pose; 
}

void Merger::clean_up_map(const ros::Time &stamp){

  auto iter = timestamped_poses_.lower_bound(stamp.toSec()); // First key that is not less than timestamp 

  // Erase from the start to the last time below the current.
  ROS_INFO_STREAM("Size of timestamped poses before erase is: " << timestamped_poses_.size());
  timestamped_poses_.erase(timestamped_poses_.begin(),std::prev(iter,1));
  ROS_INFO_STREAM("Size of timestamped poses after erase is: " << timestamped_poses_.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_graph_merger");
    ros::NodeHandle nh, pnh("~");

    Merger merger(nh, pnh);

    ros::spin();
}

