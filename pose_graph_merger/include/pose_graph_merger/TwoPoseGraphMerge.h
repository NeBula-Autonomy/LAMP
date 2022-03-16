/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef TWO_POSE_GRAPH_MERGE_H
#define TWO_POSE_GRAPH_MERGE_H

#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <pose_graph_merger/merger.h>

#include <utils/CommonFunctions.h>


// Class definition
class TwoPoseGraphMerge {

  friend class TestTwoPoseGraphMerge;

  public:
    // Constructor
    TwoPoseGraphMerge();

    // Destructor
    ~TwoPoseGraphMerge();

    bool Initialize(const ros::NodeHandle& n);


  protected:
    // TODO: make most of these pure virtual

    // Use this for any "private" things to be used in the derived class
    // Node initialization.
    // Set precisions for fixed covariance settings
    bool CreatePublishers(const ros::NodeHandle& n);
    bool CreateSubscribers(const ros::NodeHandle& n);
    std::string GetRobotName(const ros::NodeHandle& n);

    void ProcessBaseGraph(const pose_graph_msgs::PoseGraphConstPtr& msg);
    void ProcessRobotGraph(const pose_graph_msgs::PoseGraphConstPtr& msg);
    void ProcessRobotPose(const geometry_msgs::PoseStampedConstPtr& msg);

    geometry_msgs::PoseStamped GetLatestOdomPose(
            const pose_graph_msgs::PoseGraphConstPtr& msg, 
            char target_prefix);

    pose_graph_msgs::PoseGraph GetMergedGraph();
    void PublishPoses();

    // Publishers
    ros::Publisher merged_graph_pub_;
    ros::Publisher rob_node_pose_pub_;
    ros::Publisher merged_node_pose_pub_;
    ros::Publisher merged_pose_pub_;

    // Subscribers
    ros::Subscriber base_pose_graph_sub_;
    ros::Subscriber robot_pose_graph_sub_;
    ros::Subscriber robot_pose_sub_;

    // Main process name
    std::string name_;

    // Booleans
    bool first_call_;
    bool b_publish_on_slow_graph_;
    bool b_have_first_robot_graph_;

    // Symbols
    char robot_prefix_;

    // Frames
    std::string world_fid_;
    std::string world2_fid_;
    
    // Pose graph merger
    Merger merger_;

    // Poses
    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::PoseStamped merged_pose_;

    // Store latest robot graph
    pose_graph_msgs::PoseGraphConstPtr last_robot_graph_;

  private:

};

#endif