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


// Class definition
class TwoPoseGraphMerge {

  friend class TestTwoPoseGraphMerge;

  public:
    // Constructor
    TwoPoseGraphMerge();

    // Destructor
    ~TwoPoseGraphMerge();

    // Define main interface functions

    bool Initialize(const ros::NodeHandle& n);


  protected:
    // TODO: make most of these pure virtual

    // Use this for any "private" things to be used in the derived class
    // Node initialization.
    // Set precisions for fixed covariance settings
    bool CreatePublishers(const ros::NodeHandle& n);
    bool CreateSubscribers(const ros::NodeHandle& n);

    void ProcessBaseGraph(const pose_graph_msgs::PoseGraphConstPtr& msg);
    void ProcessRobotGraph(const pose_graph_msgs::PoseGraphConstPtr& msg);

    geometry_msgs::PoseStamped GetLatestOdomPose(const pose_graph_msgs::PoseGraphConstPtr& msg);

    pose_graph_msgs::PoseGraph GetMergedGraph();
    void PublishPoses();

    // Publishers
    ros::Publisher merged_graph_pub_;
    ros::Publisher rob_node_pose_pub_;
    ros::Publisher merged_node_pose_pub_;

    // Subscribers
    ros::Subscriber base_pose_graph_sub_;
    ros::Subscriber robot_pose_graph_sub_;

    // Services

    // Main process name
    std::string name_;

    // Booleans
    bool first_call_;
    char robot_prefix_;
    
    // Pose graph merger
    Merger merger_;

    // Poses
    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::PoseStamped merged_pose_;

  private:
    // Anything just in the base class

};

#endif