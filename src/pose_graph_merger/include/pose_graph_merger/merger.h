#ifndef MERGER_H
#define MERGER_H

#include <ros/ros.h>

#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphNode.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_utils/Transform3.h>
#include <geometry_utils/GeometryUtilsROS.h>

#include <tf2/transform_datatypes.h>

#include <string>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>

typedef pose_graph_msgs::PoseGraphNode GraphNode;
typedef pose_graph_msgs::PoseGraphEdge GraphEdge;

class Merger {
public:
    Merger();

    void OnFastGraphMsg(const pose_graph_msgs::PoseGraphConstPtr& msg);

    void OnSlowGraphMsg(const pose_graph_msgs::PoseGraphConstPtr& msg);

    void OnFastPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void OnSlowPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);


    // Utility functions
    void CleanUpMap(const ros::Time &stamp);
    pose_graph_msgs::PoseGraph GetCurrentGraph();

    geometry_utils::Transform3 GetPoseAtTime(const ros::Time &stamp);

private:

    pose_graph_msgs::PoseGraph current_graph_;
    pose_graph_msgs::PoseGraphConstPtr lastSlow;
    geometry_utils::Transform3 current_pose_est_;
    geometry_utils::Transform3 last_slow_pose_;
    geometry_utils::Transform3 current_fast_pose_;
    geometry_utils::Transform3 fast_pose_at_slow_;

    std::map<double, geometry_utils::Transform3> timestamped_poses_;
    
    bool b_received_first_fast_pose_;
    bool b_received_first_slow_pose_;

    bool b_block_slow_pose_update;

    ros::Subscriber fastGraphSub;
    ros::Subscriber slowGraphSub;
    ros::Subscriber fastPoseSub;
    ros::Subscriber slowPoseSub;

    ros::Publisher mergedGraphPub;
    ros::Publisher mergedPosePub;

    pose_graph_msgs::PoseGraph merged_graph_;

    // Test class fixtures
    friend class TestMerger;
};

#endif