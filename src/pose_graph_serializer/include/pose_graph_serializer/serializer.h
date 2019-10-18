#ifndef SERIALIZER_H
#define SERIALIZER_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>

#include <tf2/transform_datatypes.h>

#include <string>

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

typedef pose_graph_msgs::PoseGraphNode GraphNode;
typedef pose_graph_msgs::PoseGraphEdge GraphEdge;
typedef pose_graph_msgs::PoseGraph::ConstPtr GraphPtr;

typedef std::vector<GraphEdge> EdgeMessages;
typedef std::vector<GraphNode> NodeMessages;

class PoseGraphSerializer {
public:
  static bool ConvertFromMsg(const pose_graph_msgs::PoseGraphConstPtr& msg,
                             gtsam::Values* values,
                             EdgeMessages* edges_info,
                             NodeMessages* priors_info);

  static GraphPtr ConvertToMsg(const gtsam::Values& values,
                               const EdgeMessages& edges_info,
                               const NodeMessages& priors_info);

private:
  static bool ConvertValuesToNodeMsgs(const gtsam::Values& values,
                                      NodeMessages* nodes);
  // Test class fixtures
  friend class TestSerializer;
};

#endif