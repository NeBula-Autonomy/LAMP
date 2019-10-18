#ifndef SERIALIZER_H
#define SERIALIZER_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>

#include <pcl_ros/point_cloud.h>

#include <tf2/transform_datatypes.h>

#include <string>

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

typedef pose_graph_msgs::PoseGraphNode GraphNode;
typedef pose_graph_msgs::PoseGraphEdge GraphEdge;
typedef pose_graph_msgs::PoseGraph::ConstPtr GraphPtr;

typedef std::vector<GraphEdge> EdgeMessages;
typedef std::vector<GraphNode> NodeMessages;

// Typedef for stored point clouds.
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct PoseGraph {
  gtsam::Values values;
  gtsam::NonlinearFactorGraph nfg;

  std::string fixed_frame_id;

  // Keep a list of keyed laser scans and keyed timestamps.
  std::map<gtsam::Symbol, PointCloud::ConstPtr> keyed_scans;
  std::map<gtsam::Symbol, ros::Time> keyed_stamps; // All nodes
  std::map<double, gtsam::Symbol> stamp_to_odom_key;

  // Message filters (if any)
  std::string prefix;

  // Initial key
  gtsam::Symbol initial_key;

  // Current key
  gtsam::Symbol key;

  EdgeMessages edges;
  NodeMessages priors;
};

class PoseGraphSerializer {
public:
  static bool ConvertFromMsg(const GraphPtr& msg, PoseGraph* pose_graph);

  static GraphPtr ConvertToMsg(const PoseGraph& pose_graph);

private:
  // Test class fixtures
  friend class TestSerializer;
};

#endif