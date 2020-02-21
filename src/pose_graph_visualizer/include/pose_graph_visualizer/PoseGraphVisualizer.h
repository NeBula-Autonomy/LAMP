#ifndef POSE_GRAPH_VISUALIZER_H
#define POSE_GRAPH_VISUALIZER_H

#include <ros/ros.h>
#include <functional>
#include <unordered_map>

#include <pose_graph_visualizer/HighlightEdge.h>
#include <pose_graph_visualizer/HighlightNode.h>


#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>

#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>
#include <std_msgs/Bool.h>

#include <visualization_msgs/Marker.h>

#include <core_msgs/Artifact.h>

#include <pcl_ros/point_cloud.h>

#include <tf/transform_datatypes.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/PriorFactor.h>

#include <utils/CommonFunctions.h>
#include <utils/CommonStructs.h>

namespace gu = geometry_utils;

class PoseGraphVisualizer {
public:
  PoseGraphVisualizer() = default;
  ~PoseGraphVisualizer() = default;

  bool Initialize(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void MakeMenuMarker(const gtsam::Pose3& pose, const std::string& id_number);

  // Visualizes an edge between the two keys.
  bool HighlightEdge(gtsam::Key key1, gtsam::Key key2);
  // Removes the edge visualization between the two keys.
  // Removes all highlighting visualizations if both keys are zero.
  void UnhighlightEdge(gtsam::Key key1, gtsam::Key key2);

  // Highlights factor graph node associated with the given key.
  bool HighlightNode(gtsam::Key key);
  // Unhighlights factor graph node associated with the given key.
  // Removes all highlighting visualizations if the key is zero.
  void UnhighlightNode(gtsam::Key key);

  void VisualizePoseGraph();
  void VisualizeArtifacts();

  // Artifacts and labels.
  struct ArtifactInfo {
    // gtsam::Key pose_key;
    core_msgs::Artifact msg;
  };

  void VisualizeSingleRealisticArtifact(visualization_msgs::Marker& m,
                                        const ArtifactInfo& art);
  void VisualizeSingleSimpleArtifact(visualization_msgs::Marker& m,
                                     const ArtifactInfo& art);
  void VisualizeSingleArtifactId(visualization_msgs::Marker& m,
                                 const ArtifactInfo& art);

private:
  PoseGraph pose_graph_;

  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& msg);
  void ErasePosegraphCallback(const std_msgs::Bool::ConstPtr& msg);
  void RemoveFactorVizCallback(const std_msgs::Bool::ConstPtr& msg);
  void PoseGraphCallback(const pose_graph_msgs::PoseGraph::ConstPtr& msg);
  void
  PoseGraphNodeCallback(const pose_graph_msgs::PoseGraphNode::ConstPtr& msg);
  void
  PoseGraphEdgeCallback(const pose_graph_msgs::PoseGraphEdge::ConstPtr& msg);
  void ArtifactCallback(const core_msgs::Artifact& msg);

  bool
  HighlightNodeService(pose_graph_visualizer::HighlightNodeRequest& request,
                       pose_graph_visualizer::HighlightNodeResponse& response);
  bool
  HighlightEdgeService(pose_graph_visualizer::HighlightEdgeRequest& request,
                       pose_graph_visualizer::HighlightEdgeResponse& response);

  geometry_msgs::Point GetPositionMsg(gtsam::Key key) const;

  // Node name.
  std::string name_;

  // Frames.
  std::string base_frame_id_;
  bool artifacts_in_global_;

  std::unordered_map<std::string, ArtifactInfo>
      artifacts_; // Keyed with UUID so can build this when we get artifact
                  // messages
  std::unordered_map<std::string, gtsam::Key> artifact_id2key_hash_;
  std::unordered_map<gtsam::Key, std::string> artifact_key2id_hash_;
  std::unordered_map<std::string, std::string> artifact_ID2ParentID_;
  std::unordered_map<std::string, std::string> current_parentID2ID_;
  std::hash<std::string> parent_id2int_;
  Eigen::Vector3d GetArtifactPosition(const gtsam::Key artifact_key) const;

  // Visualization publishers.
  ros::Publisher odometry_edge_pub_;
  ros::Publisher loop_edge_pub_;
  ros::Publisher artifact_edge_pub_;
  ros::Publisher uwb_edge_pub_;
  ros::Publisher uwb_edge_between_pub_;
  ros::Publisher uwb_node_pub_;
  ros::Publisher graph_node_pub_;
  ros::Publisher graph_node_id_pub_;
  ros::Publisher keyframe_node_pub_;
  ros::Publisher closure_area_pub_;
  ros::Publisher highlight_pub_;
  ros::Publisher artifact_marker_pub_;
  ros::Publisher artifact_id_marker_pub_;
  ros::Publisher stair_marker_pub_;

  // Subscribers.
  ros::Subscriber keyed_scan_sub_;
  ros::Subscriber pose_graph_sub_;
  ros::Subscriber pose_graph_node_sub_;
  ros::Subscriber pose_graph_edge_sub_;
  ros::Subscriber artifact_sub_;
  ros::Subscriber erase_posegraph_sub_;
  ros::Subscriber remove_factor_viz_sub_;
  std::vector<ros::Subscriber> subscribers_artifacts_;

  // Robots that the base station subscribes to
  std::vector<std::string> robot_names_;

  // Services.
  ros::ServiceServer highlight_node_srv_;
  ros::ServiceServer highlight_edge_srv_;

  bool publish_interactive_markers_{true};

  bool b_use_base_reconciliation_{false};

  // Proximity threshold used by LaserLoopClosureNode.
  double proximity_threshold_{1};

  bool use_realistic_artifact_models_;
  bool b_scale_artifacts_with_confidence_;
  float confidence_scale_{1.0};
};

#endif
