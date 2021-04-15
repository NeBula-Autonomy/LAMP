/**
 * @file   LoopTransformComputation.h
 * @brief  Base class for classes to find transform of loop closures
 * @author Yun Chang
 */
#pragma once

#include <deque>
#include <map>
#include <vector>

#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace lamp_loop_closure {

class LoopTransformComputation {
 public:
  LoopTransformComputation();
  ~LoopTransformComputation();

  virtual bool Initialize(const ros::NodeHandle& n) = 0;

  // Update the pose in the pose-graph-edge to be the transform computed through
  // icp, ransac, etc. Return false if loop closure is rejected
  virtual bool ComputeTransform(
      pose_graph_msgs::PoseGraphEdge* loop_closure) = 0;

 protected:
  void PublishLoopClosures(
      const std::vector<pose_graph_msgs::PoseGraphEdge> edges) const;

  std::string param_ns_;
};

}  // namespace lamp_loop_closure