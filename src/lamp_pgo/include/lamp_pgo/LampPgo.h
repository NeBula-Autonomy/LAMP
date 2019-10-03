/*
LampPgo.h
Author: Yun Chang
Interface for ROS and KimeraRPGO
*/

#ifndef LAMP_PGO_H_
#define LAMP_PGO_H_

#include <ros/console.h>
#include <ros/ros.h>

#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>

#include "KimeraRPGO/RobustSolver.h"

class LampPgo {
 public:
  LampPgo();

  Initialize(const ros::NodeHandle& n);

 private:
  // node handle
  ros::NodeHandle nh_;

  // define publishers and subscribers
  ros::Publisher optimized_pub_;

  ros::Subscriber input_sub_;

  void InputCallback(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg);

 private:
  // Optimizer parameters
  KimeraRPGO::RobustSolverParams rpgo_params_;
  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_solver_;  // actual solver
};

#endif  // LAMP_PGO_H_
