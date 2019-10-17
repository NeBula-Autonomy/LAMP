/*
LampPgo.h
Author: Yun Chang
Interface for ROS and KimeraRPGO
*/

#ifndef LAMP_PGO_H_
#define LAMP_PGO_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>

#include "KimeraRPGO/RobustSolver.h"

class LampPgo {
public:
  // constructor destructor
  LampPgo();
  ~LampPgo();

  bool Initialize(const ros::NodeHandle& n);

private:
  // define publishers and subscribers
  ros::Publisher optimized_pub_;

  ros::Subscriber input_sub_;

  void PublishValues() const;

  void InputCallback(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg);

private:
  // Optimizer parameters
  KimeraRPGO::RobustSolverParams rpgo_params_;
  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_solver_; // actual solver

  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;
};

#endif // LAMP_PGO_H_
