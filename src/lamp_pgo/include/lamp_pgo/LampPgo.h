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
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>

#include <utils/PrefixHandling.h>

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

  ros::Subscriber remove_lc_sub_;

  ros::Subscriber remove_lc_by_id_sub_;

  void PublishValues() const;

  void InputCallback(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg);

  void RemoveLCByIdCallback(const std_msgs::String::ConstPtr& msg);

  void RemoveLCCallback(const std_msgs::Bool::ConstPtr& msg);

  void RemoveLastLoopClosure(char prefix_1, char prefix_2);

  void RemoveLastLoopClosure();

 private:
  // Optimizer parameters
  KimeraRPGO::RobustSolverParams rpgo_params_;
  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_solver_;  // actual solver

  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;

  // Parameter namespace ("robot" or "base")
  std::string param_ns_;

  // Keep track of node IDs for output message (not stored by GTSAM types)
  std::map<gtsam::Key, std::string> key_to_id_map_;
};

#endif  // LAMP_PGO_H_
