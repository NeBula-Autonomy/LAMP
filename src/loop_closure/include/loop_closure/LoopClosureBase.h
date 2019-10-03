/*
LoopClosureBase.h
Author: Yun Chang
Loop closure base class
*/

#ifndef LOOP_CLOSURE_BASE_H_
#define LOOP_CLOSURE_BASE_H_

#include <map>
#include <unordered_map>

#include <pose_graph_msgs/PoseGraphEdge.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <gtsam/inference/Symbol.h>

class LoopClosure {
 public:
  LoopClosure(const ros::NodeHandle& n);
  ~LoopClosure();

 protected:
  std::unordered_map<gtsam::Key, ros::Time> keyed_stamps_;

  // define publishers and subscribers
  ros::Publisher loop_closure_pub_;

  ros::Subscriber keyed_stamps_sub_;

  virtual bool FindLoopClosures(
      gtsam::Key new_key,
      std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges) {}

  void InputCallback();  // TODO type
};
#endif  // LOOP_CLOSURE_BASE_H_