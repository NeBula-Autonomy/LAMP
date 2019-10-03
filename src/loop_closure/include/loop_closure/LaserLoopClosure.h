/*
LaserLoopClosure.h
Author: Yun Chang
Lidar pointcloud based loop closure
*/

#ifndef LASER_LOOP_CLOSURE_H_
#define LASER_LOOP_CLOSURE_H_

#include "loop_closure/LoopClosureBase.h"

#include <map>
#include <unordered_map>

#include <ros/console.h>
#include <ros/ros.h>

#include <gtsam/inference/Symbol.h>

class LaserLoopClosure : public LoopClosure {
 public:
  LaserLoopClosure(const ros::NodeHandle& n);
  ~LaserLoopClosure();

 private:
  bool FindLoopClosures(
      gtsam::Key new_key,
      std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges);
};
#endif  // LASER_LOOP_CLOSURE_H_