/*
LaserLoopClosure.cc
Author: Yun Chang
Lidar pointcloud based loop closure
*/
#include "loop_closure/LaserLoopClosure.h"

LaserLoopClosure::LaserLoopClosure(const ros::NodeHandle& n) : LoopClosure(n){};
LaserLoopClosure::~LaserLoopClosure(){};

bool FindLoopClosures(
    gtsam::Key new_key,
    std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges) {
  // TODO
}