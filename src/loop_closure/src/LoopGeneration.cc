/**
 * @file   LoopGeneration.cc
 * @brief  Base class for classes to find potentital loop closures
 * @author Yun Chang
 */
#include <parameter_utils/ParameterUtils.h>
#include <pose_graph_msgs/PoseGraphNode.h>
#include <utils/CommonFunctions.h>

#include "loop_closure/LoopGeneration.h"

namespace pu = parameter_utils;

namespace lamp_loop_closure {

LoopGeneration::LoopGeneration() {}
LoopGeneration::~LoopGeneration() {}

bool LoopGeneration::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);  // Nodehandle for subscription/publishing
  param_ns_ = utils::GetParamNamespace(n.getNamespace());
  if (!pu::Get(param_ns_ + "/b_find_laser_loop_closures",
               b_check_for_loop_closures_))
    return false;
  return true;
}

bool LoopGeneration::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_pub_ = nl.advertise<pose_graph_msgs::LoopCandidateArray>(
      "loop_candidates", 10, false);
  return true;
}

}  // namespace lamp_loop_closure