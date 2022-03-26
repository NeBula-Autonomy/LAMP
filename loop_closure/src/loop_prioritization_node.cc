/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */
#include <loop_closure/GenericLoopPrioritization.h>
#include <loop_closure/ObservabilityLoopPrioritization.h>
#include <memory>
#include <parameter_utils/ParameterUtils.h>
#include <ros/ros.h>
#include <lamp_utils/CommonFunctions.h>

namespace pu = parameter_utils;
namespace lc = lamp_loop_closure;

int main(int argc, char** argv) {
  ros::init(argc, argv, "loop_prioritization");
  ros::NodeHandle n("~");
  int prioritization_method = 0;
  std::string param_ns = lamp_utils::GetParamNamespace(n.getNamespace());
  if (!pu::Get(param_ns + "/prioritization_method", prioritization_method)) {
    return EXIT_FAILURE;
  }

  std::unique_ptr<lc::LoopPrioritization> loop_prioritize;

  switch (prioritization_method) {
  case 0: {
    loop_prioritize = std::unique_ptr<lc::GenericLoopPrioritization>(
        new lc::GenericLoopPrioritization);
  } break;
  case 1: {
    loop_prioritize = std::unique_ptr<lc::ObservabilityLoopPrioritization>(
        new lc::ObservabilityLoopPrioritization);
  } break;
  default: {
    ROS_ERROR("loop_prioritization: Unrecognized prioritization method. ");
  }
  }
  if (!loop_prioritize->Initialize(n)) {
    ROS_ERROR(
        "%s: Failed to initialize Loop Candidate Prioritization module. .",
        ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  std::vector<ros::AsyncSpinner> async_spinners =
        loop_prioritize->SetAsyncSpinners(n);
  for (auto spinner : async_spinners)
    spinner.start();

  ros::spin();

  return EXIT_SUCCESS;
}
