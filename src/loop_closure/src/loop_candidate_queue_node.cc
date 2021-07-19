/*
 * Copyright Notes
 *
 * Authors: Yun Chang    (yunchang@mit.edu)
 */

#include <loop_closure/LoopCandidateQueue.h>
#include <loop_closure/RoundRobinLoopCandidateQueue.h>
#include <loop_closure/ObservabilityQueue.h>
#include <ros/ros.h>
#include <parameter_utils/ParameterUtils.h>
#include <utils/CommonFunctions.h>
#include <memory>

namespace pu = parameter_utils;
namespace lc = lamp_loop_closure;

int main(int argc, char** argv) {
  ros::init(argc, argv, "loop_candidate_queue");
  ros::NodeHandle n("~");
  int queue_method = 0;
  std::string param_ns = utils::GetParamNamespace(n.getNamespace());
  if (!pu::Get(param_ns + "/queue/method", queue_method)) {
    return EXIT_FAILURE;
  }
  std::unique_ptr<lc::LoopCandidateQueue> queue;

  switch (queue_method) {
    case 1: {
      queue = std::unique_ptr<lc::RoundRobinLoopCandidateQueue>(new lc::RoundRobinLoopCandidateQueue);
    }break;
    case 2: {
      queue = std::unique_ptr<lc::ObservabilityQueue>(new lc::ObservabilityQueue);
    }break;
    default:
      ROS_ERROR_STREAM("Candidate Queue: Unrecognized queue method " << queue_method);
  }

  if (!queue->Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize Loop Candidate Queue module. ",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
