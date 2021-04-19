/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <loop_closure/ObservabilityLoopCandidatePrioritization.h>
#include <ros/ros.h>

namespace lamp_loop_closure {
int main(int argc, char** argv) {
  ros::init(argc, argv, "loop_prioritization");
  ros::NodeHandle n("~");

  ObservabilityLoopCandidatePrioritization loop_prioritize;
  if (!loop_prioritize.Initialize(n)) {
    ROS_ERROR(
        "%s: Failed to initialize Loop Candidate Prioritization module. .",
        ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
}  // namespace lamp_loop_closure