/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <loop_closure/ProximityLoopCandidateGeneration.h>
#include <ros/ros.h>

namespace lamp_loop_closure {
int main(int argc, char** argv) {
  ros::init(argc, argv, "loop_generation");
  ros::NodeHandle n("~");

  ProximityLoopCandidateGeneration loop_gen;
  if (!loop_gen.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize Loop Candidate Generation module. .",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
}  // namespace lamp_loop_closure