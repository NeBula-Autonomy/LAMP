/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <loop_closure/IcpLoopComputation.h>
#include <ros/ros.h>

namespace lc = lamp_loop_closure;

int main(int argc, char** argv) {
  ros::init(argc, argv, "loop_computation");
  ros::NodeHandle n("~");

  lc::IcpLoopComputation loop_computation;
  if (!loop_computation.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize Loop Candidate Computation module.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
