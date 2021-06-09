/*
 * Copyright Notes
 *
 * Authors: Yun Chang    (yunchang@mit.edu)
 */

#include <loop_closure/LoopCandidateQueue.h>
#include <ros/ros.h>

namespace lc = lamp_loop_closure;

int main(int argc, char** argv) {
  ros::init(argc, argv, "loop_candidate_queue");
  ros::NodeHandle n("~");

  lc::LoopCandidateQueue loop_queue;
  if (!loop_queue.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize Loop Candidate Queue module. ",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
