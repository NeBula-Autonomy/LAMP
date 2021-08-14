/*
 * Copyright Notes
 *
 * Authors: Andrzej Reinke   (andrzej.m.reinke@jpl.nasa.gov)
 */

#include <loop_closure/RssiLoopClosure.h>

namespace lc = lamp_loop_closure;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rssi_loop_closure");
  ros::NodeHandle n("~");

  lc::RssiLoopClosure rssi_loop_closure;
  if (!rssi_loop_closure.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize RssiLoopClosure.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
