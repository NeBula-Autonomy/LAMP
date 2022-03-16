
#include <lamp_pgo/LampPgo.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lamp_pgo");
  ros::NodeHandle n("~");

  LampPgo pgo;
  if (!pgo.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize LAMP PGO.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}