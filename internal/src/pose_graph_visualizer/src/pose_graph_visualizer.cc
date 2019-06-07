#include <ros/ros.h>
#include <pose_graph_visualizer/PoseGraphVisualizer.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph_visualizer");
  ros::NodeHandle n("~");

  PoseGraphVisualizer pgv;
  if (!pgv.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize pose graph visualizer.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
