#include <pose_graph_visualizer/PoseGraphVisualizer.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph_visualizer");

  // Initialize public and private node handles
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PoseGraphVisualizer pgv;
  if (!pgv.Initialize(nh, pnh)) {
    ROS_ERROR("%s: Failed to initialize pose graph visualizer.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
