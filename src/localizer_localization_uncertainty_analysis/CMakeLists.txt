cmake_minimum_required(VERSION 2.8.3)
project(localizer_localization_uncertainty_analysis)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pose_graph_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        std_msgs
)
