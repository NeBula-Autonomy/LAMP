cmake_minimum_required(VERSION 2.8.3)
project(robust_pgo)

find_package(catkin REQUIRED)

## Export package settings
catkin_package(
  INCLUDE_DIRS
    RobustPGO
  LIBRARIES
    RobustPGO
  CATKIN_DEPENDS
  DEPENDS
    Boost
    Eigen3
    GTSAM
)

## Build RobustPGO
add_subdirectory(RobustPGO)
