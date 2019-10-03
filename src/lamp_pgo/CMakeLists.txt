cmake_minimum_required(VERSION 2.8.3)
project(lamp_pgo)

find_package(catkin REQUIRED)

## Export package settings
catkin_package(
  INCLUDE_DIRS
    KimeraRPGO
  LIBRARIES
    KimeraRPGO
  CATKIN_DEPENDS
  DEPENDS
    Boost
    Eigen3
    GTSAM
)

## Build RobustPGO
add_subdirectory(KimeraRPGO)
