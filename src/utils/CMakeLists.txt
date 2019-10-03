cmake_minimum_required(VERSION 2.8.3)
project(utils)

SET(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(GTSAM REQUIRED)
if (NOT GTSAM_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(NOT GTSAM_FOUND)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_utils
  core_msgs
  pose_graph_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    geometry_utils
    core_msgs
    pose_graph_msgs
    geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/CommonFunctions.cc)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY cmake/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.cmake"
)
