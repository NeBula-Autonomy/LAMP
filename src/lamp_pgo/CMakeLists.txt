cmake_minimum_required(VERSION 2.8.3)
project(lamp_pgo)

SET(CMAKE_BUILD_TYPE Release)

find_package(GTSAM REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  utils
  geometry_utils
  parameter_utils
  core_msgs
  pose_graph_msgs
  geometry_msgs
)

## Export package settings
catkin_package(
  INCLUDE_DIRS
    KimeraRPGO
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
    utils
  	geometry_utils
  	parameter_utils
  	core_msgs
    pose_graph_msgs
  	geometry_msgs
  DEPENDS
    KimeraRPGO
)

## Build RobustPGO
add_subdirectory(KimeraRPGO)

include_directories(include ${catkin_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} KimeraRPGO)
link_directories(${catkin_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/lamp_pgo.cc src/LampPgo.cc)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
