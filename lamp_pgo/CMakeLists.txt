cmake_minimum_required(VERSION 3.5)
project(lamp_pgo)

SET(CMAKE_BUILD_TYPE Release)

## Build kimera_rpgo
add_subdirectory(kimera_rpgo)

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
  INCLUDE_DIRS include kimera_rpgo
  LIBRARIES ${PROJECT_NAME} KimeraRPGO
  CATKIN_DEPENDS
  	roscpp
    utils
  	geometry_utils
  	parameter_utils
  	core_msgs
    pose_graph_msgs
  	geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} kimera_rpgo/include)
link_directories(${catkin_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS} kimera_rpgo)

add_library(${PROJECT_NAME} src/lamp_pgo.cc src/LampPgo.cc)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  KimeraRPGO
  gtsam
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  KimeraRPGO
  gtsam
)

add_executable(load_and_save_as_g2o src/load_and_save_as_g2o.cc)
target_link_libraries(load_and_save_as_g2o
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
)

add_executable(lamp_g2o src/lamp_g2o_node.cc)
target_link_libraries(lamp_g2o
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  KimeraRPGO
  gtsam
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
