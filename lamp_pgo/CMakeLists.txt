cmake_minimum_required(VERSION 3.5)
project(lamp_pgo)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

find_package(GTSAM REQUIRED)
find_package(KimeraRPGO REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  lamp_utils
  geometry_utils
  parameter_utils
  pose_graph_msgs
  geometry_msgs
)

## Export package settings
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
    lamp_utils
  	geometry_utils
  	parameter_utils
    pose_graph_msgs
  	geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

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
