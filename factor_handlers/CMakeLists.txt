cmake_minimum_required(VERSION 2.8.3)
project(factor_handlers)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

SET(CMAKE_BUILD_TYPE Release)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(GTSAM REQUIRED)
if (NOT GTSAM_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(NOT GTSAM_FOUND)

FIND_PACKAGE(Boost REQUIRED
  date_time
  serialization
  thread
  filesystem
  system
  regex
  timer
  chrono
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  lamp_utils
  geometry_utils
  parameter_utils
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav_msgs
  std_msgs
  tf_conversions
  eigen_conversions
  localizer_zero_velocity_detector
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rosbag
    lamp_utils
    geometry_utils
    parameter_utils
    pcl_ros
    pcl_conversions
    geometry_msgs
    nav_msgs
    std_msgs
    localizer_zero_velocity_detector
  DEPENDS
    Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(${PROJECT_NAME} 
  src/LampDataHandlerBase.cc 
  src/OdometryHandler.cc 
  src/ManualLoopClosureHandler.cc 
  src/PoseGraphHandler.cc 
  src/RobotPoseHandler.cc
  src/StationaryHandler.cc
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # add_subdirectory(test)
  ## Add gtest based cpp test target and link libraries
  add_rostest_gtest(test_odometry_handler test/test_odometry_handler.test test/test_odometry_handler.cc)
  target_link_libraries(test_odometry_handler ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_stationary_handler test/test_stationary_handler.test test/test_stationary_handler.cc)
  target_link_libraries(test_stationary_handler ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()