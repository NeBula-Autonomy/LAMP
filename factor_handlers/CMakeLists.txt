cmake_minimum_required(VERSION 2.8.3)
project(factor_handlers)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
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

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "${CMAKE_CXX_COMPILER} doesn't provide c++11 support.")
endif()

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