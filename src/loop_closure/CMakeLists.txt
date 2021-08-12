cmake_minimum_required(VERSION 2.8.3)
project(loop_closure)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

SET(CMAKE_BUILD_TYPE Release)

set (CMAKE_CXX_STANDARD 14)

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

find_package(teaserpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  utils
  geometry_utils
  parameter_utils
  point_cloud_filter
  point_cloud_mapper
  pcl_ros
  pcl_conversions
  std_msgs
  core_msgs
  pose_graph_msgs
  geometry_msgs
  multithreaded_gicp
)

find_package(OpenMP)
# if (OpenMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
# endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    utils
    geometry_utils
    parameter_utils
    point_cloud_filter
    point_cloud_mapper
    multithreaded_gicp
    pcl_ros
    pcl_conversions
    std_msgs
    core_msgs
    pose_graph_msgs
    geometry_msgs
  DEPENDS
    Boost
)

find_package(Eigen3 REQUIRED)

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
  src/PointCloudUtils.cc
  src/LoopGeneration.cc
  src/LoopPrioritization.cc
  src/LoopComputation.cc
  src/ProximityLoopGeneration.cc
  src/GenericLoopPrioritization.cc
  src/ObservabilityLoopPrioritization.cc
  src/IcpLoopComputation.cc
  src/LoopCandidateQueue.cc
  src/TestUtils.cc
  src/RoundRobinLoopCandidateQueue.cc
  src/ObservabilityQueue.cc
)

find_package(Eigen3 REQUIRED)
find_package( PCL 1.8 REQUIRED )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
  Eigen3::Eigen
  teaserpp::teaser_registration
  teaserpp::teaser_features
  teaserpp::teaser_io
)

add_executable(loop_generation_node src/loop_generation_node.cc)
target_link_libraries(loop_generation_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(loop_prioritization_node src/loop_prioritization_node.cc)
target_link_libraries(loop_prioritization_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(loop_candidate_queue_node src/loop_candidate_queue_node.cc)
target_link_libraries(loop_candidate_queue_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(loop_computation_node src/loop_computation_node.cc)
target_link_libraries(loop_computation_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(gen_loop_computation_test src/gen_loop_computation_test.cc)
target_link_libraries(gen_loop_computation_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(eval_loop_computation_test src/eval_loop_computation_test.cc)
target_link_libraries(eval_loop_computation_test
  ${PROJECT_NAME}
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
  add_rostest_gtest(test_icp_computation test/test_icp_computation.test test/test_icp_computation.cpp)
  target_link_libraries(test_icp_computation  ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_loop_generation test/test_loop_generation.test test/test_loop_generation.cc)
  target_link_libraries(test_loop_generation ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_loop_prioritization test/test_loop_prioritization.test test/test_loop_prioritization.cc)
  target_link_libraries(test_loop_prioritization ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_loop_computation test/test_loop_computation.test test/test_loop_computation.cc)
  target_link_libraries(test_loop_computation ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_point_cloud_utils test/test_point_cloud_utils.test test/test_point_cloud_utils.cc)
  target_link_libraries(test_point_cloud_utils ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()