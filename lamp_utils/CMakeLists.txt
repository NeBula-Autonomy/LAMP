cmake_minimum_required(VERSION 2.8.3)
project(lamp_utils)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_utils
  pose_graph_msgs
  geometry_msgs
  pcl_ros
)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    geometry_utils
    pose_graph_msgs
    geometry_msgs
    pcl_ros
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS})
add_library(${PROJECT_NAME}
  src/CommonFunctions.cc
  src/PoseGraphFileIO.cc
  src/PoseGraphMessageConversion.cc
  src/PoseGraphBookkeeping.cc
  src/PoseGraphLookupUtils.cc
  src/PointCloudUtils.cc
  src/LampPcldFilter.cc
  src/gicp.cc
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
  minizip
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# install(DIRECTORY cmake/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   FILES_MATCHING PATTERN "*.cmake"
# )

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # add_subdirectory(test)
  ## Add gtest based cpp test target and link libraries
  add_rostest_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.test test/test_${PROJECT_NAME}.cc)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_pose_graph test/test_pose_graph.test test/test_pose_graph.cc)
  target_link_libraries(test_pose_graph ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_point_cloud_utils test/test_point_cloud_utils.test test/test_point_cloud_utils.cc)
  target_link_libraries(test_point_cloud_utils ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

