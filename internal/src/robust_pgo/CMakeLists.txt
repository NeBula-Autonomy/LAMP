cmake_minimum_required(VERSION 2.8.3)
project(robust_pgo)

find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    RobustPGO/RobustPGO
    RobustPGO
    RobustPGO/RobustPGO/pcm
    RobustPGO/RobustPGO/max_clique_finder
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS
    Eigen3
)

## Build RobustPGO
add_subdirectory(RobustPGO)
