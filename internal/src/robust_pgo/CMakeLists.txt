cmake_minimum_required(VERSION 2.8.3)
project(robust_pgo)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


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

        
## Build ROS code
include_directories(
  RobustPGO/RobustPGO
  RobustPGO
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_library(${PROJECT_NAME} 
  RobustPGO/RobustPGO/RobustPGO.cpp 
  RobustPGO/RobustPGO/GenericSolver.cpp 
  RobustPGO/RobustPGO/max_clique_finder/findClique.cpp
  RobustPGO/RobustPGO/max_clique_finder/findCliqueHeu.cpp
  RobustPGO/RobustPGO/max_clique_finder/graphIO.cpp
  RobustPGO/RobustPGO/max_clique_finder/utils.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
)

