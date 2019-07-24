<<<<<<< HEAD
gtsamAddTestsGlob(base "test*.cpp" "" "RobustPGO")
=======
include_directories("${CMAKE_CURRENT_BINARY_DIR}")
gtsamAddTestsGlob(RobustPGOTests "test*.cpp" "" RobustPGO)
>>>>>>> master
