# Add Gtest for the package
catkin_add_gtest(test_${PROJECT_NAME} test_${PROJECT_NAME}.cpp)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
