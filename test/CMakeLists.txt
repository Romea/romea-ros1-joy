find_package(rostest REQUIRED)

add_rostest_gtest(${PROJECT_NAME}_mapping_test mapping.test mapping_test.cpp)
target_link_libraries(${PROJECT_NAME}_mapping_test ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}_remapping_test remapping.test remapping_test.cpp )
target_link_libraries(${PROJECT_NAME}_remapping_test ${PROJECT_NAME})
