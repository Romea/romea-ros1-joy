find_package(rostest REQUIRED)

catkin_add_gtest(${PROJECT_NAME}_trigger_test trigger_test.cpp )
target_link_libraries(${PROJECT_NAME}_trigger_test ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}_stick_test stick_test.cpp )
target_link_libraries(${PROJECT_NAME}_stick_test ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}_button_test button_test.cpp )
target_link_libraries(${PROJECT_NAME}_button_test ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}_mapping_test mapping.test mapping_test.cpp)
target_link_libraries(${PROJECT_NAME}_mapping_test ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}_remapping_test remapping.test remapping_test.cpp )
target_link_libraries(${PROJECT_NAME}_remapping_test ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}_joystick_test joystick.test joystick_test.cpp )
target_link_libraries(${PROJECT_NAME}_joystick_test ${PROJECT_NAME})
