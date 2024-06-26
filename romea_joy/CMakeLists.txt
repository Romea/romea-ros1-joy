cmake_minimum_required(VERSION 3.0.2)
project(romea_joy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

########################
### Find dependencies ##
########################
set(${PROJECT_NAME}_CATKIN_DEPS roscpp sensor_msgs romea_common_utils)

find_package(catkin COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(romea_core_common REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})


############
### Build ##
############
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${romea_core_common_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/joystick.cpp
  src/joystick_axis.cpp
  src/joystick_button.cpp
  src/joystick_directional_pad.cpp
  src/joystick_mapping.cpp
  src/joystick_stick.cpp
  src/joystick_trigger.cpp
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif(CATKIN_ENABLE_TESTING)
