cmake_minimum_required(VERSION 3.0.2)
project(sliding_mode)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -O3 -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  medusa_gimmicks_library
  auv_msgs
  dsor_utils
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

add_message_files(
  DIRECTORY msg
)

add_service_files(
  FILES
  ChangeSMParams.srv
)

# Put here the message dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  medusa_gimmicks_library
  auv_msgs
  dsor_utils
  # Include for message generation
  message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include/sliding_mode
)

# include all .cpp files
add_executable(${PROJECT_NAME}_node 
  src/algorithm/sliding_mode.cpp
  src/ros/sliding_mode_node.cpp)

## Specify libraries to link a library or executable target against
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
