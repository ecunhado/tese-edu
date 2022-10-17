cmake_minimum_required(VERSION 3.0.2)
project(yaw_ref_generator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -O3 -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  medusa_gimmicks_library
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  medusa_gimmicks_library
  # Include for message generation
  # message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include/yaw_ref_generator
)

# include all .cpp files
add_executable(${PROJECT_NAME}_node 
  src/algorithm/yaw_ref_generator.cpp
  src/ros/yaw_ref_generator_node.cpp)

## Specify libraries to link a library or executable target against
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
