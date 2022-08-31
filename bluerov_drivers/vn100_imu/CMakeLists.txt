cmake_minimum_required(VERSION 3.1)
project(vn100_imu)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 medusa_msgs
 dsor_msgs
 nmea_msgs
 geometry_msgs
 nav_msgs
 tf
 diagnostic_msgs
 roscpp
 serial_lib
 medusa_diagnostics_library
 medusa_gimmicks_library
 message_generation
)

## Generate messages in the 'msg' folder
add_message_files(FILES
   imu_pp.msg
 )

include_directories(
 include/vn100_imu_ros
 ${catkin_INCLUDE_DIRS}
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
 medusa_msgs 
 dsor_msgs
 medusa_diagnostics_library 
 medusa_gimmicks_library 
 serial_lib 
 message_runtime 
 tf
)

add_executable(${PROJECT_NAME}_node src/vn100_imu_ros/Vn100ImuNode.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${nmea_LIBRARIES} ${catkin_LIBRARIES})
