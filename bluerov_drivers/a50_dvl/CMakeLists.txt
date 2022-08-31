cmake_minimum_required(VERSION 2.8.3)
project(a50_dvl)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs
 dsor_msgs
 rospy
 geometry_msgs
 message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  DVL.msg
  DVLBeam.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )


catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime dsor_msgs
)
