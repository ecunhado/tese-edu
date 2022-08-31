cmake_minimum_required(VERSION 3.1)
project(serial_port)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES SerialPort
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(SerialPort
   src/SerialPort.cpp
   include/SerialPort.h
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(SerialPort ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(SerialPort
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS SerialPort
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

