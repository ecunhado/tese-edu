cmake_minimum_required(VERSION 3.1)
project(serial_lib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
)


catkin_package(
  LIBRARIES nmea serialLib
  INCLUDE_DIRS include
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
add_library(nmea
  include/nmeaSerial.h
  include/AsyncSerial.cpp
  include/AsyncSerial.h
)

add_library(serialLib
  include/AsyncSerial.cpp
  include/AsyncSerial.h
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(nmea ${catkin_EXPORTED_TARGETS})
add_dependencies(serialLib ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(nmea   ${catkin_LIBRARIES})
target_link_libraries(serialLib   ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS nmea
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS serialLib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
