cmake_minimum_required(VERSION 2.8.3)
project(bldc_uart_beaglebone)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES bldc_uart_beaglebone
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include/bldc_uart_beaglebone)

add_library(bldc_uart_beaglebone
  src/bldc_interface.cpp
  src/bldc_interface_uart.cpp
  src/buffer.cpp
  src/comm_uart.cpp
  src/crc.cpp
  src/packet.cpp
)

#############
## Install ##
#############

install(TARGETS bldc_uart_beaglebone
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
