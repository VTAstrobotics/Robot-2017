cmake_minimum_required(VERSION 2.8.3)
project(vesc_bbb_uart)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
   INCLUDE_DIRS src
   LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(src/${PROJECT_NAME})

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/bldc.cpp
  src/${PROJECT_NAME}/bldc_interface.c
  src/${PROJECT_NAME}/bldc_interface_uart.c
  src/${PROJECT_NAME}/buffer.c
  src/${PROJECT_NAME}/comm_uart.c
  src/${PROJECT_NAME}/crc.c
  src/${PROJECT_NAME}/packet.c
  src/${PROJECT_NAME}/timers.c
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
