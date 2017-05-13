cmake_minimum_required(VERSION 2.8.3)
project(bbb_hw)

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

include_directories(src)

add_library(${PROJECT_NAME}
  src/adc.c
  src/gpio.c
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
install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
