cmake_minimum_required(VERSION 2.8.3)
project(sunrom)
set(REPO_DIR "Sunrom1190_Linux")

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
   INCLUDE_DIRS src/${REPO_DIR}
   LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(src/${REPO_DIR})

add_library(${PROJECT_NAME}
  src/${REPO_DIR}/sunrom.cpp
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
install(DIRECTORY src/${REPO_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
