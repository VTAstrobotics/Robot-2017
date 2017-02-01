cmake_minimum_required(VERSION 2.8.3)
project(robot2017)

find_package(catkin REQUIRED COMPONENTS
  bldc_uart_beaglebone
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drivetest
  CATKIN_DEPENDS bldc_uart_beaglebone roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(drive_node src/drive_node.cpp)
target_link_libraries(drive_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY lauch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
