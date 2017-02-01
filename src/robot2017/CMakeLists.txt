cmake_minimum_required(VERSION 2.8.3)
project(robot2017)

find_package(catkin REQUIRED COMPONENTS
  bldc_uart_beaglebone
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES Teleop.msg
)

generate_messages(DEPENDENCIES 
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drivetest
  CATKIN_DEPENDS bldc_uart_beaglebone roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(drive_node src/drive_node.cpp)
add_executable(testController src/testController.cpp)

add_dependencies(drive_node robot2017_generate_messages_cpp)
add_dependencies(testController robot2017_generate_messages_cpp)

target_link_libraries(drive_node
  ${catkin_LIBRARIES}
)

target_link_libraries(testController
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
