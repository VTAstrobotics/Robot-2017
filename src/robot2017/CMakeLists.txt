cmake_minimum_required(VERSION 2.8.3)
project(robot2017)

find_package(catkin REQUIRED COMPONENTS
  vesc_bbb_uart
  roscpp
  robot_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drivetest
  CATKIN_DEPENDS vesc_bbb_uart roscpp robot_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_executable(drive_node src/drive_node.cpp src/robot_exec.cpp)
add_executable(test_controller src/test_controller.cpp)

add_dependencies(drive_node robot_msgs_generate_messages_cpp)
add_dependencies(test_controller robot_msgs_generate_messages_cpp)

target_link_libraries(drive_node
  ${catkin_LIBRARIES}
)

target_link_libraries(test_controller
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
