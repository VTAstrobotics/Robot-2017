# Top-level CMake file used to override catkin variables

cmake_minimum_required(VERSION 2.8.3)

# Disable unneeded message generators
# Keep for C++ and Python
set(ENV{ROS_LANG_DISABLE} "geneus:genlisp:gennodejs")

include(/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake)
