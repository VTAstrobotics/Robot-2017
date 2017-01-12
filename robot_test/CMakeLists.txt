# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# Name of this package.
project(robot2017)

# Find the catkin build system and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp flobotics_servo_pwm_pi std_msgs)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable.
add_executable(drive drive.cpp)

# Specify libraries against which to link.
target_link_libraries(drive ${catkin_LIBRARIES})
