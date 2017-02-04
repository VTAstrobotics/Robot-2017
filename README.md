# Robot code for Astrobotics 2017
This repository is a catkin workspace, containing the packages that comprise the 2017 robot code.

## Building
Assuming you have ROS already installed on your system, run the following commands:

1. `wstool update -t src` to initialize the other Astrobotics packages
2. `source /opt/ros/kinetic/setup.bash` if you have not already
3. `catkin_make` in the current directory to build the robot code

## Running
After building, run the following commands:

1. `source devel/setup.bash` to initialize the newly-built robot code environment
2. `roslaunch robot2017 main.launch` to launch the robot code
