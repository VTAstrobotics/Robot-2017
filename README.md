# Robot code for Astrobotics 2017
This repository is a catkin workspace, containing the packages that comprise the 2017 robot code.

## How to build
Assuming you have ROS already installed on your system, follow these steps:

1. Run `source /opt/ros/kinetic/setup.bash` if you have not already
2. Run `catkin_make` in the current directory to build the robot code
3. Run `source devel/setup.bash` to initialize the newly-built robot code environment
4. Run `roslaunch robot2017 main.launch` to launch the robot code
