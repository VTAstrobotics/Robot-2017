# Robot code for Astrobotics 2017
This repository is a catkin workspace, containing the packages that comprise the 2017 robot code.

## Building
Assuming you have ROS already installed on your system, run the following commands:

1. `source /opt/ros/kinetic/setup.bash` if you have not already
2. `git submodule deinit -f --all`- sometimes submodules won't update correctly, so force a delete and re-init
3. `git submodule init` to initialize the Git submodules
4. `git submodule update` to clone the required packages
5. `catkin_make` in the current directory to build the robot code

## Running
After building, run the following commands:

1. `source devel/setup.bash` to initialize the newly-built robot code environment
2. `roslaunch robot2017 main.launch` to launch the robot code

## Running at boot
The code can be configured to launch automatically at boot after the network is connected, on any system that supports systemd.
See `astro-robot-2017.service` for details.
