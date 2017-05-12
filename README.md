# Robot code for Astrobotics 2017
This repository is a catkin workspace, containing the packages that comprise the 2017 robot code.

## Running on BeagleBone
You will need to perform the following actions once to run the robot code on a BeagleBone.

1. `cp system/99-gpio.rules /etc/udev/rules.d` to set up permissions for the GPIOs
2. `groupadd gpio` to create a user group for GPIO access
3. `usermod -aG gpio astrobotics` to add the user `astrobotics` to the GPIO group
4. Reboot the system

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
See `system/astro-robot-2017.service` for details.
