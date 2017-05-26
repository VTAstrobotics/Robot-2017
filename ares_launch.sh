#! /bin/bash

cd $(dirname $(readlink -f $0))
. devel/setup.bash
ulimit -c unlimited
export ROS_IP=10.0.0.30
until ping -c1 $ROS_IP &>/dev/null; do :; done
roslaunch robot2017 main.launch
