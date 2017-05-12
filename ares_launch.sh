#! /bin/bash

cd $(dirname $(readlink -f $0))
. devel/setup.bash
roslaunch robot2017 main.launch
