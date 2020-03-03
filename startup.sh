#!/bin/bash

# This script is used by the cms laptop to startup cms.
# It has hard-coded values to that system.
# Don't try to use it somewhere else

source ~/autonomy/devel/setup.bash
roscd cms

npm run build

export ROS_MASTER_URI=http://luke:11311

# This is the static ip of the cms laptop
export ROS_IP=192.168.1.42

exec roslaunch cms groundstation.launch
