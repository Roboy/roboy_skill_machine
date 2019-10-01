#! /bin/bash

source /opt/ros/kinetic/setup.bash
source /home/pi/workspace/devel/setup.bash

export PYTHONPATH=$PYTHONPATH:$HOME/snowboy

export ROS_IP=192.168.0.225
export ROS_MASTER_URI=http://192.168.0.105:11311
exec "$@"
