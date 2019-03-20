#!/usr/bin/env bash

ssh root@192.168.0.222 -t "bash -ic ./ROS_MASTER_URI_receiver_armhf && source ~/.bashrc && ./roboy_plexus" > 222.log &
#ssh root@192.168.0.223 "export ROS_MASTER_URI='http://192.168.0.226:11311'; source /opt/ros/kinetic/setup.bash; ~/roboy_plexus" &
