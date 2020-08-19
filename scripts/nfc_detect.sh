#!/usr/bin/env bash

ssh pi@192.168.0.225 -t "bash -ic './hal_nfc_roboy/build/examples/nfc_detect'" 
#ssh root@192.168.0.223 -t "bash -ic ./ROS_MASTER_URI_receiver_armhf && source ~/.bashrc && ./roboy_plexus" > shoulder_left.log &
#ssh root@192.168.0.223 "export ROS_MASTER_URI='http://192.168.0.226:11311'; source /opt/ros/kinetic/setup.bash; ~/roboy_plexus" &
