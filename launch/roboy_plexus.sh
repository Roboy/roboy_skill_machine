#!/usr/bin/env bash

sshpass -p "Roboy2016" ssh root@192.168.0.222 "export ROS_MASTER_URI='http://192.168.0.226:11311'; source /opt/ros/kinetic/setup.bash; ~/roboy_plexus" &
sshpass -p "Roboy2016" ssh root@192.168.0.223 "export ROS_MASTER_URI='http://192.168.0.226:11311'; source /opt/ros/kinetic/setup.bash; ~/roboy_plexus" &
