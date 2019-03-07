#!/usr/bin/env bash

ssh pi@192.168.0.211 "export ROS_MASTER_URI='http://192.168.0.226:11311'; /home/pi/face_recognition.sh" &
sleep 2
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
python ~/catkin_ws/src/face_oracle/webcam_video_processor.py
