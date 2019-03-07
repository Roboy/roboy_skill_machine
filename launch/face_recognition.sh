#!/usr/bin/env bash

ssh pi@192.168.0.211 "export ROS_MASTER_URI='http://192.168.0.127:11311'; export ROS_IP='192.168.0.211'; /home/pi/face_recognition.sh" &
sleep 2
python ~/catkin_ws/src/face_oracle/webcam_video_processor.py
