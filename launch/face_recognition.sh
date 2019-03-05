#!/usr/bin/env bash

ssh pi@192.168.0.211 "export ROS_MASTER_URI='http://192.168.0.226:11311'; /home/pi/face_recognition.sh" &

source /opt/ros/melodic/setup.bash
source ~/workspace/melodic_ws/devel/setup.bash
python /home/roboy/workspace/melodic_ws/src/face_oracle/webcam_video_processor.py
