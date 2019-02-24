#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash
rosservice call /start_skill "skill_name: 'simple_publisher'
launch_package: 'roboy_skill_machine'
launch_file: 'simple_pub.launch'
continuous: true
node_list:
- {node_name: 'simple_publisher', node_executable: 'simple_publisher.py', node_package: 'roboy_skill_machine'}"

