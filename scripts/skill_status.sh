#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash

output=$(rosnode list | grep "$1")

if [[ $output = *"$1"* ]]; then
    exit 1
else
    exit 0
fi