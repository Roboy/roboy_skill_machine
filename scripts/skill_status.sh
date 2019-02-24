#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash

output=$(rosnode list | grep "$1")

if [[ $output = *"$1"* ]]; then
    echo "1"
else
    echo "0"
fi