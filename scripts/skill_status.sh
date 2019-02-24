#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash

output=$(rostopic echo -n 1 /skill_machine_bonds | grep "active")

if [[ $output = *"True"* ]]; then
    exit 1
else
    exit 0
fi