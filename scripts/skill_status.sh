#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash

output=$(rostopic echo -n 1 /skill_machine_bonds | grep "active")

if [[ $output = *"True"* ]]; then
    return 1
else
    return 0
fi