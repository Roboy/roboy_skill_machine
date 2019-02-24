#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash

output=$(rostopic echo -n 1 /skill_machine_bonds | grep "active")

if [[ output = *"True"* ]]; then
    echo "it's till on"
else
    echo "it's already dead dude, leave it"
fi