#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash

output=$(rostopic echo /skill_machine_bonds | grep "active")

if [[ output = "active: True" ]]; then
    echo "it's till on"
else
    echo "it's already dead dude, leave it"
fi