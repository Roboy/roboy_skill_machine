#!/usr/bin/env bash

source ~/catkin_ws/devel/setup.bash

rosservice call /kill_skill "node_name: '$1'"