#!/usr/bin/env bash

roslaunch rosbridge_server rosbridge_tcp.launch port:=9091
roslaunch roboy_face roboy_face.launch
