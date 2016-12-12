#!/bin/bash

nohup Xvfb :1 -screen 0 1024x768x16 &> xvfb.log &
DISPLAY=:1.0
export DISPLAY

source "/workspace/devel/setup.bash"
source "/usr/share/gazebo-7/setup.sh"

GAZEBO_MODEL_PATH=/workspace/src/universal_robot:${GAZEBO_MODEL_PATH} roslaunch ur_gazebo ur10.launch gui:=false &

cd /root/gzweb
GAZEBO_MODEL_PATH=/workspace/src/universal_robot:${GAZEBO_MODEL_PATH} ./start_gzweb.sh

read -n 1 -s
