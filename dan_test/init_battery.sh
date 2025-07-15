#!/bin/bash

sudo ip link set can1 up type can bitrate 250000

cd ~/next_ros2

source install/setup.bash

ros2 launch next2_bms bms.launch.py