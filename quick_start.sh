#!/bin/bash

# 快速启动脚本 - vChisel_ros2_ws系统

cd /home/bosch/vChisel_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch snap_7 snap_7.launch.py