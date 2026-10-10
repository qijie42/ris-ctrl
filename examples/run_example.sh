#!/bin/bash
# 示例脚本启动器 - 自动加载 ROS2 环境

cd ~/1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

cd ~/1_ws/ris-ctrl/examples
python3 ""
