#!/bin/bash

# 加载 ROS2 环境（请根据你的 ROS2 版本调整路径）
#source /opt/ros/humble/setup.bash
source /home/bosch/vChisel_ros2_ws/install/setup.bash

# 日志配置
LOG_FILE="/home/bosch/logs/visual.log"
MAX_SIZE=5242880  # 5MB

# 检查日志大小，超限则清空（保留最新）
if [ -f "$LOG_FILE" ]; then
    if [ $(stat -c%s "$LOG_FILE") -gt $MAX_SIZE ]; then
        echo "[$(date)] 日志超过 $(($MAX_SIZE / 1024 / 1024))MB，清空日志..." > "$LOG_FILE"
    fi
fi

# 启动 ROS2 launch 并追加日志
exec ros2 launch snap_7 snap_7.launch.py >> "$LOG_FILE" 2>&1 | tee -a "/home/bosch/logs/visual.log"
