#!/bin/bash

# 加载 ROS2 环境
source /opt/ros/humble/setup.bash
source /home/bosch/vChisel_ros2_ws/install/setup.bash

# 日志配置
LOG_DIR="/home/bosch/logs"
LOG_FILE="$LOG_DIR/visual.log"
MAX_SIZE=10485760  # 10MB
MAX_LOGS=5         # 保留最近5个日志文件

# 创建日志目录（如果不存在）
mkdir -p "$LOG_DIR"

# 日志滚动函数
rotate_logs() {
    if [ -f "$LOG_FILE" ]; then
        # 检查日志大小
        local file_size=$(stat -c%s "$LOG_FILE" 2>/dev/null || echo 0)
        
        if [ "$file_size" -gt "$MAX_SIZE" ]; then
            echo "[$(date)] 日志超过 10MB，开始滚动日志..." >> "$LOG_FILE"
            
            # 滚动日志文件
            for i in $(seq $MAX_LOGS -1 -1 1); do
                local old_log="$LOG_FILE.$i"
                local new_log="$LOG_FILE.$((i + 1))"
                
                if [ -f "$old_log" ]; then
                    if [ $i -eq $((MAX_LOGS - 1)) ]; then
                        # 删除最旧的日志
                        rm -f "$old_log"
                    else
                        # 移动日志文件
                        mv "$old_log" "$new_log"
                    fi
                fi
            done
            
            # 将当前日志移动到 .1
            mv "$LOG_FILE" "$LOG_FILE.1"
            
            # 创建新的日志文件
            touch "$LOG_FILE"
            
            echo "[$(date)] 日志滚动完成，保留最近 $MAX_LOGS 个日志文件" >> "$LOG_FILE"
        fi
    fi
}

# 启动前检查并滚动日志
rotate_logs

# 启动 ROS2 launch 并追加日志
echo "[$(date)] ========== 系统启动 ==========" >> "$LOG_FILE"
echo "[$(date)] 启动命令: ros2 launch snap_7 snap_7.launch.py" >> "$LOG_FILE"

exec ros2 launch snap_7 snap_7.launch.py >> "$LOG_FILE" 2>&1
