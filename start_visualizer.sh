#!/bin/bash

# ============================================================
# vChisel 可视化窗口启动脚本
# 功能：启动点云和图像可视化窗口（需要图形界面环境）
# ============================================================

set -e

WORKSPACE_DIR="/home/bosch/vChisel_ros2_ws"
LOG_DIR="/home/bosch/logs"
LOG_FILE="$LOG_DIR/visualizer.log"

# 创建日志目录
mkdir -p "$LOG_DIR"

# 日志函数
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $@" | tee -a "$LOG_FILE"
}

log "============================================================"
log "vChisel 可视化窗口启动"
log "============================================================"

# 检查 GUI 环境
if [ -z "$DISPLAY" ]; then
    log "错误：DISPLAY 环境变量未设置"
    log "可视化窗口需要图形界面环境"
    exit 1
fi

log "GUI 环境："
log "  DISPLAY: $DISPLAY"
log "  XAUTHORITY: ${XAUTHORITY:-$HOME/.Xauthority}"
log "  用户: $USER"

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    log "加载 ROS2 环境..."
    source /opt/ros/humble/setup.bash
fi

# 加载工作空间环境
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    log "加载工作空间环境..."
    source "$WORKSPACE_DIR/install/setup.bash"
else
    log "错误：工作空间未构建"
    log "请先运行: colcon build"
    exit 1
fi

# 等待后台服务启动
log "等待后台服务启动..."
sleep 5

# 检查后台服务是否运行
if ! ros2 node list | grep -q "norm_calc"; then
    log "警告：norm_calc 服务未运行"
    log "后台服务可能还未启动完成，继续尝试..."
    sleep 10
fi

log "启动可视化窗口..."

# 启动可视化节点
ros2 run norm_calc image_norm_viewer 2>&1 | while IFS= read -r line; do
    log "$line"
done

log "可视化窗口已关闭"
