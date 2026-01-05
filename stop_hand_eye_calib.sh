#!/bin/bash

# 手眼标定系统停止脚本

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}停止手眼标定系统...${NC}"

# 停止手眼标定服务节点
if pkill -f "hand_eye_calib_server" 2>/dev/null; then
    echo -e "${GREEN}已停止手眼标定服务节点${NC}"
else
    echo -e "${YELLOW}手眼标定服务节点未运行${NC}"
fi

# 停止Aruco标记检测节点
if pkill -f "aruco_ros.*single" 2>/dev/null; then
    echo -e "${GREEN}已停止Aruco标记检测节点${NC}"
else
    echo -e "${YELLOW}Aruco标记检测节点未运行${NC}"
fi

# 停止RealSense相机节点
if pkill -f "realsense2_camera_node" 2>/dev/null; then
    echo -e "${GREEN}已停止RealSense相机节点${NC}"
else
    echo -e "${YELLOW}RealSense相机节点未运行${NC}"
fi

# 停止图像查看窗口
if pkill -f "rqt_image_view" 2>/dev/null; then
    echo -e "${GREEN}已停止图像查看窗口${NC}"
else
    echo -e "${YELLOW}图像查看窗口未运行${NC}"
fi

# 等待进程结束
sleep 2

# 强制清理残留进程
pkill -9 -f "hand_eye_calib_server" 2>/dev/null
pkill -9 -f "aruco_ros.*single" 2>/dev/null
pkill -9 -f "realsense2_camera_node" 2>/dev/null
pkill -9 -f "rqt_image_view" 2>/dev/null

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}手眼标定系统已完全停止${NC}"
echo -e "${GREEN}========================================${NC}"