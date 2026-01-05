#!/bin/bash

# 手眼标定交互操作一键启动脚本

set -e

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}    手眼标定交互式启动脚本${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS2环境未加载，正在加载...${NC}"
    source /opt/ros/humble/setup.bash
fi

# 加载工作空间
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "${YELLOW}加载工作空间: ${WORKSPACE_DIR}${NC}"
source ${WORKSPACE_DIR}/install/setup.bash

# 检查是否已有运行中的节点
RUNNING_NODES=$(ros2 node list 2>/dev/null | grep -E "(hand_eye|aruco|realsense2_camera)" || true)
if [ ! -z "$RUNNING_NODES" ]; then
    echo -e "${RED}检测到已有运行中的手眼标定相关节点:${NC}"
    echo "$RUNNING_NODES"
    read -p "是否继续启动? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${RED}启动已取消${NC}"
        exit 1
    fi
fi

# 创建日志目录
LOG_DIR="${WORKSPACE_DIR}/logs"
mkdir -p ${LOG_DIR}

# 获取当前时间戳
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="${LOG_DIR}/hand_eye_calib_${TIMESTAMP}.log"

echo -e "${GREEN}启动手眼标定系统...${NC}"
echo -e "${GREEN}日志文件: ${LOG_FILE}${NC}"
echo -e "${GREEN}========================================${NC}"

# 启动RealSense相机节点（后台运行）
echo -e "${YELLOW}启动RealSense相机节点...${NC}"
ros2 run realsense2_camera realsense2_camera_node \
    --ros-args \
    -p enable_color:=true \
    -p enable_depth:=true \
    -p color_width:=640 \
    -p color_height:=480 \
    -p color_fps:=10.0 \
    -p depth_fps:=10.0 \
    -p initial_reset:=true \
    >> ${LOG_FILE} 2>&1 &
CAMERA_PID=$!

# 等待相机启动
echo -e "${YELLOW}等待相机启动...${NC}"
sleep 5

# 检查相机是否启动成功
if ! kill -0 $CAMERA_PID 2>/dev/null; then
    echo -e "${RED}RealSense相机节点启动失败！${NC}"
    echo -e "${RED}请检查相机连接和驱动安装${NC}"
    echo -e "${RED}请查看日志: ${LOG_FILE}${NC}"
    exit 1
fi

echo -e "${GREEN}RealSense相机节点启动成功 (PID: $CAMERA_PID)${NC}"

# 启动手眼标定服务节点（后台运行）
echo -e "${YELLOW}启动手眼标定服务节点...${NC}"
ros2 run hand_eye_calib hand_eye_calib_server >> ${LOG_FILE} 2>&1 &
SERVICE_PID=$!

# 等待服务启动
echo -e "${YELLOW}等待服务启动...${NC}"
sleep 3

# 检查服务是否启动成功
if ! kill -0 $SERVICE_PID 2>/dev/null; then
    echo -e "${RED}手眼标定服务节点启动失败！${NC}"
    echo -e "${RED}请查看日志: ${LOG_FILE}${NC}"
    kill $CAMERA_PID 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}手眼标定服务节点启动成功 (PID: $SERVICE_PID)${NC}"

# 启动Aruco标记检测节点（后台运行，使用RealSense专用启动文件）
echo -e "${YELLOW}启动Aruco标记检测节点...${NC}"
ros2 launch aruco_ros realsense_single.launch.py \
    marker_size:=0.10 \
    marker_id:=123 \
    camera_frame:=camera_color_optical_frame \
    image_topic:=/camera/camera/color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    >> ${LOG_FILE} 2>&1 &
ARUCO_PID=$!

# 等待Aruco节点启动
echo -e "${YELLOW}等待Aruco节点启动...${NC}"
sleep 3

# 检查Aruco节点是否启动成功
if ! kill -0 $ARUCO_PID 2>/dev/null; then
    echo -e "${RED}Aruco标记检测节点启动失败！${NC}"
    echo -e "${RED}请查看日志: ${LOG_FILE}${NC}"
    kill $SERVICE_PID $CAMERA_PID 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}Aruco标记检测节点启动成功 (PID: $ARUCO_PID)${NC}"

# 启动图像查看窗口（后台运行）
echo -e "${YELLOW}启动图像查看窗口...${NC}"
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw >> ${LOG_FILE} 2>&1 &
IMAGE_VIEWER_PID=$!

# 启动Aruco检测结果图像查看窗口（后台运行）
ros2 run rqt_image_view rqt_image_view /aruco_single/result >> ${LOG_FILE} 2>&1 &
ARUCO_VIEWER_PID=$!

# 等待图像查看器启动
echo -e "${YELLOW}等待图像查看器启动...${NC}"
sleep 2

echo -e "${GREEN}图像查看窗口已启动${NC}"
echo -e "${GREEN}  - 相机原始图像窗口 (PID: $IMAGE_VIEWER_PID)${NC}"
echo -e "${GREEN}  - ArUco检测结果窗口 (PID: $ARUCO_VIEWER_PID)${NC}"

# 等待相机话题数据
echo -e "${YELLOW}等待相机图像话题数据...${NC}"
for i in {1..30}; do
    if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
        echo -e "${GREEN}检测到相机图像话题${NC}"
        break
    fi
    if [ $i -eq 30 ]; then
        echo -e "${RED}等待相机图像话题超时！${NC}"
        echo -e "${RED}请检查相机连接和相机节点状态${NC}"
        kill $SERVICE_PID $ARUCO_PID $CAMERA_PID 2>/dev/null || true
        exit 1
    fi
    sleep 1
done

# 等待Aruco姿态话题数据
echo -e "${YELLOW}等待Aruco标记检测...${NC}"
for i in {1..30}; do
    if ros2 topic list 2>/dev/null | grep -q "/aruco_single/pose"; then
        echo -e "${GREEN}检测到Aruco姿态话题${NC}"
        break
    fi
    if [ $i -eq 30 ]; then
        echo -e "${RED}等待Aruco姿态话题超时！${NC}"
        echo -e "${RED}请检查Aruco节点状态和标记板是否在视野内${NC}"
        kill $SERVICE_PID $ARUCO_PID $CAMERA_PID 2>/dev/null || true
        exit 1
    fi
    sleep 1
done

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}手眼标定系统已就绪！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}系统信息:${NC}"
echo -e "  - RealSense相机: 已启动 (PID: $CAMERA_PID)"
echo -e "  - ArUco标记检测: 已启动 (PID: $ARUCO_PID)"
echo -e "  - 相机原始图像窗口: 已启动 (PID: $IMAGE_VIEWER_PID)"
echo -e "  - ArUco检测结果窗口: 已启动 (PID: $ARUCO_VIEWER_PID)"
echo -e "  - 标记ID: 123"
echo -e "  - 标记尺寸: 10cm"
echo -e "  - 相机话题: /camera/camera/color/image_raw"
echo -e "  - 姿态话题: /aruco_single/pose"
echo -e "  - 检测结果话题: /aruco_single/result"
echo ""
echo -e "${YELLOW}操作说明:${NC}"
echo -e "  1. 将ArUco标记板（ID: 123）放置在相机视野内"
echo -e "  2. 查看图像窗口确认标记板被正确检测（绿色边框）"
echo -e "  3. 移动机器人/机械臂到不同位置"
echo -e "  4. 每次移动后，按照提示输入指令"
echo -e "     - 'r': 记录当前相机姿态"
echo -e "     - 然后输入工具坐标 (格式: X,Y,Z,A,B,C)"
echo -e "       单位: 位置为mm，角度为度（ZYX欧拉角）"
echo -e "       示例: 1631.73,254.42,1154.09,2.11,0.82,178.96"
echo -e "  5. 完成多组数据采集后，输入 'c' 计算标定结果"
echo -e "  6. 输入 'q' 退出标定流程"
echo ""
echo -e "${YELLOW}建议至少采集10-20组数据以获得准确结果${NC}"
echo -e "${YELLOW}数据采集时注意:${NC}"
echo -e "  - 每次移动距离不要太小（建议>5cm）"
echo -e "  - 标记板保持在相机视野内，确保检测稳定"
echo -e "  - 观察Aruco检测结果窗口，确认标记被正确识别"
echo -e "  - 避免标记板反光、模糊或部分遮挡"
echo -e "  - 尽量覆盖不同的姿态和位置"
echo ""
echo -e "${GREEN}========================================${NC}"

# 启动交互式标定程序
ros2 run hand_eye_calib hand_eye_bringup.py

# 标定完成后清理
echo ""
echo -e "${YELLOW}停止手眼标定系统...${NC}"
kill $SERVICE_PID 2>/dev/null || true
kill $ARUCO_PID 2>/dev/null || true
kill $CAMERA_PID 2>/dev/null || true
kill $IMAGE_VIEWER_PID 2>/dev/null || true
kill $ARUCO_VIEWER_PID 2>/dev/null || true

# 等待进程结束
sleep 2

echo -e "${GREEN}手眼标定系统已停止${NC}"
echo -e "${GREEN}日志文件: ${LOG_FILE}${NC}"
echo -e "${GREEN}========================================${NC}"