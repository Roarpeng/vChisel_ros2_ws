#!/bin/bash

# 一键启动脚本 - vChisel_ros2_ws系统

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==========================================="
echo "vChisel_ros2_ws 一键启动脚本"
echo "==========================================="
echo "正在进入项目目录: $SCRIPT_DIR"
cd "$SCRIPT_DIR"

echo "正在设置ROS2 Humble环境..."
source /opt/ros/humble/setup.bash

echo "正在设置项目环境..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "项目环境设置成功"
else
    echo "警告: 未找到install/setup.bash，尝试编译项目..."
    if command -v colcon &> /dev/null; then
        colcon build --packages-select norm_calc snap_7
        source install/setup.bash
        echo "项目已编译并环境已设置"
    else
        echo "错误: colcon 未找到，无法编译项目"
        exit 1
    fi
fi

echo "检查必要的依赖..."
if ! command -v ros2 &> /dev/null; then
    echo "错误: ROS2 未安装或未正确配置"
    exit 1
fi

echo "检查snap_7 launch文件..."
if [ ! -f "src/snap_7/launch/snap_7.launch.py" ]; then
    echo "错误: 未找到snap_7.launch.py文件"
    exit 1
fi

echo "==========================================="
echo "准备启动snap_7系统..."
echo "==========================================="
exec ros2 launch snap_7 snap_7.launch.py