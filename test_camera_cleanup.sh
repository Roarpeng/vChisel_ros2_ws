#!/bin/bash

# 测试相机进程清理功能
# 该脚本用于验证相机关闭后是否还有残留进程

echo "=== 相机进程清理测试 ==="
echo ""

# 1. 检查当前相机进程
echo "1. 检查当前相机进程..."
ps aux | grep -i realsense | grep -v grep || echo "  ✓ 没有相机进程在运行"
echo ""

# 2. 启动系统（如果需要）
echo "2. 启动 snap_7 节点（在另一个终端运行以下命令）:"
echo "   source /opt/ros/humble/setup.bash"
echo "   source install/setup.bash"
echo "   ros2 launch snap_7 snap_7.launch.py"
echo ""

# 3. 等待相机启动
read -p "3. 相机启动后，按 Enter 继续..."
echo ""

# 4. 检查相机进程
echo "4. 检查相机进程（应该有 1 个）..."
ps aux | grep -i realsense | grep -v grep
CAMERA_COUNT=$(ps aux | grep -i realsense | grep -v grep | wc -l)
echo "  相机进程数量: $CAMERA_COUNT"
echo ""

# 5. 触发相机关闭（通过 PLC 或手动）
echo "5. 触发相机关闭（等待 PLC 状态从 110 变为 130 或 0）..."
echo "   或者手动运行: ros2 service call /norm_calc norm_calc/srv/NormCalcData '{seq: 0}'"
echo ""

# 6. 等待相机关闭
read -p "6. 相机关闭后，按 Enter 检查残留进程..."
echo ""

# 7. 检查残留进程
echo "7. 检查残留相机进程（应该为 0）..."
ps aux | grep -i realsense | grep -v grep || echo "  ✓ 没有残留相机进程"
REMAINING_COUNT=$(ps aux | grep -i realsense | grep -v grep | wc -l)
echo "  残留进程数量: $REMAINING_COUNT"
echo ""

# 8. 检查图像话题发布者
echo "8. 检查图像话题发布者..."
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic info /camera/camera/color/image_raw --verbose | grep -A 5 "Publisher count"
echo ""

# 9. 验证结果
echo "=== 测试结果 ==="
if [ $REMAINING_COUNT -eq 0 ]; then
    echo "✓ 测试通过：没有残留相机进程"
else
    echo "✗ 测试失败：仍有 $REMAINING_COUNT 个残留相机进程"
    echo "  请检查 cam_shutdown() 函数是否正确执行"
fi
echo ""

# 10. 清理
echo "10. 清理所有相机进程..."
pkill -9 -f realsense2_camera
sleep 1
echo "  ✓ 清理完成"