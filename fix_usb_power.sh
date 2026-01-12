#!/bin/bash

# RealSense相机USB电源管理修复脚本
# 此脚本将禁用RealSense相机的USB电源管理，防止相机自动断开

echo "正在修复RealSense相机USB电源管理..."

# 方法1: 使用udev规则（永久生效）
echo "创建udev规则..."
sudo tee /etc/udev/rules.d/99-realsense-usb.rules > /dev/null <<EOF
# Intel RealSense D435IF
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b3a", TEST=="power/control", ATTR{power/control}="on"
EOF

# 重新加载udev规则
echo "重新加载udev规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# 方法2: 立即生效（当前会话）
echo "立即应用设置..."
sudo bash -c 'echo on > /sys/bus/usb/devices/4-3/power/control'

echo ""
echo "✓ 修复完成！"
echo ""
echo "验证设置："
cat /sys/bus/usb/devices/4-3/power/control
echo ""

echo "现在可以启动系统了："
echo "  source install/setup.bash"
echo "  ros2 launch norm_calc system_with_foxglove.launch.py"
echo ""
echo "或者使用不包含相机的启动文件："
echo "  source install/setup.bash"
echo "  ros2 launch norm_calc system_without_camera.launch.py"
echo ""