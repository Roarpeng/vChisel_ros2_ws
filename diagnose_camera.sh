#!/bin/bash

echo "=========================================="
echo "RealSense相机诊断工具"
echo "=========================================="
echo ""

# 1. 检查USB连接
echo "1. 检查USB设备连接..."
lsusb | grep -i realsense
if [ $? -eq 0 ]; then
    echo "✓ RealSense相机已连接"
else
    echo "✗ RealSense相机未连接"
    echo "  请检查USB连接"
fi
echo ""

# 2. 检查video设备
echo "2. 检查video设备..."
ls -la /dev/video* 2>/dev/null | head -10
echo ""

# 3. 检查USB带宽
echo "3. 检查USB带宽..."
lsusb -t | grep -A 10 -B 2 "8086:0b3a"
echo ""

# 4. 检查是否有进程占用相机
echo "4. 检查是否有进程占用相机..."
lsof /dev/video* 2>/dev/null | grep -v COMMAND || echo "  没有进程占用video设备"
echo ""

# 5. 测试相机连接
echo "5. 测试相机连接..."
if command -v rs-enumerate-devices &> /dev/null; then
    rs-enumerate-devices 2>&1 | head -20
else
    echo "  rs-enumerate-devices 命令未找到"
    echo "  请安装: sudo apt install ros-humble-librealsense2-tools"
fi
echo ""

# 6. 检查USB电源管理
echo "6. 检查USB电源管理..."
for device in /sys/bus/usb/devices/*/power/control; do
    if [ -f "$device" ]; then
        echo "  $device: $(cat $device)"
    fi
done | head -10
echo ""

echo "=========================================="
echo "诊断完成"
echo "=========================================="
echo ""
echo "建议："
echo "1. 如果相机未连接，请检查USB线缆和端口"
echo "2. 如果USB带宽不足，请尝试使用USB 3.0端口（蓝色）"
echo "3. 如果有进程占用相机，请先结束这些进程"
echo "4. 如果USB电源管理设置为auto，可以尝试设置为on"
echo ""
echo "快速修复命令："
echo "  # 禁用USB电源管理"
echo "  sudo bash -c 'for i in /sys/bus/usb/devices/*/power/control; do echo on > \$i; done'"
echo ""
echo "  # 重置USB设备"
echo "  sudo usbreset \$(lsusb | grep RealSense | awk '{print $6}')"
echo ""