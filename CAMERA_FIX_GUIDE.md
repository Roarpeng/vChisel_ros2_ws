# RealSense相机问题修复指南

## 问题诊断

RealSense相机在启动后断开连接，错误信息：
```
xioctl(VIDIOC_QBUF) failed when requesting new frame! fd: 23 error: No such device
The device has been disconnected!
```

## 根本原因

USB电源管理设置为`auto`，导致系统在认为不需要相机时自动断电。

## 解决方案

### 1. 临时修复（当前会话）

```bash
sudo bash -c 'echo on > /sys/bus/usb/devices/4-3/power/control'
```

### 2. 永久修复（重启后生效）

```bash
# 运行修复脚本
./fix_usb_power.sh
```

或手动创建udev规则：

```bash
sudo tee /etc/udev/rules.d/99-realsense-usb.rules > /dev/null <<EOF
# Intel RealSense D435IF
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b3a", TEST=="power/control", ATTR{power/control}="on"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. 验证修复

```bash
# 检查USB电源管理设置
cat /sys/bus/usb/devices/4-3/power/control
# 应该输出：on

# 运行诊断脚本
./diagnose_camera.sh
```

## 启动系统

### 方案1：包含相机（推荐）

```bash
source install/setup.bash
ros2 launch norm_calc system_with_foxglove.launch.py
```

### 方案2：不包含相机（相机有问题时使用）

```bash
source install/setup.bash
ros2 launch norm_calc system_without_camera.launch.py
```

### 方案3：仅Foxglove + norm_calc（最简单）

```bash
source install/setup.bash
ros2 launch norm_calc system_without_camera.launch.py plc_enabled:=false camera_monitor_enabled:=false
```

## 可用参数

所有启动文件都支持以下参数：

- `foxglove_enabled:=true/false` - 启用/禁用Foxglove Bridge
- `foxglove_port:=8765` - Foxglove WebSocket端口
- `plc_enabled:=true/false` - 启用/禁用PLC客户端（system_without_camera.launch.py）
- `camera_monitor_enabled:=true/false` - 启用/禁用相机监控（system_without_camera.launch.py）

## 如果问题仍然存在

### 检查USB带宽

```bash
# 查看USB设备树
lsusb -t | grep -A 10 -B 2 "8086:0b3a"
```

确保相机连接到USB 3.0端口（通常为蓝色端口）。

### 尝试降低相机配置

编辑 `src/norm_calc/launch/system_with_foxglove.launch.py`，降低帧率和分辨率：

```python
'color_fps': 5.0,      # 从6.0降低到5.0
'depth_fps': 5.0,      # 从6.0降低到5.0
'color_width': 424,    # 从640降低到424
'color_height': 240,   # 从480降低到240
```

### 检查USB线缆

确保使用高质量的USB 3.0线缆，长度不超过3米。

### 尝试不同的USB端口

将相机连接到主板上的USB端口，而不是前置面板或USB集线器。

## 工具脚本

项目根目录提供了以下工具脚本：

- `diagnose_camera.sh` - 相机诊断工具
- `fix_usb_power.sh` - USB电源管理修复脚本
- `start_foxglove_system.sh` - 一键启动脚本

## 相关文档

- `FOXGLOVE_INTEGRATION.md` - Foxglove集成详细文档
- `FOXGLOVE_SUMMARY.md` - Foxglove集成总结
- `IFLOW.md` - 项目完整文档

## 技术支持

如果问题仍然存在，请提供以下信息：

1. 诊断脚本的完整输出：`./diagnose_camera.sh > diagnose_output.txt`
2. 系统日志：`journalctl -xe | grep -i realsense`
3. USB设备信息：`lsusb -v | grep -A 20 "8086:0b3a"`