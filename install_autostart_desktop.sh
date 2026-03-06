#!/bin/bash

# ============================================================
# vChisel 系统桌面自启动安装脚本
# 功能：创建桌面自启动文件，用户登录后自动运行
# ============================================================

WORKSPACE_DIR="/home/bosch/vChisel_ros2_ws"
AUTOSTART_DIR="/home/bosch/.config/autostart"
DESKTOP_FILE="$AUTOSTART_DIR/vchisel-system.desktop"

echo "=========================================="
echo "vChisel 系统桌面自启动安装工具"
echo "=========================================="
echo ""

# 创建自启动目录
echo "创建自启动目录..."
mkdir -p "$AUTOSTART_DIR"

# 检查是否已经安装
if [ -f "$DESKTOP_FILE" ]; then
    echo "警告：自启动文件已存在"
    echo ""
    cat "$DESKTOP_FILE"
    echo ""
    read -p "是否要重新安装？(y/n): " -n 1 -r
    echo ""
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "取消安装"
        exit 0
    fi
fi

echo "创建桌面自启动文件..."

# 创建 .desktop 文件
cat > "$DESKTOP_FILE" << EOF
[Desktop Entry]
Type=Application
Name=vChisel System
Comment=ROS2 Visual Processing System
Exec=gnome-terminal -- bash -c "cd $WORKSPACE_DIR && ./start_system.sh; exec bash"
Icon=utilities-terminal
Terminal=false
Categories=Development;
X-GNOME-Autostart-enabled=true
X-GNOME-Autostart-Delay=10
EOF

# 设置权限
chmod +x "$DESKTOP_FILE"

echo "自启动文件已创建：$DESKTOP_FILE"
echo ""

echo "=========================================="
echo "安装完成"
echo "=========================================="
echo ""
echo "自启动方式："
echo "  - 用户登录桌面后自动运行"
echo "  - 延迟10秒启动（等待系统初始化）"
echo "  - 在新终端窗口中运行"
echo ""
echo "管理命令："
echo "  查看自启动文件：cat $DESKTOP_FILE"
echo "  删除自启动：    rm $DESKTOP_FILE"
echo ""
echo "注意事项："
echo "1. 需要登录桌面环境才会自动启动"
echo "2. 启动时会打开一个新的终端窗口"
echo "3. 如果不想要终端窗口，可以修改 Exec 行"
echo ""
