#!/bin/bash

# ============================================================
# vChisel 系统开机自启动安装脚本
# 功能：创建 systemd 服务，实现开机自启动
# ============================================================

SERVICE_NAME="vchisel-system"
WORKSPACE_DIR="/home/bosch/vChisel_ros2_ws"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

echo "=========================================="
echo "vChisel 系统开机自启动安装工具"
echo "=========================================="
echo ""

# 检查是否已经安装
if [ -f "$SERVICE_FILE" ]; then
    echo "警告：服务已存在"
    echo ""
    echo "当前服务状态："
    sudo systemctl status ${SERVICE_NAME}.service --no-pager
    echo ""
    read -p "是否要重新安装？(y/n): " -n 1 -r
    echo ""
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "取消安装"
        exit 0
    fi
    
    # 停止并禁用旧服务
    echo "停止并删除旧服务..."
    sudo systemctl stop ${SERVICE_NAME}.service 2>/dev/null || true
    sudo systemctl disable ${SERVICE_NAME}.service 2>/dev/null || true
    sudo rm -f "$SERVICE_FILE"
fi

echo "创建 systemd 服务文件..."

# 创建 systemd 服务文件
sudo tee "$SERVICE_FILE" > /dev/null << 'EOF'
[Unit]
Description=vChisel ROS2 System (GUI Ready)
Documentation=https://github.com/Roarpeng/vChisel_ros2_ws

# 依赖关系：确保网络和GUI完全启动
After=network-online.target
After=graphical.target
After=display-manager.service
Wants=network-online.target
Wants=graphical.target

[Service]
Type=simple
User=bosch
Group=bosch

# 环境变量设置
Environment="ROS_DOMAIN_ID=0"
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/bosch/.Xauthority"
Environment="HOME=/home/bosch"
Environment="USER=bosch"
Environment="PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin"

# 工作目录
WorkingDirectory=/home/bosch/vChisel_ros2_ws

# 启动命令（包含GUI等待逻辑）
ExecStartPre=/bin/sleep 10
ExecStart=/home/bosch/vChisel_ros2_ws/start_system.sh
ExecStop=/bin/kill -INT ${MAINPID}

# 重启策略
Restart=on-failure
RestartSec=15
StartLimitIntervalSec=60
StartLimitBurst=3

# 日志输出
StandardOutput=append:/home/bosch/logs/visual.log
StandardError=append:/home/bosch/logs/visual.log

# 安全设置
PrivateTmp=true
NoNewPrivileges=true

# 资源限制
LimitNOFILE=65536
LimitNPROC=4096

[Install]
WantedBy=graphical.target
EOF

echo "服务文件已创建：$SERVICE_FILE"
echo ""

# 重新加载 systemd
echo "重新加载 systemd 守护进程..."
sudo systemctl daemon-reload

# 启用服务
echo "启用开机自启动..."
sudo systemctl enable ${SERVICE_NAME}.service

# 启动服务
echo ""
read -p "是否立即启动服务？(y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "启动服务..."
    sudo systemctl start ${SERVICE_NAME}.service
    sleep 3
    echo ""
    echo "服务状态："
    sudo systemctl status ${SERVICE_NAME}.service --no-pager
fi

echo ""
echo "=========================================="
echo "安装完成"
echo "=========================================="
echo ""
echo "服务名称：${SERVICE_NAME}.service"
echo "服务文件：$SERVICE_FILE"
echo ""
echo "管理命令："
echo "  启动服务：   sudo systemctl start ${SERVICE_NAME}.service"
echo "  停止服务：   sudo systemctl stop ${SERVICE_NAME}.service"
echo "  重启服务：   sudo systemctl restart ${SERVICE_NAME}.service"
echo "  查看状态：   sudo systemctl status ${SERVICE_NAME}.service"
echo "  查看日志：   sudo journalctl -u ${SERVICE_NAME}.service -f"
echo "  禁用自启：   sudo systemctl disable ${SERVICE_NAME}.service"
echo ""
echo "注意事项："
echo "1. 服务会在网络和GUI完全就绪后自动运行"
echo "2. 启动前会等待GUI完全就绪（最多等待60秒）"
echo "3. 日志会同时写入 /home/bosch/logs/visual.log"
echo "4. 服务失败后会自动重启（延迟15秒，最多重试3次）"
echo ""
echo "GUI 就绪检测："
echo "  - 检测 DISPLAY 环境变量"
echo "  - 检测 Xauthority 文件"
echo "  - 检测桌面环境进程"
echo "  - 检测用户会话状态"
echo "  - 检测 X 显示服务器响应"
echo ""
