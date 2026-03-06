#!/bin/bash

# ============================================================
# vChisel 系统开机自启动卸载脚本
# 功能：移除 systemd 服务
# ============================================================

SERVICE_NAME="vchisel-system"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

echo "=========================================="
echo "vChisel 系统开机自启动卸载工具"
echo "=========================================="
echo ""

# 检查服务是否存在
if [ ! -f "$SERVICE_FILE" ]; then
    echo "未找到服务文件：$SERVICE_FILE"
    exit 0
fi

echo "当前服务状态："
sudo systemctl status ${SERVICE_NAME}.service --no-pager || true
echo ""

read -p "确认删除此服务？(y/n): " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "取消删除"
    exit 0
fi

# 停止服务
echo "停止服务..."
sudo systemctl stop ${SERVICE_NAME}.service 2>/dev/null || true

# 禁用服务
echo "禁用开机自启动..."
sudo systemctl disable ${SERVICE_NAME}.service 2>/dev/null || true

# 删除服务文件
echo "删除服务文件..."
sudo rm -f "$SERVICE_FILE"

# 重新加载 systemd
echo "重新加载 systemd 守护进程..."
sudo systemctl daemon-reload

echo ""
echo "=========================================="
echo "服务已成功卸载"
echo "=========================================="
