#!/bin/bash

# ============================================================
# vChisel 系统桌面自启动卸载脚本
# 功能：移除桌面自启动文件
# ============================================================

AUTOSTART_DIR="/home/bosch/.config/autostart"
DESKTOP_FILE="$AUTOSTART_DIR/vchisel-system.desktop"

echo "=========================================="
echo "vChisel 系统桌面自启动卸载工具"
echo "=========================================="
echo ""

# 检查文件是否存在
if [ ! -f "$DESKTOP_FILE" ]; then
    echo "未找到自启动文件：$DESKTOP_FILE"
    exit 0
fi

echo "当前自启动文件："
cat "$DESKTOP_FILE"
echo ""

read -p "确认删除此自启动文件？(y/n): " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "取消删除"
    exit 0
fi

# 删除文件
echo "删除自启动文件..."
rm -f "$DESKTOP_FILE"

echo ""
echo "=========================================="
echo "自启动已成功卸载"
echo "=========================================="
