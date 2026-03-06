#!/bin/bash

# ============================================================
# 闲时重启 Cron 任务卸载脚本
# 功能：从 root crontab 中移除 idle_restart.sh
# ============================================================

SCRIPT_PATH="/home/bosch/vChisel_ros2_ws/idle_restart.sh"

echo "=========================================="
echo "vChisel 闲时重启 Cron 任务卸载工具"
echo "=========================================="
echo ""

# 检查是否已经安装（root crontab）
if ! sudo crontab -l 2>/dev/null | grep -q "$SCRIPT_PATH"; then
    echo "未找到相关的 root cron 任务"
    exit 0
fi

echo "当前的 root cron 任务："
sudo crontab -l | grep "$SCRIPT_PATH"
echo ""

read -p "确认删除此 cron 任务？(y/n): " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "取消删除"
    exit 0
fi

# 删除任务
echo "删除 root cron 任务..."
sudo crontab -l 2>/dev/null | grep -v "$SCRIPT_PATH" | sudo crontab -

# 验证删除
if sudo crontab -l 2>/dev/null | grep -q "$SCRIPT_PATH"; then
    echo "错误：删除失败"
    exit 1
else
    echo ""
    echo "=========================================="
    echo "Cron 任务已成功删除"
    echo "=========================================="
fi
