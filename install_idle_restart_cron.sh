#!/bin/bash

# ============================================================
# 闲时重启 Cron 任务安装脚本
# 功能：将 idle_restart.sh 添加到 root 用户的 crontab，每天凌晨3点执行
# 注意：需要使用 root crontab，因为 reboot 命令需要 root 权限
# ============================================================

SCRIPT_PATH="/home/bosch/vChisel_ros2_ws/idle_restart.sh"
CRON_JOB="0 3 * * * $SCRIPT_PATH"

echo "=========================================="
echo "vChisel 闲时重启 Cron 任务安装工具"
echo "=========================================="
echo ""
echo "注意：此脚本需要将任务添加到 root 用户的 crontab"
echo "因为 reboot 命令需要 root 权限"
echo ""

# 检查脚本是否存在
if [ ! -f "$SCRIPT_PATH" ]; then
    echo "错误：脚本不存在：$SCRIPT_PATH"
    exit 1
fi

# 检查脚本是否可执行
if [ ! -x "$SCRIPT_PATH" ]; then
    echo "添加执行权限..."
    chmod +x "$SCRIPT_PATH"
fi

# 检查是否已经安装（root crontab）
if sudo crontab -l 2>/dev/null | grep -q "$SCRIPT_PATH"; then
    echo "警告：Root cron 任务已存在"
    echo ""
    echo "当前的 root cron 任务："
    sudo crontab -l | grep "$SCRIPT_PATH"
    echo ""
    read -p "是否要重新安装？(y/n): " -n 1 -r
    echo ""
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "取消安装"
        exit 0
    fi
    
    # 删除旧的任务
    echo "删除旧的 cron 任务..."
    sudo crontab -l 2>/dev/null | grep -v "$SCRIPT_PATH" | sudo crontab -
fi

# 添加新的 cron 任务到 root crontab
echo "添加新的 cron 任务到 root crontab..."
(sudo crontab -l 2>/dev/null; echo "$CRON_JOB") | sudo crontab -

# 验证安装
echo ""
echo "=========================================="
echo "Cron 任务安装完成"
echo "=========================================="
echo ""
echo "任务内容："
sudo crontab -l | grep "$SCRIPT_PATH"
echo ""
echo "执行时间：每天凌晨 3:00"
echo "执行条件：系统运行且相机已关闭超过5分钟"
echo "执行操作：重启 PC 系统"
echo ""
echo "查看 cron 日志："
echo "  grep CRON /var/log/syslog | grep idle_restart"
echo ""
echo "手动测试（需要 root 权限）："
echo "  sudo $SCRIPT_PATH"
echo ""
echo "删除任务："
echo "  sudo crontab -e  # 然后删除相关行"
echo "  或者运行：./uninstall_idle_restart_cron.sh"
echo ""
echo "注意事项："
echo "1. 任务已添加到 root 用户的 crontab"
echo "2. 脚本会检查闲时条件，只有在满足条件时才会重启"
echo "3. 重启前会停止项目进程并清理临时文件"
echo ""
