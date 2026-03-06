#!/bin/bash

# ============================================================
# 闲时重启逻辑测试脚本
# 功能：模拟不同场景测试 idle_restart.sh 的逻辑
# 注意：此脚本仅测试逻辑判断，不会真正重启PC
# ============================================================

IDLE_RESTART_SCRIPT="/home/bosch/vChisel_ros2_ws/idle_restart.sh"
WORKSPACE_DIR="/home/bosch/vChisel_ros2_ws"

echo "=========================================="
echo "闲时重启逻辑测试"
echo "=========================================="
echo "注意：此脚本仅测试逻辑判断，不会真正重启PC"
echo ""

# 测试函数
test_case() {
    local test_name=$1
    local expected_result=$2
    
    echo "----------------------------------------"
    echo "测试：$test_name"
    echo "预期：$expected_result"
    echo "----------------------------------------"
    
    # 执行测试
    bash "$IDLE_RESTART_SCRIPT"
    local exit_code=$?
    
    echo ""
    if [ $exit_code -eq 0 ]; then
        echo "✓ 测试通过（退出码：$exit_code）"
    else
        echo "✗ 测试失败（退出码：$exit_code）"
    fi
    echo ""
}

# 准备测试环境
echo "准备测试环境..."
rm -f /tmp/vchisel_camera_status.txt
rm -f /tmp/vchisel_camera_off_time.txt
rm -f "$WORKSPACE_DIR/.vchisel_system.lock"
echo ""

# 测试1：系统未运行
echo "=========================================="
echo "测试1：系统未运行"
echo "=========================================="
test_case "系统未运行（无锁文件）" "应跳过重启（退出码0）"

# 测试2：系统运行但相机开启
echo "=========================================="
echo "测试2：系统运行但相机开启"
echo "=========================================="
echo "创建锁文件..."
echo $$ > "$WORKSPACE_DIR/.vchisel_system.lock"
echo "创建相机开启状态文件..."
echo "on" > /tmp/vchisel_camera_status.txt
test_case "系统运行，相机开启" "应跳过重启（退出码0）"

# 测试3：系统运行，相机关闭但时间不足
echo "=========================================="
echo "测试3：系统运行，相机关闭但时间不足"
echo "=========================================="
echo "创建相机关闭状态文件..."
echo "off" > /tmp/vchisel_camera_status.txt
echo "记录关闭时间为3分钟前..."
echo $(($(date +%s) - 180)) > /tmp/vchisel_camera_off_time.txt
test_case "相机关闭3分钟" "应跳过重启（退出码0）"

# 测试4：系统运行，相机关闭且时间足够（但不是凌晨3点）
echo "=========================================="
echo "测试4：系统运行，相机关闭且时间足够"
echo "（注意：此测试只能在非凌晨3点时通过）"
echo "=========================================="
echo "记录关闭时间为6分钟前..."
echo $(($(date +%s) - 360)) > /tmp/vchisel_camera_off_time.txt
test_case "相机关闭6分钟，但非凌晨3点" "应跳过重启（退出码0）"

# 清理测试环境
echo "=========================================="
echo "清理测试环境..."
echo "=========================================="
rm -f /tmp/vchisel_camera_status.txt
rm -f /tmp/vchisel_camera_off_time.txt
rm -f "$WORKSPACE_DIR/.vchisel_system.lock"

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
echo ""
echo "重要提示："
echo "1. 此脚本仅测试逻辑判断，不会真正重启PC"
echo "2. 真正的重启测试需要在凌晨3点执行，且需要 root 权限"
echo "3. 重启前会停止项目进程并清理临时文件"
echo ""
echo "手动测试重启（危险操作，仅用于测试）："
echo "  1. 设置系统时间为凌晨3点："
echo "     sudo date -s '03:00:00'"
echo ""
echo "  2. 创建必要的测试文件："
echo "     echo \$\$ > $WORKSPACE_DIR/.vchisel_system.lock"
echo "     echo 'off' > /tmp/vchisel_camera_status.txt"
echo "     echo \$(($(date +%s) - 360)) > /tmp/vchisel_camera_off_time.txt"
echo ""
echo "  3. 执行重启脚本（需要 root 权限）："
echo "     sudo $IDLE_RESTART_SCRIPT"
echo ""
echo "  4. 测试完成后恢复时间："
echo "     sudo ntpdate -s time.nist.gov"
echo "     或"
echo "     sudo timedatectl set-ntp true"
echo ""
