#!/bin/bash

# ============================================================
# GUI 就绪检测测试脚本
# 功能：测试 wait_for_gui.sh 是否正常工作
# ============================================================

WORKSPACE_DIR="/home/bosch/vChisel_ros2_ws"
WAIT_GUI_SCRIPT="$WORKSPACE_DIR/wait_for_gui.sh"

echo "=========================================="
echo "GUI 就绪检测测试工具"
echo "=========================================="
echo ""

# 检查 wait_for_gui.sh 是否存在
if [ ! -f "$WAIT_GUI_SCRIPT" ]; then
    echo "错误：未找到 GUI 检测脚本: $WAIT_GUI_SCRIPT"
    echo "请先运行: ./install_autostart.sh"
    exit 1
fi

# 检查是否可执行
if [ ! -x "$WAIT_GUI_SCRIPT" ]; then
    echo "警告：脚本不可执行，正在设置权限..."
    chmod +x "$WAIT_GUI_SCRIPT"
fi

echo "测试 1：快速检测 GUI 状态（等待 10 秒）"
echo "----------------------------------------"
if bash "$WAIT_GUI_SCRIPT" 10; then
    echo ""
    echo "✓ 测试通过：GUI 已就绪"
    TEST1_RESULT="PASS"
else
    echo ""
    echo "✗ 测试失败：GUI 未就绪或超时"
    TEST1_RESULT="FAIL"
fi

echo ""
echo "=========================================="
echo "测试结果汇总"
echo "=========================================="
echo "测试 1 (快速检测): $TEST1_RESULT"
echo ""

# 显示当前 GUI 状态
echo "=========================================="
echo "当前 GUI 状态信息"
echo "=========================================="
echo "DISPLAY: ${DISPLAY:-未设置}"
echo "XAUTHORITY: ${XAUTHORITY:-$HOME/.Xauthority}"
echo "用户: $USER"
echo "桌面进程:"
pgrep -l "gnome-shell|unity|kwin|xfwm4|mate-session|lxsession" || echo "  未检测到桌面环境进程"
echo ""

# 检查 X authority 文件
if [ -f "$HOME/.Xauthority" ]; then
    echo "Xauthority 文件: 存在"
    echo "  大小: $(stat -c%s "$HOME/.Xauthority" 2>/dev/null || echo "未知") 字节"
    echo "  权限: $(stat -c%a "$HOME/.Xauthority" 2>/dev/null || echo "未知")"
else
    echo "Xauthority 文件: 不存在"
fi

echo ""

# 检查 X 显示服务器
if command -v xdpyinfo &> /dev/null; then
    echo "X 显示服务器状态:"
    if DISPLAY=:0 xdpyinfo > /dev/null 2>&1; then
        echo "  ✓ X 显示服务器响应正常"
    else
        echo "  ✗ X 显示服务器无响应"
    fi
else
    echo "警告：xdpyinfo 未安装，无法检测 X 显示服务器状态"
fi

echo ""
echo "=========================================="
echo "建议"
echo "=========================================="

if [ "$TEST1_RESULT" == "PASS" ]; then
    echo "✓ GUI 就绪检测功能正常"
    echo "✓ 可以正常启动可视化服务"
    echo ""
    echo "下一步："
    echo "  1. 安装开机自启动: ./install_autostart.sh"
    echo "  2. 查看服务状态: sudo systemctl status vchisel-system.service"
    echo "  3. 查看日志: tail -f /home/bosch/logs/visual.log"
else
    echo "✗ GUI 就绪检测失败"
    echo ""
    echo "可能的原因："
    echo "  1. 当前不在图形界面环境"
    echo "  2. 用户未登录桌面"
    echo "  3. DISPLAY 环境变量错误"
    echo "  4. Xauthority 文件丢失或权限错误"
    echo ""
    echo "建议检查："
    echo "  1. 确认已登录桌面环境"
    echo "  2. 检查 DISPLAY 变量: echo \$DISPLAY"
    echo "  3. 检查 Xauthority: ls -l ~/.Xauthority"
    echo "  4. 查看系统日志: journalctl -xe"
fi

echo ""

# 根据测试结果返回退出码
if [ "$TEST1_RESULT" == "PASS" ]; then
    exit 0
else
    exit 1
fi
