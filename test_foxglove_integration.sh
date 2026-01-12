#!/bin/bash

# Foxglove集成测试脚本
# 用于验证Foxglove Bridge和可视化功能是否正常工作

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_test() {
    echo -e "${BLUE}[TEST]${NC} $1"
}

print_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

print_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
}

print_info() {
    echo -e "${YELLOW}[INFO]${NC} $1"
}

# 加载ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ""
echo "=========================================="
echo "  Foxglove 集成测试"
echo "=========================================="
echo ""

# 测试1: 检查Foxglove Bridge包
print_test "检查Foxglove Bridge包..."
if ros2 pkg list | grep -q "foxglove_bridge"; then
    print_pass "Foxglove Bridge已安装"
else
    print_fail "Foxglove Bridge未安装"
    print_info "安装命令: sudo apt install ros-humble-foxglove-bridge"
    exit 1
fi

# 测试2: 检查启动文件
print_test "检查Foxglove启动文件..."
if [ -f "src/norm_calc/launch/foxglove_bridge.launch.py" ]; then
    print_pass "foxglove_bridge.launch.py存在"
else
    print_fail "foxglove_bridge.launch.py不存在"
    exit 1
fi

if [ -f "src/norm_calc/launch/system_with_foxglove.launch.py" ]; then
    print_pass "system_with_foxglove.launch.py存在"
else
    print_fail "system_with_foxglove.launch.py不存在"
    exit 1
fi

# 测试3: 检查布局配置文件
print_test "检查Foxglove布局配置文件..."
if [ -f "src/norm_calc/config/foxglove_layout.json" ]; then
    print_pass "foxglove_layout.json存在"
else
    print_fail "foxglove_layout.json不存在"
    exit 1
fi

# 测试4: 检查参数辅助脚本
print_test "检查参数辅助脚本..."
if [ -f "src/norm_calc/scripts/foxglove_param_helper.py" ]; then
    print_pass "foxglove_param_helper.py存在"
else
    print_fail "foxglove_param_helper.py不存在"
    exit 1
fi

# 测试5: 验证JSON格式
print_test "验证布局配置JSON格式..."
if python3 -m json.tool src/norm_calc/config/foxglove_layout.json > /dev/null 2>&1; then
    print_pass "JSON格式正确"
else
    print_fail "JSON格式错误"
    exit 1
fi

# 测试6: 检查启动脚本
print_test "检查启动脚本..."
if [ -f "start_foxglove_system.sh" ]; then
    print_pass "start_foxglove_system.sh存在"
    if [ -x "start_foxglove_system.sh" ]; then
        print_pass "启动脚本有执行权限"
    else
        print_fail "启动脚本无执行权限"
        exit 1
    fi
else
    print_fail "start_foxglove_system.sh不存在"
    exit 1
fi

# 测试7: 检查文档
print_test "检查集成文档..."
if [ -f "FOXGLOVE_INTEGRATION.md" ]; then
    print_pass "FOXGLOVE_INTEGRATION.md存在"
else
    print_fail "FOXGLOVE_INTEGRATION.md不存在"
    exit 1
fi

echo ""
echo "=========================================="
echo "  测试总结"
echo "=========================================="
echo ""
print_pass "所有测试通过！"
echo ""
print_info "下一步："
echo "  1. 启动系统: ./start_foxglove_system.sh"
echo "  2. 下载Foxglove Studio: https://foxglove.dev/download"
echo "  3. 连接到: ws://localhost:8765"
echo "  4. 导入布局: src/norm_calc/config/foxglove_layout.json"
echo ""
print_info "更多信息请参考: FOXGLOVE_INTEGRATION.md"
echo ""