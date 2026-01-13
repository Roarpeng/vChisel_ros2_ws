#!/bin/bash

# vChisel ROS2 + Foxglove 可视化系统启动脚本
# 版本: 1.0
# 日期: 2026-01-08

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查依赖
check_dependencies() {
    print_info "检查依赖项..."
    
    # 检查ROS2环境
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2环境未加载，请先运行: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    
    # 检查Foxglove Bridge
    if ! ros2 pkg list | grep -q "foxglove_bridge"; then
        print_warning "Foxglove Bridge未安装"
        print_info "安装命令: sudo apt install ros-humble-foxglove-bridge"
        read -p "是否现在安装? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            sudo apt install ros-humble-foxglove-bridge
            print_success "Foxglove Bridge安装完成"
        else
            print_error "需要Foxglove Bridge才能使用可视化功能"
            exit 1
        fi
    fi
    
    print_success "依赖检查完成"
}

# 启动系统
launch_system() {
    print_info "启动vChisel ROS2 + Foxglove系统..."
    
    # 加载ROS2环境
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    # 获取工作空间目录
    WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    cd "$WS_DIR"
    
    # 检查是否需要构建
    if [ ! -d "install/norm_calc" ] || [ ! -d "install/snap_7" ]; then
        print_warning "检测到项目未构建或构建不完整"
        read -p "是否现在构建? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_info "开始构建项目..."
            colcon build --packages-select norm_calc snap_7
            source install/setup.bash
            print_success "构建完成"
        else
            print_error "无法启动未构建的项目"
            exit 1
        fi
    fi
    
    # 启动参数
    FOXGLOVE_PORT=${FOXGLOVE_PORT:-8765}
    FOXGLOVE_ENABLED=${FOXGLOVE_ENABLED:-true}
    
    print_info "配置:"
    echo "  - Foxglove端口: $FOXGLOVE_PORT"
    echo "  - Foxglove启用: $FOXGLOVE_ENABLED"
    echo ""
    
    # 启动系统
    print_info "正在启动系统，请稍候..."
    print_info "按 Ctrl+C 停止系统"
    echo ""
    
    ros2 launch norm_calc with_foxglove.launch.py \
        foxglove_port:=$FOXGLOVE_PORT
}

# 显示帮助信息
show_help() {
    cat << EOF
vChisel ROS2 + Foxglove 可视化系统启动脚本

用法: $0 [选项]

选项:
    -p, --port PORT        指定Foxglove WebSocket端口 (默认: 8765)
    -d, --disable-foxglove  禁用Foxglove Bridge
    -h, --help             显示此帮助信息

环境变量:
    FOXGLOVE_PORT          Foxglove WebSocket端口
    FOXGLOVE_ENABLED       是否启用Foxglove (true/false)

示例:
    # 默认启动（启用Foxglove，端口8765）
    $0

    # 指定端口启动
    $0 --port 9000

    # 禁用Foxglove启动
    $0 --disable-foxglove

    # 使用环境变量
    FOXGLOVE_PORT=9000 $0

连接Foxglove Studio:
    1. 下载Foxglove Studio: https://foxglove.dev/download
    2. 打开Foxglove Studio
    3. 连接到: ws://localhost:8765
    4. 导入布局: src/norm_calc/config/foxglove_layout.json

更多信息请参考: FOXGLOVE_INTEGRATION.md

EOF
}

# 主函数
main() {
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -p|--port)
                FOXGLOVE_PORT="$2"
                shift 2
                ;;
            -d|--disable-foxglove)
                FOXGLOVE_ENABLED=false
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                print_error "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 显示欢迎信息
    echo ""
    echo "=========================================="
    echo "  vChisel ROS2 + Foxglove 可视化系统"
    echo "=========================================="
    echo ""
    
    # 检查依赖
    check_dependencies
    
    # 启动系统
    launch_system
}

# 捕获Ctrl+C信号
trap 'print_info "正在停止系统..."; exit 0' INT TERM

# 运行主函数
main "$@"