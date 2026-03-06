#!/bin/bash

# ============================================================
# vChisel_ros2_ws 系统启动脚本
# 功能：启动视觉处理系统，包含日志管理、进程清理、错误处理
# ============================================================

# 检查是否使用 sudo
if [ "$EUID" -eq 0 ]; then
    echo "错误：请不要使用 sudo 运行此脚本！"
    echo "ROS2 节点应该以普通用户运行，使用 sudo 会导致："
    echo "  - 环境变量问题"
    echo "  - 权限问题"
    echo "  - Python 包找不到"
    echo ""
    echo "正确用法：./start_system.sh"
    exit 1
fi

set -e  # 遇到错误立即退出

# ============================================================
# 全局变量配置
# ============================================================
WORKSPACE_DIR="/home/bosch/vChisel_ros2_ws"
LOG_DIR="/home/bosch/logs"
LOG_FILE="$LOG_DIR/visual.log"
MAX_SIZE=10485760  # 10MB
MAX_LOGS=5         # 保留最近5个日志文件
LOCK_FILE="$WORKSPACE_DIR/.vchisel_system.lock"
LAUNCH_PACKAGE="snap_7"
LAUNCH_FILE="snap_7.launch.py"  # 后台服务（不包含可视化窗口）

# ============================================================
# 辅助函数
# ============================================================

# 日志记录函数
log() {
    local level=$1
    shift
    local message="$@"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [$level] $message" | tee -a "$LOG_FILE"
}

log_info() {
    log "INFO" "$@"
}

log_warn() {
    log "WARN" "$@"
}

log_error() {
    log "ERROR" "$@"
}

# 创建日志目录
create_log_dir() {
    if ! mkdir -p "$LOG_DIR" 2>/dev/null; then
        echo "错误：无法创建日志目录 $LOG_DIR"
        exit 1
    fi
    
    if ! touch "$LOG_FILE" 2>/dev/null; then
        echo "错误：无法创建日志文件 $LOG_FILE"
        exit 1
    fi
}

# 日志滚动函数（修复bug）
rotate_logs() {
    if [ ! -f "$LOG_FILE" ]; then
        return 0
    fi
    
    # 检查日志大小
    local file_size=$(stat -c%s "$LOG_FILE" 2>/dev/null || echo 0)
    
    if [ "$file_size" -gt "$MAX_SIZE" ]; then
        log_info "日志文件超过 10MB，开始滚动日志..."
        
        # 删除最旧的日志文件（.MAX_LOGS）
        local oldest_log="$LOG_FILE.$MAX_LOGS"
        if [ -f "$oldest_log" ]; then
            rm -f "$oldest_log"
        fi
        
        # 滚动日志文件：.4 -> .5, .3 -> .4, .2 -> .3, .1 -> .2
        for i in $(seq $((MAX_LOGS - 1)) -1 1); do
            local old_log="$LOG_FILE.$i"
            local new_log="$LOG_FILE.$((i + 1))"
            
            if [ -f "$old_log" ]; then
                mv "$old_log" "$new_log"
            fi
        done
        
        # 将当前日志移动到 .1
        mv "$LOG_FILE" "$LOG_FILE.1"
        
        # 创建新的日志文件
        touch "$LOG_FILE"
        
        log_info "日志滚动完成，保留最近 $MAX_LOGS 个日志文件"
    fi
}

# 清理函数（在退出时执行）
cleanup() {
    local exit_code=$?
    
    log_info "收到退出信号，开始清理..."
    
    # 清理锁文件
    if [ -f "$LOCK_FILE" ]; then
        if rm -f "$LOCK_FILE" 2>/dev/null; then
            log_info "已清理锁文件"
        else
            log_warn "无法删除锁文件: $LOCK_FILE (可能已被其他进程删除)"
        fi
    fi
    
    log_info "系统退出，退出码: $exit_code"
    log_info "========== 系统关闭 =========="
    
    exit $exit_code
}

# 清理旧的相机进程
cleanup_camera_processes() {
    log_info "检查旧相机进程..."
    
    # 检查是否有 realsense2_camera 进程
    local camera_pids=$(pgrep -f "realsense2_camera" 2>/dev/null || true)
    
    if [ -n "$camera_pids" ]; then
        log_warn "发现残留的相机进程: $camera_pids"
        log_info "正在清理旧相机进程..."
        
        # 使用项目提供的清理脚本
        if [ -f "$WORKSPACE_DIR/clean_camera_processes.py" ]; then
            python3 "$WORKSPACE_DIR/clean_camera_processes.py" >> "$LOG_FILE" 2>&1
            log_info "相机进程清理完成"
        else
            # 回退方案：使用 pkill
            pkill -9 -f "realsense2_camera" >> "$LOG_FILE" 2>&1 || true
            log_warn "使用了 pkill 清理相机进程（未找到清理脚本）"
        fi
        
        # 等待进程完全终止
        sleep 2
    else
        log_info "未发现残留的相机进程"
    fi
}

# 检查是否已有实例在运行
check_duplicate_instance() {
    # 检查锁文件是否存在
    if [ -f "$LOCK_FILE" ]; then
        local lock_pid=$(cat "$LOCK_FILE" 2>/dev/null || echo "")
        
        # 检查进程是否存在
        if [ -n "$lock_pid" ] && kill -0 "$lock_pid" 2>/dev/null; then
            log_error "系统已在运行中 (PID: $lock_pid)"
            log_error "如需强制启动，请先删除锁文件: rm -f $LOCK_FILE"
            exit 1
        else
            log_warn "发现过期的锁文件，正在清理..."
            
            # 尝试删除过期的锁文件
            if ! rm -f "$LOCK_FILE" 2>/dev/null; then
                log_error "无法删除锁文件: $LOCK_FILE"
                log_error "请手动删除: rm -f $LOCK_FILE"
                exit 1
            fi
            
            log_info "过期锁文件已清理"
        fi
    fi
    
    # 创建锁文件
    if ! echo $$ > "$LOCK_FILE" 2>/dev/null; then
        log_error "无法创建锁文件: $LOCK_FILE"
        log_error "请检查目录权限"
        exit 1
    fi
    
    log_info "已创建锁文件 (PID: $$)"
}

# 检查 ROS2 环境
check_ros2_environment() {
    log_info "检查 ROS2 环境..."
    
    # 检查 ROS2 是否安装
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        log_error "ROS2 Humble 未安装或未找到"
        exit 1
    fi
    
    # 检查工作空间是否存在
    if [ ! -d "$WORKSPACE_DIR" ]; then
        log_error "工作空间不存在: $WORKSPACE_DIR"
        exit 1
    fi
    
    # 检查工作空间是否已构建
    if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        log_error "工作空间未构建，请先运行: colcon build"
        exit 1
    fi
    
    log_info "ROS2 环境检查通过"
}

# 加载 ROS2 环境
load_ros2_environment() {
    log_info "加载 ROS2 环境..."
    
    # 加载系统环境
    source /opt/ros/humble/setup.bash
    
    # 加载工作空间环境
    cd "$WORKSPACE_DIR"
    source install/setup.bash
    
    log_info "ROS2 环境加载完成"
    log_info "工作空间: $WORKSPACE_DIR"
    log_info "ROS_DISTRO: $ROS_DISTRO"
}

# 启动系统
start_system() {
    log_info "========== 系统启动 =========="
    log_info "启动参数："
    log_info "  - 包名: $LAUNCH_PACKAGE"
    log_info "  - 启动文件: $LAUNCH_FILE"
    log_info "  - 日志文件: $LOG_FILE"
    log_info "  - 工作空间: $WORKSPACE_DIR"
    
    # 启动 ROS2 launch
    log_info "正在启动系统..."
    exec ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" >> "$LOG_FILE" 2>&1
}

# ============================================================
# 主流程
# ============================================================

# 设置信号处理
trap cleanup EXIT INT TERM

# 创建日志目录
create_log_dir

# 启动前检查并滚动日志
rotate_logs

# 等待 GUI 完全就绪
wait_for_gui() {
    log_info "等待 GUI 完全就绪..."
    
    # 检查 GUI 检测脚本是否存在
    if [ ! -f "$WORKSPACE_DIR/wait_for_gui.sh" ]; then
        log_warn "未找到 GUI 检测脚本，跳过 GUI 就绪检测"
        log_warn "建议检查: $WORKSPACE_DIR/wait_for_gui.sh"
        return 0
    fi
    
    # 执行 GUI 就绪检测（等待最多 60 秒）
    if ! bash "$WORKSPACE_DIR/wait_for_gui.sh" 60 >> "$LOG_FILE" 2>&1; then
        log_error "GUI 未在 60 秒内就绪"
        log_error "可能的原因："
        log_error "  1. 图形界面服务未启动"
        log_error "  2. 用户未登录桌面环境"
        log_error "  3. DISPLAY 或 Xauthority 配置错误"
        log_error ""
        log_error "建议检查："
        log_error "  - 系统日志: journalctl -xe"
        log_error "  - 显示管理器: systemctl status gdm3"
        log_error "  - X 日志: /var/log/Xorg.0.log"
        
        # 询问是否继续
        if [ -t 0 ]; then
            echo ""
            read -p "GUI 未就绪，是否仍要启动系统？(y/n): " -n 1 -r
            echo ""
            
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                log_info "用户取消启动"
                exit 1
            fi
            
            log_warn "用户强制启动系统（GUI 未就绪）"
        else
            # 非交互模式，直接退出
            exit 1
        fi
    fi
    
    log_info "GUI 已就绪，可以启动可视化服务"
}

# 检查 ROS2 环境
check_ros2_environment

# 加载 ROS2 环境
load_ros2_environment

# 等待 GUI 完全就绪
wait_for_gui

# 清理旧的相机进程
cleanup_camera_processes

# 检查重复实例
check_duplicate_instance

# 启动系统
start_system
