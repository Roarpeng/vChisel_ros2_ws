#!/bin/bash

# ============================================================
# vChisel_ros2_ws 系统闲时重启脚本
# 功能：在凌晨3点检查系统是否处于闲时状态，如果是则重启系统
# 定义：闲时 = 上次相机关闭后超过5分钟没有接到下次相机开机命令
# ============================================================

# ============================================================
# 全局变量配置
# ============================================================
WORKSPACE_DIR="/home/bosch/vChisel_ros2_ws"
LOG_DIR="/home/bosch/logs"
LOG_FILE="$LOG_DIR/visual.log"
CAMERA_STATUS_FILE="/tmp/vchisel_camera_status.txt"
IDLE_THRESHOLD_MINUTES=5
TARGET_HOUR=3  # 凌晨3点

# ============================================================
# 辅助函数
# ============================================================

# 日志记录函数
log() {
    local level=$1
    shift
    local message="$@"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [$level] [idle_restart] $message" | tee -a "$LOG_FILE"
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
    mkdir -p "$LOG_DIR" 2>/dev/null
    touch "$LOG_FILE" 2>/dev/null
}

# 检查当前时间是否为凌晨3点
check_time() {
    local current_hour=$(date +%H)
    
    if [ "$current_hour" -ne "$TARGET_HOUR" ]; then
        log_info "当前时间不是凌晨3点（当前：$current_hour:00），跳过重启检查"
        exit 0
    fi
    
    log_info "当前时间为凌晨3点，开始检查闲时状态..."
}

# 检查系统是否在运行
check_system_running() {
    # 检查锁文件
    if [ ! -f "$WORKSPACE_DIR/.vchisel_system.lock" ]; then
        log_info "系统未运行（无锁文件），无需重启"
        exit 0
    fi
    
    local lock_pid=$(cat "$WORKSPACE_DIR/.vchisel_system.lock" 2>/dev/null || echo "")
    
    # 检查进程是否存在
    if [ -z "$lock_pid" ] || ! kill -0 "$lock_pid" 2>/dev/null; then
        log_info "系统未运行（进程不存在），无需重启"
        rm -f "$WORKSPACE_DIR/.vchisel_system.lock"
        exit 0
    fi
    
    log_info "系统正在运行 (PID: $lock_pid)"
}

# 获取相机状态
get_camera_status() {
    # 方法1：检查相机状态文件
    if [ -f "$CAMERA_STATUS_FILE" ]; then
        local status=$(cat "$CAMERA_STATUS_FILE" 2>/dev/null || echo "")
        local timestamp=$(stat -c %Y "$CAMERA_STATUS_FILE" 2>/dev/null || echo "0")
        local current_time=$(date +%s)
        local age_seconds=$((current_time - timestamp))
        
        log_info "相机状态文件：$status，文件年龄：${age_seconds}秒"
        
        # 如果文件是最近5分钟内更新的，使用文件内容
        if [ $age_seconds -lt 300 ]; then
            echo "$status"
            return
        fi
    fi
    
    # 方法2：检查 realsense2_camera 进程
    local camera_pids=$(pgrep -f "realsense2_camera" 2>/dev/null || echo "")
    
    if [ -n "$camera_pids" ]; then
        log_info "检测到相机进程运行中：$camera_pids"
        echo "on"
    else
        log_info "未检测到相机进程"
        echo "off"
    fi
}

# 获取相机最后关闭时间
get_camera_last_off_time() {
    local status_file="/tmp/vchisel_camera_off_time.txt"
    
    if [ -f "$status_file" ]; then
        local off_time=$(cat "$status_file" 2>/dev/null || echo "0")
        echo "$off_time"
    else
        echo "0"
    fi
}

# 记录相机关闭时间
record_camera_off_time() {
    local status_file="/tmp/vchisel_camera_off_time.txt"
    date +%s > "$status_file"
}

# 检查是否处于闲时
check_idle_state() {
    local camera_status=$(get_camera_status)
    
    log_info "当前相机状态：$camera_status"
    
    # 如果相机正在运行，不是闲时
    if [ "$camera_status" = "on" ]; then
        log_info "相机正在运行，不是闲时状态"
        exit 0
    fi
    
    # 获取相机最后关闭时间
    local last_off_time=$(get_camera_last_off_time)
    local current_time=$(date +%s)
    local idle_seconds=$((current_time - last_off_time))
    local idle_minutes=$((idle_seconds / 60))
    
    log_info "相机已关闭时间：${idle_minutes}分钟（${idle_seconds}秒）"
    
    # 如果闲时超过阈值，执行重启
    if [ $idle_minutes -ge $IDLE_THRESHOLD_MINUTES ]; then
        log_info "检测到闲时状态（已关闭${idle_minutes}分钟 ≥ ${IDLE_THRESHOLD_MINUTES}分钟），准备重启系统..."
        return 0
    else
        log_info "未达到闲时阈值（已关闭${idle_minutes}分钟 < ${IDLE_THRESHOLD_MINUTES}分钟），跳过重启"
        exit 0
    fi
}

# 停止项目进程
stop_project() {
    log_info "正在停止项目进程..."
    
    # 方法1：发送 SIGINT 信号给主进程
    local lock_pid=$(cat "$WORKSPACE_DIR/.vchisel_system.lock" 2>/dev/null || echo "")
    
    if [ -n "$lock_pid" ] && kill -0 "$lock_pid" 2>/dev/null; then
        log_info "发送 SIGINT 信号给主进程 (PID: $lock_pid)..."
        kill -INT "$lock_pid"
        
        # 等待进程退出
        local count=0
        while kill -0 "$lock_pid" 2>/dev/null && [ $count -lt 30 ]; do
            sleep 1
            count=$((count + 1))
        done
        
        if kill -0 "$lock_pid" 2>/dev/null; then
            log_warn "进程未在30秒内退出，强制终止..."
            kill -9 "$lock_pid" 2>/dev/null || true
        fi
    fi
    
    # 方法2：清理所有相关进程
    log_info "清理残留进程..."
    pkill -9 -f "ros2 launch snap_7" 2>/dev/null || true
    pkill -9 -f "norm_calc_server" 2>/dev/null || true
    pkill -9 -f "snap_7_node" 2>/dev/null || true
    pkill -9 -f "image_norm_viewer" 2>/dev/null || true
    pkill -9 -f "realsense2_camera" 2>/dev/null || true
    
    # 清理锁文件
    rm -f "$WORKSPACE_DIR/.vchisel_system.lock"
    
    log_info "项目进程已停止"
    sleep 3
}

# 清理临时文件
cleanup_temp_files() {
    log_info "清理临时文件..."
    rm -f "$CAMERA_STATUS_FILE"
    rm -f "/tmp/vchisel_camera_off_time.txt"
    log_info "临时文件已清理"
}

# 重启PC系统
reboot_pc() {
    log_info "=========================================="
    log_info "即将重启PC系统"
    log_info "=========================================="
    log_info "重启原因：闲时自动重启"
    log_info "触发时间：$(date '+%Y-%m-%d %H:%M:%S')"
    log_info "闲时条件：相机已关闭超过 ${IDLE_THRESHOLD_MINUTES} 分钟"
    log_info ""
    
    # 检查是否为交互式运行（测试模式）
    if [ -t 0 ] && [ "$FORCE_REBOOT" != "true" ]; then
        log_warn "检测到交互式运行模式"
        log_warn "此脚本设计为通过 cron 自动运行"
        log_warn ""
        log_warn "如果确定要重启PC，请设置环境变量："
        log_warn "  FORCE_REBOOT=true $0"
        log_warn ""
        log_warn "或者使用 sudo 运行（cron 任务方式）："
        log_warn "  sudo $0"
        log_warn ""
        log_info "跳过重启（安全保护）"
        exit 0
    fi
    
    # 同步文件系统
    log_info "同步文件系统..."
    sync
    
    # 等待日志写入完成
    sleep 2
    
    log_info "正在重启系统..."
    log_info "系统将在5秒后重启..."
    
    # 使用 reboot 命令重启（需要 root 权限）
    reboot
}

# ============================================================
# 主流程
# ============================================================

# 创建日志目录
create_log_dir

log_info "========== 闲时重启检查开始 =========="

# 检查时间
check_time

# 检查系统是否在运行
check_system_running

# 检查是否处于闲时
check_idle_state

# 停止项目进程
stop_project

# 清理临时文件
cleanup_temp_files

# 重启PC系统
reboot_pc

log_info "========== 闲时重启检查完成 =========="
