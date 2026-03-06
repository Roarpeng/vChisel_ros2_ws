#!/bin/bash

# ============================================================
# GUI 就绪检测脚本
# 功能：等待系统完全进入GUI界面，确保可视化功能正常工作
# ============================================================

# 默认参数
MAX_WAIT=${1:-60}  # 最大等待时间（秒），默认60秒
CHECK_INTERVAL=2   # 检查间隔（秒）

# 颜色输出（如果支持）
if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    NC='\033[0m' # No Color
else
    RED=''
    GREEN=''
    YELLOW=''
    NC=''
fi

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检测 DISPLAY 环境变量
check_display() {
    if [ -z "$DISPLAY" ]; then
        # 尝试从常见位置获取 DISPLAY
        if [ -f /tmp/.X11-unix/* ]; then
            export DISPLAY=:0
            log_info "检测到 DISPLAY=:0"
            return 0
        else
            return 1
        fi
    fi
    return 0
}

# 检测 Xauthority 文件
check_xauthority() {
    local xauth_file="$HOME/.Xauthority"
    
    if [ ! -f "$xauth_file" ]; then
        log_warn "Xauthority 文件不存在: $xauth_file"
        return 1
    fi
    
    if [ ! -r "$xauth_file" ]; then
        log_warn "Xauthority 文件不可读: $xauth_file"
        return 1
    fi
    
    return 0
}

# 检测桌面环境进程
check_desktop_process() {
    # 常见桌面环境进程
    local desktop_processes=(
        "gnome-shell"      # GNOME
        "unity"            # Unity
        "kwin"             # KDE
        "xfwm4"            # XFCE
        "mate-session"     # MATE
        "lxsession"        # LXDE
        "deepin-wm"        # Deepin
        "enlightenment"    # Enlightenment
    )
    
    for proc in "${desktop_processes[@]}"; do
        if pgrep -x "$proc" > /dev/null 2>&1; then
            log_info "检测到桌面环境进程: $proc"
            return 0
        fi
    done
    
    # 如果没有检测到已知的桌面进程，检查是否有 Xorg 或 Wayland
    if pgrep -x "Xorg" > /dev/null 2>&1 || pgrep -x "Xwayland" > /dev/null 2>&1; then
        log_info "检测到 X 显示服务器运行中"
        return 0
    fi
    
    return 1
}

# 检测用户会话是否活跃
check_user_session() {
    # 检查是否有活跃的用户会话
    if command -v loginctl &> /dev/null; then
        local sessions=$(loginctl list-sessions --no-legend 2>/dev/null | grep "$USER" | wc -l)
        if [ "$sessions" -gt 0 ]; then
            log_info "检测到 $sessions 个活跃的用户会话"
            return 0
        fi
    fi
    
    # 如果没有 loginctl，使用 who 命令
    if command -v who &> /dev/null; then
        local user_sessions=$(who | grep "$USER" | grep -E "(:[0-9]+|tty)" | wc -l)
        if [ "$user_sessions" -gt 0 ]; then
            log_info "检测到 $user_sessions 个用户登录会话"
            return 0
        fi
    fi
    
    return 1
}

# 检测 GUI 是否可以正常显示（通过 xdpyinfo）
check_gui_responsive() {
    if ! command -v xdpyinfo &> /dev/null; then
        log_warn "xdpyinfo 未安装，跳过响应测试"
        return 0
    fi
    
    # 尝试连接 X 显示
    if DISPLAY=:0 xdpyinfo > /dev/null 2>&1; then
        log_info "X 显示服务器响应正常"
        return 0
    else
        return 1
    fi
}

# 主检测函数
check_gui_ready() {
    local checks=0
    local passed=0
    
    log_info "开始检测 GUI 就绪状态..."
    
    # 检查 1: DISPLAY 环境变量
    ((checks++))
    if check_display; then
        ((passed++))
        log_info "✓ DISPLAY 环境变量正常"
    else
        log_warn "✗ DISPLAY 环境变量未设置"
    fi
    
    # 检查 2: Xauthority 文件
    ((checks++))
    if check_xauthority; then
        ((passed++))
        log_info "✓ Xauthority 文件正常"
    else
        log_warn "✗ Xauthority 文件异常"
    fi
    
    # 检查 3: 桌面环境进程
    ((checks++))
    if check_desktop_process; then
        ((passed++))
        log_info "✓ 桌面环境进程运行中"
    else
        log_warn "✗ 未检测到桌面环境进程"
    fi
    
    # 检查 4: 用户会话
    ((checks++))
    if check_user_session; then
        ((passed++))
        log_info "✓ 用户会话活跃"
    else
        log_warn "✗ 未检测到活跃用户会话"
    fi
    
    # 检查 5: GUI 响应
    ((checks++))
    if check_gui_responsive; then
        ((passed++))
        log_info "✓ GUI 响应正常"
    else
        log_warn "✗ GUI 无响应"
    fi
    
    log_info "检测结果: $passed/$checks 项通过"
    
    # 至少需要通过 3 项才算就绪
    if [ "$passed" -ge 3 ]; then
        return 0
    else
        return 1
    fi
}

# 等待 GUI 就绪
wait_for_gui() {
    local waited=0
    
    log_info "等待 GUI 完全就绪（最多等待 ${MAX_WAIT} 秒）..."
    log_info "检查间隔: ${CHECK_INTERVAL} 秒"
    echo ""
    
    while [ "$waited" -lt "$MAX_WAIT" ]; do
        if check_gui_ready; then
            echo ""
            log_info "=========================================="
            log_info "GUI 已完全就绪！"
            log_info "=========================================="
            log_info "等待时间: ${waited} 秒"
            log_info "DISPLAY: ${DISPLAY:-:0}"
            log_info "XAUTHORITY: ${XAUTHORITY:-$HOME/.Xauthority}"
            log_info "用户: $USER"
            log_info "桌面环境已就绪，可以启动可视化服务"
            return 0
        fi
        
        echo ""
        log_warn "GUI 尚未就绪，继续等待... (${waited}/${MAX_WAIT} 秒)"
        sleep "$CHECK_INTERVAL"
        ((waited += CHECK_INTERVAL))
    done
    
    echo ""
    log_error "=========================================="
    log_error "等待超时：GUI 未在 ${MAX_WAIT} 秒内就绪"
    log_error "=========================================="
    log_error "可能的原因："
    log_error "  1. 图形界面服务未启动"
    log_error "  2. 用户未登录桌面环境"
    log_error "  3. DISPLAY 或 Xauthority 配置错误"
    log_error "  4. 系统性能问题导致启动缓慢"
    echo ""
    log_error "建议检查："
    log_error "  - 系统日志: journalctl -xe"
    log_error "  - 显示管理器状态: systemctl status gdm3 或 systemctl status lightdm"
    log_error "  - X 日志: /var/log/Xorg.0.log"
    
    return 1
}

# ============================================================
# 主流程
# ============================================================

# 检查参数
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    echo "用法: $0 [最大等待秒数]"
    echo ""
    echo "功能：等待系统 GUI 完全就绪"
    echo ""
    echo "参数："
    echo "  最大等待秒数  可选，默认 60 秒"
    echo ""
    echo "示例："
    echo "  $0           # 等待最多 60 秒"
    echo "  $0 120       # 等待最多 120 秒"
    echo ""
    echo "退出码："
    echo "  0 - GUI 已就绪"
    echo "  1 - GUI 未就绪或超时"
    exit 0
fi

# 执行等待
if wait_for_gui; then
    exit 0
else
    exit 1
fi
