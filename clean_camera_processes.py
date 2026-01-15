#!/usr/bin/env python3
"""
彻底清理所有相机进程（包括子进程）
"""
import subprocess
import time
import signal
import os

def get_all_camera_processes():
    """获取所有相机相关进程（包括子进程）"""
    try:
        # 使用 pgrep 获取所有 realsense2_camera 相关进程的 PID
        result = subprocess.run(['pgrep', '-f', 'realsense2_camera'],
                              capture_output=True, text=True)
        if result.returncode == 0:
            pids = [int(pid) for pid in result.stdout.strip().split('\n')]
            return pids
        return []
    except Exception as e:
        print(f"Error getting PIDs: {e}")
        return []

def kill_process_tree(pid):
    """终止进程树（包括所有子进程）"""
    try:
        # 获取进程的所有子进程
        result = subprocess.run(['pstree', '-p', str(pid)],
                              capture_output=True, text=True)
        if result.returncode == 0:
            # 从 pstree 输出中提取所有 PID
            import re
            pids = re.findall(r'\((\d+)\)', result.stdout)
            for child_pid in reversed(pids):  # 从子进程开始终止
                try:
                    os.kill(int(child_pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass

        # 最后终止主进程
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return True
    except Exception as e:
        print(f"Error killing process tree: {e}")
        return False

def forceful_cleanup():
    """强制清理所有相机进程"""
    print("开始强制清理所有相机进程...")

    # 方法1: 使用 pkill -9
    print("1. 使用 pkill -9 清理...")
    try:
        subprocess.run(['pkill', '-9', '-f', 'realsense2_camera'],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)
    except Exception as e:
        print(f"   pkill 失败: {e}")

    # 方法2: 逐个终止进程树
    print("2. 检查残留进程...")
    pids = get_all_camera_processes()
    if pids:
        print(f"   发现 {len(pids)} 个残留进程: {pids}")
        for pid in pids:
            print(f"   终止进程树 {pid}...")
            kill_process_tree(pid)
        time.sleep(1)

    # 方法3: 再次检查
    print("3. 最终验证...")
    remaining = get_all_camera_processes()
    if remaining:
        print(f"   ✗ 仍有 {len(remaining)} 个残留进程: {remaining}")
        return False
    else:
        print("   ✓ 所有相机进程已清理")
        return True

if __name__ == '__main__':
    success = forceful_cleanup()
    exit(0 if success else 1)