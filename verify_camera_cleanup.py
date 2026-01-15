#!/usr/bin/env python3
"""
验证相机进程清理功能的测试脚本
"""
import subprocess
import time
import sys

def check_camera_processes():
    """检查当前运行的相机进程"""
    try:
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
        lines = result.stdout.split('\n')
        camera_processes = [line for line in lines if 'realsense2_camera' in line and 'grep' not in line]
        return camera_processes
    except Exception as e:
        print(f"Error checking processes: {e}")
        return []

def kill_all_camera_processes():
    """强制终止所有相机进程"""
    try:
        subprocess.run(['pkill', '-9', '-f', 'realsense2_camera'],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)
        return True
    except Exception as e:
        print(f"Error killing processes: {e}")
        return False

def main():
    print("=" * 60)
    print("相机进程清理验证脚本")
    print("=" * 60)
    print()

    # 1. 检查初始状态
    print("1. 检查初始状态...")
    processes = check_camera_processes()
    if processes:
        print(f"   发现 {len(processes)} 个相机进程:")
        for proc in processes:
            print(f"   - {proc[:100]}...")
    else:
        print("   ✓ 没有相机进程在运行")
    print()

    # 2. 清理所有进程
    print("2. 清理所有相机进程...")
    if kill_all_camera_processes():
        print("   ✓ 清理命令已执行")
    else:
        print("   ✗ 清理命令执行失败")
    print()

    # 3. 验证清理结果
    print("3. 验证清理结果...")
    time.sleep(1)
    remaining = check_camera_processes()
    if remaining:
        print(f"   ✗ 仍有 {len(remaining)} 个残留进程:")
        for proc in remaining:
            print(f"   - {proc[:100]}...")
        return False
    else:
        print("   ✓ 所有相机进程已清理")
    print()

    # 4. 启动一个测试相机进程
    print("4. 启动测试相机进程（5秒后自动关闭）...")
    try:
        # 启动一个临时的相机进程
        cmd = ['ros2', 'run', 'realsense2_camera', 'realsense2_camera_node',
               '--ros-args', '-p', 'enable_color:=false', '-p', 'enable_depth:=false']
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)

        # 检查进程是否启动
        processes = check_camera_processes()
        if processes:
            print(f"   ✓ 测试进程已启动 (PID: {proc.pid})")
        else:
            print("   ✗ 测试进程启动失败")
            return False

        # 5. 终止测试进程
        print()
        print("5. 终止测试进程...")
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        time.sleep(1)

        # 6. 验证进程是否完全终止
        print("6. 验证进程是否完全终止...")
        remaining = check_camera_processes()
        if remaining:
            print(f"   ✗ 仍有 {len(remaining)} 个残留进程:")
            for proc in remaining:
                print(f"   - {proc[:100]}...")
            return False
        else:
            print("   ✓ 进程已完全终止")

    except FileNotFoundError:
        print("   ⚠ 跳过测试（ros2 未找到或相机不可用）")
    except Exception as e:
        print(f"   ⚠ 测试过程中出错: {e}")
    print()

    # 7. 最终清理
    print("7. 最终清理...")
    kill_all_camera_processes()
    print("   ✓ 完成")
    print()

    print("=" * 60)
    print("验证完成")
    print("=" * 60)
    return True

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)