# 相机进程清理问题修复总结

## 问题描述

**症状**：即使 PLC 状态从 110 变为 130 或 0，`norm_calc_server` 仍在接收图像数据，日志显示：
```
[norm_calc_server-1] [INFO] [1768444898.163456568] [norm_calc]: [Data Flow] RGB Image Received
[norm_calc_server-1] [INFO] [1768444900.164556892] [norm_calc]: [Data Flow] RGB Image Received
```

## 根本原因分析

### 1. 多个相机进程同时运行

通过 `ps aux | grep realsense2_camera` 发现，系统中有 **3 个相机进程**在同时运行：
- PID 21750: 08:34 启动，运行了 8 分 31 秒
- PID 65946: 08:43 启动，运行了 7 分 51 秒
- PID 501748: 10:47 启动，运行了 21 秒

这些进程都在发布图像到相同的话题 `/camera/camera/color/image_raw`。

### 2. 进程清理不彻底

原始 `cam_shutdown()` 函数只终止了 `self.cam_proc`（当前保存的进程引用），但没有清理其他残留的相机进程：
```python
def cam_shutdown(self):
    if self.cam_proc and self.cam_proc.poll() is None:
        self.cam_proc.terminate()
        # ...
```

这导致：
- 第一次启动相机时，`self.cam_proc` 指向 PID 21750
- 第二次启动相机时，`self.cam_proc` 指向 PID 65946，但 PID 21750 仍在运行
- 第三次启动相机时，`self.cam_proc` 指向 PID 501748，但 PID 21750 和 65946 仍在运行

### 3. 子进程问题

`realsense2_camera_node` 会启动子进程，简单的 `pkill -f realsense2_camera` 可能无法清理所有子进程。

### 4. QoS 缓存问题

相机节点使用 `TRANSIENT_LOCAL` 持久性策略，这意味着即使发布者断开，订阅者仍可能收到缓存的数据。

## 解决方案

### 1. 增强进程清理逻辑

修改 `cam_shutdown()` 函数，使用多种方法彻底清理所有相机进程：

```python
def cam_shutdown(self):
    # 1. 优雅终止存储的进程引用
    if self.cam_proc and self.cam_proc.poll() is None:
        self.cam_proc.terminate()
        try:
            self.cam_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.cam_proc.kill()

    # 2. 使用多种方法强制终止所有相机进程
    # 方法1: pkill -9 清理所有 realsense2_camera 进程
    subprocess.run(['pkill', '-9', '-f', 'realsense2_camera'], ...)

    # 方法2: pkill -9 清理 realsense2_camera_node 进程
    subprocess.run(['pkill', '-9', '-f', 'realsense2_camera_node'], ...)

    # 方法3: killall 作为最后手段
    subprocess.run(['killall', '-9', 'realsense2_camera_node'], ...)

    # 3. 验证清理结果
    result = subprocess.run(['pgrep', '-f', 'realsense2_camera'], ...)
    if result.returncode == 0:
        self.get_logger().warning(f'Warning: {len(remaining)} camera processes still running')

    # 4. 重置状态
    self.camStatus = False
    # ...
```

### 2. 改进相机启动前清理

在 `cam_bringup()` 函数中，使用相同的多方法清理逻辑：
```python
def cam_bringup(self):
    # 在启动新相机之前，彻底清理所有旧相机进程
    subprocess.run(['pkill', '-9', '-f', 'realsense2_camera'], ...)
    subprocess.run(['pkill', '-9', '-f', 'realsense2_camera_node'], ...)
    subprocess.run(['killall', '-9', 'realsense2_camera_node'], ...)

    # 验证清理结果
    result = subprocess.run(['pgrep', '-f', 'realsense2_camera'], ...)
    # ...
```

### 3. 增加等待时间

在清理命令之间增加适当的等待时间（0.5-1秒），确保进程完全终止。

### 4. 添加验证日志

在清理后使用 `pgrep` 验证是否还有残留进程，并记录警告信息。

## 修改文件

- `src/snap_7/snap_7/plc_client_node.py`
  - `cam_shutdown()` 函数：增强清理逻辑
  - `cam_bringup()` 函数：改进启动前清理

## 测试工具

创建了以下测试工具来验证修复效果：

1. **`clean_camera_processes.py`**：彻底清理所有相机进程的脚本
2. **`verify_camera_cleanup.py`**：验证相机进程清理功能的测试脚本
3. **`test_camera_cleanup.sh`**：交互式测试脚本

## 验证步骤

1. **清理所有残留进程**：
   ```bash
   python3 clean_camera_processes.py
   ```

2. **启动系统**：
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch snap_7 snap_7.launch.py
   ```

3. **检查相机进程**：
   ```bash
   ps aux | grep realsense2_camera | grep -v grep
   ```
   应该只有 1 个进程

4. **触发相机关闭**（通过 PLC 或手动）：
   ```bash
   ros2 service call /norm_calc norm_calc/srv/NormCalcData '{seq: 0}'
   ```

5. **验证清理结果**：
   ```bash
   ps aux | grep realsense2_camera | grep -v grep
   ```
   应该没有进程

6. **检查图像话题发布者**：
   ```bash
   ros2 topic info /camera/camera/color/image_raw --verbose
   ```
   Publisher count 应该为 0

## 预期效果

修复后，当 PLC 状态从 110 变为 130 或 0 时：
1. 所有相机进程（包括子进程）被彻底终止
2. `norm_calc_server` 不再接收图像数据
3. 日志中不再出现 `[Data Flow] RGB Image Received` 消息
4. 相机进程数量始终为 0 或 1（运行时）

## 注意事项

1. **权限问题**：确保运行 `plc_client_node` 的用户有足够的权限终止进程
2. **进程竞争**：在快速启动/关闭循环中，可能需要增加等待时间
3. **QoS 策略**：如果问题仍然存在，可能需要调整 QoS 策略（从 `TRANSIENT_LOCAL` 改为 `VOLATILE`）
4. **监控残留**：定期检查是否有残留进程，确保系统稳定运行

## 后续改进建议

1. **进程管理**：考虑使用进程组（process group）来管理所有相机进程
2. **状态同步**：添加相机状态话题，让 `norm_calc_server` 能够知道相机是否真的在运行
3. **超时机制**：增加进程清理超时机制，防止无限等待
4. **错误恢复**：添加自动清理机制，定期检查并清理残留进程
5. **日志优化**：减少清理日志的详细程度，避免日志泛滥

## 版本信息

- 修复日期：2026-01-15
- 分支：visual_base
- 提交：待提交