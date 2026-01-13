#!/usr/bin/env python3
"""
Camera monitoring script for industrial reliability
Monitors RealSense camera status and handles reconnection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import subprocess
import threading
import time
from std_msgs.msg import Bool

# 注意：不再使用 pyrealsense2 检测设备，改用 lsusb 避免资源竞争
import sys
import os


class CameraMonitor(Node):
    def __init__(self):
        super().__init__("camera_monitor")

        # Camera status tracking
        self.camera_process = None
        self.camera_running = False
        self.last_image_time = None
        self.last_camera_info_time = None

        # Parameters
        self.declare_parameter("timeout_seconds", 3.0)  # 秒数后认为相机无响应
        self.declare_parameter("reconnect_attempts", 5)  # 重连尝试次数
        self.declare_parameter("reconnect_delay", 2.0)  # 重连延迟（秒）

        self.timeout_seconds = (
            self.get_parameter("timeout_seconds").get_parameter_value().double_value
        )
        self.reconnect_attempts = (
            self.get_parameter("reconnect_attempts").get_parameter_value().integer_value
        )
        self.reconnect_delay = (
            self.get_parameter("reconnect_delay").get_parameter_value().double_value
        )

        # 连接尝试计数器
        self.connection_attempts = 0

        # 物理设备连接状态缓存（避免频繁检测导致资源竞争）
        self.physical_device_connected = False

        # Subscribe to camera topics to monitor data flow
        self.image_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/depth/camera_info",
            self.camera_info_callback,
            10,
        )

        # Publisher for camera status
        self.camera_status_pub = self.create_publisher(Bool, "camera_status", 10)

        # Timer for monitoring
        self.monitor_timer = self.create_timer(
            0.5, self.monitor_callback
        )  # 更频繁的监控

        # Timer for physical device check
        self.device_check_timer = self.create_timer(
            2.0, self.check_physical_device
        )  # 更频繁的物理检测

        self.get_logger().info("Camera monitor initialized")

    def image_callback(self, msg):
        """处理图像数据到达"""
        self.last_image_time = self.get_clock().now()

    def camera_info_callback(self, msg):
        """处理相机信息数据到达"""
        self.last_camera_info_time = self.get_clock().now()

    def check_physical_device(self):
        """检查物理设备连接状态 - 使用轻量级方法避免资源竞争"""
        try:
            # 使用 lsusb 检查 RealSense 设备（不会锁定设备）
            # Intel RealSense 的 USB Vendor ID 是 8086
            result = subprocess.run(
                ["lsusb", "-d", "8086:"],  # 只检查 Intel 设备
                capture_output=True,
                text=True,
                timeout=2,
            )

            # 检查是否有 RealSense 设备
            if result.returncode == 0 and result.stdout.strip():
                # 找到了 Intel 设备，进一步检查是否是 RealSense
                # D435的Product ID是 0B3A
                if (
                    "0b3a" in result.stdout.lower()
                    or "realsense" in result.stdout.lower()
                ):
                    self.physical_device_connected = True
                    status_msg = Bool()
                    status_msg.data = True
                    self.camera_status_pub.publish(status_msg)
                    return True

            # 没有检测到 RealSense 设备
            self.get_logger().warning("No RealSense camera detected via USB!")
            self.physical_device_connected = False
            status_msg = Bool()
            status_msg.data = False
            self.camera_status_pub.publish(status_msg)
            return False

        except subprocess.TimeoutExpired:
            self.get_logger().warning("USB device check timed out")
            return self.physical_device_connected  # 返回上次的状态
        except Exception as e:
            self.get_logger().warning(f"Error checking physical device: {e}")
            return self.physical_device_connected  # 返回上次的状态

    def monitor_callback(self):
        """监控回调函数"""
        current_time = self.get_clock().now()

        # 检查是否收到相机数据
        image_timeout = (
            self.last_image_time is None
            or (current_time - self.last_image_time).nanoseconds / 1e9
            > self.timeout_seconds
        )

        info_timeout = (
            self.last_camera_info_time is None
            or (current_time - self.last_camera_info_time).nanoseconds / 1e9
            > self.timeout_seconds
        )

        # 使用缓存的物理连接状态，避免频繁调用 pyrealsense2 导致资源竞争
        # 物理状态由 device_check_timer 每2秒独立检测一次
        physical_connected = self.physical_device_connected

        # 发布相机状态
        status_msg = Bool()
        status_msg.data = not (image_timeout or info_timeout) and physical_connected
        self.camera_status_pub.publish(status_msg)

        # 如果相机无响应，尝试重启
        if (image_timeout or info_timeout) and physical_connected:
            self.get_logger().warning(
                "Camera appears to be unresponsive, checking process..."
            )
            self.check_and_restart_camera()
        elif not physical_connected:
            self.get_logger().warning("Physical camera disconnected, cannot restart")

    def check_and_restart_camera(self):
        """检查并重启相机进程"""
        # 检查物理连接
        if not self.check_physical_device():
            self.get_logger().error("Physical camera not connected, cannot restart")
            return

        if self.camera_process and self.camera_process.poll() is not None:
            # 进程仍在运行但无数据，终止它
            self.get_logger().info("Terminating unresponsive camera process...")
            try:
                self.camera_process.terminate()
                try:
                    self.camera_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.camera_process.kill()
            except Exception as e:
                self.get_logger().error(f"Error killing camera process: {e}")

        # 重新启动相机
        success = self.start_camera_process()
        if success:
            self.connection_attempts = 0  # 重置连接尝试计数
        else:
            self.connection_attempts += 1
            if self.connection_attempts >= self.reconnect_attempts:
                self.get_logger().error(
                    f"Failed to connect after {self.reconnect_attempts} attempts"
                )
                self.connection_attempts = 0  # 重置计数器，继续尝试

    def start_camera_process(self):
        """启动相机进程"""
        try:
            # [修改] 使用自定义 SDK 节点替代官方 ROS 包
            # 路径假定脚本在同一目录下，或者已安装到 lib/snap_7/
            # 在 ROS2 colcon build 后，python 脚本通常安装在 install/snap_7/lib/snap_7/

            # 构造命令： ros2 run snap_7 custom_realsense_node.py
            # 注意：需要在 setup.py 中注册 entry_point
            cmd = ["ros2", "run", "snap_7", "custom_realsense_node"]

            self.camera_process = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

            # 等待进程启动
            time.sleep(3)  # 增加等待时间

            if self.camera_process.poll() is not None:
                self.get_logger().error(
                    f"Camera process exited immediately with code {self.camera_process.poll()}"
                )
                self.camera_running = False
                return False
            else:
                self.get_logger().info("Camera process restarted successfully")
                self.camera_running = True
                return True

        except FileNotFoundError:
            self.get_logger().error(
                "ros2 executable not found. Ensure ROS2 is sourced."
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start camera process: {e}")
            self.camera_running = False
        return False


def main(args=None):
    rclpy.init(args=args)
    node = CameraMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理相机进程
        if node.camera_process and node.camera_process.poll() is None:
            node.camera_process.terminate()
            try:
                node.camera_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                node.camera_process.kill()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
