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
import pyrealsense2 as rs
import sys
import os


class CameraMonitor(Node):
    def __init__(self):
        super().__init__('camera_monitor')
        
        # Camera status tracking
        self.camera_process = None
        self.camera_running = False
        self.last_image_time = None
        self.last_camera_info_time = None
        
        # Parameters
        self.declare_parameter('timeout_seconds', 3.0)  # 秒数后认为相机无响应
        self.declare_parameter('reconnect_attempts', 5)  # 重连尝试次数
        self.declare_parameter('reconnect_delay', 2.0)  # 重连延迟（秒）
        
        self.timeout_seconds = self.get_parameter('timeout_seconds').get_parameter_value().double_value
        self.reconnect_attempts = self.get_parameter('reconnect_attempts').get_parameter_value().integer_value
        self.reconnect_delay = self.get_parameter('reconnect_delay').get_parameter_value().double_value
        
        # 连接尝试计数器
        self.connection_attempts = 0
        
        # Subscribe to camera topics to monitor data flow
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', 
            self.image_callback, 
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publisher for camera status
        self.camera_status_pub = self.create_publisher(Bool, 'camera_status', 10)
        
        # Timer for monitoring
        self.monitor_timer = self.create_timer(0.5, self.monitor_callback)  # 更频繁的监控
        
        # Timer for physical device check
        self.device_check_timer = self.create_timer(2.0, self.check_physical_device)  # 更频繁的物理检测
        
        self.get_logger().info('Camera monitor initialized')
    
    def image_callback(self, msg):
        """处理图像数据到达"""
        self.last_image_time = self.get_clock().now()
    
    def camera_info_callback(self, msg):
        """处理相机信息数据到达"""
        self.last_camera_info_time = self.get_clock().now()
    
    def check_physical_device(self):
        """检查物理设备连接状态"""
        try:
            # 使用 pyrealsense2 检查物理设备
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                # 没有检测到RealSense设备
                self.get_logger().warning('No RealSense camera detected physically!')
                # 发布离线状态
                status_msg = Bool()
                status_msg.data = False
                self.camera_status_pub.publish(status_msg)
                return False
            else:
                # 额外检查设备是否实际可访问
                device_connected = False
                for device in devices:
                    try:
                        # 尝试获取设备信息以确认设备真正可访问
                        device_name = device.get_info(rs.camera_info.name) if device else "Unknown"
                        serial_number = device.get_info(rs.camera_info.serial_number) if device else "Unknown"
                        self.get_logger().info(f'Detected RealSense Camera: {device_name}, Serial: {serial_number}')
                        device_connected = True
                        break  # 只需要找到一个可用的设备即可
                    except Exception as e:
                        self.get_logger().warning(f'Could not access device: {e}')
                        continue
                
                if device_connected:
                    self.get_logger().info('RealSense camera is physically connected and accessible')
                    # 发布在线状态
                    status_msg = Bool()
                    status_msg.data = True
                    self.camera_status_pub.publish(status_msg)
                    return True
                else:
                    self.get_logger().warning('No accessible RealSense camera found')
                    status_msg = Bool()
                    status_msg.data = False
                    self.camera_status_pub.publish(status_msg)
                    return False
        except ImportError:
            # 如果pyrealsense2不可用，使用lsusb检查
            try:
                result = subprocess.run(['lsusb'], capture_output=True, text=True)
                # 检查是否有Intel RealSense设备
                if 'Intel' in result.stdout and ('RealSense' in result.stdout or '8086:' in result.stdout):
                    status_msg = Bool()
                    status_msg.data = True
                    self.camera_status_pub.publish(status_msg)
                    return True
                else:
                    status_msg = Bool()
                    status_msg.data = False
                    self.camera_status_pub.publish(status_msg)
                    return False
            except Exception as e:
                self.get_logger().warning(f'Error checking USB devices: {e}')
                return False
        except Exception as e:
            self.get_logger().warning(f'Error checking physical device: {e}')
            status_msg = Bool()
            status_msg.data = False
            self.camera_status_pub.publish(status_msg)
            return False
    
    def monitor_callback(self):
        """监控回调函数"""
        current_time = self.get_clock().now()
        
        # 检查是否收到相机数据
        image_timeout = (self.last_image_time is None or 
                        (current_time - self.last_image_time).nanoseconds / 1e9 > self.timeout_seconds)
        
        info_timeout = (self.last_camera_info_time is None or 
                       (current_time - self.last_camera_info_time).nanoseconds / 1e9 > self.timeout_seconds)
        
        # 如果物理检测失败，也认为相机不在线
        physical_connected = self.check_physical_device()
        
        # 发布相机状态
        status_msg = Bool()
        status_msg.data = not (image_timeout or info_timeout) and physical_connected
        self.camera_status_pub.publish(status_msg)
        
        # 如果相机无响应，尝试重启
        if (image_timeout or info_timeout) and physical_connected:
            self.get_logger().warning('Camera appears to be unresponsive, checking process...')
            self.check_and_restart_camera()
        elif not physical_connected:
            self.get_logger().warning('Physical camera disconnected, cannot restart')
    
    def check_and_restart_camera(self):
        """检查并重启相机进程"""
        # 检查物理连接
        if not self.check_physical_device():
            self.get_logger().error('Physical camera not connected, cannot restart')
            return
        
        if self.camera_process and self.camera_process.poll() is not None:
            # 进程仍在运行但无数据，终止它
            self.get_logger().info('Terminating unresponsive camera process...')
            try:
                self.camera_process.terminate()
                try:
                    self.camera_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.camera_process.kill()
            except Exception as e:
                self.get_logger().error(f'Error killing camera process: {e}')
        
        # 重新启动相机
        success = self.start_camera_process()
        if success:
            self.connection_attempts = 0  # 重置连接尝试计数
        else:
            self.connection_attempts += 1
            if self.connection_attempts >= self.reconnect_attempts:
                self.get_logger().error(f'Failed to connect after {self.reconnect_attempts} attempts')
                self.connection_attempts = 0  # 重置计数器，继续尝试
    
    def start_camera_process(self):
        """启动相机进程"""
        try:
            cmd = ['ros2', 'run', 'realsense2_camera', 'realsense2_camera_node']
            self.camera_process = subprocess.Popen(
                cmd, 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL
            )
            
            # 等待进程启动
            time.sleep(3)  # 增加等待时间
            
            if self.camera_process.poll() is not None:
                self.get_logger().error(f'Camera process exited immediately with code {self.camera_process.poll()}')
                self.camera_running = False
                return False
            else:
                self.get_logger().info('Camera process restarted successfully')
                self.camera_running = True
                return True
                
        except FileNotFoundError:
            self.get_logger().error('ros2 executable not found. Ensure ROS2 is sourced.')
        except Exception as e:
            self.get_logger().error(f'Failed to start camera process: {e}')
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


if __name__ == '__main__':
    main()