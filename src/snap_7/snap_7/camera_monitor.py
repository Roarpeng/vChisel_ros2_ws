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


class CameraMonitor(Node):
    def __init__(self):
        super().__init__('camera_monitor')
        
        # Camera status tracking
        self.camera_process = None
        self.camera_running = False
        self.last_image_time = None
        self.last_camera_info_time = None
        
        # Parameters
        self.declare_parameter('timeout_seconds', 5.0)  # 秒数后认为相机无响应
        self.declare_parameter('reconnect_attempts', 3)  # 重连尝试次数
        self.declare_parameter('reconnect_delay', 2.0)  # 重连延迟（秒）
        
        self.timeout_seconds = self.get_parameter('timeout_seconds').get_parameter_value().double_value
        self.reconnect_attempts = self.get_parameter('reconnect_attempts').get_parameter_value().integer_value
        self.reconnect_delay = self.get_parameter('reconnect_delay').get_parameter_value().double_value
        
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
        self.monitor_timer = self.create_timer(1.0, self.monitor_callback)
        
        self.get_logger().info('Camera monitor initialized')
    
    def image_callback(self, msg):
        """处理图像数据到达"""
        self.last_image_time = self.get_clock().now()
    
    def camera_info_callback(self, msg):
        """处理相机信息数据到达"""
        self.last_camera_info_time = self.get_clock().now()
    
    def monitor_callback(self):
        """监控回调函数"""
        current_time = self.get_clock().now()
        
        # 检查是否收到相机数据
        image_timeout = (self.last_image_time is None or 
                        (current_time - self.last_image_time).nanoseconds / 1e9 > self.timeout_seconds)
        
        info_timeout = (self.last_camera_info_time is None or 
                       (current_time - self.last_camera_info_time).nanoseconds / 1e9 > self.timeout_seconds)
        
        # 发布相机状态
        status_msg = Bool()
        status_msg.data = not (image_timeout or info_timeout)
        self.camera_status_pub.publish(status_msg)
        
        # 如果相机无响应，尝试重启
        if image_timeout or info_timeout:
            self.get_logger().warning('Camera appears to be unresponsive, checking process...')
            self.check_and_restart_camera()
    
    def check_and_restart_camera(self):
        """检查并重启相机进程"""
        if self.camera_process and self.camera_process.poll() is None:
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
        self.start_camera_process()
    
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
            time.sleep(2)
            
            if self.camera_process.poll() is not None:
                self.get_logger().error(f'Camera process exited immediately with code {self.camera_process.poll()}')
                self.camera_running = False
            else:
                self.get_logger().info('Camera process restarted successfully')
                self.camera_running = True
                
        except FileNotFoundError:
            self.get_logger().error('ros2 executable not found. Ensure ROS2 is sourced.')
        except Exception as e:
            self.get_logger().error(f'Failed to start camera process: {e}')
            self.camera_running = False


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