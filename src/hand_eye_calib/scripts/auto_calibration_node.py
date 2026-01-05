#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动化手眼标定节点
Auto Hand-Eye Calibration Node

功能：
1. 调用 /get_robot_pose 服务获取机器人位姿
2. 订阅 /aruco_single/pose 获取标定板位姿
3. 调用手眼标定服务计算变换矩阵
4. 提供服务接口用于UI/PLC触发
5. 发布预览图像（带ArUco检测结果）
6. 与PLC状态机交互

状态码（与PLC交互）：
- 300: 启动手眼标定模式
- 310: 采集点位（机器人已到位）
- 320: 计算标定矩阵
- 330: 保存标定结果
- 340: 退出标定
"""

import os
import sys
import time
import copy
import yaml
import numpy as np
from datetime import datetime
from pathlib import Path

import cv2

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from hand_eye_calib.srv import HandEyeCalibData
from norm_calc.srv import GetRobotPose


class AutoCalibrationNode(Node):
    def __init__(self):
        super().__init__('auto_calibration_node')

        # 回调组（允许嵌套服务调用）
        self.callback_group = ReentrantCallbackGroup()

        # 参数
        self.declare_parameter('min_points', 8)
        self.declare_parameter('max_points', 20)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('aruco_topic', '/aruco_single/pose')
        self.declare_parameter('preview_topic', '/calib/handeye/preview_image')
        self.declare_parameter('calib_dir', os.path.expanduser('~/.vchisel/calibrations'))

        # 获取参数
        self.min_points = self.get_parameter('min_points').get_parameter_value().integer_value
        self.max_points = self.get_parameter('max_points').get_parameter_value().integer_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.aruco_topic = self.get_parameter('aruco_topic').get_parameter_value().string_value
        self.preview_topic = self.get_parameter('preview_topic').get_parameter_value().string_value
        self.calib_dir = self.get_parameter('calib_dir').get_parameter_value().string_value

        # 确保目录存在
        Path(self.calib_dir).mkdir(parents=True, exist_ok=True)
        Path(os.path.join(self.calib_dir, 'calib_images')).mkdir(parents=True, exist_ok=True)

        # CV Bridge
        self.bridge = CvBridge()

        # 标定状态
        self.calib_active = False
        self.camera_poses = PoseArray()  # ArUco标定板位姿列表
        self.robot_poses = PoseArray()   # 机器人位姿列表
        self.captured_images = []
        self.current_frame = None
        self.current_aruco_pose = None
        self.aruco_detected = False
        self.last_aruco_time = None

        # 标定结果
        self.hand_eye_transform = None
        self.reprojection_error = None
        self.quality_score = 0

        # 订阅图像话题
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # 订阅ArUco位姿话题
        self.aruco_sub = self.create_subscription(
            PoseStamped,
            self.aruco_topic,
            self.aruco_callback,
            10
        )

        # 发布预览图像
        self.preview_pub = self.create_publisher(
            Image,
            self.preview_topic,
            10
        )

        # 发布压缩预览图像
        self.preview_compressed_pub = self.create_publisher(
            CompressedImage,
            f'{self.preview_topic}/compressed',
            10
        )

        # 服务客户端：获取机器人位姿
        self.robot_pose_client = self.create_client(
            GetRobotPose,
            'get_robot_pose',
            callback_group=self.callback_group
        )

        # 服务客户端：手眼标定计算
        self.handeye_client = self.create_client(
            HandEyeCalibData,
            'hand_eye_calib',
            callback_group=self.callback_group
        )

        # 服务：启动标定
        self.start_srv = self.create_service(
            Trigger,
            'calib/handeye/start',
            self.handle_start_calibration,
            callback_group=self.callback_group
        )

        # 服务：采集点位
        self.capture_srv = self.create_service(
            Trigger,
            'calib/handeye/capture',
            self.handle_capture_point,
            callback_group=self.callback_group
        )

        # 服务：计算标定
        self.compute_srv = self.create_service(
            Trigger,
            'calib/handeye/compute',
            self.handle_compute_calibration,
            callback_group=self.callback_group
        )

        # 服务：保存结果
        self.save_srv = self.create_service(
            Trigger,
            'calib/handeye/save',
            self.handle_save_calibration,
            callback_group=self.callback_group
        )

        # 服务：停止标定
        self.stop_srv = self.create_service(
            Trigger,
            'calib/handeye/stop',
            self.handle_stop_calibration,
            callback_group=self.callback_group
        )

        # 服务：获取状态
        self.status_srv = self.create_service(
            Trigger,
            'calib/handeye/status',
            self.handle_get_status,
            callback_group=self.callback_group
        )

        # 定时器：发布预览图像
        self.preview_timer = self.create_timer(0.1, self.publish_preview)

        # 定时器：检查ArUco检测状态
        self.aruco_check_timer = self.create_timer(0.5, self.check_aruco_status)

        self.get_logger().info('Auto calibration node started')
        self.get_logger().info(f'Min points: {self.min_points}, Max points: {self.max_points}')
        self.get_logger().info(f'Calibration directory: {self.calib_dir}')

    def image_callback(self, msg):
        """接收相机图像"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def aruco_callback(self, msg):
        """接收ArUco标定板位姿"""
        self.current_aruco_pose = msg.pose
        self.aruco_detected = True
        self.last_aruco_time = time.time()

    def check_aruco_status(self):
        """检查ArUco检测状态（超时判断）"""
        if self.last_aruco_time is not None:
            if time.time() - self.last_aruco_time > 0.5:
                self.aruco_detected = False

    def draw_status_overlay(self, image):
        """在图像上绘制状态信息"""
        result_image = image.copy()

        # 状态颜色
        if self.aruco_detected:
            status_text = "ArUco DETECTED"
            status_color = (0, 255, 0)
        else:
            status_text = "ArUco NOT DETECTED"
            status_color = (0, 0, 255)

        # 绘制状态信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2

        # 背景半透明框
        overlay = result_image.copy()
        cv2.rectangle(overlay, (10, 10), (400, 160), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, result_image, 0.5, 0, result_image)

        # 状态文本
        cv2.putText(result_image, f'Status: {status_text}', (20, 35), font, font_scale, status_color, thickness)
        cv2.putText(result_image, f'Captured Points: {len(self.robot_poses.poses)}/{self.min_points}',
                    (20, 65), font, font_scale, (255, 255, 0), thickness)

        if self.calib_active:
            cv2.putText(result_image, 'Mode: CALIBRATION ACTIVE', (20, 95), font, font_scale, (0, 255, 255), thickness)
        else:
            cv2.putText(result_image, 'Mode: IDLE', (20, 95), font, font_scale, (128, 128, 128), thickness)

        if self.current_aruco_pose is not None and self.aruco_detected:
            pose = self.current_aruco_pose
            cv2.putText(result_image, f'Marker: X={pose.position.x:.3f} Y={pose.position.y:.3f} Z={pose.position.z:.3f}',
                        (20, 125), font, 0.5, (200, 200, 200), 1)

        if self.quality_score > 0:
            cv2.putText(result_image, f'Quality Score: {self.quality_score}/100',
                        (20, 150), font, font_scale, (0, 255, 0) if self.quality_score > 70 else (0, 165, 255), thickness)

        return result_image

    def publish_preview(self):
        """发布预览图像"""
        if self.current_frame is None:
            return

        if not self.calib_active:
            return

        # 绘制状态叠加
        preview_image = self.draw_status_overlay(self.current_frame)

        # 发布原始图像
        try:
            img_msg = self.bridge.cv2_to_imgmsg(preview_image, 'bgr8')
            self.preview_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish preview: {e}')

        # 发布压缩图像
        try:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.format = 'jpeg'
            compressed_msg.data = np.array(cv2.imencode('.jpg', preview_image, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]).tobytes()
            self.preview_compressed_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish compressed preview: {e}')

    def get_robot_pose_sync(self):
        """同步调用获取机器人位姿服务"""
        if not self.robot_pose_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('get_robot_pose service not available')
            return None, False, 'Service not available'

        request = GetRobotPose.Request()
        future = self.robot_pose_client.call_async(request)

        # 等待结果
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()
            if response.success:
                # 将位姿数组转换为Pose消息
                pose = Pose()
                pose.position.x = response.pose[0]
                pose.position.y = response.pose[1]
                pose.position.z = response.pose[2]
                pose.orientation.x = response.pose[3]  # A
                pose.orientation.y = response.pose[4]  # B
                pose.orientation.z = response.pose[5]  # C
                pose.orientation.w = 0.0  # 未使用
                return pose, True, response.message
            else:
                return None, False, response.message
        else:
            return None, False, 'Service call timeout'

    def handle_start_calibration(self, request, response):
        """处理启动标定请求"""
        if self.calib_active:
            response.success = False
            response.message = 'Calibration already active'
            return response

        # 重置状态
        self.calib_active = True
        self.camera_poses = PoseArray()
        self.robot_poses = PoseArray()
        self.captured_images = []
        self.hand_eye_transform = None
        self.reprojection_error = None
        self.quality_score = 0

        # 创建本次标定的图像存储目录
        session_dir = os.path.join(
            self.calib_dir,
            'calib_images',
            f'handeye_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        )
        Path(session_dir).mkdir(parents=True, exist_ok=True)
        self.session_dir = session_dir

        self.get_logger().info('Hand-eye calibration started')
        response.success = True
        response.message = f'Calibration started. Capture at least {self.min_points} points.'
        return response

    def handle_capture_point(self, request, response):
        """处理采集点位请求"""
        if not self.calib_active:
            response.success = False
            response.message = 'Calibration not active. Call start first.'
            return response

        if len(self.robot_poses.poses) >= self.max_points:
            response.success = False
            response.message = f'Maximum points ({self.max_points}) already captured'
            return response

        # 检查ArUco是否检测到
        if not self.aruco_detected or self.current_aruco_pose is None:
            response.success = False
            response.message = 'ArUco marker not detected. Please adjust camera/marker position.'
            return response

        # 获取机器人位姿
        robot_pose, success, message = self.get_robot_pose_sync()
        if not success:
            response.success = False
            response.message = f'Failed to get robot pose: {message}'
            return response

        # 保存数据
        self.camera_poses.poses.append(copy.deepcopy(self.current_aruco_pose))
        self.robot_poses.poses.append(copy.deepcopy(robot_pose))

        # 保存当前图像
        if self.current_frame is not None:
            self.captured_images.append(self.current_frame.copy())
            img_path = os.path.join(self.session_dir, f'handeye_{len(self.robot_poses.poses):02d}.png')
            cv2.imwrite(img_path, self.current_frame)

        point_num = len(self.robot_poses.poses)
        self.get_logger().info(f'Captured point {point_num}/{self.min_points}')
        self.get_logger().info(f'  Robot: X={robot_pose.position.x:.2f}, Y={robot_pose.position.y:.2f}, Z={robot_pose.position.z:.2f}')
        self.get_logger().info(f'  Camera: X={self.current_aruco_pose.position.x:.4f}, Y={self.current_aruco_pose.position.y:.4f}, Z={self.current_aruco_pose.position.z:.4f}')

        response.success = True
        response.message = f'Point {point_num} captured successfully.'
        return response

    def handle_compute_calibration(self, request, response):
        """处理计算标定请求"""
        if not self.calib_active:
            response.success = False
            response.message = 'Calibration not active. Call start first.'
            return response

        if len(self.robot_poses.poses) < self.min_points:
            response.success = False
            response.message = f'Need at least {self.min_points} points. Currently have {len(self.robot_poses.poses)}.'
            return response

        self.get_logger().info(f'Computing hand-eye calibration with {len(self.robot_poses.poses)} points...')

        # 调用手眼标定服务
        if not self.handeye_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = 'hand_eye_calib service not available'
            return response

        try:
            req = HandEyeCalibData.Request()
            req.eye_data_list = self.camera_poses
            req.hand_data_list = self.robot_poses

            future = self.handeye_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

            if future.done():
                result = future.result()
                self.get_logger().info('Hand-eye calibration computed successfully')
                self.get_logger().info(f'Result: {result}')

                # 计算质量评分（基于点数和标定结果）
                # 简化评分：点数越多越好，最多100分
                self.quality_score = min(100, int(len(self.robot_poses.poses) / self.min_points * 80))

                # TODO: 从标定结果中提取变换矩阵
                self.hand_eye_transform = result

                response.success = True
                response.message = f'Calibration computed. Quality score: {self.quality_score}/100'
                return response
            else:
                response.success = False
                response.message = 'Calibration service call timeout'
                return response

        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')
            response.success = False
            response.message = f'Calibration failed: {str(e)}'
            return response

    def handle_save_calibration(self, request, response):
        """处理保存标定请求"""
        if self.hand_eye_transform is None:
            response.success = False
            response.message = 'No calibration result to save. Compute first.'
            return response

        try:
            # 准备保存数据
            calib_data = {
                'calibration_type': 'hand_eye',
                'calibration_date': datetime.now().isoformat(),
                'num_points': len(self.robot_poses.poses),
                'quality_score': self.quality_score,
                'transform': str(self.hand_eye_transform)  # TODO: 正确序列化变换矩阵
            }

            # 保存到YAML文件
            output_path = os.path.join(self.calib_dir, 'hand_eye_transform.yaml')

            # 备份旧文件
            if os.path.exists(output_path):
                backup_dir = os.path.join(self.calib_dir, 'backups')
                Path(backup_dir).mkdir(parents=True, exist_ok=True)
                backup_path = os.path.join(
                    backup_dir,
                    f'hand_eye_transform_{datetime.now().strftime("%Y%m%d_%H%M%S")}.yaml'
                )
                os.rename(output_path, backup_path)
                self.get_logger().info(f'Backed up previous calibration to {backup_path}')

            with open(output_path, 'w') as f:
                yaml.dump(calib_data, f, default_flow_style=False)

            self.get_logger().info(f'Hand-eye calibration saved to {output_path}')

            response.success = True
            response.message = f'Calibration saved to {output_path}'
            return response

        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')
            response.success = False
            response.message = f'Failed to save: {str(e)}'
            return response

    def handle_stop_calibration(self, request, response):
        """处理停止标定请求"""
        self.calib_active = False
        self.camera_poses = PoseArray()
        self.robot_poses = PoseArray()
        self.captured_images = []

        self.get_logger().info('Calibration stopped')
        response.success = True
        response.message = 'Calibration stopped'
        return response

    def handle_get_status(self, request, response):
        """获取标定状态"""
        status_info = {
            'active': self.calib_active,
            'captured_points': len(self.robot_poses.poses),
            'min_points': self.min_points,
            'aruco_detected': self.aruco_detected,
            'has_result': self.hand_eye_transform is not None,
            'quality_score': self.quality_score
        }

        response.success = True
        response.message = str(status_info)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AutoCalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
