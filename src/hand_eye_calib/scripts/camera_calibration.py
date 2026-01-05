#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机内参标定节点
Camera Intrinsic Calibration Node

功能：
1. 订阅相机图像话题
2. 检测棋盘格标定板
3. 采集标定图像
4. 计算相机内参矩阵和畸变系数
5. 保存标定结果
6. 发布预览图像（带检测结果叠加）

状态码（与PLC交互）：
- 350: 启动相机内参标定
- 360: 采集一张内参标定图像
- 370: 计算相机内参
- 380: 保存相机内参
"""

import os
import sys
import time
import yaml
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge


class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')

        # 标定板参数
        self.declare_parameter('board_rows', 6)
        self.declare_parameter('board_cols', 9)
        self.declare_parameter('square_size', 25.0)  # mm
        self.declare_parameter('min_images', 15)
        self.declare_parameter('max_images', 30)

        # 图像话题
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('preview_topic', '/calib/preview_image')

        # 存储路径
        self.declare_parameter('calib_dir', os.path.expanduser('~/.vchisel/calibrations'))

        # 获取参数
        self.board_rows = self.get_parameter('board_rows').get_parameter_value().integer_value
        self.board_cols = self.get_parameter('board_cols').get_parameter_value().integer_value
        self.square_size = self.get_parameter('square_size').get_parameter_value().double_value
        self.min_images = self.get_parameter('min_images').get_parameter_value().integer_value
        self.max_images = self.get_parameter('max_images').get_parameter_value().integer_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.preview_topic = self.get_parameter('preview_topic').get_parameter_value().string_value
        self.calib_dir = self.get_parameter('calib_dir').get_parameter_value().string_value

        # 确保标定目录存在
        Path(self.calib_dir).mkdir(parents=True, exist_ok=True)
        Path(os.path.join(self.calib_dir, 'calib_images')).mkdir(parents=True, exist_ok=True)

        # CV Bridge
        self.bridge = CvBridge()

        # 标定状态
        self.calib_active = False
        self.captured_images = []
        self.object_points = []  # 3D点
        self.image_points = []   # 2D点
        self.image_size = None
        self.current_frame = None
        self.last_detection_result = None

        # 标定结果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.reprojection_error = None

        # 准备标定板的3D点（假设Z=0平面）
        self.objp = np.zeros((self.board_rows * self.board_cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.board_cols, 0:self.board_rows].T.reshape(-1, 2)
        self.objp *= self.square_size

        # 订阅图像话题
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # 发布预览图像
        self.preview_pub = self.create_publisher(
            Image,
            self.preview_topic,
            10
        )

        # 发布压缩预览图像（用于Web UI）
        self.preview_compressed_pub = self.create_publisher(
            CompressedImage,
            f'{self.preview_topic}/compressed',
            10
        )

        # 服务：启动标定
        self.start_srv = self.create_service(
            Trigger,
            'calib/intrinsic/start',
            self.handle_start_calibration
        )

        # 服务：采集图像
        self.capture_srv = self.create_service(
            Trigger,
            'calib/intrinsic/capture',
            self.handle_capture_image
        )

        # 服务：计算标定
        self.compute_srv = self.create_service(
            Trigger,
            'calib/intrinsic/compute',
            self.handle_compute_calibration
        )

        # 服务：保存结果
        self.save_srv = self.create_service(
            Trigger,
            'calib/intrinsic/save',
            self.handle_save_calibration
        )

        # 服务：停止标定
        self.stop_srv = self.create_service(
            Trigger,
            'calib/intrinsic/stop',
            self.handle_stop_calibration
        )

        # 服务：获取标定状态
        self.status_srv = self.create_service(
            Trigger,
            'calib/intrinsic/status',
            self.handle_get_status
        )

        # 定时器：发布预览图像
        self.preview_timer = self.create_timer(0.1, self.publish_preview)

        self.get_logger().info(f'Camera calibration node started')
        self.get_logger().info(f'Board size: {self.board_cols}x{self.board_rows}, Square size: {self.square_size}mm')
        self.get_logger().info(f'Calibration directory: {self.calib_dir}')

    def image_callback(self, msg):
        """接收相机图像"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.image_size is None:
                self.image_size = (self.current_frame.shape[1], self.current_frame.shape[0])
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def detect_chessboard(self, image):
        """检测棋盘格角点"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 棋盘格检测标志
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK

        # 检测角点
        ret, corners = cv2.findChessboardCorners(gray, (self.board_cols, self.board_rows), flags)

        if ret:
            # 亚像素精度优化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # 计算覆盖面积
            coverage = self.calculate_coverage(corners, image.shape)

            # 计算清晰度（Laplacian方差）
            sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()

            return True, corners, coverage, sharpness
        else:
            return False, None, 0.0, 0.0

    def calculate_coverage(self, corners, image_shape):
        """计算标定板在图像中的覆盖面积百分比"""
        if corners is None:
            return 0.0

        # 获取角点的边界框
        x_coords = corners[:, 0, 0]
        y_coords = corners[:, 0, 1]
        min_x, max_x = np.min(x_coords), np.max(x_coords)
        min_y, max_y = np.min(y_coords), np.max(y_coords)

        # 计算覆盖面积
        board_area = (max_x - min_x) * (max_y - min_y)
        image_area = image_shape[0] * image_shape[1]

        return (board_area / image_area) * 100.0

    def draw_detection_result(self, image, detected, corners, coverage, sharpness):
        """在图像上绘制检测结果"""
        result_image = image.copy()

        if detected and corners is not None:
            # 绘制角点
            cv2.drawChessboardCorners(result_image, (self.board_cols, self.board_rows), corners, detected)

            # 状态文本（绿色）
            status_text = "DETECTED"
            status_color = (0, 255, 0)
        else:
            # 状态文本（红色）
            status_text = "NOT DETECTED"
            status_color = (0, 0, 255)

        # 绘制状态信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2

        # 背景半透明框
        overlay = result_image.copy()
        cv2.rectangle(overlay, (10, 10), (350, 130), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, result_image, 0.5, 0, result_image)

        # 状态文本
        cv2.putText(result_image, f'Status: {status_text}', (20, 35), font, font_scale, status_color, thickness)
        cv2.putText(result_image, f'Coverage: {coverage:.1f}%', (20, 60), font, font_scale, (255, 255, 255), thickness)
        cv2.putText(result_image, f'Sharpness: {sharpness:.0f}', (20, 85), font, font_scale, (255, 255, 255), thickness)
        cv2.putText(result_image, f'Captured: {len(self.captured_images)}/{self.min_images}', (20, 110), font, font_scale, (255, 255, 0), thickness)

        return result_image

    def publish_preview(self):
        """发布预览图像"""
        if self.current_frame is None:
            return

        if not self.calib_active:
            return

        # 检测棋盘格
        detected, corners, coverage, sharpness = self.detect_chessboard(self.current_frame)
        self.last_detection_result = {
            'detected': detected,
            'corners': corners,
            'coverage': coverage,
            'sharpness': sharpness
        }

        # 绘制检测结果
        preview_image = self.draw_detection_result(self.current_frame, detected, corners, coverage, sharpness)

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

    def handle_start_calibration(self, request, response):
        """处理启动标定请求"""
        if self.calib_active:
            response.success = False
            response.message = 'Calibration already active'
            return response

        # 重置状态
        self.calib_active = True
        self.captured_images = []
        self.object_points = []
        self.image_points = []
        self.camera_matrix = None
        self.dist_coeffs = None
        self.reprojection_error = None

        # 创建本次标定的图像存储目录
        session_dir = os.path.join(
            self.calib_dir,
            'calib_images',
            f'intrinsic_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        )
        Path(session_dir).mkdir(parents=True, exist_ok=True)
        self.session_dir = session_dir

        self.get_logger().info('Camera intrinsic calibration started')
        response.success = True
        response.message = f'Calibration started. Capture at least {self.min_images} images.'
        return response

    def handle_capture_image(self, request, response):
        """处理采集图像请求"""
        if not self.calib_active:
            response.success = False
            response.message = 'Calibration not active. Call start first.'
            return response

        if self.current_frame is None:
            response.success = False
            response.message = 'No camera image available'
            return response

        if len(self.captured_images) >= self.max_images:
            response.success = False
            response.message = f'Maximum images ({self.max_images}) already captured'
            return response

        # 检测棋盘格
        detected, corners, coverage, sharpness = self.detect_chessboard(self.current_frame)

        if not detected:
            response.success = False
            response.message = 'Chessboard not detected. Please adjust position.'
            return response

        # 检查图像质量
        if coverage < 5.0:
            response.success = False
            response.message = f'Coverage too low ({coverage:.1f}%). Move board closer.'
            return response

        if sharpness < 50.0:
            response.success = False
            response.message = f'Image too blurry (sharpness: {sharpness:.0f}). Hold steady.'
            return response

        # 保存图像和角点
        self.captured_images.append(self.current_frame.copy())
        self.object_points.append(self.objp)
        self.image_points.append(corners)

        # 保存图像到文件
        img_path = os.path.join(self.session_dir, f'intrinsic_{len(self.captured_images):02d}.png')
        cv2.imwrite(img_path, self.current_frame)

        self.get_logger().info(f'Captured image {len(self.captured_images)}/{self.min_images} (coverage: {coverage:.1f}%, sharpness: {sharpness:.0f})')

        response.success = True
        response.message = f'Image {len(self.captured_images)} captured. Coverage: {coverage:.1f}%, Sharpness: {sharpness:.0f}'
        return response

    def handle_compute_calibration(self, request, response):
        """处理计算标定请求"""
        if not self.calib_active:
            response.success = False
            response.message = 'Calibration not active. Call start first.'
            return response

        if len(self.captured_images) < self.min_images:
            response.success = False
            response.message = f'Need at least {self.min_images} images. Currently have {len(self.captured_images)}.'
            return response

        self.get_logger().info(f'Computing calibration with {len(self.captured_images)} images...')

        try:
            # 执行标定
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.object_points,
                self.image_points,
                self.image_size,
                None,
                None
            )

            if not ret:
                response.success = False
                response.message = 'Calibration computation failed'
                return response

            self.camera_matrix = mtx
            self.dist_coeffs = dist

            # 计算重投影误差
            total_error = 0
            for i in range(len(self.object_points)):
                img_points2, _ = cv2.projectPoints(
                    self.object_points[i],
                    rvecs[i],
                    tvecs[i],
                    mtx,
                    dist
                )
                error = cv2.norm(self.image_points[i], img_points2, cv2.NORM_L2) / len(img_points2)
                total_error += error

            self.reprojection_error = total_error / len(self.object_points)

            self.get_logger().info(f'Calibration computed. Reprojection error: {self.reprojection_error:.4f} pixels')
            self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')

            # 计算质量评分 (0-100)
            # 重投影误差 < 0.5 为满分，> 2.0 为0分
            quality_score = max(0, min(100, int((2.0 - self.reprojection_error) / 1.5 * 100)))

            response.success = True
            response.message = f'Calibration computed. Reprojection error: {self.reprojection_error:.4f} px. Quality: {quality_score}/100'
            return response

        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')
            response.success = False
            response.message = f'Calibration failed: {str(e)}'
            return response

    def handle_save_calibration(self, request, response):
        """处理保存标定请求"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            response.success = False
            response.message = 'No calibration result to save. Compute first.'
            return response

        try:
            # 准备保存数据
            calib_data = {
                'camera_matrix': {
                    'rows': 3,
                    'cols': 3,
                    'data': self.camera_matrix.flatten().tolist()
                },
                'distortion_coefficients': {
                    'rows': 1,
                    'cols': len(self.dist_coeffs.flatten()),
                    'data': self.dist_coeffs.flatten().tolist()
                },
                'image_width': self.image_size[0],
                'image_height': self.image_size[1],
                'reprojection_error': float(self.reprojection_error),
                'calibration_date': datetime.now().isoformat(),
                'num_images': len(self.captured_images),
                'board_size': f'{self.board_cols}x{self.board_rows}',
                'square_size_mm': self.square_size
            }

            # 保存到YAML文件
            output_path = os.path.join(self.calib_dir, 'camera_intrinsics.yaml')

            # 备份旧文件
            if os.path.exists(output_path):
                backup_dir = os.path.join(self.calib_dir, 'backups')
                Path(backup_dir).mkdir(parents=True, exist_ok=True)
                backup_path = os.path.join(
                    backup_dir,
                    f'camera_intrinsics_{datetime.now().strftime("%Y%m%d_%H%M%S")}.yaml'
                )
                os.rename(output_path, backup_path)
                self.get_logger().info(f'Backed up previous calibration to {backup_path}')

            with open(output_path, 'w') as f:
                yaml.dump(calib_data, f, default_flow_style=False)

            self.get_logger().info(f'Calibration saved to {output_path}')

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
        self.captured_images = []
        self.object_points = []
        self.image_points = []

        self.get_logger().info('Calibration stopped')
        response.success = True
        response.message = 'Calibration stopped'
        return response

    def handle_get_status(self, request, response):
        """获取标定状态"""
        status_info = {
            'active': self.calib_active,
            'captured_images': len(self.captured_images),
            'min_images': self.min_images,
            'has_result': self.camera_matrix is not None,
            'reprojection_error': self.reprojection_error
        }

        if self.last_detection_result:
            status_info['board_detected'] = self.last_detection_result['detected']
            status_info['coverage'] = self.last_detection_result['coverage']
            status_info['sharpness'] = self.last_detection_result['sharpness']

        response.success = True
        response.message = str(status_info)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
