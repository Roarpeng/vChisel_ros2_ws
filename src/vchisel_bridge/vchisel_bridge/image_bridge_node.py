#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图像桥接节点
Image Bridge Node

功能：
1. 订阅ROS图像话题（captured_image）
2. 将图像转换为base64编码的JPEG
3. 发布到WebSocket可以传输的话题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
import cv2
from cv_bridge import CvBridge
import base64
import json
import numpy as np
from datetime import datetime


class ImageBridgeNode(Node):
    def __init__(self):
        super().__init__('image_bridge_node')

        # 参数
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('max_width', 800)
        self.declare_parameter('max_height', 600)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.max_width = self.get_parameter('max_width').value
        self.max_height = self.get_parameter('max_height').value

        self.bridge = CvBridge()

        # 存储最新的图像和法向量数据
        self.latest_image = None
        self.latest_normals = None
        self.image_updated = False

        # 订阅原始图像
        self.image_sub = self.create_subscription(
            Image,
            'captured_image',
            self.image_callback,
            10
        )

        # 订阅法向量结果
        self.normals_sub = self.create_subscription(
            PoseArray,
            'visual_norm_result',
            self.normals_callback,
            10
        )

        # 发布base64编码的图像
        self.image_pub = self.create_publisher(
            String,
            '/vchisel/ui/camera_image',
            10
        )

        # 发布带有法向量标注的图像
        self.annotated_pub = self.create_publisher(
            String,
            '/vchisel/ui/annotated_image',
            10
        )

        # 也发布压缩图像（可选，用于本地显示）
        self.compressed_pub = self.create_publisher(
            CompressedImage,
            '/vchisel/ui/compressed_image',
            10
        )

        # 定时器用于限制发布频率
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_images)

        self.get_logger().info(f'Image bridge node started (quality={self.jpeg_quality}, max_size={self.max_width}x{self.max_height})')

    def image_callback(self, msg: Image):
        """处理图像消息"""
        try:
            # 转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
            self.image_updated = True
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def normals_callback(self, msg: PoseArray):
        """处理法向量结果"""
        self.latest_normals = msg

    def resize_image(self, image: np.ndarray) -> np.ndarray:
        """调整图像大小以减少传输数据量"""
        h, w = image.shape[:2]

        # 计算缩放比例
        scale = min(self.max_width / w, self.max_height / h)

        if scale < 1.0:
            new_w = int(w * scale)
            new_h = int(h * scale)
            return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)

        return image

    def draw_normals(self, image: np.ndarray, normals: PoseArray) -> np.ndarray:
        """在图像上绘制法向量"""
        if normals is None or len(normals.poses) == 0:
            return image

        annotated = image.copy()
        h, w = annotated.shape[:2]

        for pose in normals.poses:
            # 从位置获取像素坐标（假设已经是像素坐标）
            px = int(pose.position.x)
            py = int(pose.position.y)

            # 确保在图像范围内
            if 0 <= px < w and 0 <= py < h:
                # 绘制点
                cv2.circle(annotated, (px, py), 5, (0, 255, 0), -1)

                # 从四元数获取法向量方向（简化：用z方向）
                # 绘制法向量箭头
                nx = pose.orientation.x * 30
                ny = pose.orientation.y * 30

                end_x = int(px + nx)
                end_y = int(py + ny)

                cv2.arrowedLine(annotated, (px, py), (end_x, end_y),
                               (0, 255, 255), 2, tipLength=0.3)

        return annotated

    def image_to_base64(self, image: np.ndarray) -> str:
        """将图像转换为base64编码的JPEG"""
        # 调整大小
        resized = self.resize_image(image)

        # 编码为JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        _, buffer = cv2.imencode('.jpg', resized, encode_param)

        # 转换为base64
        base64_str = base64.b64encode(buffer).decode('utf-8')

        return base64_str

    def publish_images(self):
        """发布图像"""
        if self.latest_image is None:
            return

        try:
            timestamp = datetime.now().isoformat()

            # 发布原始相机图像
            base64_image = self.image_to_base64(self.latest_image)

            image_msg = String()
            image_data = {
                'type': 'camera_image',
                'data': base64_image,
                'timestamp': timestamp,
                'width': self.latest_image.shape[1],
                'height': self.latest_image.shape[0]
            }
            image_msg.data = json.dumps(image_data)
            self.image_pub.publish(image_msg)

            # 发布带有法向量标注的图像
            if self.latest_normals is not None:
                annotated = self.draw_normals(self.latest_image, self.latest_normals)
                base64_annotated = self.image_to_base64(annotated)

                annotated_msg = String()
                annotated_data = {
                    'type': 'annotated_image',
                    'data': base64_annotated,
                    'timestamp': timestamp,
                    'width': annotated.shape[1],
                    'height': annotated.shape[0],
                    'normal_count': len(self.latest_normals.poses)
                }
                annotated_msg.data = json.dumps(annotated_data)
                self.annotated_pub.publish(annotated_msg)

            # 也发布压缩图像
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.format = 'jpeg'
            resized = self.resize_image(self.latest_image)
            _, buffer = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            compressed_msg.data = buffer.tobytes()
            self.compressed_pub.publish(compressed_msg)

            if self.image_updated:
                self.get_logger().debug('Published image to UI')
                self.image_updated = False

        except Exception as e:
            self.get_logger().error(f'Failed to publish images: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
