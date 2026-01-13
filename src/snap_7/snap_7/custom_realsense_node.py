#!/usr/bin/env python3
"""
Custom RealSense Driver for Concrete Chiseling (Eye-in-Hand)
Optimized for Static Capture with Temporal Filtering
"""

import sys

# [HACK] 强制使用 numpy 1.x 兼容模式，规避 cv_bridge 与 numpy 2.x 的冲突
import numpy as np

try:
    if np.__version__.startswith("2."):
        import warnings

        warnings.filterwarnings(
            "ignore", message="A module that was compiled using NumPy 1.x"
        )
except:
    pass

import sys

# [HACK] 强制使用 numpy 1.x 兼容模式，规避 cv_bridge 与 numpy 2.x 的冲突
import numpy as np

try:
    if np.__version__.startswith("2."):
        import warnings

        warnings.filterwarnings(
            "ignore", message="A module that was compiled using NumPy 1.x"
        )
except:
    pass

# [HACK] 确保使用 ROS 系统自带的 cv_bridge，而不是 pip 安装的版本
import sys

sys.path.insert(0, "/opt/ros/humble/lib/python3.10/site-packages")
sys.path.insert(0, "/opt/ros/humble/local/lib/python3.10/dist-packages")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import time


class CustomRealSenseNode(Node):
    def __init__(self):
        super().__init__("realsense2_camera")  # 使用相同的节点名以便兼容

        # Publishers
        self.pub_color = self.create_publisher(
            Image, "/camera/camera/color/image_raw", 10
        )
        self.pub_depth = self.create_publisher(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", 10
        )
        self.pub_info = self.create_publisher(
            CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", 10
        )

        self.bridge = CvBridge()

        # --- RealSense Config ---
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.rgb8, 30)

        # --- Filters Setup ---
        # 1. Decimation (降低分辨率以减少计算，可选，这里保持原分辨率精度)
        # self.decimation = rs.decimation_filter()

        # 2. Hole Filling (补孔 - 关键)
        self.hole_filling = rs.hole_filling_filter()
        self.hole_filling.set_option(rs.option.holes_fill, 1)  # 1 = Nearest Neighbor

        # 3. Spatial Filter (空间平滑)
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 2)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial.set_option(rs.option.filter_smooth_delta, 20)
        self.spatial.set_option(rs.option.holes_fill, 0)  # 交给 hole_filling 做

        # 4. Temporal Filter (时间滤波 - 静止拍照神器)
        self.temporal = rs.temporal_filter()
        self.temporal.set_option(rs.option.filter_smooth_alpha, 0.4)  # 平滑系数
        self.temporal.set_option(rs.option.filter_smooth_delta, 20)  # 差异阈值

        # 启动管线
        self.profile = self.pipeline.start(config)
        self.get_logger().info("RealSense Pipeline Started (Custom SDK Mode)")

        # 获取内参
        self.depth_profile = self.profile.get_stream(rs.stream.depth)
        self.intrinsics = self.depth_profile.as_video_stream_profile().get_intrinsics()

        # 对齐对象
        self.align = rs.align(rs.stream.color)

        # 定时器 (30FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)

        # 预热计数
        self.frame_count = 0
        self.warmup_frames = 30  # 丢弃前30帧

    def get_camera_info(self, frame_time):
        info = CameraInfo()
        info.header.stamp = frame_time
        info.header.frame_id = "camera_color_optical_frame"
        info.width = self.intrinsics.width
        info.height = self.intrinsics.height
        info.distortion_model = "plumb_bob"
        info.d = [float(c) for c in self.intrinsics.coeffs]
        info.k = [
            self.intrinsics.fx,
            0.0,
            self.intrinsics.ppx,
            0.0,
            self.intrinsics.fy,
            self.intrinsics.ppy,
            0.0,
            0.0,
            1.0,
        ]
        info.p = [
            self.intrinsics.fx,
            0.0,
            self.intrinsics.ppx,
            0.0,
            0.0,
            self.intrinsics.fy,
            self.intrinsics.ppy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return info

    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames()

            # 对齐
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return

            # 预热期
            self.frame_count += 1
            if self.frame_count < self.warmup_frames:
                return

            # === 应用滤波器链 ===
            # 顺序: Hole -> Spatial -> Temporal
            filtered_depth = self.hole_filling.process(depth_frame)
            filtered_depth = self.spatial.process(filtered_depth)
            filtered_depth = self.temporal.process(filtered_depth)

            # 转换数据
            depth_image = np.asanyarray(filtered_depth.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 发布 ROS 消息
            now = self.get_clock().now().to_msg()

            # Color
            msg_color = self.bridge.cv2_to_imgmsg(color_image, "rgb8")
            msg_color.header.stamp = now
            msg_color.header.frame_id = "camera_color_optical_frame"
            self.pub_color.publish(msg_color)

            # Depth
            msg_depth = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
            msg_depth.header.stamp = now
            msg_depth.header.frame_id = "camera_color_optical_frame"
            self.pub_depth.publish(msg_depth)

            # Camera Info
            msg_info = self.get_camera_info(now)
            self.pub_info.publish(msg_info)

        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CustomRealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
