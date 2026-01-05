#!/usr/bin/env python3
"""
标定系统启动文件
Calibration System Launch File

启动以下节点：
1. hand_eye_calib_server - 手眼标定计算服务
2. camera_calibration_node - 相机内参标定节点
3. auto_calibration_node - 自动化手眼标定节点
4. aruco_single - ArUco标定板检测节点
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 声明参数
    board_rows_arg = DeclareLaunchArgument(
        'board_rows',
        default_value='6',
        description='Chessboard rows for camera calibration'
    )

    board_cols_arg = DeclareLaunchArgument(
        'board_cols',
        default_value='9',
        description='Chessboard columns for camera calibration'
    )

    square_size_arg = DeclareLaunchArgument(
        'square_size',
        default_value='25.0',
        description='Chessboard square size in mm'
    )

    min_calib_images_arg = DeclareLaunchArgument(
        'min_calib_images',
        default_value='15',
        description='Minimum images for camera calibration'
    )

    min_handeye_points_arg = DeclareLaunchArgument(
        'min_handeye_points',
        default_value='8',
        description='Minimum points for hand-eye calibration'
    )

    calib_dir_arg = DeclareLaunchArgument(
        'calib_dir',
        default_value=os.path.expanduser('~/.vchisel/calibrations'),
        description='Calibration data directory'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Camera image topic'
    )

    # 手眼标定计算服务节点
    hand_eye_calib_server = Node(
        package='hand_eye_calib',
        executable='hand_eye_calib_server',
        name='hand_eye_calib_server',
        output='screen'
    )

    # 相机内参标定节点
    camera_calibration_node = Node(
        package='hand_eye_calib',
        executable='camera_calibration.py',
        name='camera_calibration_node',
        output='screen',
        parameters=[{
            'board_rows': LaunchConfiguration('board_rows'),
            'board_cols': LaunchConfiguration('board_cols'),
            'square_size': LaunchConfiguration('square_size'),
            'min_images': LaunchConfiguration('min_calib_images'),
            'image_topic': LaunchConfiguration('image_topic'),
            'calib_dir': LaunchConfiguration('calib_dir'),
        }]
    )

    # 自动化手眼标定节点
    auto_calibration_node = Node(
        package='hand_eye_calib',
        executable='auto_calibration_node.py',
        name='auto_calibration_node',
        output='screen',
        parameters=[{
            'min_points': LaunchConfiguration('min_handeye_points'),
            'image_topic': LaunchConfiguration('image_topic'),
            'calib_dir': LaunchConfiguration('calib_dir'),
        }]
    )

    return LaunchDescription([
        # 参数声明
        board_rows_arg,
        board_cols_arg,
        square_size_arg,
        min_calib_images_arg,
        min_handeye_points_arg,
        calib_dir_arg,
        image_topic_arg,

        # 节点
        hand_eye_calib_server,
        camera_calibration_node,
        auto_calibration_node,
    ])
