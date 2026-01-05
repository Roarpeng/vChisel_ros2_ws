#!/usr/bin/env python3
"""
vChisel Calibration Mode Launch File
标定模式启动文件

仅启动标定相关节点，用于系统初始设置或重新标定

用法:
  ros2 launch vchisel_bringup calibration_mode.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 参数
    plc_ip_arg = DeclareLaunchArgument(
        'plc_ip',
        default_value='192.168.0.1',
        description='PLC IP地址'
    )

    # 获取包路径
    snap_7_share = FindPackageShare('snap_7')
    hand_eye_calib_share = FindPackageShare('hand_eye_calib')

    # 启动信息
    startup_info = LogInfo(
        msg='\n' + '='*60 + '\n'
            '  vChisel Calibration Mode\n'
            '  视觉凿击系统 - 标定模式\n'
            '='*60
    )

    # RealSense相机
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'pointcloud.enable': True,
        }]
    )

    # PLC节点 (用于读取机器人位姿)
    plc_node = Node(
        package='snap_7',
        executable='snap_7_node',
        name='plc_client',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'plc_ip': LaunchConfiguration('plc_ip'),
            'db_number': 2120,
            'db_start': 64,
        }]
    )

    # 标定服务
    calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([hand_eye_calib_share, 'launch', 'calibration.launch.py'])
        )
    )

    return LaunchDescription([
        plc_ip_arg,
        startup_info,
        realsense_node,
        plc_node,
        calibration_launch,
    ])
