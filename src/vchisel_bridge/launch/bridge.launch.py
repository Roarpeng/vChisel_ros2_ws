#!/usr/bin/env python3
"""
vChisel Bridge Launch File
UI桥接层启动文件

启动以下节点：
1. bridge_node - 系统状态聚合
2. alarm_manager - 报警管理
3. rosbridge_server - WebSocket服务 (可选，需要安装rosbridge_suite)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 参数声明
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='WebSocket port for rosbridge'
    )

    max_alarm_history_arg = DeclareLaunchArgument(
        'max_alarm_history',
        default_value='100',
        description='Maximum alarm history entries'
    )

    enable_rosbridge_arg = DeclareLaunchArgument(
        'enable_rosbridge',
        default_value='true',
        description='Enable rosbridge WebSocket server (requires rosbridge_suite)'
    )

    # Bridge 节点
    bridge_node = Node(
        package='vchisel_bridge',
        executable='bridge_node.py',
        name='bridge_node',
        output='screen'
    )

    # Alarm Manager 节点
    alarm_manager_node = Node(
        package='vchisel_bridge',
        executable='alarm_manager.py',
        name='alarm_manager',
        output='screen',
        parameters=[{
            'max_history': LaunchConfiguration('max_alarm_history'),
        }]
    )

    # Image Bridge 节点 - 将ROS图像转换为Web可显示格式
    image_bridge_node = Node(
        package='vchisel_bridge',
        executable='image_bridge_node.py',
        name='image_bridge',
        output='screen',
        parameters=[{
            'jpeg_quality': 80,
            'max_width': 800,
            'max_height': 600,
            'publish_rate': 10.0,
        }]
    )

    # rosbridge_server (可选)
    # 注意：需要安装 rosbridge_suite 包: sudo apt install ros-humble-rosbridge-suite
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rosbridge')),
        parameters=[{
            'port': LaunchConfiguration('rosbridge_port'),
        }]
    )

    return LaunchDescription([
        rosbridge_port_arg,
        max_alarm_history_arg,
        enable_rosbridge_arg,
        bridge_node,
        alarm_manager_node,
        image_bridge_node,
        rosbridge_node,
    ])
