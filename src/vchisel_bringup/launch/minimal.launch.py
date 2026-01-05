#!/usr/bin/env python3
"""
vChisel Minimal Launch File
最小化启动文件 - 仅核心功能

仅启动核心节点用于生产运行：
1. PLC通信
2. 法向量计算
3. UI桥接

用法:
  ros2 launch vchisel_bringup minimal.launch.py
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

    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='WebSocket端口'
    )

    # 获取包路径
    norm_calc_share = FindPackageShare('norm_calc')
    vchisel_bridge_share = FindPackageShare('vchisel_bridge')

    # 启动信息
    startup_info = LogInfo(
        msg='\n' + '='*60 + '\n'
            '  vChisel Minimal Mode\n'
            '  视觉凿击系统 - 最小化模式\n'
            '='*60
    )

    # PLC节点
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

    # 法向量计算
    norm_calc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([norm_calc_share, 'launch', 'norm_calc_launch.py'])
        )
    )

    # UI桥接
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([vchisel_bridge_share, 'launch', 'bridge.launch.py'])
        ),
        launch_arguments={
            'rosbridge_port': LaunchConfiguration('rosbridge_port'),
        }.items()
    )

    return LaunchDescription([
        plc_ip_arg,
        rosbridge_port_arg,
        startup_info,
        plc_node,
        norm_calc_launch,
        bridge_launch,
    ])
