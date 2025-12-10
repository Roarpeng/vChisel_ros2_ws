from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取包路径
    norm_calc_pkg_share = FindPackageShare(package='norm_calc').find('norm_calc')
    snap_7_pkg_share = FindPackageShare(package='snap_7').find('snap_7')
    
    # 获取参数文件路径
    params_file = PathJoinSubstitution([norm_calc_pkg_share, 'config', 'norm_calc_params.yaml'])

    # 启动norm_calc服务节点
    norm_calc_node = Node(
        package='norm_calc',
        executable='norm_calc_node',
        name='norm_calc',
        output='screen',
        parameters=[params_file]
    )

    # 启动PLC客户端节点
    plc_client_node = Node(
        package='snap_7',
        executable='snap_7_node',
        name='plc_client_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'db_number': 2120},  # 使用与测试脚本一致的DB号
            {'db_start': 64},
            {'plc_address': '192.168.1.36'},
            {'plc_rack': 0},
            {'plc_slot': 1},
            {'poll_rate': 20.0}
        ]
    )

    # 启动相机监控节点（工业级可靠性增强）
    camera_monitor_node = Node(
        package='snap_7',
        executable='camera_monitor.py',
        name='camera_monitor',
        output='screen',
        parameters=[
            {'timeout_seconds': 5.0},
            {'reconnect_attempts': 3},
            {'reconnect_delay': 2.0}
        ]
    )

    # 启动图像和法向信息联合可视化节点（新增）- 这是您需要的主要功能
    image_norm_viewer_node = Node(
        package='norm_calc',
        executable='image_norm_viewer_node',
        name='image_norm_viewer',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ]
    )

    # 启动RealSense相机节点
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra': False,  # Disable infrared emission
            'enable_infra1': False,  # Disable infrared 1
            'enable_infra2': False,  # Disable infrared 2
            'depth_module.profile': '640x480x10',  # Reduced frame rate
            'color_width': 640,
            'color_height': 480,
            'color_fps': 10.0,  # Reduced frame rate from 30 to 10 FPS
            'depth_fps': 10.0,  # Reduced frame rate from 30 to 10 FPS
            # 添加相机可靠性增强参数
            'initial_reset': True,  # 启动时重置设备
        }]
    )

    # 可选：启动PCL点云可视化节点（如果需要的话，暂时注释掉以避免崩溃）
    # norm_viewer_node = Node(
    #     package='norm_calc',
    #     executable='norm_viewer_node',
    #     name='norm_viewer',
    #     output='screen'
    # )

    return LaunchDescription([
        # 启动所有节点
        norm_calc_node,
        plc_client_node,
        camera_monitor_node,
        image_norm_viewer_node,
        realsense_node,
        # norm_viewer_node  # 暂时注释掉有问题的节点
    ])