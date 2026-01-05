#!/usr/bin/env python3
"""
vChisel Full System Launch File
完整系统启动文件

启动以下子系统：
1. RealSense相机 (可选)
2. PLC通信节点 (snap_7)
3. 法向量计算服务 (norm_calc)
4. 手眼标定服务 (hand_eye_calib) - 可选
5. 配方管理 (recipe_manager)
6. UI桥接层 (vchisel_bridge + rosbridge)

用法:
  ros2 launch vchisel_bringup full_system.launch.py
  ros2 launch vchisel_bringup full_system.launch.py use_camera:=true
  ros2 launch vchisel_bringup full_system.launch.py simulation:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================================
    # 参数声明
    # ============================================================

    # 是否启用相机
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='启动RealSense相机节点'
    )

    # 是否启用标定模式
    enable_calibration_arg = DeclareLaunchArgument(
        'enable_calibration',
        default_value='false',
        description='启动手眼标定服务节点'
    )

    # 是否启用仿真模式（不连接真实PLC）
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='使用PLC仿真模式'
    )

    # PLC连接参数
    plc_ip_arg = DeclareLaunchArgument(
        'plc_ip',
        default_value='192.168.0.1',
        description='PLC IP地址'
    )

    plc_rack_arg = DeclareLaunchArgument(
        'plc_rack',
        default_value='0',
        description='PLC机架号'
    )

    plc_slot_arg = DeclareLaunchArgument(
        'plc_slot',
        default_value='1',
        description='PLC插槽号'
    )

    # DB块参数
    db_number_arg = DeclareLaunchArgument(
        'db_number',
        default_value='2120',
        description='PLC数据块号'
    )

    db_start_arg = DeclareLaunchArgument(
        'db_start',
        default_value='64',
        description='DB起始偏移量'
    )

    # rosbridge端口
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='WebSocket端口'
    )

    # 是否启用rosbridge
    enable_rosbridge_arg = DeclareLaunchArgument(
        'enable_rosbridge',
        default_value='false',
        description='启用rosbridge WebSocket服务器（需要安装rosbridge_suite）'
    )

    # 配方目录
    recipe_dir_arg = DeclareLaunchArgument(
        'recipe_dir',
        default_value='~/.vchisel/recipes',
        description='配方存储目录'
    )

    # ============================================================
    # 获取包路径
    # ============================================================
    snap_7_share = FindPackageShare('snap_7')
    norm_calc_share = FindPackageShare('norm_calc')
    hand_eye_calib_share = FindPackageShare('hand_eye_calib')
    recipe_manager_share = FindPackageShare('recipe_manager')
    vchisel_bridge_share = FindPackageShare('vchisel_bridge')

    # ============================================================
    # 1. RealSense相机节点 (可选)
    # ============================================================
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'align_depth.enable': True,
            'pointcloud.enable': True,
        }]
    )

    # ============================================================
    # 2. PLC通信节点 (snap_7)
    # ============================================================
    # 真实PLC模式
    plc_real_node = Node(
        package='snap_7',
        executable='snap_7_node',
        name='plc_client',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('simulation')),
        parameters=[{
            'plc_ip': LaunchConfiguration('plc_ip'),
            'plc_rack': LaunchConfiguration('plc_rack'),
            'plc_slot': LaunchConfiguration('plc_slot'),
            'db_number': LaunchConfiguration('db_number'),
            'db_start': LaunchConfiguration('db_start'),
        }]
    )

    # 仿真PLC模式
    plc_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([snap_7_share, 'launch', 'plc_sim.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('simulation'))
    )

    # ============================================================
    # 3. 法向量计算节点 (norm_calc)
    # ============================================================
    norm_calc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([norm_calc_share, 'launch', 'norm_calc_launch.py'])
        )
    )

    # ============================================================
    # 4. 手眼标定服务 (可选)
    # ============================================================
    calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([hand_eye_calib_share, 'launch', 'calibration.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('enable_calibration'))
    )

    # ============================================================
    # 5. 配方管理节点
    # ============================================================
    recipe_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([recipe_manager_share, 'launch', 'recipe_manager.launch.py'])
        ),
        launch_arguments={
            'recipe_dir': LaunchConfiguration('recipe_dir'),
        }.items()
    )

    # ============================================================
    # 6. UI桥接层 (包含rosbridge)
    # ============================================================
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([vchisel_bridge_share, 'launch', 'bridge.launch.py'])
        ),
        launch_arguments={
            'rosbridge_port': LaunchConfiguration('rosbridge_port'),
            'enable_rosbridge': LaunchConfiguration('enable_rosbridge'),
        }.items()
    )

    # ============================================================
    # 启动信息
    # ============================================================
    startup_info = LogInfo(
        msg='vChisel System Starting... / 视觉凿击系统启动中...'
    )

    # ============================================================
    # 返回启动描述
    # ============================================================
    return LaunchDescription([
        # 参数
        use_camera_arg,
        enable_calibration_arg,
        simulation_arg,
        plc_ip_arg,
        plc_rack_arg,
        plc_slot_arg,
        db_number_arg,
        db_start_arg,
        rosbridge_port_arg,
        enable_rosbridge_arg,
        recipe_dir_arg,

        # 启动信息
        startup_info,

        # 节点和子系统
        realsense_node,
        plc_real_node,
        plc_sim_launch,
        norm_calc_launch,
        calibration_launch,
        recipe_manager_launch,
        bridge_launch,
    ])
