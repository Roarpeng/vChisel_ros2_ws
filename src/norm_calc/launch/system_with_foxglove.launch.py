from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """
    完整系统启动文件（集成Foxglove可视化）
    包含：norm_calc服务、PLC客户端、相机监控、RealSense相机、Foxglove Bridge
    """
    
    # 获取包路径
    norm_calc_pkg_share = FindPackageShare(package='norm_calc').find('norm_calc')
    snap_7_pkg_share = FindPackageShare(package='snap_7').find('snap_7')
    
    # 获取参数文件路径
    params_file = PathJoinSubstitution([norm_calc_pkg_share, 'config', 'norm_calc_params.yaml'])
    
    # ==================== 声明启动参数 ====================
    foxglove_enabled_arg = DeclareLaunchArgument(
        'foxglove_enabled',
        default_value='true',
        description='是否启用Foxglove Bridge'
    )
    
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket端口'
    )
    
    # ==================== 核心节点 ====================
    
    # 1. norm_calc服务节点
    norm_calc_node = Node(
        package='norm_calc',
        executable='norm_calc_server',
        name='norm_calc',
        output='screen',
        parameters=[params_file]
    )
    
    # 2. PLC客户端节点
    plc_client_node = Node(
        package='snap_7',
        executable='snap_7_node',
        name='plc_client_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'db_number': 2120},
            {'db_start': 64},
            {'plc_address': '192.168.1.36'},
            {'plc_rack': 0},
            {'plc_slot': 1},
            {'poll_rate': 20.0}
        ]
    )
    
    # 3. 相机监控节点
    camera_monitor_node = Node(
        package='snap_7',
        executable='camera_monitor',
        name='camera_monitor',
        output='screen',
        parameters=[
            {'timeout_seconds': 5.0},
            {'reconnect_attempts': 3},
            {'reconnect_delay': 2.0}
        ]
    )
    
    # 4. RealSense相机节点
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'depth_module.profile': '640x480x10',
            'color_width': 640,
            'color_height': 480,
            'color_fps': 10.0,
            'depth_fps': 10.0,
            'initial_reset': True,
        }]
    )
    
    # 5. Foxglove Bridge节点（可选）
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('foxglove_enabled'), "' == 'true'"])),
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'address': '0.0.0.0',
            'topics': '[]',  # 订阅所有话题
            'tls': False,
            'max_update_ms': 100,
            'num_threads': 4,
        }]
    )
    
    # ==================== 启动顺序控制 ====================
    
    # 确保相机节点先启动
    camera_ready_event = RegisterEventHandler(
        OnProcessExit(
            target_action=realsense_node,
            on_exit=[norm_calc_node, plc_client_node, camera_monitor_node]
        )
    )
    
    return LaunchDescription([
        # 启动参数
        foxglove_enabled_arg,
        foxglove_port_arg,
        
        # 核心节点
        realsense_node,
        foxglove_bridge_node,
        norm_calc_node,
        plc_client_node,
        camera_monitor_node,
        
        # 事件处理器
        camera_ready_event,
    ])