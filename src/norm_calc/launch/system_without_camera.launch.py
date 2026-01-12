from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    系统启动文件（不包含RealSense相机）
    包含：norm_calc服务、PLC客户端、相机监控、Foxglove Bridge
    """
    
    # 获取包路径
    norm_calc_pkg_share = FindPackageShare(package='norm_calc').find('norm_calc')
    
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
    
    plc_enabled_arg = DeclareLaunchArgument(
        'plc_enabled',
        default_value='true',
        description='是否启用PLC客户端'
    )
    
    camera_monitor_enabled_arg = DeclareLaunchArgument(
        'camera_monitor_enabled',
        default_value='true',
        description='是否启用相机监控'
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
    
    # 2. PLC客户端节点（可选）
    plc_client_node = Node(
        package='snap_7',
        executable='snap_7_node',
        name='plc_client_node',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('plc_enabled'), "' == 'true'"])),
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
    
    # 3. 相机监控节点（可选）
    camera_monitor_node = Node(
        package='snap_7',
        executable='camera_monitor',
        name='camera_monitor',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('camera_monitor_enabled'), "' == 'true'"])),
        output='screen',
        parameters=[
            {'timeout_seconds': 5.0},
            {'reconnect_attempts': 3},
            {'reconnect_delay': 2.0}
        ]
    )
    
    # 4. Foxglove Bridge节点（可选）
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('foxglove_enabled'), "' == 'true'"])),
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'address': '0.0.0.0',
            'topics': '[]',
            'tls': False,
            'max_update_ms': 100,
            'num_threads': 4,
        }]
    )
    
    # ==================== 返回启动描述 ====================
    
    return LaunchDescription([
        # 启动参数
        foxglove_enabled_arg,
        foxglove_port_arg,
        plc_enabled_arg,
        camera_monitor_enabled_arg,
        
        # 核心节点
        norm_calc_node,
        plc_client_node,
        camera_monitor_node,
        foxglove_bridge_node,
    ])