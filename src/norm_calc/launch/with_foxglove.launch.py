from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

"""
在现有系统基础上添加 Foxglove Bridge
用法: ros2 launch norm_calc with_foxglove.launch.py

注意: 相机需要单独启动！
  ros2 launch realsense2_camera rs_launch.py
"""

def generate_launch_description():
    # 获取包路径
    norm_calc_pkg = FindPackageShare(package='norm_calc').find('norm_calc')
    snap_7_pkg = FindPackageShare(package='snap_7').find('snap_7')
    
    # Foxglove 参数
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket端口'
    )
    
    # 包含现有的 snap_7 launch (包含 norm_calc + image_norm_viewer + plc_node)
    snap_7_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([snap_7_pkg, 'launch', 'snap_7.launch.py'])
        )
    )
    
    # Foxglove Bridge - 只订阅用于可视化的话题
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,
            'tls': False,
            'num_threads': 2,
            # 订阅必要话题（压缩图像 + 结果 + 参数）
            'topic_whitelist': [
                # 法向计算发布的图像和结果
                '/captured_image',
                '/visual_norm_result', 
                '/debug_processed_cloud',
                # 参数事件（用于调参）
                '/parameter_events',
                # TF
                '/tf',
                '/tf_static',
            ],
            # 启用参数服务（用于 Foxglove 调参）
            'capabilities': ['parameters', 'parametersSubscribe', 'services'],
        }]
    )
    
    return LaunchDescription([
        foxglove_port_arg,
        snap_7_launch,
        foxglove_bridge_node,
    ])
