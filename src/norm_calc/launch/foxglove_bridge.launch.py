from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Foxglove Bridge启动文件
    用于将ROS2数据桥接到Foxglove Studio进行可视化
    """
    
    # 声明启动参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='WebSocket服务器端口'
    )
    
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='WebSocket服务器监听地址'
    )
    
    topic_list_arg = DeclareLaunchArgument(
        'topics',
        default_value='[]',
        description='要订阅的话题列表（JSON数组格式），空列表表示订阅所有话题'
    )
    
    # Foxglove Bridge节点
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'topics': LaunchConfiguration('topics'),
            'tls': False,  # 禁用TLS以简化配置
            'max_update_ms': 100,  # 最大更新间隔（毫秒）
            'num_threads': 4,  # 线程数
        }]
    )
    
    return LaunchDescription([
        port_arg,
        address_arg,
        topic_list_arg,
        foxglove_bridge_node,
    ])