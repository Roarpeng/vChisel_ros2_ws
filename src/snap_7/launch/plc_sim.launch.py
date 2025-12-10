from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='127.0.0.1',
        description='IP address for the PLC simulator server'
    )
    
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='102',
        description='Port for the PLC simulator server'
    )
    
    # Get launch configurations
    server_ip = LaunchConfiguration('server_ip')
    server_port = LaunchConfiguration('server_port')
    
    # Define the PLC simulator node
    plc_sim_node = Node(
        package='snap_7',
        executable='plc_sim_server',
        name='plc_simulator',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'server_ip': server_ip,
            'server_port': server_port
        }]
    )
    
    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        plc_sim_node
    ])