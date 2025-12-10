from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    # Launch the PLC simulator
    plc_sim_node = Node(
        package='snap_7',
        executable='plc_sim_server',
        name='plc_simulator',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'server_ip': '127.0.0.1',
            'server_port': 102
        }]
    )

    # Launch the PLC client configured to connect to the simulator
    plc_client_node = Node(
        package='snap_7',
        executable='snap_7_node',
        name='plc_client',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'plc_address': '127.0.0.1',
            'plc_rack': 0,
            'plc_slot': 1,
            'db_number': 2120,
            'db_start': 0,
            'poll_rate': 10.0
        }]
    )

    return LaunchDescription([
        plc_sim_node,
        plc_client_node
    ])