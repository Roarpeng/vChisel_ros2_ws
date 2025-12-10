from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# Launch norm_calc and plc_client_node
# ros2 launch snap_7 snap_7.launch.py 

def generate_launch_description():
    pkg_share = FindPackageShare(package='norm_calc').find('norm_calc')
    # include existing norm_calc launch
    norm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'norm_calc_launch.py']))
    )

    plc_node = Node(
        package='snap_7',
        executable='snap_7_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'db_number': 2120},  # Changed to 2120 to match the working test script
            {'db_start': 64}
        ]
    )

    return LaunchDescription([
        norm_launch,
        plc_node
    ])
