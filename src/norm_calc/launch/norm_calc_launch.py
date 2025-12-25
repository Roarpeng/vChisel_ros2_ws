from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# Launch realsense2_camera and norm_calc node with parameters

def generate_launch_description():
    pkg_share = FindPackageShare(package='norm_calc').find('norm_calc')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'norm_calc_params.yaml'])

    # realsense_node = Node(
    #     package='realsense2_camera',
    #     executable='realsense2_camera_node',
    #     name='realsense2_camera',#若修改name则norm_calc_server.cpp中sub订阅的话题需要同步修改。
    #     output='screen',
    #     parameters=[{
    #         # keep default camera params; user can override via ros2 param if needed
    #     }]
    # )

    norm_calc_node = Node(
        package='norm_calc',
        executable='norm_calc_server',
        name='norm_calc',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        # realsense_node,
        norm_calc_node
    ])


# 另一个终端中模拟客户端：ros2 service call /norm_calc norm_calc/srv/NormCalcData "{seq: 1}"