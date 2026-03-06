from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import launch.conditions

# Launch realsense2_camera and norm_calc node with parameters

def generate_launch_description():
    pkg_share = FindPackageShare(package='norm_calc').find('norm_calc')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'norm_calc_params.yaml'])

    # 声明启动参数：是否启动可视化节点（默认不启动）
    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='false',
        description='Enable image norm viewer (requires GUI)'
    )

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

    # 可视化节点（条件启动）
    image_norm_viewer_node = Node(
        package='norm_calc',
        executable='image_norm_viewer',
        name='image_norm_viewer',
        output='screen',
        parameters=[params_file],
        condition=launch.conditions.IfCondition(LaunchConfiguration('enable_viewer'))
    )

    return LaunchDescription([
        enable_viewer_arg,
        # realsense_node,
        norm_calc_node,
        image_norm_viewer_node
    ])


# 另一个终端中模拟客户端：ros2 service call /norm_calc norm_calc/srv/NormCalcData "{seq: 1}"