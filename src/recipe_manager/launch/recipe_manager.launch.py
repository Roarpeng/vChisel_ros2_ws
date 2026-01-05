#!/usr/bin/env python3
"""
Recipe Manager Launch File
配方管理器启动文件
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 参数声明
    recipe_dir_arg = DeclareLaunchArgument(
        'recipe_dir',
        default_value=os.path.expanduser('~/.vchisel/recipes'),
        description='Directory for recipe files'
    )

    backup_dir_arg = DeclareLaunchArgument(
        'backup_dir',
        default_value=os.path.expanduser('~/.vchisel/recipes/backups'),
        description='Directory for recipe backups'
    )

    default_recipe_arg = DeclareLaunchArgument(
        'default_recipe',
        default_value='default',
        description='Default recipe name'
    )

    max_backups_arg = DeclareLaunchArgument(
        'max_backups',
        default_value='10',
        description='Maximum number of backups to keep'
    )

    # Recipe Manager 节点
    recipe_manager_node = Node(
        package='recipe_manager',
        executable='recipe_node.py',
        name='recipe_manager_node',
        output='screen',
        parameters=[{
            'recipe_dir': LaunchConfiguration('recipe_dir'),
            'backup_dir': LaunchConfiguration('backup_dir'),
            'default_recipe': LaunchConfiguration('default_recipe'),
            'max_backups': LaunchConfiguration('max_backups'),
        }]
    )

    return LaunchDescription([
        recipe_dir_arg,
        backup_dir_arg,
        default_recipe_arg,
        max_backups_arg,
        recipe_manager_node,
    ])
