"""
robocup_planner_sim.launch.py

Full simulation stack:
  order_server  — publishes /sml/task (TRANSIENT_LOCAL, auto_publish)
  mock_nav_node — serves navigate_to_station action
  mock_arm_node — serves /amr_robot_command service
  mock_wb_node  — serves wb_task action
  planner_node  — robocup_planner (subscriber + executor)

Usage:
  ros2 launch robocup_planner robocup_planner_sim.launch.py
  ros2 launch robocup_planner robocup_planner_sim.launch.py side:=b tier:=advanced stage:=lifecycle
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robocup_planner')
    params_file = os.path.join(pkg, 'config', 'params.yaml')

    side_arg  = DeclareLaunchArgument('side',  default_value='a',
                                      description='Arena side: a or b')
    tier_arg  = DeclareLaunchArgument('tier',  default_value='beginner',
                                      description='Tier: entry/beginner/advanced/expert')
    stage_arg = DeclareLaunchArgument('stage', default_value='production',
                                      description='Stage: production/recycling/lifecycle')

    side  = LaunchConfiguration('side')
    tier  = LaunchConfiguration('tier')
    stage = LaunchConfiguration('stage')

    order_server = Node(
        package='sml_system_pkg',
        executable='order_server',
        name='order_server',
        parameters=[
            params_file,
            {'start_side': side, 'tier': tier, 'stage': stage},
        ],
        output='screen',
    )

    mock_nav = Node(
        package='sml_system_pkg',
        executable='mock_nav_node',
        name='mock_nav_node',
        parameters=[
            params_file,
            {'side': side},
        ],
        output='screen',
    )

    mock_arm = Node(
        package='sml_system_pkg',
        executable='mock_arm_node',
        name='mock_arm_node',
        parameters=[params_file],
        output='screen',
    )

    mock_wb = Node(
        package='sml_system_pkg',
        executable='mock_wb_node',
        name='mock_wb_node',
        parameters=[params_file],
        output='screen',
    )

    planner = Node(
        package='robocup_planner',
        executable='planner_node',
        name='robocup_planner',
        parameters=[params_file, {'side': side}],
        output='screen',
    )

    return LaunchDescription([
        side_arg,
        tier_arg,
        stage_arg,
        order_server,
        mock_nav,
        mock_arm,
        mock_wb,
        planner,
    ])
