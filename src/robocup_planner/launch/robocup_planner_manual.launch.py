"""
robocup_planner_manual.launch.py

Same simulated backend as robocup_planner_sim.launch.py (mock nav/arm/wb +
planner), plus the visualization GUI, but WITHOUT the auto-publishing
order_server — intended to be paired with a manually-run interactive task
publisher in a separate terminal, e.g.:

  ros2 run eai_task_server manual_order_server
  ros2 run eai_task_server task_complexity_publisher --ros-args -p interactive:=true

Usage:
  ros2 launch robocup_planner robocup_planner_manual.launch.py
  ros2 launch robocup_planner robocup_planner_manual.launch.py side:=b gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robocup_planner')
    params_file = os.path.join(pkg, 'config', 'params.yaml')

    side_arg = DeclareLaunchArgument('side', default_value='a',
                                      description='Arena side: a or b')
    gui_arg = DeclareLaunchArgument('gui', default_value='true',
                                     description='Launch sml_worldcup_gui alongside the stack')

    side = LaunchConfiguration('side')
    gui = LaunchConfiguration('gui')

    mock_nav = Node(
        package='sml_system_pkg',
        executable='mock_nav_node',
        name='mock_nav_node',
        parameters=[params_file, {'side': side}],
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

    worldcup_gui = Node(
        package='sml_worldcup_gui',
        executable='sml_worldcup_gui',
        name='sml_worldcup_gui',
        parameters=[{'side': side}],
        output='screen',
        condition=IfCondition(gui),
    )

    return LaunchDescription([
        side_arg,
        gui_arg,
        mock_nav,
        mock_arm,
        mock_wb,
        planner,
        worldcup_gui,
    ])
