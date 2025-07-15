from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    camera_pkg_dir = get_package_share_directory('camera_perception_pkg')
    decision_pkg_dir = get_package_share_directory('decision_making_pkg')

    return LaunchDescription([
        # 개별 노드 실행
        Node(
            package='decision_making_pkg',
            executable='motion_planner_node',
            name='motion_planner_node',
            output='screen',
            #parameters=[{'param_name': 'param_value'}],
            #remappings=[('/old/topic', '/new/topic')]
        ),

        # camera_perception_pkg의 launch 파일 포함 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(camera_pkg_dir, 'launch', 'camera_group.launch.py')
            )
        ),

        # decision_making_pkg의 launch 파일 포함 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(decision_pkg_dir, 'launch', 'path_planner.launch.py')
            )
        ),
    ])
