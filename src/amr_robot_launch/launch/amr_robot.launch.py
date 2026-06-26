"""AMR 로봇팔 pick-and-place 시스템 전체 실행 런치 파일.

실행되는 노드 (총 4개):
  - vision_node          (vision_pkg)          : 카메라 + YOLO 기반 타겟 pose 탐색, /get_target_pose
  - gripper_node         (arm_controller_pkg)  : Arduino 그리퍼 제어, /gripper/open, /gripper/grip
  - cargo_manager_node   (arm_controller_pkg)  : 슬롯 상태 관리, /cargo
  - amr_robot_node       (arm_controller_pkg)  : 로봇팔 제어 오케스트레이터, /amr_robot_command

amr_robot_node는 생성자에서 wait_for_service로 의존 서비스들을 자체적으로
기다리기 때문에, 노드 시작 순서가 엄격하게 보장되지 않아도 동작한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vision_visualize_arg = DeclareLaunchArgument(
        'vision_visualize',
        default_value='false',
        description='Show OpenCV visualization window for vision_node',
    )

    vision_node = Node(
        package='vision_pkg',
        executable='vision_node',
        name='vision_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'visualize': LaunchConfiguration('vision_visualize'),
        }],
    )

    gripper_node = Node(
        package='arm_controller_pkg',
        executable='gripper_node',
        name='gripper_node',
        output='screen',
        emulate_tty=True,
    )

    cargo_manager_node = Node(
        package='arm_controller_pkg',
        executable='cargo_manager_node',
        name='cargo_manager_node',
        output='screen',
        emulate_tty=True,
    )

    amr_robot_node = Node(
        package='arm_controller_pkg',
        executable='amr_robot_node',
        name='amr_robot_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        vision_visualize_arg,
        vision_node,
        gripper_node,
        cargo_manager_node,
        amr_robot_node,
    ])
