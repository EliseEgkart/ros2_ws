from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # 1) YOLO 추론 런치 포함
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('perception_yolov8_pkg') + '/launch/multi_camera_yolov8.launch.py'
            )
        )
    )

    # 2) 디버그 비주얼라이저 노드 정의
    cameras = [
        ('front_up',   '/camera_front_up/image_raw',   '/detections/front_up'),
        ('front_down', '/camera_front_down/image_raw', '/detections/front_down'),
        ('left',       '/camera_left/image_raw',       '/detections/left'),
        ('right',      '/camera_right/image_raw',      '/detections/right'),
    ]

    for name, img_topic, det_topic in cameras:
        ld.add_action(
            Node(
                package='debug_pkg',
                executable='yolov8_visualizer_node',
                name=f'yolov8_visualizer_{name}',
                output='screen',
                parameters=[
                    {'image_topic': img_topic},
                    {'detections_topic': det_topic},
                    {'image_reliability': 1},  # ← 정수형으로 지정
                ],
                remappings=[
                    ('yolov8_visualized_img', f'/yolov8/{name}/visualized_img'),
                    ('dgb_bb_markers', f'/yolov8/{name}/bb_markers'),
                    ('dgb_kp_markers', f'/yolov8/{name}/kp_markers'),
                ],
            )
        )

    return ld
