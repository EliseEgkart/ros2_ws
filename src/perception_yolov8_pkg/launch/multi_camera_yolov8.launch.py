from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    camera_names = ['front_up', 'front_down', 'left', 'right']
    
    yolo_nodes = []

    for name in camera_names:
        image_topic = f'/camera_{name}/image_raw'
        detection_topic = f'/detections/{name}'

        yolo_node = Node(
            package='perception_yolov8_pkg',
            executable='yolov8_node',
            name=f'yolov8_{name}',
            output='screen',
            parameters=[
                {'model': 'best.pt'},
                {'device': 'cuda:0'},
                {'threshold': 0.5},
                {'enable': True},
                {'image_reliability': 1},
                {'image_topic': image_topic},
            ],
            remappings=[
                ('detections', detection_topic)
            ]
        )

        yolo_nodes.append(yolo_node)

    ld.add_action(GroupAction(yolo_nodes))

    return ld
