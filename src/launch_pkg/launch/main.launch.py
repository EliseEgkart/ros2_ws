'''
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 각 패키지 디렉토리 경로 가져오기
    yolov8_pkg_dir = get_package_share_directory('perception_yolov8_pkg')
    debug_pkg_dir = get_package_share_directory('debug_pkg')
    usb_cam_pkg_dir = get_package_share_directory('usb_cam')

    # 개별 런치 포함
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolov8_pkg_dir, 'launch', 'multi_camera_yolov8.launch.py')
        )
    )

    debug_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(debug_pkg_dir, 'launch', 'multi_camera_yolov8_debug.launch.py')
        )
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usb_cam_pkg_dir, 'launch', 'multiple_camera.launch.py')
        )
    )

    # LaunchDescription 객체에 모두 추가
    return LaunchDescription([
        camera_launch,
        yolov8_launch,
        debug_launch,
    ])
'''
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 경로
    usb_cam_dir      = get_package_share_directory('usb_cam')
    yolov8_dir       = get_package_share_directory('perception_yolov8_pkg')
    debug_dir        = get_package_share_directory('debug_pkg')
    sensor_init_dir  = get_package_share_directory('sensor_initialize')
    velodyne_driver_dir  = get_package_share_directory('velodyne_driver')
    velodyne_pointcloud_dir  = get_package_share_directory('velodyne_pointcloud')
    # lidar_pre_dir    = get_package_share_directory('lidar_preprocessing_pkg')  # 전처리 패키지
    # fusion_pkg_dir   = get_package_share_directory('sensor_fusion_pkg')

    return LaunchDescription([

        # 0) 라이다 드라이버 퍼블리셔
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(velodyne_driver_dir, 'launch', 'velodyne_driver_node-VLP16-launch.py')
            )
        ),
        # 0) 라이다 포인트클라우드 퍼블리셔
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(velodyne_pointcloud_dir, 'launch', 'velodyne_transform_node-VLP16-launch.py')
            )
        ),

        # 1) 카메라 퍼블리셔
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(usb_cam_dir, 'launch', 'multiple_camera.launch.py')
            )
        ),

        # 2) YOLOv8 추론
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(yolov8_dir, 'launch', 'multi_camera_yolov8.launch.py')
            )
        ),

        # 3) 디버그 시각화
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(debug_dir, 'launch', 'multi_camera_yolov8_debug.launch.py')
            )
        ),

        # 4) TF 브로드캐스트 (카메라 ⇆ LiDAR)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensor_init_dir, 'launch', 'sensor_tf.launch.py')
            )
        ),

        # 5) LiDAR 전처리
        Node(
            package='sensor_initialize',
            executable='lidar_preprocessing_node',
            name='lidar_preprocessor',
            output='screen',
            parameters=[{
                # 필요하면 파라미터 여기
            }]
        ),

         # 6) 센서 융합 노드
         Node(
             package='sensor_fusion_pkg',
             executable='bbox_projector_node',
             name='bbox_projector',
             output='screen',
             parameters=[{
                 'camera_frame':   'camera_front_up',
                 'camera_frame':   'camera_front_up_frame',
                 'detection_topic': '/detections/front_up',
                 'lidar_topic':    '/lidar_preprocessed',
                 'lidar_frame':    'velodyne'
             }]
         ),
    ])

