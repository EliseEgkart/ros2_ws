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
