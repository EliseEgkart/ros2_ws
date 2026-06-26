import os
from typing import Any, Dict, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class WaypointMarkerPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_marker_publisher')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('marker_topic', '/waypoint_markers')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('arrow_length', 0.45)
        self.declare_parameter('arrow_shaft_diameter', 0.07)
        self.declare_parameter('arrow_head_diameter', 0.16)
        self.declare_parameter('text_scale', 0.11)
        self.declare_parameter('text_z_offset', 0.35)
        self.declare_parameter('red', 1.0)
        self.declare_parameter('green', 0.0)
        self.declare_parameter('blue', 0.0)
        self.declare_parameter('alpha', 1.0)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        marker_topic = str(self.get_parameter('marker_topic').value)
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, qos)
        self._last_mtime: Optional[float] = None
        self._last_marker_array = MarkerArray()
        self._warned_missing = False

        period = float(self.get_parameter('publish_period_sec').value)
        self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'Waypoint marker publisher ready: topic="{marker_topic}", '
            f'file="{self._resolve_waypoints_file()}"'
        )

    def _timer_callback(self) -> None:
        path = self._resolve_waypoints_file()
        if not os.path.exists(path):
            if not self._warned_missing:
                self.get_logger().warn(f'Waypoints file not found: {path}')
                self._warned_missing = True
            return

        self._warned_missing = False
        mtime = os.path.getmtime(path)
        if self._last_mtime != mtime:
            self._last_marker_array = self._load_markers(path)
            self._last_mtime = mtime

        self._marker_pub.publish(self._last_marker_array)

    def _resolve_waypoints_file(self) -> str:
        configured = str(self.get_parameter('waypoints_file').value).strip()
        if configured:
            return os.path.abspath(os.path.expanduser(os.path.expandvars(configured)))

        source_candidate = os.path.abspath(
            os.path.join(
                os.getcwd(),
                'src',
                'robocup_navigator',
                'params',
                'stations_robocup.yaml',
            )
        )
        if os.path.exists(source_candidate):
            return source_candidate

        share_dir = get_package_share_directory('robocup_navigator')
        return os.path.join(share_dir, 'params', 'stations_robocup.yaml')

    def _load_markers(self, path: str) -> MarkerArray:
        try:
            with open(path, 'r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to read waypoint marker YAML: {exc}')
            return self._delete_all_markers()

        if not isinstance(data, dict):
            self.get_logger().error(f'YAML root must be a map: {path}')
            return self._delete_all_markers()

        frame_id = str(data.get('frame_id') or 'map')
        waypoints = data.get('waypoints', {})
        if not isinstance(waypoints, dict):
            self.get_logger().error('waypoints must be a map')
            return self._delete_all_markers()

        marker_array = self._delete_all_markers()
        marker_id = 1
        for name, entry in waypoints.items():
            pose = self._parse_waypoint(entry)
            if pose is None:
                self.get_logger().warn(f'Skipping invalid waypoint marker: {name}')
                continue

            marker_array.markers.append(
                self._make_arrow_marker(marker_id, frame_id, str(name), pose)
            )
            marker_id += 1
            marker_array.markers.append(
                self._make_text_marker(marker_id, frame_id, str(name), pose)
            )
            marker_id += 1

        self.get_logger().info(
            f'Loaded {len(marker_array.markers) - 1} waypoint markers from {path}'
        )
        return marker_array

    def _parse_waypoint(self, entry: Any) -> Optional[Dict[str, float]]:
        try:
            if 'pose' in entry:
                values = entry['pose']
                if len(values) != 7:
                    raise ValueError('pose list must have 7 elements')
                x, y, z, qx, qy, qz, qw = values
            else:
                position = entry.get('position', {})
                orientation = entry.get('orientation', {})
                x = position.get('x')
                y = position.get('y')
                z = position.get('z', 0.0)
                qx = orientation.get('x', 0.0)
                qy = orientation.get('y', 0.0)
                qz = orientation.get('z', 0.0)
                qw = orientation.get('w', 1.0)

            if x is None or y is None:
                raise ValueError('position.x and position.y are required')

            return {
                'x': float(x),
                'y': float(y),
                'z': float(z),
                'qx': float(qx),
                'qy': float(qy),
                'qz': float(qz),
                'qw': float(qw),
            }
        except Exception:
            return None

    def _make_arrow_marker(
        self,
        marker_id: int,
        frame_id: str,
        name: str,
        pose: Dict[str, float],
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, name, 'waypoint_arrows')
        marker.type = Marker.ARROW
        marker.scale.x = float(self.get_parameter('arrow_length').value)
        marker.scale.y = float(self.get_parameter('arrow_shaft_diameter').value)
        marker.scale.z = float(self.get_parameter('arrow_head_diameter').value)
        marker.pose.position.x = pose['x']
        marker.pose.position.y = pose['y']
        marker.pose.position.z = pose['z'] + 0.08
        marker.pose.orientation.x = pose['qx']
        marker.pose.orientation.y = pose['qy']
        marker.pose.orientation.z = pose['qz']
        marker.pose.orientation.w = pose['qw']
        return marker

    def _make_text_marker(
        self,
        marker_id: int,
        frame_id: str,
        name: str,
        pose: Dict[str, float],
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, name, 'waypoint_names')
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = name
        marker.scale.z = float(self.get_parameter('text_scale').value)
        marker.pose.position.x = pose['x']
        marker.pose.position.y = pose['y']
        marker.pose.position.z = pose['z'] + float(
            self.get_parameter('text_z_offset').value
        )
        marker.pose.orientation.w = 1.0
        return marker

    def _base_marker(
        self,
        marker_id: int,
        frame_id: str,
        name: str,
        namespace: str,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.color.r = float(self.get_parameter('red').value)
        marker.color.g = float(self.get_parameter('green').value)
        marker.color.b = float(self.get_parameter('blue').value)
        marker.color.a = float(self.get_parameter('alpha').value)
        marker.frame_locked = False
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        marker.text = name
        return marker

    def _delete_all_markers(self) -> MarkerArray:
        marker = Marker()
        marker.action = Marker.DELETEALL
        return MarkerArray(markers=[marker])


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()