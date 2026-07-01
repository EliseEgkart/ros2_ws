from pathlib import Path
from threading import Event, Lock
from typing import List, Optional

import math
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from nav2_msgs.action import FollowWaypoints
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from robocup_pkg.action import NavTask


class StationProfile:
    def __init__(self, station_id: int, name: str, sequence: List[str],
                 post_process: bool):
        self.station_id = station_id
        self.name = name
        self.sequence = sequence
        self.post_process = post_process


class RotationProfile:
    def __init__(self, waypoint_name: str, direction: str, angle_deg: float):
        self.waypoint_name = waypoint_name
        self.direction = direction
        self.angle_deg = angle_deg
        self.angle_rad = math.radians(angle_deg)


class RobocupNavigator(Node):
    """Action server that converts station IDs into stable Nav2 sequences."""

    def __init__(self, node_name='robocup_navigator',
                 enable_action_interfaces=True):
        super().__init__(node_name)
        self._cbg = ReentrantCallbackGroup()
        self._busy_lock = Lock()
        self._busy = False
        self._active_nav_goal_handle = None
        self._pending_post_process_profile: Optional[StationProfile] = None

        share_params_dir = (
            Path(get_package_share_directory('robocup_navigator')) / 'params'
        )
        source_params_dir = (
            Path.home() / 'ros2_ws/src/robocup_navigator/params'
        )
        default_yaml_path = source_params_dir / 'robocup_waypoint.yaml'
        if not default_yaml_path.exists():
            default_yaml_path = share_params_dir / 'robocup_waypoint.yaml'
        default_yaml = str(default_yaml_path)

        default_params_path = source_params_dir / 'robocup_navigator_params.yaml'
        if not default_params_path.exists():
            default_params_path = share_params_dir / 'robocup_navigator_params.yaml'
        default_params = self._load_default_params_file(default_params_path)

        def default_param(name: str, fallback):
            return default_params.get(name, fallback)

        default_rotation_profiles_path = (
            source_params_dir / 'robocup_rotation_profiles.yaml'
        )
        if not default_rotation_profiles_path.exists():
            default_rotation_profiles_path = (
                share_params_dir / 'robocup_rotation_profiles.yaml'
            )
        default_rotation_profiles_yaml = str(default_rotation_profiles_path)

        self.declare_parameter('stations_file', default_yaml)
        self.declare_parameter('frame_id', default_param('frame_id', 'map'))
        self.declare_parameter(
            'navigate_action_name',
            default_param('navigate_action_name', 'navigate_to_station'),
        )
        self.declare_parameter(
            'post_process_service_name',
            default_param(
                'post_process_service_name',
                '/robocup_navigator/post_process',
            ),
        )

        self.declare_parameter(
            'rotation_profiles_file',
            default_rotation_profiles_yaml,
        )

        self.declare_parameter(
            'follow_waypoints_action_name',
            default_param('follow_waypoints_action_name', 'follow_waypoints'),
        )
        self.declare_parameter(
            'follow_waypoints_server_timeout_sec',
            default_param('follow_waypoints_server_timeout_sec', 10.0),
        )
        self.declare_parameter(
            'nav_result_timeout_sec',
            default_param('nav_result_timeout_sec', 120.0),
        )

        self.declare_parameter(
            'cmd_vel_topic',
            default_param('cmd_vel_topic', 'cmd_vel'),
        )
        self.declare_parameter('scan_topic', default_param('scan_topic', '/scan'))
        self.declare_parameter(
            'front_edge_sample_count',
            default_param('front_edge_sample_count', 5),
        )
        self.declare_parameter(
            'front_edge_skip_count',
            default_param('front_edge_skip_count', 0),
        )
        self.declare_parameter(
            'scan_stale_timeout_sec',
            default_param('scan_stale_timeout_sec', 0.5),
        )
        self.declare_parameter(
            'front_min_valid_range',
            default_param('front_min_valid_range', 0.005),
        )
        self.declare_parameter(
            'front_max_valid_range',
            default_param('front_max_valid_range', 5.0),
        )
        self.declare_parameter('scan_debug', default_param('scan_debug', True))
        self.declare_parameter(
            'scan_debug_period_sec',
            default_param('scan_debug_period_sec', 1.0),
        )

        self.declare_parameter(
            'approach_after_goal',
            default_param('approach_after_goal', True),
        )
        self.declare_parameter(
            'target_front_distance',
            default_param('target_front_distance', 0.47),
        )
        self.declare_parameter(
            'target_distance_tolerance',
            default_param('target_distance_tolerance', 0.01),
        )
        self.declare_parameter(
            'approach_speed',
            default_param('approach_speed', 0.08),
        )
        self.declare_parameter(
            'approach_min_speed',
            default_param('approach_min_speed', 0.05),
        )
        self.declare_parameter(
            'approach_slowdown_distance',
            default_param('approach_slowdown_distance', 0.15),
        )
        self.declare_parameter(
            'approach_timeout_sec',
            default_param('approach_timeout_sec', 12.0),
        )
        self.declare_parameter(
            'fail_on_alignment_timeout',
            default_param('fail_on_alignment_timeout', False),
        )

        self.declare_parameter(
            'backup_after_goal',
            default_param('backup_after_goal', True),
        )
        self.declare_parameter(
            'backup_distance',
            default_param('backup_distance', 0.20),
        )
        self.declare_parameter(
            'backup_speed',
            default_param('backup_speed', 0.14),
        )
        self.declare_parameter(
            'backup_accel_limit',
            default_param('backup_accel_limit', 0.30),
        )
        self.declare_parameter(
            'backup_timeout_sec',
            default_param('backup_timeout_sec', 5.0),
        )
        self.declare_parameter(
            'post_backup_settle_sec',
            default_param('post_backup_settle_sec', 0.20),
        )

        self.declare_parameter(
            'rotate_after_backup',
            default_param('rotate_after_backup', True),
        )
        self.declare_parameter(
            'rotate_angular_speed',
            default_param('rotate_angular_speed', 1.4),
        )
        self.declare_parameter(
            'rotate_angular_accel_limit',
            default_param('rotate_angular_accel_limit', 1.50),
        )
        self.declare_parameter(
            'rotate_timeout_sec',
            default_param('rotate_timeout_sec', 8.0),
        )
        self.declare_parameter(
            'motion_period_sec',
            default_param('motion_period_sec', 0.05),
        )

        self._load_parameters()

        self._latest_front_distance: Optional[float] = None
        self._latest_scan_time = None
        self._latest_scan_frame_id: Optional[str] = None
        self._last_scan_debug_time = None
        self._last_front_candidates = []
        self._last_front_valid_distances = []
        self._last_front_median: Optional[float] = None

        self._waypoints_map, self._stations, self._frame_id = (
            self._load_station_file()
        )
        self._rotation_profiles = self._load_rotation_profile_file()

        follow_action = self.get_parameter(
            'follow_waypoints_action_name'
        ).value
        nav_action = self.get_parameter('navigate_action_name').value
        post_process_service = self.get_parameter(
            'post_process_service_name'
        ).value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self._follow_client = ActionClient(
            self,
            FollowWaypoints,
            follow_action,
            callback_group=self._cbg,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
            callback_group=self._cbg,
        )
        self._action_server = None
        self._post_process_srv = None
        if enable_action_interfaces:
            self._action_server = ActionServer(
                self,
                NavTask,
                nav_action,
                execute_callback=self._execute_callback,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._cbg,
            )
            self._post_process_srv = self.create_service(
                Trigger,
                post_process_service,
                self._post_process_callback,
                callback_group=self._cbg,
            )

        self.get_logger().info(
            f'Robocup navigator ready: node="{node_name}", '
            f'action_interfaces={enable_action_interfaces}, '
            f'action="{nav_action}", '
            f'post_process="{post_process_service}", '
            f'cmd_vel="{cmd_vel_topic}", '
            f'stations={sorted(self._stations.keys())}, scan="{scan_topic}"'
        )

    def _load_default_params_file(self, path: Path):
        if not path.exists():
            self.get_logger().warn(
                f'Navigator params file not found: {path}. '
                'Using built-in fallback defaults.'
            )
            return {}

        try:
            with path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to read navigator params file: {exc}. '
                'Using built-in fallback defaults.'
            )
            return {}

        params = {}
        if isinstance(data, dict):
            node_section = data.get('robocup_navigator', {})
            if isinstance(node_section, dict):
                raw_params = node_section.get('ros__parameters', {})
                if isinstance(raw_params, dict):
                    params = raw_params

        self.get_logger().info(
            f'Loaded navigator default params: {len(params)} from {path}'
        )
        return params

    def _load_parameters(self):
        self._follow_server_timeout_sec = float(
            self.get_parameter('follow_waypoints_server_timeout_sec').value
        )
        self._nav_result_timeout_sec = float(
            self.get_parameter('nav_result_timeout_sec').value
        )
        self._front_edge_sample_count = int(
            self.get_parameter('front_edge_sample_count').value
        )
        self._front_edge_skip_count = int(
            self.get_parameter('front_edge_skip_count').value
        )
        self._scan_stale_timeout_sec = float(
            self.get_parameter('scan_stale_timeout_sec').value
        )
        self._front_min_valid_range = float(
            self.get_parameter('front_min_valid_range').value
        )
        self._front_max_valid_range = float(
            self.get_parameter('front_max_valid_range').value
        )
        self._scan_debug = bool(self.get_parameter('scan_debug').value)
        self._scan_debug_period_sec = float(
            self.get_parameter('scan_debug_period_sec').value
        )
        self._approach_after_goal = bool(
            self.get_parameter('approach_after_goal').value
        )
        self._target_front_distance = float(
            self.get_parameter('target_front_distance').value
        )
        self._target_distance_tolerance = float(
            self.get_parameter('target_distance_tolerance').value
        )
        self._approach_speed = float(
            self.get_parameter('approach_speed').value
        )
        self._approach_min_speed = float(
            self.get_parameter('approach_min_speed').value
        )
        self._approach_slowdown_distance = float(
            self.get_parameter('approach_slowdown_distance').value
        )
        self._approach_timeout_sec = float(
            self.get_parameter('approach_timeout_sec').value
        )
        self._fail_on_alignment_timeout = bool(
            self.get_parameter('fail_on_alignment_timeout').value
        )
        self._backup_after_goal = bool(
            self.get_parameter('backup_after_goal').value
        )
        self._backup_distance = float(
            self.get_parameter('backup_distance').value
        )
        self._backup_speed = float(self.get_parameter('backup_speed').value)
        self._backup_accel_limit = float(
            self.get_parameter('backup_accel_limit').value
        )
        self._backup_timeout_sec = float(
            self.get_parameter('backup_timeout_sec').value
        )
        self._post_backup_settle_sec = float(
            self.get_parameter('post_backup_settle_sec').value
        )
        self._rotate_after_backup = bool(
            self.get_parameter('rotate_after_backup').value
        )

        self._rotate_angular_speed = float(
            self.get_parameter('rotate_angular_speed').value
        )
        self._rotate_angular_accel_limit = float(
            self.get_parameter('rotate_angular_accel_limit').value
        )
        self._rotate_timeout_sec = float(
            self.get_parameter('rotate_timeout_sec').value
        )
        self._motion_period_sec = float(
            self.get_parameter('motion_period_sec').value
        )

    def _load_rotation_profile_file(self):
        path = Path(
            str(self.get_parameter('rotation_profiles_file').value)
        ).expanduser()

        if not path.exists():
            self.get_logger().warn(
                f'Rotation profile file not found: {path}. '
                'Goal rotations are disabled.'
            )
            return {}

        try:
            with path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(
                f'Failed to read rotation profile file: {exc}. '
                'Goal rotations are disabled.'
            )
            return {}

        raw_profiles = (
            data.get('rotation_profiles', {})
            if isinstance(data, dict)
            else {}
        )
        profiles = {}
        for waypoint_name, entry in raw_profiles.items():
            if not self._is_goal_waypoint(waypoint_name):
                self.get_logger().warn(
                    f'Ignoring rotation profile for non-goal waypoint: '
                    f'{waypoint_name}'
                )
                continue
            if not isinstance(entry, dict):
                self.get_logger().warn(
                    f'Ignoring invalid rotation profile: {waypoint_name}'
                )
                continue
            direction = str(entry.get('direction', '')).lower()
            if direction not in ('clockwise', 'counterclockwise'):
                self.get_logger().warn(
                    f'Ignoring {waypoint_name}: invalid direction '
                    f'"{direction}".'
                )
                continue
            try:
                angle_deg = float(entry.get('angle_deg', 0.0))
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f'Ignoring {waypoint_name}: invalid angle_deg.'
                )
                continue
            if angle_deg <= 0.0:
                self.get_logger().warn(
                    f'Ignoring {waypoint_name}: angle_deg must be > 0.'
                )
                continue
            profiles[waypoint_name] = RotationProfile(
                waypoint_name,
                direction,
                angle_deg,
            )

        self.get_logger().info(
            f'Loaded waypoint rotation profiles: {len(profiles)} from {path}'
        )
        return profiles

    def _is_goal_waypoint(self, waypoint_name: str):
        return (
            waypoint_name.startswith('station_')
            and waypoint_name.endswith('_goal')
            and not waypoint_name.endswith('_sub_goal')
        )

    def _load_station_file(self):
        path = Path(
            str(self.get_parameter('stations_file').value)
        ).expanduser()
        frame_id = self.get_parameter('frame_id').value

        if not path.exists():
            self.get_logger().error(f'Station file not found: {path}')
            return {}, {}, frame_id

        try:
            with path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to read station file: {exc}')
            return {}, {}, frame_id

        if isinstance(data, dict) and data.get('frame_id'):
            frame_id = data['frame_id']

        waypoints_map = (
            data.get('waypoints', {}) if isinstance(data, dict) else {}
        )
        raw_stations = (
            data.get('stations', {}) if isinstance(data, dict) else {}
        )
        stations = {}

        for raw_id, entry in raw_stations.items():
            try:
                station_id = int(raw_id)
                name = str(entry.get('name', f'station_{station_id}'))
                sequence = list(entry.get('sequence', []))
                post_process = bool(entry.get('post_process', True))
            except Exception as exc:
                self.get_logger().error(
                    f'Invalid station entry "{raw_id}": {exc}'
                )
                continue

            missing = [name for name in sequence if name not in waypoints_map]
            if missing:
                self.get_logger().error(
                    f'Station {station_id} references missing waypoints: '
                    f'{missing}'
                )
                continue

            if not sequence:
                self.get_logger().warn(
                    f'Station {station_id} has empty sequence; ignored.'
                )
                continue

            stations[station_id] = StationProfile(
                station_id,
                name,
                sequence,
                post_process,
            )

        return waypoints_map, stations, frame_id

    def _goal_callback(self, goal_request):
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn(
                    f'Rejecting station {goal_request.station_id}: busy'
                )
                return GoalResponse.REJECT
            self._busy = True

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().warn(
            f'Cancel requested for station {goal_handle.request.station_id}'
        )
        nav_goal_handle = self._active_nav_goal_handle
        if nav_goal_handle is not None:
            nav_goal_handle.cancel_goal_async()
        self._publish_zero_velocity()
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        station_id = int(goal_handle.request.station_id)
        result = NavTask.Result()

        try:
            success, reason = self._run_station_sequence(
                goal_handle,
                station_id,
            )
            result.success = success
            result.fail_reason = reason

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.fail_reason = 'CANCELED'
                return result

            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            return result

        except Exception as exc:
            self.get_logger().error(
                f'Unhandled station navigation exception: {exc}'
            )
            self._publish_zero_velocity()
            result.success = False
            result.fail_reason = 'NAV_FAILED'
            goal_handle.abort()
            return result

        finally:
            self._active_nav_goal_handle = None
            self._publish_zero_velocity()
            with self._busy_lock:
                self._busy = False

    def _run_station_sequence(self, goal_handle, station_id: int):
        # Negative station_id = sub_goal phase only (approach leg).
        # Positive/zero station_id = goal phase only (docking leg).
        sub_goal_only = station_id < 0
        real_id = abs(station_id)

        profile = self._stations.get(real_id)
        if profile is None:
            self.get_logger().error(
                f'Unknown station_id={station_id} (resolved={real_id})'
            )
            return False, 'UNKNOWN_STATION'

        if sub_goal_only:
            sequence = [w for w in profile.sequence if w.endswith('_sub_goal')]
            phase_label = 'sub_goal'
        else:
            sequence = [w for w in profile.sequence if not w.endswith('_sub_goal')]
            phase_label = 'goal'

        self.get_logger().info(
            f'[STATION START] id={station_id}, name="{profile.name}", '
            f'phase={phase_label}, sequence={sequence}'
        )

        if not sequence:
            # No waypoints for this phase (e.g., station has no sub_goal).
            self._publish_feedback(
                goal_handle,
                f'ARRIVED station={station_id} name={profile.name}',
            )
            return True, ''

        self._publish_feedback(
            goal_handle,
            f'MOVING station={station_id} name={profile.name}',
        )

        for index, waypoint_name in enumerate(sequence):
            if goal_handle.is_cancel_requested:
                return False, 'CANCELED'

            ok, reason = self._navigate_to_waypoint(
                goal_handle,
                waypoint_name,
                index,
                len(sequence),
            )
            if not ok:
                return False, reason

        # Alignment and post-process are only needed at the final docking goal.
        if not sub_goal_only:
            if profile.post_process and self._approach_after_goal:
                ok, reason = self._run_front_alignment(goal_handle, profile)
                if not ok:
                    return False, reason

            self._pending_post_process_profile = (
                profile if profile.post_process else None
            )

        self._publish_feedback(
            goal_handle,
            f'ARRIVED station={station_id} name={profile.name}',
        )
        self.get_logger().info(
            f'[STATION DONE] id={station_id}, name="{profile.name}"'
        )
        return True, ''

    def _post_process_callback(self, request, response):
        del request

        with self._busy_lock:
            if self._busy:
                response.success = False
                response.message = 'BUSY'
                return response
            self._busy = True

        try:
            profile = self._pending_post_process_profile
            if profile is None:
                response.success = True
                response.message = 'NO_PENDING_POST_PROCESS'
                return response

            ok, reason = self._run_departure_post_process(profile)
            if ok:
                self._pending_post_process_profile = None
                response.success = True
                response.message = ''
            else:
                response.success = False
                response.message = reason

            return response

        except Exception as exc:
            self.get_logger().error(f'Post-process exception: {exc}')
            self._publish_zero_velocity()
            response.success = False
            response.message = 'POST_PROCESS_FAILED'
            return response

        finally:
            self._publish_zero_velocity()
            with self._busy_lock:
                self._busy = False

    def _navigate_to_waypoint(self, goal_handle, waypoint_name: str,
                              index: int, total: int):
        pose = self._entry_to_pose(waypoint_name)
        if pose is None:
            return False, 'NAV_FAILED'

        if not self._follow_client.wait_for_server(
            timeout_sec=self._follow_server_timeout_sec
        ):
            self.get_logger().error(
                'FollowWaypoints action server unavailable.'
            )
            return False, 'NAV_FAILED'

        self._publish_feedback(
            goal_handle,
            f'MOVING waypoint={waypoint_name} {index + 1}/{total}',
        )
        self.get_logger().info(
            f'[NAV START] waypoint="{waypoint_name}" {index + 1}/{total}'
        )

        nav_goal = FollowWaypoints.Goal()
        nav_goal.poses = [pose]

        done = Event()
        state = {
            'accepted': False,
            'done': False,
            'missed': [],
            'exception': None,
        }

        def on_goal_response(future):
            try:
                nav_goal_handle = future.result()
                self._active_nav_goal_handle = nav_goal_handle
                state['accepted'] = bool(nav_goal_handle.accepted)
                if not nav_goal_handle.accepted:
                    done.set()
                    return

                result_future = nav_goal_handle.get_result_async()
                result_future.add_done_callback(on_result)
            except Exception as exc:
                state['exception'] = exc
                done.set()

        def on_result(future):
            try:
                wrapped = future.result()
                state['status'] = wrapped.status
                state['missed'] = list(wrapped.result.missed_waypoints)
                state['done'] = True
            except Exception as exc:
                state['exception'] = exc
            finally:
                done.set()

        send_future = self._follow_client.send_goal_async(nav_goal)
        send_future.add_done_callback(on_goal_response)

        deadline = time.monotonic() + self._nav_result_timeout_sec
        while not done.is_set():
            if goal_handle.is_cancel_requested:
                nav_goal_handle = self._active_nav_goal_handle
                if nav_goal_handle is not None:
                    nav_goal_handle.cancel_goal_async()
                return False, 'CANCELED'

            if time.monotonic() >= deadline:
                self.get_logger().error(
                    f'[NAV TIMEOUT] waypoint="{waypoint_name}"'
                )
                nav_goal_handle = self._active_nav_goal_handle
                if nav_goal_handle is not None:
                    nav_goal_handle.cancel_goal_async()
                return False, 'TIMEOUT'

            time.sleep(self._motion_period_sec)

        self._active_nav_goal_handle = None

        if state['exception'] is not None:
            self.get_logger().error(
                f'[NAV ERROR] waypoint="{waypoint_name}": '
                f'{state["exception"]}'
            )
            return False, 'NAV_FAILED'

        if not state['accepted']:
            self.get_logger().error(
                f'[NAV REJECTED] waypoint="{waypoint_name}"'
            )
            return False, 'NAV_FAILED'

        if state['missed']:
            self.get_logger().error(
                f'[NAV MISSED] waypoint="{waypoint_name}", '
                f'missed={state["missed"]}'
            )
            return False, 'NAV_FAILED'

        self.get_logger().info(f'[NAV DONE] waypoint="{waypoint_name}"')
        return True, ''

    def _entry_to_pose(self, name: str) -> Optional[PoseStamped]:
        entry = self._waypoints_map.get(name)
        if entry is None:
            self.get_logger().error(f'Waypoint "{name}" not found.')
            return None

        try:
            if 'pose' in entry:
                pose_list = entry['pose']
                if len(pose_list) != 7:
                    raise ValueError('pose list must have 7 elements')
                x, y, z, qx, qy, qz, qw = pose_list
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

            return PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self._frame_id,
                ),
                pose=Pose(
                    position=Point(x=float(x), y=float(y), z=float(z)),
                    orientation=Quaternion(
                        x=float(qx),
                        y=float(qy),
                        z=float(qz),
                        w=float(qw),
                    ),
                ),
            )
        except Exception as exc:
            self.get_logger().error(f'Invalid waypoint "{name}": {exc}')
            return None

    def _run_departure_post_process(self, profile: StationProfile):
        if self._backup_after_goal:
            ok, reason = self._run_backup(None, profile)
            if not ok:
                return False, reason

        if self._rotate_after_backup:
            waypoint_name = self._last_goal_waypoint(profile)
            ok, reason = self._run_rotation(None, profile, waypoint_name)
            if not ok:
                return False, reason

        return True, ''

    def _last_goal_waypoint(self, profile: StationProfile):
        for waypoint_name in reversed(profile.sequence):
            if self._is_goal_waypoint(waypoint_name):
                return waypoint_name
        return None

    def _run_front_alignment(self, goal_handle, profile: StationProfile):
        self._publish_zero_velocity()
        start = time.monotonic()

        self._publish_feedback(
            goal_handle,
            f'ALIGNING station={profile.station_id} name={profile.name}',
        )
        self.get_logger().info(
            f'[ALIGN START] station={profile.station_id}, '
            f'target={self._target_front_distance:.3f} m'
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_zero_velocity()
                return False, 'CANCELED'

            elapsed = time.monotonic() - start
            front_distance = self._get_recent_front_distance()

            if front_distance is None:
                self._publish_zero_velocity()
                if elapsed >= self._approach_timeout_sec:
                    self.get_logger().warn(
                        '[ALIGN TIMEOUT] valid scan data unavailable.'
                    )
                    if self._fail_on_alignment_timeout:
                        return False, 'TIMEOUT'
                    return True, ''
                time.sleep(self._motion_period_sec)
                continue

            error = front_distance - self._target_front_distance
            if abs(error) <= self._target_distance_tolerance:
                self.get_logger().info(
                    f'[ALIGN DONE] front_distance={front_distance:.3f} m'
                )
                self._publish_zero_velocity()
                return True, ''

            if elapsed >= self._approach_timeout_sec:
                self.get_logger().warn(
                    f'[ALIGN TIMEOUT] front_distance={front_distance:.3f} m, '
                    f'target={self._target_front_distance:.3f} m'
                )
                self._publish_zero_velocity()
                if self._fail_on_alignment_timeout:
                    return False, 'TIMEOUT'
                return True, ''

            speed = self._compute_alignment_speed(abs(error))
            cmd = Twist()
            cmd.linear.x = speed if error > 0.0 else -speed
            self._cmd_vel_pub.publish(cmd)
            time.sleep(self._motion_period_sec)

        return False, 'NAV_FAILED'

    def _run_backup(self, goal_handle, profile: StationProfile):
        if self._backup_distance <= 0.0 or self._backup_speed <= 0.0:
            self.get_logger().warn('Backup skipped: invalid distance/speed.')
            return True, ''

        self._publish_zero_velocity()

        if goal_handle is not None:
            self._publish_feedback(
                goal_handle,
                f'BACKING_UP station={profile.station_id} name={profile.name}',
            )
        self.get_logger().info(
            f'[BACKUP START] distance={self._backup_distance:.3f} m, '
            f'speed={self._backup_speed:.3f} m/s, '
            f'accel_limit={self._backup_accel_limit:.3f} m/s^2'
        )

        if not self._publish_linear_motion_profile(
            -abs(self._backup_distance),
            abs(self._backup_speed),
            self._backup_accel_limit,
            self._backup_timeout_sec,
            'BACKUP',
            goal_handle,
        ):
            return False, 'CANCELED'

        if self._post_backup_settle_sec > 0.0:
            self._publish_zero_velocity()
            time.sleep(self._post_backup_settle_sec)

        self.get_logger().info('[BACKUP DONE]')
        return True, ''

    def _run_rotation(self, goal_handle, profile: StationProfile,
                      waypoint_name: Optional[str]):
        if not waypoint_name:
            self.get_logger().info(
                f'Rotation skipped: station={profile.station_id} has no '
                'goal waypoint.'
            )
            return True, ''

        rotation_profile = self._rotation_profiles.get(waypoint_name)
        if rotation_profile is None:
            self.get_logger().info(
                f'Rotation skipped: no profile for waypoint={waypoint_name}.'
            )
            return True, ''

        if (
                rotation_profile.angle_rad <= 0.0
                or self._rotate_angular_speed <= 0.0):
            self.get_logger().warn(
                f'Rotation skipped for {waypoint_name}: invalid angle/speed.'
            )
            return True, ''

        self._publish_zero_velocity()

        direction_text = (
            'ROTATING_CLOCKWISE'
            if rotation_profile.direction == 'clockwise'
            else 'ROTATING_COUNTERCLOCKWISE'
        )

        if goal_handle is not None:
            self._publish_feedback(
                goal_handle,
                f'{direction_text} station={profile.station_id} '
                f'name={profile.name} waypoint={waypoint_name}',
            )

        self.get_logger().info(
            f'[ROTATE START] waypoint={waypoint_name}, '
            f'direction={rotation_profile.direction}, '
            f'angle={rotation_profile.angle_deg:.1f} deg, '
            f'speed={self._rotate_angular_speed:.3f} rad/s, '
            f'accel_limit={self._rotate_angular_accel_limit:.3f} rad/s^2'
        )

        if rotation_profile.direction == 'clockwise':
            angle_rad = -abs(rotation_profile.angle_rad)
        else:
            angle_rad = abs(rotation_profile.angle_rad)

        if not self._publish_angular_motion_profile(
            angle_rad,
            abs(self._rotate_angular_speed),
            self._rotate_angular_accel_limit,
            self._rotate_timeout_sec,
            'ROTATE',
            goal_handle,
        ):
            return False, 'CANCELED'

        self.get_logger().info('[ROTATE DONE]')
        return True, ''

    def _scan_callback(self, msg: LaserScan):
        front_distance = self._get_front_distance(msg)
        self._scan_debug_log(msg, front_distance)

        if front_distance is not None:
            self._latest_front_distance = front_distance
            self._latest_scan_time = self.get_clock().now()
            self._latest_scan_frame_id = msg.header.frame_id

    def _get_front_distance(self, msg: LaserScan) -> Optional[float]:
        if not msg.ranges:
            self._last_front_candidates = []
            self._last_front_valid_distances = []
            self._last_front_median = None
            return None

        ranges = list(msg.ranges)
        total_count = len(ranges)
        sample_count = max(1, self._front_edge_sample_count)
        skip_count = max(0, self._front_edge_skip_count)
        required_count = (sample_count + skip_count) * 2

        if total_count < required_count:
            self._last_front_candidates = []
            self._last_front_valid_distances = []
            self._last_front_median = None
            return None

        front_candidates = []
        front_candidates.extend(ranges[skip_count: skip_count + sample_count])

        if skip_count == 0:
            front_candidates.extend(ranges[-sample_count:])
        else:
            front_candidates.extend(
                ranges[-skip_count - sample_count: -skip_count]
            )

        valid_distances = []
        for raw_distance in front_candidates:
            if not math.isfinite(raw_distance):
                continue
            if raw_distance < self._front_min_valid_range:
                continue
            if raw_distance > self._front_max_valid_range:
                continue
            valid_distances.append(raw_distance)

        self._last_front_candidates = front_candidates
        self._last_front_valid_distances = list(valid_distances)

        if not valid_distances:
            self._last_front_median = None
            return None

        valid_distances.sort()
        mid = len(valid_distances) // 2
        if len(valid_distances) % 2 == 1:
            median_distance = valid_distances[mid]
        else:
            median_distance = 0.5 * (
                valid_distances[mid - 1] + valid_distances[mid]
            )

        self._last_front_valid_distances = valid_distances
        self._last_front_median = median_distance
        return median_distance

    def _get_recent_front_distance(self) -> Optional[float]:
        if self._latest_scan_time is None:
            return None

        age = (
            self.get_clock().now() - self._latest_scan_time
        ).nanoseconds / 1e9

        if age > self._scan_stale_timeout_sec:
            return None

        return self._latest_front_distance

    def _scan_debug_log(self, msg: LaserScan,
                        front_distance: Optional[float]):
        if not self._scan_debug:
            return

        now = self.get_clock().now()
        if self._last_scan_debug_time is not None:
            elapsed = (
                now - self._last_scan_debug_time
            ).nanoseconds / 1e9
            if elapsed < self._scan_debug_period_sec:
                return

        self._last_scan_debug_time = now
        candidates = [
            round(v, 3) if math.isfinite(v) else 'inf'
            for v in self._last_front_candidates
        ]
        valid = [round(v, 3) for v in self._last_front_valid_distances]

        self.get_logger().info(
            f'[SCAN MEDIAN DEBUG] frame={msg.header.frame_id}, '
            f'ranges_len={len(msg.ranges)}, '
            f'front_candidates={candidates}, '
            f'valid_sorted={valid}, '
            f'median={self._last_front_median}, '
            f'front_distance={front_distance}'
        )

    def _compute_alignment_speed(self, abs_error: float) -> float:
        if abs_error <= self._target_distance_tolerance:
            return 0.0

        slowdown = max(
            self._approach_slowdown_distance,
            self._target_distance_tolerance + 0.001,
        )
        if abs_error >= slowdown:
            return abs(self._approach_speed)

        ratio = abs_error / slowdown
        speed = abs(self._approach_speed) * ratio
        return max(
            abs(self._approach_min_speed),
            min(speed, abs(self._approach_speed)),
        )

    def _limit_motion_duration(self, duration_sec: float, timeout_sec: float,
                               label: str) -> float:
        if timeout_sec <= 0.0:
            return max(0.0, duration_sec)

        if duration_sec > timeout_sec:
            self.get_logger().warn(
                f'[{label} TIMEOUT LIMIT] required={duration_sec:.2f}s, '
                f'timeout={timeout_sec:.2f}s'
            )
            return timeout_sec

        return max(0.0, duration_sec)

    def _motion_profile_segments(self, distance: float, max_speed: float,
                                 accel_limit: float, label: str):
        distance = abs(float(distance))
        max_speed = abs(float(max_speed))
        accel_limit = abs(float(accel_limit))

        if distance <= 0.0 or max_speed <= 0.0:
            return 0.0, 0.0, 0.0, 0.0

        if accel_limit <= 0.0:
            self.get_logger().warn(
                f'[{label}] accel_limit <= 0.0; using constant velocity.'
            )
            return 0.0, distance / max_speed, 0.0, max_speed

        full_ramp_time = max_speed / accel_limit
        full_ramp_distance = 0.5 * accel_limit * full_ramp_time ** 2

        if 2.0 * full_ramp_distance >= distance:
            peak_speed = math.sqrt(distance * accel_limit)
            ramp_time = peak_speed / accel_limit
            cruise_time = 0.0
        else:
            peak_speed = max_speed
            ramp_time = full_ramp_time
            cruise_distance = distance - 2.0 * full_ramp_distance
            cruise_time = cruise_distance / peak_speed

        total_time = 2.0 * ramp_time + cruise_time
        return ramp_time, cruise_time, total_time, peak_speed

    def _profile_speed_at(self, elapsed: float, ramp_time: float,
                          cruise_time: float, total_time: float,
                          peak_speed: float) -> float:
        if total_time <= 0.0:
            return 0.0

        if ramp_time <= 0.0:
            return peak_speed

        if elapsed < ramp_time:
            return peak_speed * elapsed / ramp_time

        cruise_end = ramp_time + cruise_time
        if elapsed < cruise_end:
            return peak_speed

        remaining = max(0.0, total_time - elapsed)
        return peak_speed * remaining / ramp_time

    def _publish_linear_motion_profile(self, distance: float, max_speed: float,
                                       accel_limit: float, timeout_sec: float,
                                       label: str, goal_handle) -> bool:
        sign = 1.0 if distance >= 0.0 else -1.0
        ramp_time, cruise_time, total_time, peak_speed = (
            self._motion_profile_segments(
                distance,
                max_speed,
                accel_limit,
                label,
            )
        )
        duration_sec = self._limit_motion_duration(
            total_time,
            timeout_sec,
            label,
        )
        return self._publish_profiled_velocity(
            duration_sec,
            ramp_time,
            cruise_time,
            total_time,
            peak_speed,
            sign,
            True,
            goal_handle,
        )

    def _publish_angular_motion_profile(self, angle_rad: float,
                                        max_angular_speed: float,
                                        accel_limit: float,
                                        timeout_sec: float, label: str,
                                        goal_handle) -> bool:
        sign = 1.0 if angle_rad >= 0.0 else -1.0
        ramp_time, cruise_time, total_time, peak_speed = (
            self._motion_profile_segments(
                angle_rad,
                max_angular_speed,
                accel_limit,
                label,
            )
        )
        duration_sec = self._limit_motion_duration(
            total_time,
            timeout_sec,
            label,
        )
        return self._publish_profiled_velocity(
            duration_sec,
            ramp_time,
            cruise_time,
            total_time,
            peak_speed,
            sign,
            False,
            goal_handle,
        )

    def _publish_profiled_velocity(self, duration_sec: float,
                                   ramp_time: float, cruise_time: float,
                                   total_time: float, peak_speed: float,
                                   sign: float, linear: bool,
                                   goal_handle) -> bool:
        start = time.monotonic()
        deadline = start + duration_sec

        try:
            while rclpy.ok():
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    return False

                now = time.monotonic()
                remaining = deadline - now
                if remaining <= 0.0:
                    return True

                elapsed = now - start
                speed = sign * self._profile_speed_at(
                    elapsed,
                    ramp_time,
                    cruise_time,
                    total_time,
                    peak_speed,
                )
                cmd = Twist()
                if linear:
                    cmd.linear.x = speed
                else:
                    cmd.angular.z = speed
                self._cmd_vel_pub.publish(cmd)
                time.sleep(min(self._motion_period_sec, remaining))

            return False
        finally:
            self._publish_zero_velocity()

    def _publish_velocity_for_duration(self, linear_x: float, angular_z: float,
                                       duration_sec: float, goal_handle) -> bool:
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        deadline = time.monotonic() + duration_sec

        try:
            while rclpy.ok():
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    return False

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return True

                self._cmd_vel_pub.publish(cmd)
                time.sleep(min(self._motion_period_sec, remaining))

            return False
        finally:
            self._publish_zero_velocity()

    def _publish_feedback(self, goal_handle, status: str):
        if goal_handle is None:
            return

        feedback = NavTask.Feedback()
        feedback.status = status
        goal_handle.publish_feedback(feedback)

    def _publish_zero_velocity(self):
        try:
            if rclpy.ok():
                self._cmd_vel_pub.publish(Twist())
        except Exception:
            pass

    def destroy_node(self):
        nav_goal_handle = self._active_nav_goal_handle
        if nav_goal_handle is not None:
            nav_goal_handle.cancel_goal_async()
        self._publish_zero_velocity()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobocupNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
