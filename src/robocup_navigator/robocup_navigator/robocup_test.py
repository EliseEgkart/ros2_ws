from pathlib import Path
from typing import List, Optional

import re
import threading
import time

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from robocup_navigator.navigator import RobocupNavigator, StationProfile


class _TestGoalHandle:
    def __init__(self, node, cancel_checker):
        self._node = node
        self._cancel_checker = cancel_checker

    @property
    def is_cancel_requested(self):
        return self._cancel_checker()

    def publish_feedback(self, feedback):
        self._node.get_logger().info(f'[TEST FEEDBACK] {feedback.status}')


class RobocupTestNavigator(RobocupNavigator):
    """Autonomous full-sequence test runner with docking/departure motions."""

    def __init__(self):
        super().__init__(
            node_name='robocup_test',
            enable_action_interfaces=False,
        )

        self.declare_parameter('sequence', '')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('repeat', False)
        self.declare_parameter('repeat_delay_sec', 0.0)
        self.declare_parameter('continue_on_miss', False)
        self.declare_parameter('honor_station_post_process', True)
        self.declare_parameter('status_topic', '')

        status_topic = str(self.get_parameter('status_topic').value)
        self._status_pub = (
            self.create_publisher(String, status_topic, 10)
            if status_topic
            else None
        )

        self._stop_requested = False
        self._run_thread = None
        self._test_goal_handle = _TestGoalHandle(
            self,
            lambda: self._stop_requested,
        )
        self._sequence = self._load_full_sequence()

        self.get_logger().info(
            'Robocup test runner ready: '
            f'waypoints={len(self._sequence)}, sequence={self._sequence}'
        )

        self._auto_start_timer = None
        if bool(self.get_parameter('auto_start').value):
            self._auto_start_timer = self.create_timer(
                0.5,
                self._auto_start_once,
                callback_group=self._cbg,
            )
        else:
            self.get_logger().info('auto_start is false; waiting idle.')

    def _auto_start_once(self):
        if self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None
        self.start()

    def start(self):
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('Robocup test is already running.')
                return
            self._busy = True

        self._stop_requested = False
        self._run_thread = threading.Thread(
            target=self._run_worker,
            daemon=True,
        )
        self._run_thread.start()

    def stop(self):
        self._stop_requested = True
        nav_goal_handle = self._active_nav_goal_handle
        if nav_goal_handle is not None:
            nav_goal_handle.cancel_goal_async()
        self._publish_zero_velocity()

    def _run_worker(self):
        try:
            while rclpy.ok() and not self._stop_requested:
                ok, reason = self._run_full_sequence_once()
                if ok:
                    self._publish_status('test_done')
                    self.get_logger().info('[TEST DONE] Full sequence complete.')
                else:
                    self._publish_status(f'test_failed:{reason}')
                    self.get_logger().error(
                        f'[TEST FAILED] reason={reason}'
                    )

                if (
                        not ok
                        or not bool(self.get_parameter('repeat').value)
                        or self._stop_requested):
                    break

                delay = float(self.get_parameter('repeat_delay_sec').value)
                self.get_logger().info(
                    f'[TEST REPEAT] Restarting after {delay:.2f}s.'
                )
                self._sleep_with_stop(delay)

        except Exception as exc:
            self.get_logger().error(f'Unhandled robocup test exception: {exc}')
            self._publish_status('test_failed:EXCEPTION')
        finally:
            self._publish_zero_velocity()
            with self._busy_lock:
                self._busy = False

    def _run_full_sequence_once(self):
        if not self._sequence:
            return False, 'EMPTY_SEQUENCE'

        self._publish_status('test_start')
        total = len(self._sequence)
        self.get_logger().info(
            f'[TEST START] current -> {" -> ".join(self._sequence)}'
        )

        for index, waypoint_name in enumerate(self._sequence):
            if self._stop_requested:
                return False, 'CANCELED'

            ok, reason = self._navigate_to_waypoint(
                self._test_goal_handle,
                waypoint_name,
                index,
                total,
            )
            if not ok:
                if bool(self.get_parameter('continue_on_miss').value):
                    self.get_logger().warn(
                        f'[TEST CONTINUE] waypoint={waypoint_name}, '
                        f'reason={reason}'
                    )
                    continue
                return False, reason

            if self._is_goal_waypoint(waypoint_name):
                ok, reason = self._run_goal_post_process(waypoint_name)
                if not ok:
                    return False, reason

        return True, ''

    def _run_goal_post_process(self, waypoint_name: str):
        profile = self._profile_for_goal_waypoint(waypoint_name)
        if profile is None:
            self.get_logger().warn(
                f'[POST SKIP] No station profile for {waypoint_name}.'
            )
            return True, ''

        honor_post_process = bool(
            self.get_parameter('honor_station_post_process').value
        )
        if honor_post_process and not profile.post_process:
            self.get_logger().info(
                f'[POST SKIP] station={profile.station_id} '
                'post_process=false.'
            )
            return True, ''

        if self._approach_after_goal:
            ok, reason = self._run_front_alignment(
                self._test_goal_handle,
                profile,
            )
            if not ok:
                return False, reason

        if self._backup_after_goal:
            ok, reason = self._run_backup(self._test_goal_handle, profile)
            if not ok:
                return False, reason

        if self._rotate_after_backup:
            ok, reason = self._run_rotation(
                self._test_goal_handle,
                profile,
                waypoint_name,
            )
            if not ok:
                return False, reason

        return True, ''

    def _profile_for_goal_waypoint(
            self,
            waypoint_name: str,
    ) -> Optional[StationProfile]:
        station_id = self._station_id_from_goal_waypoint(waypoint_name)
        if station_id is not None:
            profile = self._stations.get(station_id)
            if profile is not None and waypoint_name in profile.sequence:
                return profile

        for profile in self._stations.values():
            if waypoint_name in profile.sequence:
                return profile

        if station_id is None:
            return None

        return StationProfile(
            station_id=station_id,
            name=f'station_{station_id}',
            sequence=[waypoint_name],
            post_process=True,
        )

    def _station_id_from_goal_waypoint(self, waypoint_name: str) -> Optional[int]:
        match = re.match(r'^station_(\d+)_goal$', waypoint_name)
        if match is None:
            return None
        return int(match.group(1))

    def _load_full_sequence(self) -> List[str]:
        seq_param = self.get_parameter('sequence').value
        if seq_param:
            sequence = self._parse_sequence(seq_param)
        else:
            sequence = self._load_yaml_sequence()
            sequence = self._apply_shared_station_7_order(sequence)

        valid_sequence = []
        missing = []
        for name in sequence:
            if name in self._waypoints_map:
                valid_sequence.append(name)
            else:
                missing.append(name)

        if missing:
            self.get_logger().warn(
                f'Sequence contains missing waypoint(s): {missing}'
            )

        return valid_sequence

    def _apply_shared_station_7_order(self, sequence: List[str]) -> List[str]:
        """Keep YAML unchanged while treating 71/72 as station-7 variants.

        Station 7 is a shared desk. 71 is the A-side station 7 and 72 is the
        B-side station 7, so the full test route should visit A-side station 7
        after station 6, then continue to B-side station 7 before station 8.
        """
        station_71_block = ['station_71_sub_goal', 'station_71_goal']
        station_72_block = ['station_72_sub_goal', 'station_72_goal']
        anchor = 'station_6_goal'

        required_names = station_71_block + station_72_block + [anchor]
        if any(name not in sequence for name in required_names):
            return sequence

        reordered = [
            name
            for name in sequence
            if name not in station_71_block and name not in station_72_block
        ]

        try:
            anchor_index = reordered.index(anchor)
        except ValueError:
            return sequence

        insert_index = anchor_index + 1
        shared_station_7_block = station_71_block + station_72_block
        reordered[insert_index:insert_index] = shared_station_7_block

        if reordered != sequence:
            self.get_logger().info(
                'Applied internal shared station-7 order: '
                'station_6 -> station_71(A) -> station_72(B) -> station_8'
            )

        return reordered

    def _load_yaml_sequence(self) -> List[str]:
        path = Path(
            str(self.get_parameter('stations_file').value)
        ).expanduser()
        if not path.exists():
            self.get_logger().error(f'Station file not found: {path}')
            return list(self._waypoints_map.keys())

        try:
            with path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to read station file: {exc}')
            return list(self._waypoints_map.keys())

        yaml_sequence = data.get('sequence') if isinstance(data, dict) else None
        if isinstance(yaml_sequence, list):
            return [str(name) for name in yaml_sequence]

        self.get_logger().warn(
            'YAML "sequence" is missing or invalid; using waypoint map order.'
        )
        return list(self._waypoints_map.keys())

    def _parse_sequence(self, value) -> List[str]:
        if isinstance(value, list):
            return [str(item) for item in value]

        text = str(value).strip()
        if not text:
            return []

        try:
            parsed = yaml.safe_load(text)
        except Exception:
            parsed = None

        if isinstance(parsed, list):
            return [str(item) for item in parsed]

        return [
            item.strip()
            for item in text.strip('[]').split(',')
            if item.strip()
        ]

    def _sleep_with_stop(self, duration_sec: float):
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and not self._stop_requested:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            time.sleep(min(0.1, remaining))

    def _publish_status(self, status: str):
        if self._status_pub is None:
            return

        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def destroy_node(self):
        self.stop()
        if (
                self._run_thread is not None
                and self._run_thread.is_alive()
                and threading.current_thread() is not self._run_thread):
            self._run_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobocupTestNavigator()
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
