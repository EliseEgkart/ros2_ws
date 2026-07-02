"""
ROS2 node for sml_worldcup_gui.

Subscribes only to interfaces that actually exist in this codebase (unlike
the previous GUI implementation, which listened for invented
`planner_state`/`manager_status` String topics that nothing ever
published):

  - sml_messages/Task on `topic_name`            (order list + arena_layout)
  - nav_msgs/Odometry on `odom_topic`             (live AMR position — real
                                                    robot via
                                                    robocup_navigator/current_pose.py,
                                                    or the simulated mock
                                                    published by
                                                    mock_nav_node.py)
  - liveness of the real robocup_pkg interfaces (navigate_to_station,
    wb_task, /amr_robot_command, /robocup_navigator/post_process), probed
    non-blockingly on a timer, instead of trusting a self-reported status
    string.

All rclpy callbacks push tagged events onto a thread-safe queue; the Tk main
loop drains it via GuiBridge.drain(). This mirrors the ROS-thread /
Tk-main-thread split the previous GUI used (that part of its design was
sound) without the parts that couldn't work.
"""

from __future__ import annotations

import queue
import threading
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from sml_messages.msg import Task
from robocup_pkg.action import NavTask, WbTask
from robocup_pkg.srv import ArmCommand
from std_srvs.srv import Trigger


def _default_waypoint_yaml() -> str:
    """Resolve robocup_waypoint.yaml from the installed robocup_planner share
    dir, same file the real planner and navigator use. Returns '' (not an
    error) if robocup_planner isn't built yet — callers must treat a missing
    file as "no live AMR position telemetry available", not crash."""
    try:
        from ament_index_python.packages import get_package_share_directory
        import os
        return os.path.join(
            get_package_share_directory('robocup_planner'), 'config', 'robocup_waypoint.yaml'
        )
    except Exception:
        return ''


TASK_QOS = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE,
)
LATCHED_TASK_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


@dataclass
class HealthStatus:
    task_publishers: int = 0
    odom_publishers: int = 0
    nav_action_up: bool = False
    wb_action_up: bool = False
    arm_service_up: bool = False
    post_process_service_up: bool = False


class GuiRosNode(Node):
    def __init__(self):
        super().__init__('sml_worldcup_gui')

        self.declare_parameter('topic_name', '/eai/task')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('nav_action', 'navigate_to_station')
        self.declare_parameter('wb_action', 'wb_task')
        self.declare_parameter('arm_service', '/amr_robot_command')
        self.declare_parameter('post_process_service', '/robocup_navigator/post_process')
        self.declare_parameter('health_check_period_sec', 1.0)
        self.declare_parameter('side', 'all')
        self.declare_parameter('waypoint_yaml', _default_waypoint_yaml())

        topic_name = self.get_parameter('topic_name').value
        odom_topic = self.get_parameter('odom_topic').value
        self.side = str(self.get_parameter('side').value).strip().lower()
        self.waypoint_yaml_path = str(self.get_parameter('waypoint_yaml').value).strip()

        self.events: 'queue.Queue' = queue.Queue()

        # Matches planner_node.py's dual volatile + transient-local subscription
        # pattern, so the GUI reliably sees the task regardless of which QoS
        # the active publisher used (task_publisher.py vs. manual_order_server.py
        # vs. task_complexity_publisher.py don't all agree).
        self.create_subscription(Task, topic_name, self._on_task, TASK_QOS)
        self.create_subscription(Task, topic_name, self._on_task, LATCHED_TASK_QOS)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self._nav_client = ActionClient(self, NavTask, self.get_parameter('nav_action').value)
        self._wb_client = ActionClient(self, WbTask, self.get_parameter('wb_action').value)
        self._arm_client = self.create_client(ArmCommand, self.get_parameter('arm_service').value)
        self._post_process_client = self.create_client(
            Trigger, self.get_parameter('post_process_service').value
        )

        self._topic_name = topic_name
        self._odom_topic = odom_topic
        period = float(self.get_parameter('health_check_period_sec').value)
        self.create_timer(max(0.2, period), self._check_health)

    # ------------------------------------------------------------------
    # Subscription callbacks (ROS thread)
    # ------------------------------------------------------------------

    def _on_task(self, msg: Task) -> None:
        self.events.put(('task', msg))

    def _on_odom(self, msg: Odometry) -> None:
        self.events.put(('odom', msg))

    def _check_health(self) -> None:
        status = HealthStatus(
            task_publishers=self.count_publishers(self._topic_name),
            odom_publishers=self.count_publishers(self._odom_topic),
            nav_action_up=self._nav_client.wait_for_server(timeout_sec=0.0),
            wb_action_up=self._wb_client.wait_for_server(timeout_sec=0.0),
            arm_service_up=self._arm_client.service_is_ready(),
            post_process_service_up=self._post_process_client.service_is_ready(),
        )
        self.events.put(('health', status))


class GuiBridge:
    """Owns the rclpy Node + its spin thread; exposes a drain() the Tk main
    loop can poll without blocking."""

    def __init__(self, args=None):
        rclpy.init(args=args)
        self.node = GuiRosNode()
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def drain(self, max_items: int = 200) -> List[tuple]:
        events = []
        for _ in range(max_items):
            try:
                events.append(self.node.events.get_nowait())
            except queue.Empty:
                break
        return events

    def shutdown(self) -> None:
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
