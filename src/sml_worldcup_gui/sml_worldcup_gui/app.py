#!/usr/bin/env python3
"""
sml_worldcup_gui entry point.

Wires together:
  - ros_bridge.GuiBridge    — rclpy node + spin thread + event queue
  - waypoints               — loads real robocup_waypoint.yaml positions
  - canvas_view.ArenaCanvas — arena drawing; owns the real->canvas transform
                               so station boxes and the AMR marker always
                               share the exact same coordinate mapping
  - side_panel.SidePanel    — orders / connection health / selected station

Works identically against the simulated stack (mock_nav_node.py's odometry
publisher) and the real robot (robocup_navigator/current_pose.py), since
both publish nav_msgs/Odometry on the same topic.
"""

from __future__ import annotations

import tkinter as tk

from . import waypoints
from .canvas_view import ArenaCanvas
from .ros_bridge import GuiBridge
from .side_panel import SidePanel

POLL_INTERVAL_MS = 100


class WorldCupGuiApp:
    def __init__(self, root: tk.Tk, bridge: GuiBridge):
        self.root = root
        self.bridge = bridge
        node = bridge.node

        self.root.title('SML World Cup GUI')
        self.root.geometry('1280x720')
        self.root.configure(bg='#0b1220')

        container = tk.Frame(root, bg='#0b1220')
        container.pack(fill=tk.BOTH, expand=True)

        real_positions = waypoints.load_station_goal_positions(node.waypoint_yaml_path)
        if not real_positions:
            node.get_logger().warning(
                f'[GUI] No station positions loaded from waypoint_yaml='
                f'"{node.waypoint_yaml_path}" — stations and the AMR marker '
                'will show as "no waypoint" until robocup_waypoint.yaml is available.'
            )

        self.side_panel = SidePanel(container)
        canvas_frame = tk.Frame(container, bg='#0b1220')
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas = ArenaCanvas(canvas_frame, real_positions, on_select=self.side_panel.show_station)

        side_param = node.side
        view = {'a': 'a', 'side_a': 'a', 'b': 'b', 'side_b': 'b'}.get(side_param, 'all')
        self.canvas.set_side_filter(view)

        self._poll()

    def _poll(self) -> None:
        for kind, payload in self.bridge.drain():
            if kind == 'task':
                self.canvas.set_task(payload)
                self.side_panel.update_task(payload)
            elif kind == 'odom':
                self._on_odom(payload)
            elif kind == 'health':
                self.side_panel.update_health(payload)
        self.root.after(POLL_INTERVAL_MS, self._poll)

    def _on_odom(self, odom_msg) -> None:
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        self.canvas.set_amr_real_position((x, y), live=True)


def main(args=None) -> None:
    bridge = GuiBridge(args=args)
    bridge.start()

    root = tk.Tk()
    app = WorldCupGuiApp(root, bridge)

    def on_close():
        bridge.shutdown()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()


if __name__ == '__main__':
    main()
