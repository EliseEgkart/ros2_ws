#!/usr/bin/env python3
"""
sml_worldcup_gui entry point.

Wires together:
  - ros_bridge.GuiBridge   — rclpy node + spin thread + event queue
  - waypoints              — real-meters -> schematic-canvas transform
  - canvas_view.ArenaCanvas — the arena drawing
  - side_panel.SidePanel   — orders / connection health / selected station

Works identically against the simulated stack (mock_nav_node.py's new
odometry publisher) and the real robot (robocup_navigator/current_pose.py),
since both publish nav_msgs/Odometry on the same topic.
"""

from __future__ import annotations

import tkinter as tk

from . import layout_schema as L
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

        self.side_panel = SidePanel(container)
        canvas_frame = tk.Frame(container, bg='#0b1220')
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas = ArenaCanvas(canvas_frame, on_select=self.side_panel.show_station)

        side_param = node.side
        view = {'a': 'a', 'side_a': 'a', 'b': 'b', 'side_b': 'b'}.get(side_param, 'all')
        self.canvas.set_side_filter(view)

        self._transform = self._build_transform(node.waypoint_yaml_path, view)
        if self._transform is None:
            node.get_logger().warning(
                '[GUI] No AMR position transform available (waypoint_yaml missing '
                'station_0_goal/station_6_goal) — live AMR marker will not be shown '
                'until robocup_waypoint.yaml has both.'
            )

        self._poll()

    def _build_transform(self, waypoint_yaml_path: str, view: str):
        real_positions = waypoints.load_station_goal_positions(waypoint_yaml_path)
        side_key = 'side_b' if view == 'b' else 'side_a'
        anchor0_box = L.START_GOAL_BOXES[1] if side_key == 'side_b' else L.START_GOAL_BOXES[0]
        canvas_anchor0 = (anchor0_box['x'], anchor0_box['y'])
        customer_slot = L.build_side_slots(side_key, {'customer': 1})[0]
        canvas_anchor1 = (customer_slot.x, customer_slot.y)
        return waypoints.build_transform(real_positions, canvas_anchor0, canvas_anchor1)

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
        if self._transform is None:
            self.canvas.set_amr_position(None, live=False)
            return
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        canvas_xy = self._transform.apply(x, y)
        self.canvas.set_amr_position(canvas_xy, live=True)


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
