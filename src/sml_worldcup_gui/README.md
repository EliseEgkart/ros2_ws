# sml_worldcup_gui

Tkinter GUI for the World Cup 2026 arena, suitable for **both** a simulated
demo run (mock stack) and real-world monitoring — the two modes share the
exact same code path, driven by whichever nodes actually publish the
interfaces below.

This is a full rewrite (2026-07-02). The previous version listened for
`planner_state`/`manager_status` string topics that nothing in this
codebase ever publishes, and for `nav_msgs/Odometry` that only the real
robot produced — during simulation it could never show live status or a
moving AMR marker. See `algorithm_description.md`'s "Known Issues" history
for the full writeup.

## What it actually subscribes to

| Data | Source | Topic/Interface |
|---|---|---|
| Orders + arena layout | `sml_messages/Task` | `topic_name` (default `/eai/task`) |
| Live AMR position | `nav_msgs/Odometry` | `odom_topic` (default `/odom`) — published by `robocup_navigator/current_pose.py` on the real robot, or by `mock_nav_node.py` during simulation (added alongside this GUI rewrite) |
| Connection health | non-blocking liveness probes | `navigate_to_station` action, `wb_task` action, `/amr_robot_command` service, `/robocup_navigator/post_process` service — the actual `robocup_pkg` interfaces `robocup_planner` uses |

If the AMR position marker never appears, check the `AMR ODOM` indicator in
the side panel first — it means nothing is publishing `/odom` yet (normal
if you're running an older `mock_nav_node.py` build without the odometry
patch, or before the real robot's localization has started).

## Station layout — name-based, not id-based

The different `eai_task_server` task generators (`task_publisher.py`,
`entry_server_0702.py`/`beginner_server_0702.py`, `manual_order_server.py`,
`task_complexity_publisher.py`) do **not** agree on numeric `station_id`
values for the same physical station. They do all agree on station
**names** (`side_a_storage_1`, `side_a_workbench_2`, `shared_storage_1`,
...). This GUI groups incoming `arena_layout` entries by parsing the name
(`station_roles.py`), not by `station_id`, and lays each role out in a
schematic matching `map.jpg` ("SML Example Layout — World Cup 2026"):
Side A left, Side B right, one shared-storage column in the center, a
customer row along the top, two start/goal boxes at bottom-center. The
layout is count-adaptive — 2 or 3 storage stations per side both render
correctly.

## AMR marker placement

Real-world station coordinates come from `robocup_planner/config/robocup_waypoint.yaml`
(the same file the real planner's `DistanceCalculator` and the real
navigator use), resolved via `ament_index` by default. As of this writing
that file only defines `station_0`..`station_8` — it's still a
work-in-progress on the navigator team's side. The GUI computes a
best-effort similarity transform (rotation + scale + translation) from two
anchor points (canonical station `0` = start/goal, `6` = customer_1) to its
schematic canvas space; if either anchor is missing from the YAML, the live
marker is hidden (grayed out) rather than guessed at.

## Run

```bash
cd ~/ros2_ws
colcon build --packages-select sml_messages robocup_pkg sml_system_pkg robocup_planner sml_worldcup_gui
source install/setup.bash

ros2 run sml_worldcup_gui sml_worldcup_gui --ros-args -p side:=a
```

`side` accepts `a`, `b`, or `all` (default) and crops/zooms the viewport
accordingly — it does not filter which stations are drawn, since the whole
arena_layout may still be useful to see.

Other parameters (all optional, `--ros-args -p name:=value`):

| Parameter | Default | Meaning |
|---|---|---|
| `topic_name` | `/eai/task` | Task subscription |
| `odom_topic` | `/odom` | AMR position subscription |
| `nav_action` | `navigate_to_station` | Navigator liveness probe |
| `wb_action` | `wb_task` | Workbench liveness probe |
| `arm_service` | `/amr_robot_command` | Arm liveness probe |
| `post_process_service` | `/robocup_navigator/post_process` | Post-process liveness probe |
| `waypoint_yaml` | resolved via `ament_index` against `robocup_planner`'s installed share dir | Real station coordinates for the AMR-position transform |
| `health_check_period_sec` | `1.0` | How often the connection indicators refresh |

The `worldcup_gui` console-script alias still works for compatibility with
older invocations.

## File structure

```
sml_worldcup_gui/
├── layout_schema.py    # schematic canvas positions matching map.jpg
├── station_roles.py    # parses arena_layout station names into roles
├── waypoints.py         # robocup_waypoint.yaml -> canvas similarity transform
├── ros_bridge.py        # rclpy node, thread-safe event queue
├── canvas_view.py       # tk.Canvas arena renderer
├── side_panel.py        # orders / connection health / selected station
└── app.py                # entry point wiring the above together
```
