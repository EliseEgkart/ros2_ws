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

## Station layout — driven by real waypoints, not a hand-drawn schematic

**Revision 2 (2026-07-02, same day as the initial rewrite)**: real-hardware
testing showed the AMR marker landing away from the station boxes. Root
cause: the first cut of this GUI rendered stations on a hand-drawn
schematic grid matching `map.jpg`, and computed a *separate* AMR-position
transform from only 2 anchor points — those two coordinate systems were
never guaranteed to agree except at the anchors. This is now fixed
structurally, not patched:

- Every station's canvas position **and** the AMR marker are computed by
  applying the exact same `waypoints.FitTransform` to real
  `robocup_waypoint.yaml` meters (`waypoints.py`, `canvas_view.py`). There
  is one coordinate transform, rebuilt from all currently-known station
  positions whenever a new `Task` arrives — so a station box and the AMR
  marker sitting on it are the same computation applied to (in the limit)
  the same real point, not two guesses that happen to agree.
- `resolve_station_xy()` mirrors `mock_nav_node.py`'s own `_station_coord()`
  resolution order exactly (raw `station_id` first, then the B-side→A-side
  canonical id) — a station renders at the same real position the AMR
  actually navigates to for it, not a different one derived independently.
- Any station whose `station_id` has no `robocup_waypoint.yaml` entry yet
  (the navigator team's file is still WIP — as of this writing it only
  defines `station_0`..`station_8`) is listed in a small "no waypoint" tray
  at the bottom of the window instead of being silently dropped or placed
  at a guessed position.
- Station **grouping** (which role a station is) still comes from parsing
  the `arena_layout` entry's **name** (`station_roles.py`), not its numeric
  `station_id` — the different `eai_task_server` task generators
  (`task_publisher.py`, `entry_server_0702.py`/`beginner_server_0702.py`,
  `manual_order_server.py`, `task_complexity_publisher.py`) don't agree on
  IDs for the same physical station, but they do all agree on names
  (`side_a_storage_1`, `shared_storage_1`, ...).

One consequence worth knowing: the rendered arrangement reflects the real
arena's true relative geometry (scaled/centered to fit the window, with a
consistent y-flip so it isn't mirrored top-to-bottom), **not** necessarily
`map.jpg`'s exact left/right or top/bottom placement — there's no verified
mapping yet from "which way is up in the real coordinate frame" to "which
way is up in the reference diagram". Positions will only visually match
`map.jpg` 1:1 once that's confirmed with the navigator team; until then,
internal self-consistency (station ↔ AMR alignment) was prioritized over
matching the picture, since that's what makes the GUI usable for real
monitoring.

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
| `waypoint_yaml` | resolved via `ament_index` against `robocup_planner`'s installed share dir | Real station coordinates driving both station placement and the AMR marker |
| `health_check_period_sec` | `1.0` | How often the connection indicators refresh |

The `worldcup_gui` console-script alias still works for compatibility with
older invocations.

## File structure

```
sml_worldcup_gui/
├── layout_schema.py    # role colors/labels + canvas geometry constants
├── station_roles.py    # parses arena_layout station names into roles
├── waypoints.py         # robocup_waypoint.yaml -> the one shared real-to-canvas transform
├── ros_bridge.py        # rclpy node, thread-safe event queue
├── canvas_view.py       # tk.Canvas arena renderer (stations + AMR marker)
├── side_panel.py        # orders / connection health / selected station
└── app.py                # entry point wiring the above together
```
