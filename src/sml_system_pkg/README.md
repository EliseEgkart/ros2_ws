# sml_system_pkg

RoboCup SML 공용 인프라 패키지.  
**계획·실행 노드는 `robocup_planner`로 이전되었습니다.** 이 패키지는 주문 서버와 mock 노드를 제공합니다.

## 포함 노드

| 노드 | 역할 |
|---|---|
| `order_server` | `/sml/task` 발행 (TRANSIENT_LOCAL QoS) |
| `mock_nav_node` | `navigate_to_station` 액션 mock (sub_goal/goal 2단계 지원) |
| `mock_arm_node` | `/amr_robot_command` 서비스 mock |
| `mock_wb_node` | `wb_task` 액션 mock |
| `sml_gui_node` | 시각화 GUI |

> `sml_planning_node` 와 `sml_manager_node` 는 `robocup_planner/planner_node` 로 대체되어 entry point에서 제거되었습니다.

## Documentation

- [경기별 적재·하역 로직](docs/경기별_적재_하역_로직.md)

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sml_messages robocup_pkg sml_system_pkg robocup_planner
source install/setup.bash
```

## 시뮬레이션 (권장 — 한번에 실행)

```bash
ros2 launch robocup_planner robocup_planner_sim.launch.py
```

## mock 노드 단독 실행

```bash
# 터미널 1
ros2 run sml_system_pkg mock_nav_node

# 터미널 2
ros2 run sml_system_pkg mock_arm_node

# 터미널 3
ros2 run sml_system_pkg mock_wb_node

# 터미널 4 — robocup_planner
ros2 run robocup_planner planner_node \
  --ros-args --params-file ~/ros2_ws/install/robocup_planner/share/robocup_planner/config/params.yaml

# 터미널 5 — 주문 발행
ros2 run sml_system_pkg order_server \
  --ros-args -p auto_publish:=true -p start_side:=a -p tier:=beginner -p stage:=production
```

## mock_nav_node — sub_goal / goal 2단계 내비게이션

`navigate_to_station` 액션에 전달하는 `station_id` 부호로 동작을 구분합니다:

| station_id | 동작 | 피드백 status |
|---|---|---|
| 양수 (`+N`) | docking goal로 이동 (정밀 주차) | `ARRIVED` |
| 음수 (`-N`) | station N의 sub_goal(접근 웨이포인트)로 정지 | `AT_SUBGOAL` |

지연 시간 모델:
- sub_goal 도착: 전체 거리의 80%
- 이어지는 goal 레그: 전체 거리의 20% (AMR이 이미 sub_goal에 대기 중)
- `use_distance_time: true` 설정 시 `station_coord_json_path`의 좌표 기반 계산
