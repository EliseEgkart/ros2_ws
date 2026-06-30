# RoboCup SML ROS2 워크스페이스

RoboCup SML(Smart Manufacturing Line) 경기를 위한 AMR + 로봇팔 통합 ROS2 워크스페이스입니다.

---

## 패키지 구조

```
src/
├── all_in_one_package/      # AMR 기본 시스템 런치 (LiDAR · Nav2 · serial)
├── amr/                     # Nav2 맵/파라미터 패키지
├── amr_navigator/           # 웨이포인트 추종 유틸리티
├── amr_robot_launch/        # 로봇팔 전체 런치 (vision · gripper · cargo · arm)
├── arm_controller_pkg/      # 로봇팔 제어 노드 (amr_robot_node 등)
├── arm_interfaces/          # 로봇팔 내부 서비스 인터페이스 (Cargo.srv 등)
├── cam_lid_calib/           # 카메라-라이다 캘리브레이션 도구
├── eai_task_server/         # 공식 EAI Task 발행 패키지
├── laser_filters/           # 라이다 필터
├── line_manager/            # 라인 관리 노드
├── robocup_navigator/       # navigate_to_station 액션 서버
├── robocup_planner/         # 독립 계획·실행 플래너 (대안 스택)
├── ros2_laser_scan_merger/  # 듀얼 LiDAR 스캔 병합
├── serial_test/             # AMR 시리얼 통신 노드
├── sllidar_ros2/            # RPLIDAR 드라이버
├── sml_msgs/                # SML 공용 메시지/서비스/액션 인터페이스
├── sml_system_pkg/          # SML 계획·관리·주문 노드
└── vision_pkg/              # YOLO 기반 비전 노드
```

---

## 시스템 아키텍처

### 실전 스택 (robocup_planner — 현행)

```
[order_server]
      │ /sml/task (sml_msgs/Task, TRANSIENT_LOCAL)
      ▼
[robocup_planner]   ← sml_planning_node + sml_manager_node 를 대체하는 단일 노드
      ├─── Action navigate_to_station ──▶ [robocup_navigator]
      │    (station_id < 0 → sub_goal,         │ Nav2 FollowWaypoints
      │     station_id > 0 → docking goal)      ▼
      │                                    [amr (Nav2)]
      │                                         │ /robocup_navigator/post_process
      │                                         ▼
      │                                   [robocup_planner (복귀)]
      │
      ├─── Service /amr_robot_command ──▶ [amr_robot_node]
      │    (sml_msgs/srv/ArmCommand)            ├─ /cargo         → [cargo_manager_node]
      │                                         ├─ /get_target_pose → [vision_node]
      │                                         ├─ /gripper/grip  → [gripper_node]
      │                                         └─ /gripper/open  → [gripper_node]
      │
      └─── Action wb_task ──▶ [실제 워크벤치 하드웨어]
           (sml_msgs/WbTask)
```

### 두 단계 내비게이션 (sub_goal / goal)

모든 station 접근은 두 단계로 분리됩니다:

```
navigate_subgoal(id)          wait_for_intransit_assembly()   navigate_goal(id)
─── 고속 주행 ───────────────▶  ── 팔 완료 대기 ──────────────▶  ── 저속 정밀 도킹 ──
  (팔: ASSEMBLE 비동기 가능)        (cargo 7/8 완료 확인)           (팔: 유휴 상태)
```

- `navigate_to_station` 액션에 **음수 station_id** 전달 시 서브골(접근 웨이포인트) 정지
- **양수** 전달 시 docking goal(최종 주차 위치)로 이동

### 주요 인터페이스

| 인터페이스 | 타입 | 설명 |
|---|---|---|
| `/sml/task` | `sml_msgs/msg/Task` | 주문 수신 (TRANSIENT_LOCAL) |
| `navigate_to_station` | `sml_msgs/action/NavTask` | AMR 이동 (음수=sub_goal) |
| `/amr_robot_command` | `sml_msgs/srv/ArmCommand` | 로봇팔 제어 |
| `wb_task` | `sml_msgs/action/WbTask` | 워크벤치 조립/분해 |
| `/workbench/product_ready` | `std_msgs/Int32` | 워크벤치 완료 신호 |
| `/robocup_navigator/post_process` | `std_srvs/srv/Trigger` | 이탈 동작 트리거 |
| `/cargo` | `arm_interfaces/srv/Cargo` | 슬롯 상태 관리 (내부) |

### AMR 슬롯 구조

| 슬롯 | 용도 |
|---|---|
| 슬롯 1 | 워크벤치 완성품 |
| 슬롯 2~6 | 재료 (최대 5종) |
| 슬롯 7~8 | 주행 중 조립(in-transit) 전용 |

---

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

특정 패키지만 재빌드:

```bash
colcon build --packages-select robocup_planner sml_system_pkg
source install/setup.bash
```

---

## 실행

### 시뮬레이션 (mock 노드 — 하드웨어 없이 전체 구동 확인)

```bash
ros2 launch robocup_planner robocup_planner_sim.launch.py
```

side, tier, stage를 인자로 지정할 수 있습니다:

```bash
ros2 launch robocup_planner robocup_planner_sim.launch.py side:=b tier:=beginner stage:=lifecycle
```

시뮬레이션 스택은 다음 5개 노드를 한번에 기동합니다:

| 노드 | 역할 |
|---|---|
| `order_server` | `/sml/task` 발행 (preset 자동 발행) |
| `mock_nav_node` | `navigate_to_station` 액션 mock |
| `mock_arm_node` | `/amr_robot_command` 서비스 mock |
| `mock_wb_node` | `wb_task` 액션 mock |
| `robocup_planner` | 계획 + 실행 통합 노드 |

### 실전 (Terminator 권장)

#### Terminator 바로 실행

```bash
cd ~/ros2_ws
bash tools/terminator/open_robocup_terminator.sh
```

#### Terminator 시스템 설치 후 실행

```bash
bash tools/terminator/install_terminator_config.sh
terminator -u -l robocup
```

#### 창 구성 (6-pane)

```
┌─────────────────┬────────────────┬────────────────────────────────┐
│  1 all_in_one   │  2 navigator   │       3 robot_launch           │
├─────────────────┴────────────────┴┬─────────────────┬────────────┤
│         4 robocup_planner         │  5 order_server │  6 debug   │
└───────────────────────────────────┴─────────────────┴────────────┘
```

| 창 | 명령어 |
|---|---|
| 1 all_in_one | `ros2 launch all_in_one_package all_in_one_launch.py` |
| 2 navigator | `ros2 run robocup_navigator robocup_navigator --ros-args -p side_mode:=A` |
| 3 robot_launch | `ros2 launch amr_robot_launch amr_robot.launch.py vision_visualize:=true` |
| 4 robocup_planner | `ros2 run robocup_planner planner_node --ros-args --params-file ~/ros2_ws/install/robocup_planner/share/robocup_planner/config/params.yaml` |
| 5 order_server | `ros2 run sml_system_pkg order_server --ros-args -p auto_publish:=true -p start_side:=a -p tier:=beginner -p stage:=production` |
| 6 debug | `ros2 topic list` |

#### 권장 실행 순서

```
1 → 3 → 4 → 5 → 2
```

> **주의**: 창 1(`all_in_one`)이 Nav2 초기화를 완료한 후 창 2(`navigator`)를 실행해야 합니다.

---

## 수동 실행

각 터미널에서 먼저 소싱합니다:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

**터미널 1 — AMR 기본 시스템 (LiDAR · Nav2)**
```bash
ros2 launch all_in_one_package all_in_one_launch.py
```

**터미널 2 — 네비게이션 액션 서버**
```bash
ros2 run robocup_navigator robocup_navigator --ros-args -p side_mode:=A
```

**터미널 3 — 로봇팔 시스템 (vision · gripper · cargo · arm)**
```bash
ros2 launch amr_robot_launch amr_robot.launch.py vision_visualize:=true
```

**터미널 4 — 계획 + 실행 노드 (robocup_planner)**
```bash
ros2 run robocup_planner planner_node \
  --ros-args --params-file ~/ros2_ws/install/robocup_planner/share/robocup_planner/config/params.yaml
```

**터미널 5 — 주문 서버**
```bash
ros2 run sml_system_pkg order_server \
  --ros-args -p auto_publish:=true -p start_side:=a -p tier:=beginner -p stage:=production
```

---

## 경기장 설정 (A / B 경기장)

### A 경기장

| 항목 | 값 |
|---|---|
| Station 번호 | 1 ~ 8 |
| 워크벤치 위치 | station 6 |
| 실행 side | `side:=a` |

### B 경기장

| 항목 | 값 |
|---|---|
| Station 번호 | 9 ~ 16 |
| 워크벤치 위치 | station 15 |
| 실행 side | `side:=b` |

B 경기장 전환 시 `side:=b`로 변경:

```bash
# 시뮬레이션
ros2 launch robocup_planner robocup_planner_sim.launch.py side:=b

# 실전
ros2 run robocup_navigator robocup_navigator --ros-args -p side_mode:=B
ros2 run sml_system_pkg order_server \
  --ros-args -p auto_publish:=true -p start_side:=b -p tier:=beginner -p stage:=production
```

---

## 맵 생성 및 웨이포인트 설정

### 1. 맵 생성

```bash
ros2 launch all_in_one_package generate_map_launch.py
```

### 2. 맵 저장

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/amr/map/robocup_map
```

### 3. 웨이포인트 편집

```bash
ros2 run robocup_navigator waypoint_editor
```

편집 후 Save를 눌러 `params/robocup_waypoint.yaml`에 저장합니다.
자세한 사용법: [`src/robocup_navigator/WAYPOINT_EDITOR.md`](src/robocup_navigator/WAYPOINT_EDITOR.md)

### 4. 현재 위치를 웨이포인트로 저장

```bash
ros2 run robocup_navigator robocup_current_pose --ros-args -p waypoint_name:=storage_shelf_goal
```

---

## 로봇팔 하드웨어 정보

| 항목 | 값 |
|---|---|
| 로봇팔 IP | `10.0.2.8` |
| 그리퍼 시리얼 포트 | `/dev/ttyARDUINO` |
| 지원 액션 | `LOAD`, `UNLOAD`, `ASSEMBLE` |

`amr_robot.launch.py`의 `vision_visualize` 인자로 카메라 시각화를 켤 수 있습니다:

```bash
ros2 launch amr_robot_launch amr_robot.launch.py vision_visualize:=true
```

### 재료 ID 매핑

| ID | 재료 |
|---|---|
| 1 | 2x2 red |
| 2 | 2x2 green |
| 3 | 2x2 blue |
| 4 | 2x2 yellow |
| 5 | 4x2 red |
| 6 | 4x2 green |
| 7 | 4x2 blue |
| 8 | 4x2 yellow |

---

## 디버깅 명령

```bash
# 토픽 목록
ros2 topic list

# 주문 수신 확인
ros2 topic echo /sml/task --once

# 네비게이션 액션 단독 테스트
ros2 action send_goal /navigate_to_station sml_msgs/action/NavTask "{station_id: 1}" --feedback

# 로봇팔 서비스 단독 테스트
ros2 service call /amr_robot_command sml_msgs/srv/ArmCommand \
  "{action: 'LOAD', object_ids: [1], location: 1, station_id: 1, slide_ids: []}"

# 슬롯 상태 확인
ros2 service call /cargo arm_interfaces/srv/Cargo \
  "{action: 'STATUS', object_id: 0, slot: 0}"

# 노드 목록
ros2 node list

# 네비게이션 액션 서버 확인
ros2 action info /navigate_to_station
```

---

## 트러블슈팅

### 로봇팔 연결 실패 (`robot not connected`)

`amr_robot_node`가 `10.0.2.8` 연결에 실패한 상태입니다.
- 로봇팔 전원과 네트워크 연결을 확인합니다.
- 실제 로봇팔 IP가 다르면 런치할 때 `robot_ip:=<IP>`를 지정합니다.
  예: `ros2 launch amr_robot_launch amr_robot.launch.py robot_ip:=10.0.2.8`
- `robot_ready = False` 상태에서는 모든 ARM 요청이 즉시 실패 반환됩니다.

### 워크벤치 서버 없음 (`WB 서버 없음`)

`wb_task` ActionServer에 연결할 수 없는 상태입니다.
- 실제 워크벤치 하드웨어의 ROS2 노드가 실행 중인지 확인합니다.
- 워크벤치 노드가 동일한 ROS_DOMAIN_ID로 실행 중인지 확인합니다.

### Fast DDS Shared Memory 오류

```
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port...
```

이전 ROS2 프로세스의 잔여 공유 메모리가 원인입니다:

```bash
sudo rm -f /dev/shm/fastrtps_port*
```

### Terminator 창이 즉시 닫히는 경우

`install/setup.bash`가 없는 것이 원인입니다. 먼저 빌드를 완료합니다:

```bash
colcon build && source install/setup.bash
```

### 다른 컴퓨터에 Terminator 설치

```bash
cd ~/ros2_ws
chmod +x tools/terminator/*.sh
bash tools/terminator/install_terminator_config.sh
terminator -u -l robocup
```

설치 스크립트가 현재 경로를 자동으로 설정 파일에 반영합니다.

---

## 관련 문서

- [robocup_planner 패키지](src/robocup_planner/README.md)
- [sml_system_pkg (mock/order 노드)](src/sml_system_pkg/README.md)
- [sml_msgs 인터페이스](src/sml_msgs/README.md)
- [Waypoint Editor 사용법](src/robocup_navigator/WAYPOINT_EDITOR.md)
- [시리얼 통신 프로토콜](comm.md)
