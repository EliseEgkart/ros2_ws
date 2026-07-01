# robocup_planner

RoboCup SML 경기용 계획·실행 통합 노드.  
`sml_planning_node` + `sml_manager_node` 의 역할을 단일 노드로 수행합니다.

---

## 아키텍처

```
/eai/task
      │
      ▼
[PlannerNode]
  ┌── 계획 단계 ──────────────────────────────────────────────┐
  │  order_list     → aidlist / net_aidlist (재활용 차감)     │
  │  arena_layout   → midlist (거리순 pickup 시퀀스)          │
  │  produce_ids    → workbench_products / intransit_products  │
  └────────────────────────────────────────────────────────────┘
      │
      ▼
[Executor] (백그라운드 스레드)
  Phase 1: 재활용 픽업 (customer → workbench 분해)
  Phase 2: 재료 픽업 루프
    ├── navigate_subgoal(id)          ← 고속 주행
    ├── wait_for_intransit_assembly() ← cargo 7/8 조립 완료 대기
    └── navigate_goal(id)             ← 저속 정밀 도킹
  Phase 3: 납품
```

---

## 인터페이스

| 방향 | 인터페이스 | 타입 | 설명 |
|---|---|---|---|
| Sub | `/eai/task` | `sml_messages/msg/Task` | 주문 수신 트리거 |
| Sub | `/workbench/product_ready` | `std_msgs/Int32` | 워크벤치 완료 신호 |
| Act | `navigate_to_station` | `robocup_pkg/action/NavTask` | AMR 이동 |
| Srv | `/robocup_navigator/post_process` | `std_srvs/srv/Trigger` | station 작업 후 후진/회전 이탈 |
| Act | `wb_task` | `robocup_pkg/action/WbTask` | 워크벤치 조립/분해 |
| Srv | `/amr_robot_command` | `robocup_pkg/srv/ArmCommand` | 로봇팔 제어 |

---

## 두 단계 내비게이션 (sub_goal / goal)

모든 station 접근이 두 단계로 분리됩니다:

1. **`navigate_subgoal(id)`** — `station_id = -abs(id)` 로 전달. AMR이 sub_goal 웨이포인트에 정지.  
   이 구간 동안 cargo 7/8 ASSEMBLE 비동기 실행 가능.
2. **`wait_for_intransit_assembly()`** — 진행 중인 ASSEMBLE 완료 대기.
3. **`navigate_goal(id)`** — `station_id = +abs(id)` 로 전달. 저속 정밀 도킹.  
   이 구간에는 팔 명령 없음.

납품 시 customer counter에서도 동일 패턴을 적용합니다:  
sub_goal 도착 후 in-transit assembly가 아직 실행 중이면 그 자리에서 완료를 기다린 뒤 docking goal로 진입합니다.

---

## AMR 슬롯 구조

| 슬롯 | 용도 |
|---|---|
| 1 | 워크벤치 완성품 |
| 2~6 | 재료 (최대 5종) |
| 7~8 | 주행 중 조립(in-transit) 전용 |

cargo 7/8에는 `is_intransit_eligible` 조건을 만족하는 제품만 할당됩니다 (레이어 중첩 없는 단순 조립 구조).

---

## 파라미터 (`config/params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `task_topic` | `/eai/task` | 주문 수신 토픽 |
| `nav_action` | `navigate_to_station` | 내비게이션 액션 이름 |
| `post_process_service` | `/robocup_navigator/post_process` | station 작업 후 이탈 서비스 |
| `wb_action` | `wb_task` | 워크벤치 액션 이름 |
| `arm_service` | `/amr_robot_command` | 로봇팔 서비스 이름 |
| `wb_ready_topic` | `/workbench/product_ready` | 워크벤치 완료 토픽 |
| `waypoint_yaml` | *(ament_index 자동 해석)* | 웨이포인트 YAML 경로 |

`waypoint_yaml`은 빌드 시 `config/robocup_waypoint.yaml`을 `share/robocup_planner/config/`에 설치하며, 노드 시작 시 `ament_index`로 자동 해석합니다.

---

## 빌드 및 실행

```bash
# 빌드
colcon build --packages-select sml_messages robocup_pkg sml_system_pkg robocup_planner
source install/setup.bash

# 시뮬레이션 (mock 노드 포함 전체 스택)
ros2 launch robocup_planner robocup_planner_sim.launch.py

# 인자 지정 예시
ros2 launch robocup_planner robocup_planner_sim.launch.py \
  side:=b tier:=beginner stage:=lifecycle
```

### launch 인자

| 인자 | 기본값 | 선택값 |
|---|---|---|
| `side` | `a` | `a`, `b` |
| `tier` | `beginner` | `entry`, `beginner`, `advanced`, `expert` |
| `stage` | `production` | `production`, `recycling`, `lifecycle` |

---

## 파일 구조

```
robocup_planner/
├── config/
│   ├── params.yaml              # 노드 파라미터 (전체 스택)
│   └── robocup_waypoint.yaml    # 웨이포인트 좌표 (distance calculator용)
├── launch/
│   └── robocup_planner_sim.launch.py   # 시뮬레이션 통합 런치
└── robocup_planner/
    ├── planner_node.py          # 메인 노드 (PlannerNode)
    ├── product_catalog.py       # 제품·재료 카탈로그
    ├── execution/
    │   ├── executor.py          # 반응형 실행 루프 (Executor)
    │   └── cargo_state.py       # 실시간 cargo 2~6 상태 추적
    └── planning/
        ├── aidlist_builder.py   # aidlist / net_aidlist 계산
        ├── cargo_allocator.py   # in-transit 슬롯(7/8) 할당
        ├── distance_calculator.py  # 웨이포인트 기반 거리 계산
        └── midlist_builder.py   # pickup 시퀀스 생성 (batch 지원)
```

---

## 디버깅

```bash
# 주문 확인
ros2 topic echo /eai/task --once

# 내비게이션 액션 단독 테스트 (sub_goal)
ros2 action send_goal /navigate_to_station robocup_pkg/action/NavTask "{station_id: -2}" --feedback

# 내비게이션 액션 단독 테스트 (goal)
ros2 action send_goal /navigate_to_station robocup_pkg/action/NavTask "{station_id: 2}" --feedback

# 로봇팔 서비스 단독 테스트
ros2 service call /amr_robot_command robocup_pkg/srv/ArmCommand \
  "{action: 'LOAD', object_ids: [1], location: 1, station_id: 1, slide_ids: []}"
```
