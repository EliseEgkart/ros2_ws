# RoboCup Navigation 실행 가이드

이 문서는 `robocup_navigator`를 실행해서 MasterPC의
`navigate_to_station` 액션 요청을 받는 방법을 정리한다.

`robocup_navigator`는 자체 인터페이스를 만들지 않는다.
액션 타입은 `sml_msgs/action/NavTask`를 사용한다.

## 1. 역할

`robocup_navigator`는 station id를 받아서 다음 순서로 움직인다.

```text
station_id 수신
-> sub goal 이동
-> main goal 이동
-> /scan 기반 전방 거리 정렬
-> 액션 success 반환
-> manager가 로봇팔 LOAD/UNLOAD 실행
-> manager가 /robocup_navigator/post_process 호출
-> 후진
-> 좌회전
-> 다음 goal 대기
```

액션 이름:

```text
/navigate_to_station
```

로봇팔 작업 이후 이탈 동작 서비스:

```text
/robocup_navigator/post_process
```

서비스 타입:

```text
std_srvs/srv/Trigger
```

액션 타입:

```text
sml_msgs/action/NavTask
```

인터페이스:

```text
# Goal
int32 station_id
---
# Result
bool success
string fail_reason
---
# Feedback
string status
```

## 2. 현재 station 매핑

설정 파일:

```text
src/robocup_navigator/params/stations_robocup.yaml
```

기본 매핑:

```text
station_id=1 -> storage_shelf
station_id=2 -> workbench
station_id=3 -> customer_counter
```

각 station은 `sequence`를 가진다.
현재는 기존 자동 주행 순서와 동일하게 `sub_goal -> goal` 구조다.

```yaml
stations:
  1:
    name: storage_shelf
    sequence:
      - storage_shelf_sub_goal
      - storage_shelf_goal
    post_process: true
```

대회장에서 station id가 바뀌면 코드가 아니라
`stations_robocup.yaml`의 `stations` 매핑을 수정한다.

## 3. 빌드

워크스페이스 루트에서 실행한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sml_msgs robocup_navigator
source install/setup.bash
```

실행 파일 확인:

```bash
ros2 pkg executables robocup_navigator
```

정상 출력:

```text
robocup_navigator robocup_navigator
```

액션 타입 확인:

```bash
ros2 interface show sml_msgs/action/NavTask
```

## 4. 로봇 기본 시스템 실행

Nav2, localization, lidar, scan merger가 먼저 떠 있어야 한다.

일반 주행 실행:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch all_in_one_package all_in_one_launch.py
```

지도 작성이 필요할 때:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch all_in_one_package generate_map_launch.py
```

`all_in_one_launch.py`는 현재 다음 계열을 순차 실행한다.

```text
serial_test
sllidar_ros2
ros2_laser_scan_merger
amr localization
amr navigation
```

`robocup_navigator`는 별도 터미널에서 실행한다.

## 5. robocup_navigator 실행

새 터미널:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robocup_navigator robocup_navigator
```

정상 시작 로그:

```text
Robocup navigator ready: action="navigate_to_station", stations=[1, 2, 3], scan="/scan"
```

이 노드는 goal 하나를 처리한 뒤 종료하지 않고 다음 goal을 기다린다.
이미 주행 중일 때 새 goal이 오면 reject한다.

## 6. 단독 액션 테스트

Nav2가 준비된 상태에서 다른 터미널에서 실행한다.

storage shelf:

```bash
ros2 action send_goal /navigate_to_station sml_msgs/action/NavTask "{station_id: 1}" --feedback
```

workbench:

```bash
ros2 action send_goal /navigate_to_station sml_msgs/action/NavTask "{station_id: 2}" --feedback
```

customer counter:

```bash
ros2 action send_goal /navigate_to_station sml_msgs/action/NavTask "{station_id: 3}" --feedback
```

성공 시 result:

```text
success: true
fail_reason: ''
```

알 수 없는 station id:

```bash
ros2 action send_goal /navigate_to_station sml_msgs/action/NavTask "{station_id: 99}" --feedback
```

예상 result:

```text
success: false
fail_reason: UNKNOWN_STATION
```

## 7. MasterPC 흐름과 연동

MasterPC 쪽 `sml_manager_node`는 AMR 이동 시
`/navigate_to_station` 액션을 호출한다.

실행 순서 예:

터미널 1, 로봇 기본 시스템:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch all_in_one_package all_in_one_launch.py
```

터미널 2, 자율주행 액션 서버:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robocup_navigator robocup_navigator
```

터미널 3, planning:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run sml_system_pkg sml_planning_node
```

터미널 4, manager:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run sml_system_pkg sml_manager_node
```

터미널 5, workbench mock:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run sml_system_pkg mock_wb_node
```

터미널 6, order server:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run sml_system_pkg order_server
```

주의:
현재 워크스페이스에는 `/amr_robot_command` 서비스 서버 구현이 없다.
`sml_manager_node`는 ARM 단계에서 이 서비스를 기다리므로,
전체 MasterPC 흐름을 끝까지 돌리려면 실제 로봇팔 노드 또는
`sml_msgs/srv/ArmCommand` 호환 mock 서버가 필요하다.

## 8. 주요 파라미터

기본값은 `robocup_navigator/navigator.py`에 선언되어 있다.

자주 조정하는 값:

```text
stations_file: station/waypoint yaml 경로
scan_topic: /scan
target_front_distance: 0.47
target_distance_tolerance: 0.01
approach_speed: 0.08
approach_timeout_sec: 12.0
backup_distance: 0.20
backup_speed: 0.08
rotate_angle_deg: 150.0
rotate_angular_speed: 0.5
post_process_service_name: /robocup_navigator/post_process
```

예시:

```bash
ros2 run robocup_navigator robocup_navigator --ros-args \
  -p target_front_distance:=0.47 \
  -p backup_distance:=0.20 \
  -p rotate_angle_deg:=150.0
```

라이다 정렬 timeout을 실패로 처리하고 싶으면:

```bash
ros2 run robocup_navigator robocup_navigator --ros-args \
  -p fail_on_alignment_timeout:=true
```

기본값은 `false`라서 라이다 정렬 timeout이 나도 후진/회전 단계로 넘어간다.

## 9. 동작 확인 명령

액션 서버 확인:

```bash
ros2 action list | grep navigate_to_station
```

액션 정보:

```bash
ros2 action info /navigate_to_station
```

Nav2 FollowWaypoints 서버 확인:

```bash
ros2 action list | grep follow_waypoints
```

scan 확인:

```bash
ros2 topic echo /scan --once
```

cmd_vel 확인:

```bash
ros2 topic echo /cmd_vel
```

현재 로봇 위치를 waypoint YAML 형태로 출력:

```bash
ros2 run robocup_navigator robocup_current_pose --ros-args \
  -p waypoint_name:=storage_shelf_goal
```

기본적으로 `map -> base_link` TF를 읽어서 아래 형태로 출력한다.
파일에는 저장하지 않는다.
TF는 여러 번 샘플링하며, 위치나 yaw가 크게 튀면 잘못된 waypoint 출력을 막고 실패한다.

```yaml
frame_id: map

waypoints:
  storage_shelf_goal:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

현재 `/odom` 토픽 기준 pose를 출력하려면:

```bash
ros2 run robocup_navigator robocup_current_pose --ros-args \
  -p pose_source:=odom \
  -p odom_topic:=/odom \
  -p waypoint_name:=storage_shelf_goal
```

이 방식은 로봇에 이동 명령을 보내지 않고 `/odom`을 한 번 읽어서 출력한다.
출력 `frame_id`는 보통 `odom`이다.

프레임을 바꾸고 싶으면:

```bash
ros2 run robocup_navigator robocup_current_pose --ros-args \
  -p target_frame:=map \
  -p source_frame:=base_link \
  -p waypoint_name:=new_waypoint
```

TF 샘플링 기준을 조정하려면:

```bash
ros2 run robocup_navigator robocup_current_pose --ros-args \
  -p target_frame:=map \
  -p source_frame:=base_link \
  -p waypoint_name:=new_waypoint \
  -p tf_sample_count:=10 \
  -p max_tf_position_jump_m:=0.05 \
  -p max_tf_yaw_jump_rad:=0.15
```

`/goal_pose`는 RViz에서 마우스로 찍은 goal이고, `map -> base_link`는 실제 로봇 현재 위치다.
두 값이 갑자기 크게 다르면 localization 수렴 상태나 중복 TF publisher를 확인한다.

## 10. 실패 원인

`fail_reason`은 현재 아래 값을 사용한다.

```text
UNKNOWN_STATION: station_id가 stations_robocup.yaml에 없음
NAV_FAILED: Nav2 goal reject, missed waypoint, 내부 예외
TIMEOUT: Nav2 결과 timeout 또는 설정에 따른 alignment timeout
CANCELED: 액션 cancel 요청
```

## 11. 대회 전 체크리스트

1. `stations_robocup.yaml`의 station id가 MasterPC의 `arena_layout.station_id`와 일치하는지 확인한다.
2. 각 station의 `sub_goal -> goal` pose를 실제 대회장 map에서 다시 찍는다.
3. `/scan`에서 전방이 ranges 배열 양끝에 잡히는지 확인한다.
4. station별로 액션 단독 테스트를 반복한다.
5. `target_front_distance`, `backup_distance`, `rotate_angle_deg`를 실제 로봇팔 작업 자세 기준으로 튜닝한다.
6. MasterPC 연동 전 `/navigate_to_station` 단독 성공률을 먼저 확보한다.


## START GUIDE

ros2 launch all_in_one_package all_in_one_launch.py

ros2 run robocup_navigator robocup_navigator --ros-args -p side_mode:=A

ros2 launch amr_robot_launch amr_robot.launch.py vision_visualize:=true

ros2 run sml_system_pkg mock_wb_node

ros2 run sml_system_pkg sml_planning_node

ros2 run sml_system_pkg sml_manager_node

ros2 run sml_system_pkg order_server

## Mockup test
ros2 run sml_system_pkg mock_nav_node

ros2 run sml_system_pkg mock_arm_node

ros2 run sml_system_pkg mock_wb_node

ros2 run sml_system_pkg sml_planning_node

ros2 run sml_system_pkg sml_manager_node

ros2 run sml_system_pkg order_server
