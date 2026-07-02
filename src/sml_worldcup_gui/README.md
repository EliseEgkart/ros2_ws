# SML World Cup 2026 GUI

ROS 2의 `/eai/task` (`sml_messages/msg/Task`)를 구독해 World Cup 2026
경기장 레이아웃과 주문 및 스테이션 자원을 표시한다.

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sml_messages sml_worldcup_gui
source install/setup.bash
```

## 실행

```bash
ros2 run sml_worldcup_gui sml_worldcup_gui
```

전체 Arena 대신 실제 경기 side만 확대해서 표시할 수 있다.

```bash
ros2 run sml_worldcup_gui sml_worldcup_gui --ros-args -p side:=a
ros2 run sml_worldcup_gui sml_worldcup_gui --ros-args -p side:=b
ros2 run sml_worldcup_gui sml_worldcup_gui --ros-args -p side:=all
```

`a` 또는 `b`를 선택하면 해당 side의 6개 station과 shared storage만 표시한다.

## 반투명 Text Info 창

메인 창 좌상단의 `TEXT INFO` 토글을 누르면 별도 진행 정보 창이 열린다.

- Task 수신 시각과 주문 수
- Planner 경로
- 현재 및 완료 구간
- AMR 위치와 상태
- Task / Planner / Manager / Location 실시간 로그

정보 창 상단의 `Alpha` 슬라이더로 투명도를 30~100% 범위에서 조절할 수 있다.
창의 X 버튼이나 `TEXT INFO` 토글을 다시 누르면 정보 창이 숨겨진다.

## 연결 상태 인디케이터

메인 창 좌상단에서 각 입력 토픽의 publisher 연결 상태를 0.5초마다 확인한다.

- `TASK`: Task 발행 노드
- `PLANNER`: Planner 상태 발행 노드
- `MANAGER`: Manager 상태 발행 노드
- `ODOM`: AMR Odometry 발행 노드

초록색은 하나 이상의 publisher가 연결된 상태, 빨간색은 publisher가 없는 상태,
회색 `OFF`는 해당 모니터가 비활성화된 상태다. 표시 숫자는 연결된 publisher 수다.

기본적으로 `/home/user/ros2_ws/src/GUI/config/sml_worldcup_2026_layout.json`을 읽으며,
해당 파일이 없는 배포 환경에서는 패키지에 설치된 JSON을 사용한다.

다른 Task 토픽이나 레이아웃 파일을 사용할 수도 있다.

```bash
ros2 run sml_worldcup_gui sml_worldcup_gui --ros-args \
  -p topic_name:=/eai/task \
  -p side:=a \
  -p layout_file:=/home/user/ros2_ws/src/GUI/config/sml_worldcup_2026_layout.json
```

기존 실행 이름인 `worldcup_gui`도 호환 alias로 유지한다. GUI와 기존
`task_listener`는 같은 토픽을 동시에 구독할 수 있다.
