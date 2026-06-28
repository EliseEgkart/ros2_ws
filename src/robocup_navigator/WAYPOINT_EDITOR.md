# Waypoint Editor 사용 설명

`waypoint_editor`는 ROS2 map YAML과 waypoint YAML을 GUI로 열어서 맵 위에서 직접 waypoint 위치와 방향을 수정하는 도구입니다.

## 실행

```bash
source install/setup.bash
ros2 run robocup_navigator waypoint_editor
```

기본 경로:

- Map: `$HOME/ros2_ws/src/amr/map/robocup_map.yaml`
- Waypoint: `$HOME/ros2_ws/src/robocup_navigator/params/robocup_waypoint.yaml`

다른 파일을 열 때:

```bash
ros2 run robocup_navigator waypoint_editor \
  --map /path/to/map.yaml \
  --waypoints /path/to/waypoint.yaml
```

## 기본 편집 흐름

1. 왼쪽 waypoint 목록에서 수정할 waypoint를 선택합니다.
2. `Mode`를 `edit`로 둡니다.
3. 맵에서 왼쪽 클릭하면 선택한 waypoint의 위치가 이동합니다.
4. 왼쪽 클릭 후 드래그하면 waypoint의 방향 yaw가 설정됩니다.
5. `Save`를 눌러 YAML에 저장합니다.

저장 시 기존 waypoint YAML은 같은 폴더에 `.bak-YYYYMMDD-HHMMSS` 형식으로 백업됩니다.

## 모드

### edit

waypoint를 실제로 수정하는 모드입니다.

- 왼쪽 클릭: 선택 waypoint 위치 지정
- 왼쪽 드래그: 선택 waypoint 방향 지정
- 오른쪽 클릭: 가장 가까운 waypoint 선택

### measure

맵 위 거리와 각도를 재는 모드입니다.

- 왼쪽 드래그: 줄자 생성
- 표시값: 거리, dx, dy, yaw
- `Clear Measure`: 줄자 표시 삭제

이 모드에서는 waypoint가 수정되지 않습니다.

### drag

맵을 이동하는 모드입니다.

- 왼쪽 드래그: 맵 화면 이동
- waypoint 위치와 방향은 수정되지 않음

중간 마우스 버튼 드래그도 모든 모드에서 맵 이동으로 사용할 수 있습니다.

### diff pair

차동 구조 정렬용으로 `sub_goal`과 `goal`을 한 번에 찍는 오버레이 모드입니다.

- 우측 `Diff pair toggle`: diff pair를 토글 방식으로 켜고 끄기
- `Auto next station`: diff pair 완료 후 station 번호를 자동으로 +1
- 왼쪽 드래그 시작점: `station_N_sub_goal`
- 왼쪽 드래그 끝점: `station_N_goal`
- 두 waypoint의 yaw: sub_goal에서 goal을 향하는 동일한 방향

우측 `Competition Points`의 `Station` 값이 `N`으로 사용됩니다. 예를 들어 `Station`이 `3`이면 드래그 한 번으로 `station_3_sub_goal`과 `station_3_goal`이 함께 생성되거나 갱신됩니다.

`Snap to grid`가 켜져 있으면 sub_goal과 goal 위치 모두 현재 grid 간격에 맞춰집니다.

## Robot Preview

우측 `Robot Preview` 패널에서 waypoint 기준 로봇 footprint를 미리 볼 수 있습니다.

- `Show robot footprint`: 로봇 크기 표시 켜기/끄기
- `Size X m`: 로봇 전후 길이
- `Size Y m`: 로봇 좌우 폭
- `Offset X m`: waypoint 기준 로봇 중심의 전방 오프셋
- `Offset Y m`: waypoint 기준 로봇 중심의 좌측 오프셋
- `Targets = selected`: 현재 선택 waypoint에만 표시
- `Targets = point list`: 우측 포인트 리스트에서 선택한 waypoint에 표시
- `Targets = all`: 모든 waypoint에 표시
- `Select All`: 포인트 리스트 전체 선택 후 표시

오프셋은 waypoint yaw 기준의 로컬 좌표입니다. 예를 들어 `Offset X m`이 `0.20`이면 로봇 중심이 waypoint보다 진행 방향으로 20 cm 앞에 표시됩니다.

## Rotation

우측 `Diff Pair` 아래 `Rotation` 패널은 goal waypoint별 후처리 회전을 설정합니다.

- 적용 대상: `station_N_goal`만 유효
- `station_N_sub_goal`의 rotation 설정은 저장/실행하지 않음
- `Target`: 현재 회전 설정 대상과 상태 표시
- `Use rotation`: 선택한 goal의 회전 활성화
- 기본값: `counterclockwise`, `150.0` deg
- `Drag to set angle`: 선택한 goal의 yaw를 기준으로 마우스 드래그 방향까지의 회전 각도 설정
- `Show backup + turn path`: navigator 후처리 궤적 미리보기
- `Direction`: `clockwise` 또는 `counterclockwise`
- `Angle deg`: goal 도착 후 회전할 각도
- `Backup m`: goal pose 기준 후진 거리, 기본값 `0.20`
- `Speed rad/s`: 회전 각속도, 기본값 `1.4`
- `Set Manual`: 직접 입력한 `Direction`, `Angle deg`를 현재 goal에 반영
- `Remove`: 현재 선택 goal의 회전 설정 제거

회전 설정은 waypoint YAML이 아니라 별도 파일에 저장됩니다.

```text
$HOME/ros2_ws/src/robocup_navigator/params/robocup_rotation_profiles.yaml
```

저장 형식:

```yaml
rotation_profiles:
  station_1_goal:
    direction: counterclockwise
    angle_deg: 150.0
```

지도에서는 goal 주변에 시계/반시계 방향 arc로 회전 방향이 표시됩니다.

`Show backup + turn path`를 켜면 goal pose에서 뒤로 `Backup m`만큼 이동한 지점과, 그 지점에서 제자리 회전할 때 로봇 footprint가 차지하는 외접 반경이 함께 표시됩니다. navigator의 실제 회전 명령은 `linear.x=0`, `angular.z`만 사용하므로 로봇 중심 회전반경은 `0.00 m`입니다.

## Competition Points

우측 `Competition Points` 패널은 대회용 waypoint 이름을 빠르게 다루기 위한 영역입니다.

- `Station`: 현재 station 번호
- `Prev` / `Next`: station 번호 이동
- `Select Sub`: `station_N_sub_goal` 선택 또는 생성
- `Select Goal`: `station_N_goal` 선택 또는 생성
- `Create 1-20 Pairs`: `station_1_sub_goal`부터 `station_20_goal`까지 전체 pair 생성

`Create 1-20 Pairs`는 이미 존재하는 waypoint 좌표를 덮어쓰지 않습니다. 없는 waypoint만 맵 중앙에 placeholder로 만들고, global `sequence`는 `station_1_sub_goal`, `station_1_goal`, ..., `station_20_sub_goal`, `station_20_goal` 순서로 정렬합니다.

## Grid와 Snap

### Show grid

맵 위에 일정 간격의 격자를 표시합니다. 좌표를 눈으로 맞추거나 station 간 간격을 비교할 때 사용합니다.

### Grid m

격자 간격을 미터 단위로 설정합니다.

예:

- `0.05`: 5 cm 간격
- `0.10`: 10 cm 간격
- `0.50`: 50 cm 간격
- `1.00`: 1 m 간격

### Snap to grid

클릭한 위치를 가장 가까운 격자 교차점으로 자동 보정하는 기능입니다.

예를 들어 `Grid m`이 `0.10`이고 `Snap to grid`가 켜져 있으면, waypoint를 찍을 때 좌표가 10 cm 단위로 정렬됩니다. 손으로 클릭할 때 생기는 미세한 오차를 줄이고, 여러 waypoint를 같은 라인이나 일정 간격에 맞출 때 유용합니다.

정밀하게 자유 위치를 찍어야 할 때는 `Snap to grid`를 끄면 됩니다.

## 버튼

- `Save`: 현재 waypoint YAML에 저장
- `Save As`: 다른 파일로 저장
- `Undo`: 마지막 편집 되돌리기
- `Redo`: 되돌린 편집 다시 적용
- `Apply Fields`: 왼쪽 입력창의 이름, 좌표, yaw 값을 선택 waypoint에 적용
- `New Custom`: station 형식이 아닌 일반 waypoint 생성
- `Delete`: 선택 waypoint 삭제
- `Reload`: 파일을 다시 읽기
- `Clear Measure`: 측정 줄자 삭제
- `Center`: 선택 waypoint를 화면 중앙으로 이동

## 키보드와 마우스

- 마우스 휠: 확대/축소
- 중간 버튼 드래그: 맵 이동
- `Ctrl+S`: 저장
- `Ctrl+Z`: Undo
- `Ctrl+Y`: Redo
- `Ctrl+Shift+Z`: Redo
- `Delete`: 선택 waypoint 삭제
- `Alt+방향키`: 선택 waypoint 미세 이동
- `Alt+Q`: 선택 waypoint yaw +5도
- `Alt+E`: 선택 waypoint yaw -5도

`Snap to grid`가 켜져 있으면 `Alt+방향키` 이동 간격도 현재 grid 간격을 사용합니다. 꺼져 있으면 기본 0.05 m 단위로 이동합니다.

## YAML 형식

에디터는 기존 navigator가 사용하는 YAML 구조를 유지합니다.

- `waypoints`
- `sequence`
- `frame_id`
- `stations`

따라서 저장한 파일은 기존 `robocup_navigator`에서 그대로 사용할 수 있습니다.
