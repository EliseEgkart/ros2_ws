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
- `New/Update Station`: `station_N_goal` 또는 `station_N_sub_goal` 생성/갱신
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
