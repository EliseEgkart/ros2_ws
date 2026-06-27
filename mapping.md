# AMR 맵 생성 및 웨이포인트 설정 절차

## 1. 새로운 맵 생성

새로운 환경에서 AMR 매핑을 진행할 때 아래 명령어를 실행한다.

```sh
ros2 launch all_in_one_package generate_map_launch.py
```

---

## 2. 매핑 이후 맵 저장

매핑이 완료되면 아래 명령어로 맵을 저장한다.

```sh
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/amr/map/robocup_map
```

위 명령어를 실행하면 다음 경로에 `robocup_map` 파일이 저장된다.

```sh
~/ros2_ws/src/amr/map
```

저장된 맵 경로는 웨이포인트 편집기에서 자동으로 사용되도록 설정되어 있다.

---

## 3. 웨이포인트 편집

웨이포인트 편집기를 실행한다.

```sh
ros2 run robocup_navigator waypoint_editor
```

편집기에서 웨이포인트를 설정한 뒤 `save`를 눌러 저장한다.

저장된 웨이포인트 경로는 `all_in_one_launch`에서 자동으로 사용되도록 설정되어 있다.

---

## 4. START GUIDE

아래 명령어를 실행한다.

```sh
rs
```

터미널은 총 6개를 실행한다.

```text
1  2  3  (4)
5  6  7  (8)
```

괄호로 표시된 `4`, `8`번 터미널은 필요 시 사용한다.