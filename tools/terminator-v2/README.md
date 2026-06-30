# Terminator v2

## 핵심 명령어

```bash
# 설치
cd ~/ros2_ws
./tools/terminator-v2/install_rslv2_alias.sh
source ~/.bashrc

# 실행
rslv2

# 로그 확인
ls -td logs/* | head -1
```

## Pane 구성

| 창 | 명령어 |
|---|---|
| 1 all_in_one | `ros2 launch all_in_one_package all_in_one_launch.py` |
| 2 navigator | `ros2 run robocup_navigator robocup_navigator` |
| 3 robot_launch | `ros2 launch amr_robot_launch amr_robot.launch.py vision_visualize:=true` |
| 4 robocup_planner | `ros2 run robocup_planner planner_node --ros-args --params-file ~/ros2_ws/install/robocup_planner/share/robocup_planner/config/params.yaml` |
| 5 order_server | `ros2 run sml_system_pkg order_server --ros-args -p auto_publish:=true -p start_side:=a -p tier:=beginner -p stage:=production` |
| 6 empty | 빈 ROS2 준비 터미널 |
| 7 empty | 빈 ROS2 준비 터미널 |
| 8 empty | 빈 ROS2 준비 터미널 |

## 권장 실행 순서

```text
1 -> 3 -> 4 -> 5 -> 2
```

## Navigator 튜닝

| 용도 | 파일 |
|---|---|
| 전면 라이다 정렬, 후진 거리/속도, side, post-process timeout | `src/robocup_navigator/params/robocup_navigator_params.yaml` |
| station별 이탈 회전 방향/각도 | `src/robocup_navigator/params/robocup_rotation_profiles.yaml` |

Navigator가 위 yaml을 기본값으로 직접 읽는다. 튜닝값을 바꾼 뒤에는 navigator를 재시작한다.

로그는 `logs/YYYYMMDD_HHMM/1.log`부터 `8.log`까지 생성된다.
