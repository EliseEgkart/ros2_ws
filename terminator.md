# Terminator ROS2 8분할 실행 가이드

목표는 Terminator를 `1 2 3 4 / 5 6 7 8` 구조로 열고, 각 창에서 ROS2 환경을 자동 소싱한 뒤 명령어를 입력줄에 미리 채워두는 것이다. 사용자는 명령어를 확인하거나 수정한 뒤 Enter를 눌러 실행한다.

## 파일 구성

```text
tools/terminator/
  ros2_preset_prompt.sh             # ROS2 소싱 + 명령어 사전 입력
  terminator_robocup_8pane.config   # 8분할 Terminator 레이아웃
  open_robocup_terminator.sh        # 워크스페이스 설정 파일로 바로 실행
  install_terminator_config.sh      # ~/.config/terminator/config로 설치
```

## 바로 실행

홈 설정을 건드리지 않고 테스트하려면 아래 명령을 사용한다.

```bash
cd ~/ros2_ws
./tools/terminator/open_robocup_terminator.sh
```

이 방식은 `terminator -g tools/terminator/terminator_robocup_8pane.config -l robocup_8pane`로 실행된다.

## 기본 Terminator 레이아웃으로 설치

`terminator -l robocup_8pane`만으로 실행하고 싶다면 설치 스크립트를 실행한다. 기존 `~/.config/terminator/config`는 자동으로 백업된다.

```bash
cd ~/ros2_ws
./tools/terminator/install_terminator_config.sh
terminator -l robocup_8pane
```

## 창 배치와 사전 입력 명령

```text
1 2 3 4
5 6 7 8
```

| 번호 | 사전 입력 명령 |
|---|---|
| 1 | `ros2 launch all_in_one_package all_in_one_launch.py` |
| 2 | `ros2 run robocup_navigator robocup_navigator --ros-args -p side_mode:=A` |
| 3 | `ros2 launch amr_robot_launch amr_robot.launch.py vision_visualize:=true` |
| 4 | `ros2 run sml_system_pkg mock_wb_node` |
| 5 | `ros2 run sml_system_pkg sml_planning_node_plan_a` |
| 6 | `ros2 run sml_system_pkg sml_manager_node` |
| 7 | `ros2 run sml_system_pkg order_server` |
| 8 | `ros2 topic list` |

주의: 기존 문서의 `sml_planning_node`는 현재 `src/sml_system_pkg/setup.py`에 등록되어 있지 않다. 등록된 실행 파일은 `sml_planning_node_plan_a`, `sml_planning_node_plan_b`이며, 설정 기본값은 `plan_a`다.

## 동작 방식

각 창은 다음 순서로 동작한다.

```text
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
명령어를 프롬프트에 미리 입력
사용자 Enter 입력 후 실행
```

다른 워크스페이스나 ROS 배포판을 써야 하면 실행 전에 환경변수를 지정한다.

```bash
ROS2_WS=/home/moonshot/ros2_ws ROS_DISTRO=humble ./tools/terminator/open_robocup_terminator.sh
```

## 추천 실행 순서

```text
1 -> 3 -> 4 -> 7 -> 5 -> 6 -> 2
```

8번 창은 디버깅용으로 쓰며, 기본값은 `ros2 topic list`다.
