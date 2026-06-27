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

설치 후에는 `terminator -u -l robocup_8pane`로 실행한다. `-u`는 이미 떠 있는 Terminator 인스턴스를 재사용하지 않고 새 창으로 레이아웃을 열게 한다.

```bash
cd ~/ros2_ws
./tools/terminator/install_terminator_config.sh
terminator -u -l robocup_8pane
```

## 다른 컴퓨터에 동일하게 설치

전제 조건은 다음과 같다.

```text
Ubuntu 환경
Terminator 설치됨
ROS2 Humble 설치됨: /opt/ros/humble/setup.bash
워크스페이스 경로: ~/ros2_ws
워크스페이스 빌드 완료: ~/ros2_ws/install/setup.bash 존재
```

Terminator가 설치되어 있지 않으면 먼저 설치한다.

```bash
sudo apt update
sudo apt install terminator
```

새 컴퓨터에 이 저장소 또는 `ros2_ws`를 같은 위치로 복사한다.

```bash
cd ~
# 예시: 저장소를 git으로 받는 경우
git clone <REPOSITORY_URL> ros2_ws
```

워크스페이스를 빌드한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Terminator 설정 파일과 실행 스크립트 권한을 준비한다.

```bash
cd ~/ros2_ws
chmod +x tools/terminator/*.sh
./tools/terminator/install_terminator_config.sh
```

설치 후 실행한다.

```bash
terminator -u -l robocup_8pane
```

정상 동작하면 8분할 창이 열리고 각 창에 명령어가 입력된 상태로 대기한다. 실제 ROS 노드는 Enter를 누를 때 실행된다.

### 다른 경로에 설치한 경우

현재 Terminator 설정 파일은 `/home/moonshot/ros2_ws/tools/terminator/ros2_preset_prompt.sh`를 직접 참조한다. 다른 컴퓨터의 사용자명이 다르거나 워크스페이스 경로가 다르면 아래 파일에서 경로를 새 컴퓨터에 맞게 바꾼다.

```text
tools/terminator/terminator_robocup_8pane.config
```

예를 들어 새 컴퓨터의 경로가 `/home/robot/ros2_ws`라면 아래처럼 치환한다.

```bash
cd ~/ros2_ws
sed -i 's#/home/moonshot/ros2_ws#/home/robot/ros2_ws#g' tools/terminator/terminator_robocup_8pane.config
./tools/terminator/install_terminator_config.sh
```

그 다음 실행한다.

```bash
terminator -u -l robocup_8pane
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

## 문제 해결

아래 경고는 기존 Terminator 창이 숨김 단축키를 이미 잡고 있다는 뜻이다. 레이아웃 실패 원인은 아니다.

```text
Binding '<Control><Alt>a' failed!
Unable to bind hide_window key, another instance/window has it.
```

이 경고가 보여도 8분할이 열리면 정상이다. 그래도 한 칸만 열리면 디버그 로그를 확인한다.

```bash
terminator -u -d -g /home/moonshot/ros2_ws/tools/terminator/terminator_robocup_8pane.config -l robocup_8pane
```

가장 흔한 실행 실수는 `-u` 없이 실행해서 이미 떠 있는 Terminator 인스턴스에 요청이 전달되는 경우다.

각 pane이 잠깐 생겼다가 바로 닫히면 `tools/terminator/ros2_preset_prompt.sh` 안에서 실행되는 source 단계가 실패한 것이다. ROS setup 파일은 `set -u` 상태에서 내부 변수가 unbound 처리되어 종료될 수 있으므로, 이 래퍼 스크립트에서는 `set -u`를 사용하지 않는다.
