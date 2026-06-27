기본 터미널로 실행하면 아무 명령어도 입력되지 않은 일반 터미널 상태로 열립니다.
즉, Terminator 레이아웃의 각 분할 터미널에 custom command를 따로 넣지 않으면 그냥 빈 bash 터미널처럼 나옵니다.

아래 내용 그대로 Markdown에 복붙하면 됩니다.

# Terminator ROS2 분할 터미널 START GUIDE

## 목적

Ubuntu Terminator의 분할 레이아웃을 사용해서 터미널을 8개로 나누고, 각 터미널에 ROS2 실행 명령어를 미리 입력해둔다.

터미널 구조는 아래와 같다.

```text
1 2 3 4
5 6 7 8

각 터미널에는 아래 명령어를 미리 입력해둔다.

번호	명령어
1	ros2 launch all_in_one_package all_in_one_launch.py
2	ros2 run robocup_navigator robocup_navigator --ros-args -p side_mode:=A
3	ros2 launch amr_robot_launch amr_robot.launch.py vision_visualize:=true
4	ros2 run sml_system_pkg mock_wb_node
5	ros2 run sml_system_pkg sml_planning_node
6	ros2 run sml_system_pkg sml_manager_node
7	ros2 run sml_system_pkg order_server
8	작업용 빈 터미널
1. 기본 개념

Terminator에서 레이아웃만 저장하면 터미널 분할 구조만 저장된다.

즉, 일반적으로 Terminator를 기본 터미널처럼 실행하면 아무 명령어도 입력되지 않은 빈 터미널 상태로 열린다.

terminator

이 경우 각 터미널은 일반 bash 터미널처럼 열린다.

명령어를 미리 입력해두고 싶다면 Terminator 레이아웃의 각 터미널에 Custom command를 설정해야 한다.

2. 추천 방식

ROS2 노드는 실행 순서가 중요할 수 있으므로, 완전 자동 실행보다는 아래 방식이 안전하다.

터미널이 열리면 명령어가 미리 입력되어 있음
↓
사용자가 Enter를 누르면 실행됨

이 방식은 다음과 같은 장점이 있다.

실행 전에 명령어 확인 가능
필요하면 명령어 수정 가능
ROS2 노드 실행 순서를 직접 제어 가능
센서, 시리얼, launch 순서 문제를 줄일 수 있음
3. 공통 스크립트 생성

먼저 Terminator에서 사용할 명령어 프리셋 스크립트를 만든다.

mkdir -p ~/.terminator_cmds
nano ~/.terminator_cmds/ros2_preset_prompt.sh

아래 내용을 그대로 넣는다.

#!/usr/bin/env bash

cd ~/ros2_ws || {
  echo "[ERROR] ~/ros2_ws 경로를 찾을 수 없습니다."
  exec bash
}

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "[WARN] ~/ros2_ws/install/setup.bash 파일이 없습니다."
  echo "       colcon build 또는 source 경로를 확인하세요."
fi

CMD="$*"

echo
echo "======================================"
echo " ROS2 COMMAND PRESET"
echo "======================================"
echo

read -e -i "$CMD" -p "$ " USER_CMD

if [ -n "$USER_CMD" ]; then
  eval "$USER_CMD"
fi

exec bash

저장 후 실행 권한을 준다.

chmod +x ~/.terminator_cmds/ros2_preset_prompt.sh
4. Terminator 각 분할 터미널 Custom Command 설정

Terminator 레이아웃에서 각 터미널의 Custom command에 아래 명령어를 넣는다.

1번 터미널
bash -lc '~/.terminator_cmds/ros2_preset_prompt.sh "ros2 launch all_in_one_package all_in_one_launch.py"'
2번 터미널
bash -lc '~/.terminator_cmds/ros2_preset_prompt.sh "ros2 run robocup_navigator robocup_navigator --ros-args -p side_mode:=A"'
3번 터미널
bash -lc '~/.terminator_cmds/ros2_preset_prompt.sh "ros2 launch amr_robot_launch amr_robot.launch.py vision_visualize:=true"'
4번 터미널
bash -lc '~/.terminator_cmds/ros2_preset_prompt.sh "ros2 run sml_system_pkg mock_wb_node"'
5번 터미널
bash -lc '~/.terminator_cmds/ros2_preset_prompt.sh "ros2 run sml_system_pkg sml_planning_node"'
6번 터미널
bash -lc '~/.terminator_cmds/ros2_preset_prompt.sh "ros2 run sml_system_pkg sml_manager_node"'
7번 터미널
bash -lc '~/.terminator_cmds/ros2_preset_prompt.sh "ros2 run sml_system_pkg order_server"'
8번 터미널

8번은 디버깅용 빈 터미널로 사용하는 것을 추천한다.

bash -lc 'cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash; exec bash'

8번 터미널에서는 아래와 같은 디버깅 명령어를 필요할 때 직접 입력한다.

ros2 topic list
ros2 node list
ros2 service list
ros2 action list
ros2 topic echo /cmd_vel
ros2 topic echo /tf
ros2 run rqt_graph rqt_graph
5. Terminator 레이아웃 실행

저장된 Terminator 레이아웃 이름이 robocup_layout이라면 아래처럼 실행한다.

terminator -l robocup_layout

그러면 각 터미널에 명령어가 미리 입력된 상태로 열린다.

예시:

$ ros2 launch all_in_one_package all_in_one_launch.py

이 상태에서 Enter를 누르면 실행된다.

6. 추천 실행 순서

전체 노드를 한 번에 실행하기보다는 아래 순서로 실행하는 것을 추천한다.

1번 실행
↓
3번 실행
↓
4번 실행
↓
7번 실행
↓
5번 실행
↓
6번 실행
↓
2번 실행
7. 각 터미널 역할 정리
번호	명령어	역할
1	ros2 launch all_in_one_package all_in_one_launch.py	AMR 기본 구동 launch
2	ros2 run robocup_navigator robocup_navigator --ros-args -p side_mode:=A	Robocup navigator 실행
3	ros2 launch amr_robot_launch amr_robot.launch.py vision_visualize:=true	AMR robot launch 및 vision visualize
4	ros2 run sml_system_pkg mock_wb_node	Mock WB node 실행
5	ros2 run sml_system_pkg sml_planning_node	Planning node 실행
6	ros2 run sml_system_pkg sml_manager_node	Manager node 실행
7	ros2 run sml_system_pkg order_server	Order server 실행
8	빈 터미널	디버깅, topic echo, node 확인용
8. 완전 자동 실행 방식

Enter 입력 없이 Terminator가 열리자마자 명령어가 바로 실행되게 할 수도 있다.

하지만 ROS2 시스템에서는 실행 순서, 센서 연결, 포트 연결, launch 준비 시간 문제가 있을 수 있으므로 권장하지 않는다.

그래도 자동 실행이 필요하면 아래 스크립트를 만든다.

nano ~/.terminator_cmds/ros2_preset_auto.sh

내용은 아래와 같다.

#!/usr/bin/env bash

cd ~/ros2_ws || {
  echo "[ERROR] ~/ros2_ws 경로를 찾을 수 없습니다."
  exec bash
}

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "[WARN] ~/ros2_ws/install/setup.bash 파일이 없습니다."
fi

CMD="$*"

echo
echo "======================================"
echo " AUTO RUN"
echo "======================================"
echo "$CMD"
echo

eval "$CMD"

exec bash

권한을 준다.

chmod +x ~/.terminator_cmds/ros2_preset_auto.sh

예를 들어 1번 터미널을 자동 실행하려면 Custom Command를 아래처럼 설정한다.

bash -lc '~/.terminator_cmds/ros2_preset_auto.sh "ros2 launch all_in_one_package all_in_one_launch.py"'
9. 정리
기본 Terminator 실행
terminator

결과:

빈 터미널로 열림
명령어 자동 입력 없음
레이아웃 실행
terminator -l robocup_layout

결과:

저장된 분할 레이아웃으로 열림
각 터미널에 Custom Command가 있으면 해당 명령어가 준비됨
가장 추천하는 방식
분할 레이아웃 저장
+
각 터미널에 Custom Command 설정
+
명령어는 미리 입력만 해두고 Enter로 실행

이 방식이 대회/실험 환경에서 가장 안정적이다.


정리하면, **기본 터미널로 키면 아무것도 입력 안 된 상태가 맞고**, 위처럼 `Custom command`를 넣은 Terminator 레이아웃으로 실행할 때만 명령어가 미리 준비됩니다.