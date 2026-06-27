# AMR 맵 생성 및 웨이포인트 설정 절차

## 1. 새로운 매핑

```sh
ros2 launch all_in_one_package generate_map_launch.py
```

---

## 2. 매핑한 이후에 맵 저장

```sh
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/amr/map/*맵이름*
```

---

## 3. 맵이름 변경

`all_in_one_package all_in_one_launch.py`

```python
'''
Fourth Launch: amr localization_launch after 15 seconds
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('amr'), 
                            'launch', 
                            'nav2_bringup',
                            'localization_launch.py'
                        )
                    ),
                    launch_arguments={'map': os.path.expandvars('$HOME/ros2_ws/src/amr/map/*맵이름*.yaml')}.items(),
                ),
            ]
        ),
'''
```

---

## 4. `ros2 launch all_in_one_package all_in_one_launch.py` 실행

`for 웨이포인트 찍어야지`

```sh
ros2 launch all_in_one_package all_in_one_launch.py
```

---

## 5. 웨이포인트

```sh
ros2 run amr_navigator goal_pose_generator --ros-args   -p output_file:=/home/st02/ros2_ws/src/robocup_navigator/params/robocup_waypoint.yaml
```

```sh
저장되는 16개 웨이포인트 이름 예시 ~
```

---

## 6. 새롭게 만든 yaml 파일 `robocup/navigator.py` 에서 yaml 파일 수정

```python
        default_yaml = str(
            Path(get_package_share_directory('robocup_navigator'))
            / 'params'
            / '*파일이름지정*.yaml'
        )
```

---

## 7. 마커 업데이트

`all_in_one_package all_in_one_launch.py`

```python
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='amr_navigator',
                    executable='waypoint_marker_publisher',
                    name='waypoint_marker_publisher',
                    output='screen',
                    parameters=[{
                        'waypoints_file': '/home/st02/ros2_ws/src/robocup_navigator/params/*파일이름지정*.yaml'
                    }],
                ),
            ]
        ),
```

---

## 8. final test

```sh
ros2 launch amr_navigator yaml_autonomous_waypoint_follower.launch.py waypoints_file:=/home/st02/ros2_ws/src/robocup_navigator/params/*파일이름지정*.yaml
```
