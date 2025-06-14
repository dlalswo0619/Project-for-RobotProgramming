# Project-for-RobotProgramming
터틀봇3 자동화 물류이동 프로젝트
프로젝트 플로우
1. 시작점에서 카메라를 이용한 물체인식
2. 인식한 물체 순서대로 목표점 토픽발행
3. path_planning알고리즘을 이용한 목표점 방문
4. 시작점 복귀

//Project Demo(real world)
실행 순서
[TURTLEBOT3_SBC]
ssh pi@***.***.***.***
cd turtlebot_ws/
ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/camera


새로운 터미널을 연후

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py

[REMOTE_PC]
cd turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml

//출발지에서 2d pose estimate

[REMOTE_PC]

ros2 launch turtlebot3_group6 lift.launch.py
// 물체인식 후 경로탐색 시작

//중단 서비스 실행
ros2 service call /pause_navigation std_srvs/srv/SetBool "{data: true}"
//재개 서비스 실행
ros2 service call /pause_navigation std_srvs/srv/SetBool "{data: false}"

