# TURTLEBOT3_GROUP6설치방법
1. turtlebot3_ws/src/turtlebot3에 위 turtlebot3_group6패키지를 위치시킨다<br/>
2. burger.yaml파일을 turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/humble 에 대치한다.<br/>
3. colcon build --symlink-install<br/>
4. source install/setup.bash<br/>
5. 비전 모델은 $HOME/에다가 위치시키면 된다.(학습시킨 모델이 대용량이라 googledrive로 따로올렸습니다.)<br/>
  https://drive.google.com/file/d/1y_w7JW5soeC50vP7OtDYudfFI5iWB7GE/view?usp=sharing
