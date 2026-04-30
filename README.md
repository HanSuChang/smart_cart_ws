ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map_cleaned.yaml


PID제어할 때

ros2 param set /controller_server FollowPath.max_vel_x 0.25 (직진 속도 0.0 ~ 0.25)








ros2 param get /controller_server FollowPath.min_speed_theta  (회전 속도) - 최소
Double value is: 0.0

ros2 param get /controller_server FollowPath.max_vel_theta    (회전 속도) - 최대
Double value is: 1.8 








가속도 제한 (로봇이 덜컥거리면 낮추세요)
ros2 param set /controller_server FollowPath.acc_lim_x 1.5











1. 경로 추종 속도 및 가속도 (기본 성능)

로봇이 너무 느리거나 답답하게 움직이면 이 값을 올리고, 너무 튄다 싶으면 낮추세요.
Bash

# 최대 선속도 (기본 0.22인데 조금씩 올려보세요)
ros2 param set /controller_server FollowPath.max_vel_x 0.25

# 최대 각속도 (회전 속도)
ros2 param set /controller_server FollowPath.max_vel_theta 2.0

# 가속도 제한 (로봇이 덜컥거리면 낮추세요)
ros2 param set /controller_server FollowPath.acc_lim_x 1.5

2. 경로 밀착도 튜닝 (Critics Scale)

DWB에서 PID의 P 게인과 비슷한 역할을 하는 것이 scale입니다.

    PathDist: 경로에 얼마나 딱 붙어서 갈 것인가? (높을수록 경로 이탈 안 함)

    GoalDist: 목적지에 얼마나 빨리 도착할 것인가? (높을수록 경로 무시하고 목적지로 돌진)

Bash

# 경로에 더 딱 붙어서 가게 하려면 (기본값보다 높여보세요)
ros2 param set /controller_server FollowPath.PathDist.scale 40.0

# 목적지 도달보다 경로 유지가 중요하다면 GoalDist를 낮춥니다
ros2 param set /controller_server FollowPath.GoalDist.scale 20.0

3. 목표 지점 정밀도 (Goal Tolerance)

목적지 근처에서 뱅글뱅글 돌거나 멈추지 않을 때 건드립니다.
Bash

# 도착 인정 거리 (0.05 = 5cm 안으로 들어오면 도착)
ros2 param set /controller_server FollowPath.xy_goal_tolerance 0.05

# 도착 후 방향 오차 (0.1 = 약 5.7도)
ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance 0.1


















===============캠 띄우는 법=======================


ssh song@192.168.0.222





(사람 추종 캠)
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video1 \
  -p image_size:=[640,480] \
  -p pixel_format:=YUYV \
  -r __node:=webcam \
  -r /image_raw:=/webcam/image_raw \
  -r /image_raw/compressed:=/webcam/image_raw/compressed





(물체인식 캠)
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video4 \
  -p image_size:=[640,480] \
  -p pixel_format:=YUYV \
  -r __node:=webcam2 \
  -r /image_raw:=/webcam2/image_raw \
  -r /image_raw/compressed:=/webcam2/image_raw/compressed


















