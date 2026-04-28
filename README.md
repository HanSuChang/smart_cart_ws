ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$Home/Downlads/map_cleaned.yaml



PID제어할 때

ros2 param set /controller_server FollowPath.max_vel_x 0.25 (직진 속도 0.0 ~ 0.25)








ros2 param get /controller_server FollowPath.min_speed_theta  (회전 속도) - 최소
Double value is: 0.0

ros2 param get /controller_server FollowPath.max_vel_theta    (회전 속도) - 최대
Double value is: 1.8 








가속도 제한 (로봇이 덜컥거리면 낮추세요)
ros2 param set /controller_server FollowPath.acc_lim_x 1.5
