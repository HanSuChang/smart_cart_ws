# Smart Cart launch 가이드

## ⚠️ 중요: 한 launch 로 전체 시스템 가동 불가능
Pi 와 PC 는 **별도 컴퓨터**입니다. launch 명령은 그 머신에서만 노드를 띄울 수 있어요.
ROS2 토픽은 네트워크로 자동 공유되지만, 프로세스는 각 머신에서 따로 시작해야 합니다.

따라서 **최소 2개 터미널** (Pi + PC) 필요. RViz2 까지 띄우면 3개.

---

## ✅ 실행 패턴 A — 권장 (sc_bringup 통합)

### 터미널 1: 라즈베리파이4 (SSH 접속)
```bash
ssh pi@<라즈베리파이_IP>
cd ~/smart_cart_ws/smart_cart_ws
source install/setup.bash
export ROS_DOMAIN_ID=27 ROS_LOCALHOST_ONLY=0

ros2 launch sc_bringup rpi.launch.py
# → 자동으로 다음 노드 가동:
#   • turtlebot3_bringup  (LDS-02 + 모터 + IMU)
#   • usb_cam x 2        (사람추종 + 바구니 카메라)
#   • safety_monitor     (LiDAR 비상정지)
#   • follow_controller  (50cm + Kalman → /cmd_vel)
#   • pan_tilt_controller (/servo_control id=0,1)
#   • lid_controller     (/servo_control id=2)
```

### 터미널 2: 우분투 PC
```bash
cd ~/smart_cart_ws/smart_cart_ws
source install/setup.bash
export ROS_DOMAIN_ID=27 ROS_LOCALHOST_ONLY=0

ros2 launch sc_bringup pc.launch.py
# → 자동으로 다음 노드 가동:
#   • person_tracker   (YOLOv8n + BoT-SORT + HSV)
#   • item_classifier  (YOLOv8n)
#   • basket_vision    (OpenCV 모션 감지)
#   • status_publisher (통합 상태 → GUI)
#   • rosbridge_websocket (port 9090)
#   • Nav2             (자동 주행)
#   • frictionless_gui (PyQt5 + Flask)
```

### (옵션) 터미널 3: PC — RViz2 (2D Pose Estimate 용)
```bash
rviz2
# 또는 nav2_bringup 의 기본 rviz 사용:
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

---

## 실행 패턴 B — Robotis 표준 (turtlebot3_bringup 따로)

### 터미널 1 (Pi SSH): turtlebot3_bringup 만
```bash
ssh pi@<IP>
ros2 launch turtlebot3_bringup robot.launch.py
```

### 터미널 2 (Pi SSH): 나머지 Pi 노드들
```bash
ssh pi@<IP>
ros2 launch sc_bringup rpi.launch.py run_tb3:=false
# → turtlebot3_bringup 빼고 카메라/safety/follow/pan_tilt/lid 만
```

### 터미널 3 (PC):
```bash
ros2 launch sc_bringup pc.launch.py
```

### (옵션) 터미널 4 (PC):
```bash
rviz2
```

---

## 🔍 동작 시나리오

1. **PC에서 GUI 가 뜸** → 맵 (map.pgm) 자동 로드, "사람 추종 카메라" / "바구니 카메라" 검은 화면 (NO SIGNAL)
2. 잠시 후 (Pi 카메라 송출 시작 / YOLO 부팅) → 두 카메라 화면에 영상 + YOLO 라벨링 박스 뜸
3. **(선택) RViz2 에서 "2D Pose Estimate" 클릭** → 맵 위 로봇 현재 위치 클릭+드래그
4. AMCL 가 `/amcl_pose` 발행 시작 → GUI 맵 위에 녹색 로봇 점 표시
5. GUI 상단 **"화장실 노드 정하기"** 클릭 → 맵 좌클릭 → 파란점
   GUI 상단 **"충전소 노드 정하기"** 클릭 → 맵 좌클릭 → 빨간점
   GUI 상단 **"노드 저장"** 클릭 → `~/.smart_cart/waypoints.json` 영구 저장
6. GUI 좌측 **"화장실 이동"** → "네/아니오" → "네" → Nav2 가 경로 계산 → 로봇 이동
7. GUI 좌측 **"사람 추종 시작"** → 20초 학습 + 5초 카운트다운 → 자동으로 추종 시작

---

## 🌐 네트워크 검증
```bash
# Pi 와 PC 가 서로의 토픽을 보는지 (양쪽 다)
ros2 topic list | grep -E "webcam|scan|odom|cmd_vel|person_bbox"

# 영상 송수신
ros2 topic hz /webcam/image_raw       # Pi → PC, 약 30Hz
ros2 topic hz /yolo/follow_image/compressed  # PC 안에서, 약 30Hz

# 사람 박스 송수신
ros2 topic hz /person_bbox            # PC → Pi 로 갈 때 정상

# 모터 명령
ros2 topic echo /cmd_vel              # Pi 안에서 발행되는 것 보임
```
