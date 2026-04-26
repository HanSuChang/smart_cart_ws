# Smart Cart — TurtleBot3 Waffle Pi 스마트 쇼핑 카트

## 프로젝트 개요

TurtleBot3 Waffle Pi 기반 자율 추종 쇼핑 카트. 사람을 인식하고 따라다니며, 물체를 인식하면 바구니 뚜껑이 자동으로 열린다.

## 하드웨어

| 장치 | 용도 |
|---|---|
| TurtleBot3 Waffle Pi | 메인 로봇 (LDS-02 + RPi4 + OpenCR) |
| USB 웹캠 | 사람 추종 (YOLOv8 + DeepSORT) |
| RPi 카메라 | 물체 인식 (Roboflow 학습 모델) |
| MG996R 서보 x2 + 메탈 브라켓 | 웹캠 Pan/Tilt 회전 |
| MG996R 서보 x1 | 바구니 뚜껑 개폐 |
| 바구니 (실 연결) | TurtleBot3 뒤에서 끌고 다님 |
| 안드로이드 앱 (별도) | 상품 QR 스캔 결제 |

## 팀 역할

| 패키지 | 담당 | 설명 |
|---|---|---|
| `sc_interfaces` | 공통 | PersonBbox, ItemDetected, ServoControl, CartStatus msg |
| `sc_cpp` | C++ 담당 | follow_controller, safety_monitor, lid_controller, pan_tilt_controller, status_publisher |
| `sc_python` | Python 담당 | person_tracker (YOLOv8+DeepSORT), item_classifier (Roboflow) |
| `sc_gui` | GUI 담당 | 카트 상태 실시간 표시 |
| `sc_bringup` | 공통 | launch, config, rviz, urdf, worlds |
| `firmware/` | HW 담당 | OpenCR 서보 제어 (PlatformIO) |

## 토픽 인터페이스

```
[Python → C++]
  /person_bbox    sc_interfaces/msg/PersonBbox
  /item_detected  sc_interfaces/msg/ItemDetected

[C++ → Hardware]
  /cmd_vel        geometry_msgs/msg/Twist
  /servo_control  sc_interfaces/msg/ServoControl

[C++ 내부]
  /safety_stop    std_msgs/msg/Bool
  /item_confirm   std_msgs/msg/String

[C++ → GUI]
  /cart_status    sc_interfaces/msg/CartStatus
```

## 빠른 시작

```bash
# 1. 의존성
pip install -r requirements.txt

# 2. 빌드
chmod +x build.sh && ./build.sh

# 3. 실행
ros2 launch sc_bringup robot.launch.py            # 실제 로봇
ros2 launch sc_bringup sim.launch.py               # Gazebo
ros2 launch sc_bringup robot.launch.py debug:=true  # 디버그 영상

# 4. PID 실시간 튜닝
ros2 param set /follow_controller linear_kp 0.003
ros2 param set /follow_controller target_bbox_height 280.0
```

## 빌드 순서 (반드시 지키기)

```
1. sc_interfaces  →  2. sc_cpp  →  3. sc_python / sc_gui  →  4. sc_bringup
```

## Git 워크플로우

```bash
git checkout -b feature/내기능명
# 작업 후
git add src/sc_cpp/
git commit -m "feat: follow_controller PID 구현"
git push origin feature/내기능명
# → PR 생성 → 리뷰 → merge
```
