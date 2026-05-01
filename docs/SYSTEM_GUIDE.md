# Smart Cart 시스템 구조 & 기능 설명서 (최종)

ROS2 Humble · TurtleBot3 Waffle Pi · OpenCR 1.0 · YOLOv8n + BoT-SORT + HSV · PyQt5 GUI · Nav2

---

## 목차
1. [전체 아키텍처](#1-전체-아키텍처)
2. [패키지 구조](#2-패키지-구조)
3. [토픽 카탈로그 (Pub/Sub)](#3-토픽-카탈로그-pubsub)
4. [기능별 동작 흐름](#4-기능별-동작-흐름)
5. [핵심 파라미터](#5-핵심-파라미터)
6. [실행 가이드 (분산 launch)](#6-실행-가이드)
7. [OpenCR 하드웨어 배선 (Pan/Tilt/Lid)](#7-opencr-하드웨어-배선)
8. [디버그 명령어](#8-디버그-명령어)
9. [트러블슈팅](#9-트러블슈팅)

---

## 1. 전체 아키텍처

```
┌─────────────────────────────┐    LAN (DOMAIN_ID=27)    ┌─────────────────────────────┐
│  라즈베리파이4 (TurtleBot3)   │ ◀──────────WiFi──────▶  │   우분투 PC (관제)            │
│  rpi.launch.py              │                          │   pc.launch.py              │
│                             │                          │                             │
│  • turtlebot3_bringup       │                          │  • person_tracker (YOLOv8n) │
│    (LDS-02 + 모터 + IMU)     │                          │  • item_classifier (YOLOv8n)│
│  • usb_cam #1 (사람추종)     │ ──/webcam/image_raw──▶   │  • basket_vision (OpenCV)   │
│  • usb_cam #2 (바구니)       │ ──/webcam2/image_raw──▶  │  • status_publisher         │
│  • safety_monitor           │                          │  • rosbridge_websocket      │
│  • follow_controller        │ ◀──/person_bbox─────     │  • Nav2                     │
│  • pan_tilt_controller      │ ◀──/smart_cart/mode──    │  • frictionless_gui (PyQt5) │
│  • lid_controller           │ ◀──/smart_cart/learn──   │  • cart_gui (Flask 결제)     │
│  • micro-ROS Agent          │                          │                             │
└──────────┬──────────────────┘                          └──────────────┬──────────────┘
           │ USB 시리얼                                                  │
           ▼                                                            ▼
   ┌──────────────────┐                                           ┌──────────────┐
   │  OpenCR 1.0      │                                           │   사용자      │
   │  Pan/Tilt/Lid    │                                           │  (GUI 조작)   │
   │  서보 PWM 출력    │                                           └──────────────┘
   └──────────────────┘
```

### 분산 배치 이유
| 노드 | 위치 | 이유 |
|---|---|---|
| YOLOv8n / OpenCV / Nav2 | **PC** | 라즈베리파이4 CPU 부족 |
| 카메라/모터/LiDAR I/O | **Pi** | 하드웨어 직결 |
| follow_controller | **Pi** | /cmd_vel 로컬 발행 → 네트워크 끊겨도 안전 |
| pan_tilt / lid | **Pi** | /servo_control 을 OpenCR로 즉시 전달 |
| GUI / Flask | **PC** | 디스플레이 + 외부 결제 웹 |

---

## 2. 패키지 구조

```
smart_cart_ws/
├── docs/
│   └── SYSTEM_GUIDE.md          ← 본 문서
├── firmware/
│   ├── opencr_pan_tilt/
│   │   ├── README.md
│   │   └── opencr_pan_tilt.ino  ← Pan/Tilt P 제어 펌웨어 (Uno→OpenCR 포팅)
│   ├── opencr_lid/
│   │   ├── README.md
│   │   └── opencr_lid.ino       ← Lid 서보 placeholder
│   └── opencr_servo/            ← 기존 참고용
└── src/
    ├── sc_interfaces/           ← ROS2 메시지/서비스 정의
    │   ├── msg/
    │   │   ├── PersonBbox.msg       사람 좌표 (Python AI → C++ 제어)
    │   │   ├── ItemDetected.msg     YOLO 분류 결과
    │   │   ├── ServoControl.msg     서보 명령 (id=0 Pan, 1 Tilt, 2 Lid)
    │   │   ├── CartStatus.msg       통합 상태
    │   │   ├── TrackerState.msg     Kalman 8-state + 거리 (신규)
    │   │   ├── Waypoint.msg         단일 웨이포인트 (신규)
    │   │   ├── WaypointList.msg     웨이포인트 목록 (신규)
    │   │   ├── PaymentEvent.msg     결제 이벤트 (신규)
    │   │   └── BasketEvent.msg      바구니 OpenCV 이벤트 (신규)
    │   └── srv/
    │       └── SaveWaypoints.srv    웨이포인트 영구 저장 (신규)
    ├── sc_cpp/                  ← 실시간 제어 C++ (Pi 측)
    │   ├── src/
    │   │   ├── follow_controller.cpp     PID + Kalman → /cmd_vel
    │   │   ├── safety_monitor.cpp        LiDAR 비상정지
    │   │   ├── pan_tilt_controller.cpp   사람 좌표 → 서보 (id=0,1)
    │   │   ├── lid_controller.cpp        뚜껑 → 서보 (id=2)
    │   │   ├── status_publisher.cpp      통합 상태
    │   │   └── kalman_filter.cpp         8-state 칼만필터
    │   └── include/sc_cpp/*.hpp
    ├── sc_python/               ← AI/비전 (PC 측)
    │   └── sc_python/
    │       ├── person_tracker.py     YOLO+BoT-SORT+HSV → /person_bbox
    │       ├── item_classifier.py    YOLO 분류 → /item_detected
    │       └── basket_vision.py      OpenCV 모션 → /basket/event
    ├── sc_gui/                  ← PyQt5 + Flask + ROS 브릿지 (PC 측)
    │   └── sc_gui/
    │       ├── main.py               (Flask + PyQt5 + ROS bridge 동시)
    │       ├── robot_gui.py          PyQt5 메인 GUI (단순 버전)
    │       ├── waypoint_manager.py   맵 + 화장실/충전소 노드
    │       └── cart_gui.py           Flask 결제 + ROS 브릿지
    └── sc_bringup/              ← launch / config / map
        ├── launch/
        │   ├── rpi.launch.py        ★ Pi 측 실행
        │   ├── pc.launch.py         ★ PC 측 실행
        │   ├── robot.launch.py      통합(시뮬) launch
        │   └── README.md
        ├── config/                  *.yaml 파라미터
        └── maps/                    map_cleaned.pgm/yaml
```

---

## 3. 토픽 카탈로그 (Pub/Sub)

### 3.1 카메라 영상

| 토픽 | 타입 | Pub | Sub | 설명 |
|---|---|---|---|---|
| `/webcam/image_raw` | sensor_msgs/Image | usb_cam (Pi) | person_tracker | 사람 추종 카메라 raw |
| `/webcam/image_raw/compressed` | sensor_msgs/CompressedImage | usb_cam | GUI (백업) | jpeg 압축 |
| `/webcam2/image_raw` | sensor_msgs/Image | usb_cam (Pi) | item_classifier, basket_vision | 바구니 카메라 raw |
| `/webcam2/image_raw/compressed` | sensor_msgs/CompressedImage | usb_cam | GUI (백업) | jpeg 압축 |
| `/yolo/follow_image/compressed` | sensor_msgs/CompressedImage | **person_tracker** | **GUI** | YOLO 박스 + ID + sim 그려진 사람 추종 화면 |
| `/yolo/item_image/compressed` | sensor_msgs/CompressedImage | **item_classifier** | **GUI** | YOLO 박스 그려진 물체 화면 |
| `/basket/annotated/compressed` | sensor_msgs/CompressedImage | **basket_vision** | **GUI** | OpenCV 모션 박스 |

### 3.2 사람 추종 / 학습

| 토픽 | 타입 | Pub | Sub | 설명 |
|---|---|---|---|---|
| `/person_bbox` | sc_interfaces/PersonBbox | person_tracker | follow_controller, pan_tilt_controller, status_publisher | 사람 좌표 + ID + valid |
| `/smart_cart/learn` | std_msgs/String | **GUI** | person_tracker | "start_with_delay" / "save" / "reset" |
| `/smart_cart/learn_status` | std_msgs/String | person_tracker | GUI | "learning:14" / "wait:3" / "ready" / "idle" |
| `/smart_cart/mode` | std_msgs/String | **GUI**, person_tracker(자동) | follow_controller, person_tracker | "idle" / "follow" / "navigate" |
| `/tracker/state` | sc_interfaces/TrackerState | follow_controller | GUI | Kalman 8-state + 거리 |
| `/follow_status` | std_msgs/String | follow_controller | GUI | idle/following/predicting/lost/recovering |

### 3.3 주행 / 안전

| 토픽 | 타입 | Pub | Sub | 설명 |
|---|---|---|---|---|
| `/scan` | sensor_msgs/LaserScan | LDS-02 | safety_monitor, follow_controller, status_publisher | 360° 라이다 |
| `/cmd_vel` | geometry_msgs/Twist | follow_controller, Nav2 | turtlebot3 모터, status_publisher, GUI | 속도 명령 |
| `/safety_stop` | std_msgs/Bool | safety_monitor, **GUI(E-STOP)** | follow_controller, status_publisher | 비상 정지 |
| `/odom` | nav_msgs/Odometry | turtlebot3 | Nav2, GUI | 오도메트리 |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | AMCL | GUI | 맵 좌표계 위치 |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | RViz2 | AMCL, GUI | 2D Pose Estimate 결과 |
| `/navigate_to_pose` (Action) | nav2_msgs/NavigateToPose | **GUI** | Nav2 | 자동 주행 |

### 3.4 바구니 / 물체 / 결제 / 뚜껑

| 토픽 | 타입 | Pub | Sub | 설명 |
|---|---|---|---|---|
| `/item_detected` | sc_interfaces/ItemDetected | item_classifier | lid_controller, status_publisher, GUI | YOLO 분류 결과 |
| `/basket/event` | sc_interfaces/BasketEvent | basket_vision | lid_controller, GUI | "insert"/"stable" + 스냅샷 경로 |
| `/item_confirm` | std_msgs/String | lid_controller | GUI, cart_gui | 연속 N프레임 확정 물체명 |
| `/lid_state` | std_msgs/String | lid_controller | GUI | "open" / "closed" |
| `/servo_control` | sc_interfaces/ServoControl | pan_tilt_controller(0,1), lid_controller(2) | OpenCR 펌웨어 | 서보 각도 명령 |
| `/smart_cart/destination` | std_msgs/String | **GUI** | lid_controller | "toilet"/"charger" → 자동 잠금 |
| `/payment/event` | sc_interfaces/PaymentEvent | **GUI**, **Flask cart_gui** | lid_controller, GUI | "request"/"paid" |

### 3.5 통합 상태 & 웨이포인트

| 토픽 | 타입 | Pub | Sub | 설명 |
|---|---|---|---|---|
| `/cart_status` | sc_interfaces/CartStatus | status_publisher | GUI | 모든 정보 통합 |
| `/waypoint_list` | sc_interfaces/WaypointList | **GUI** | (전체) | 화장실/충전소 좌표 |

---

## 4. 기능별 동작 흐름

### 4.1 사람 추종 (가장 복잡한 시나리오)

```
[STEP 0] GUI "사람 추종 시작" 클릭
   GUI──/smart_cart/learn "start_with_delay"──▶ person_tracker (PC)

[STEP 1] 15초 학습 (HSV 히스토그램 수집)
   webcam1 ──/webcam/image_raw──▶ person_tracker
     • YOLOv8n 사람 검출
     • BoT-SORT ID 부여
     • 가장 큰 bbox(가장 가까운 사람) HSV 히스토그램 → master_db 누적
     • person_tracker──/yolo/follow_image/compressed──▶ GUI (학습 카운트다운 화면)
     • person_tracker──/smart_cart/learn_status "learning:14"──▶ GUI
     • /person_bbox is_valid=false → follow_controller 정지

[STEP 2] 5초 대기 카운트다운
   • person_tracker──/smart_cart/learn_status "wait:4"──▶ GUI
   • 화면 "WAIT 4s -> FOLLOW"
   • 5초 후 자동:
     person_tracker──/smart_cart/mode "follow"──▶ follow_controller, person_tracker

[STEP 3] 추종 시작 (정상 동작)
   webcam1 ──/webcam/image_raw──▶ person_tracker (PC)
     • YOLO 검출된 모든 사람 중 master_db HSV 유사도 > 0.7 → MASTER 선택
     • person_tracker──/person_bbox (is_valid=true)──WiFi──▶ follow_controller (Pi)
     • person_tracker──/yolo/follow_image/compressed (MASTER 박스)──▶ GUI

   follow_controller (Pi):
     • Kalman.update(x,y,w,h)
     • 거리 추정: dist = target_dist × bbox_h_at_target / pred_h
     • Linear PID: error = dist - 0.8m → /cmd_vel.linear.x
     • Angular PID: error = (image_width/2) - bbox_center_x → /cmd_vel.angular.z
     • follow_controller──/cmd_vel──▶ TurtleBot3 모터 (로컬)
     • follow_controller──/tracker/state, /follow_status "following"──▶ GUI

   pan_tilt_controller (Pi):
     • /person_bbox 받아서 사람 중심 → P 제어 (Kp=0.1)
     • /servo_control(id=0 Pan, id=1 Tilt) ──▶ OpenCR

[STEP 4] 사용자가 카메라 밖으로 나감 (사각지대)
   • person_tracker가 사람 못 찾음 → /person_bbox is_valid=false
   • follow_controller: bbox_callback 에서 update 안 함, control_loop 에서 Kalman.predict() 만 호출
     → 직전 속도로 다음 위치 추정 (등속도 모델)
     → state="predicting", /follow_status 발행
   • prediction_timeout_sec(1.5초) 초과 시:
     → stop_robot() + state="recovering"
     → GUI 에 "복귀 중 — BoT-SORT+HSV 재인식 대기" 표시

[STEP 5] 사용자 재출현
   • 카메라 시야로 다시 들어옴
   • person_tracker 가 YOLO + BoT-SORT (NEW ID) 검출
   • 학습된 master_db (HSV) 와 매칭 → similarity > 0.7
   • "이 사람이 그 사용자!" → /person_bbox is_valid=true
   • follow_controller가 Kalman.update() 호출 → 정상 추종 복귀
```

### 4.2 화장실 이동 (안전 잠금)

```
GUI "화장실 이동" 클릭
   ↓
QMessageBox "화장실로 이동하시겠습니까?" Yes/No
   ↓ Yes
GUI──/smart_cart/destination "toilet"──▶ lid_controller (Pi)
GUI──/navigate_to_pose (action)──▶ Nav2 (PC)
   ↓
[lid_controller]
   close_lid("destination:toilet")
   → /servo_control(id=2, angle=0) → OpenCR → Lid 서보 닫힘
   → /lid_state "closed" 발행
   → GUI 라벨 "뚜껑: closed"
   ↓
[Nav2]
   /amcl_pose 보면서 경로 계산
   /cmd_vel ──WiFi──▶ Pi 모터
   ↓ 도착
goal.on('result') → GUI 로그 "[NAV2] arrived"
```

### 4.3 충전소 이동
GUI "충전소 이동" 클릭 (알림창 없음, 즉시) → `/smart_cart/destination "charger"` + `/navigate_to_pose` 액션 → 충전소 좌표로 자동 주행.

### 4.4 결제

```
GUI "결제" 클릭
   ↓
GUI──/payment/event "request"──▶ lid_controller, GUI(자기자신)
GUI──webbrowser.open()──▶ http://127.0.0.1:5000 (Flask)
   ↓
사용자 휴대폰으로 QR 스캔
   ↓
Flask /process_payment → 영수증 페이지
   ↓
cart_gui (CartRosBridgeNode)──/payment/event "paid"──▶ lid_controller, GUI
   ↓
[lid_controller] close_lid("payment_paid") → /servo_control(id=2) → 잠금
[GUI] QMessageBox "결제 완료, 금액: XXX원"
```

### 4.5 바구니에 물건 담기

```
사용자가 바구니에 물건 넣음
   ↓
webcam2 ──/webcam2/image_raw──▶ basket_vision (PC, OpenCV)
   • MOG2 배경차분 → 모션 픽셀 > threshold → in_motion=True
   • 모션 종료 후 0.5초 안정화 → "insert" 이벤트 확정
   • 스냅샷 저장 (~/.smart_cart/basket_snapshots/basket_<TS>.jpg)
   • basket_vision──/basket/event "insert"+snapshot──▶ lid_controller, GUI
   ↓
   [동시에]
   webcam2 ──/webcam2/image_raw──▶ item_classifier (PC, YOLO)
   • YOLOv8n predict → bbox + 클래스 + 신뢰도
   • item_classifier──/item_detected──▶ lid_controller, GUI
   ↓
[lid_controller]
   같은 item_name 이 confirm_frames(5) 이상 연속 + confidence > 0.7
   → open_lid → /servo_control(id=2, angle=120) → 뚜껑 OPEN
   → /item_confirm "<name>" 발행
   → 5초 타이머 → 자동 close
[GUI / Flask]
   /item_detected 받아 Flask /api/add_item POST → 장바구니 자동 추가
```

### 4.6 비상 정지

```
GUI "EMERGENCY STOP" 클릭
   ↓
GUI──/smart_cart/mode "idle"──▶ follow_controller (cmd_vel 발행 중지)
GUI──/safety_stop True──▶ follow_controller, status_publisher

[독립 안전 루프 — 네트워크 끊겨도 동작]
LDS-02 ──/scan──▶ safety_monitor (Pi 로컬)
   • 전방 ±20도 0.35m 이내 장애물 감지
   • safety_monitor──/safety_stop True──▶ follow_controller (Pi 로컬)
   • follow_controller: stop_robot()
```

### 4.7 웨이포인트 설정 / 저장

```
GUI 맵 패널 "화장실 노드 정하기" 토글 ON
   ↓
사용자가 맵 좌클릭
   • 픽셀 → map frame 변환 (origin, resolution)
   • WaypointDB.set_waypoint('toilet', x, y) → ~/.smart_cart/waypoints.json 저장
   • 파란점 그려짐, mode 자동 OFF

"노드 저장" 버튼
   • GUI──/waypoint_list──▶ (전체) 발행
   • 다음 launch 재시작에도 자동 로드
```

### 4.8 RViz2 2D Pose Estimate 연동

```
RViz2 실행 (rviz2)
   ↓
"2D Pose Estimate" 도구 → 맵 위 클릭+드래그
   ↓
RViz2──/initialpose──▶ AMCL (Nav2)
                    └──▶ GUI (로그 표시)
   ↓
AMCL 위치 추정 시작
AMCL──/amcl_pose──▶ GUI
   ↓
GUI 맵 위 녹색 점이 실시간 갱신
```

---

## 5. 핵심 파라미터

### 5.1 `tracker_params.yaml` (person_tracker — PC)
```yaml
learning_duration: 15.0          # 학습 시간 (초)
post_learn_delay:  5.0           # 학습 후 follow 전환 전 대기
similarity_threshold: 0.7        # HSV Re-ID 임계값
img_size: 320                    # YOLO 추론 해상도
```

### 5.2 `follow_params.yaml` (follow_controller — Pi)
```yaml
target_distance_m: 0.8           # ★ 사용자와 80cm 유지
bbox_height_at_target: 150.0     # 80cm 거리에서 bbox 픽셀 높이 (캘리브레이션)
linear_kp / kd: 1.5 / 0.3        # 거리 PID
angular_kp / kd: 0.004 / 0.001   # 좌우 정렬 PID
max_linear_vel:  0.22            # 최대 전진 속도 (m/s)
bbox_timeout_sec: 1.5            # 사람 못 찾으면 정지
prediction_timeout_sec: 1.5      # Kalman 예측 유지
min_safe_dist:   0.25            # LiDAR 안전 거리
```

### 5.3 `safety_params.yaml`
```yaml
stop_distance_m:  0.35           # 전방 장애물 감지 거리
front_angle_deg:  30             # 전방 감시 각도 (±15°)
```

### 5.4 `lid_params.yaml`
```yaml
confidence_threshold: 0.70
confirm_frames:       5          # 연속 N프레임 확정
open_angle:    120.0
close_angle:   0.0
open_duration_sec: 5.0           # 자동 닫기 타이머
```

### 5.5 `servo_params.yaml` (pan_tilt_controller)
```yaml
pan_kp:  0.1                     # 좌우 P 제어
tilt_kp: 0.08
pan_min/max:  0/180              # Pan 범위
tilt_min/max: 45/135             # Tilt 범위 (전신 담기용)
```

---

## 6. 실행 가이드

### 6.1 환경 설정 (양쪽 머신 공통, 한 번만)
```bash
echo 'export ROS_DOMAIN_ID=27' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/smart_cart_ws/smart_cart_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 6.2 빌드
```bash
cd ~/smart_cart_ws/smart_cart_ws
colcon build --symlink-install
source install/setup.bash
```

### 6.3 실행 — 최소 2개 터미널 필요

**터미널 1 — Pi (SSH)**
```bash
ssh pi@<라즈베리파이_IP>
ros2 launch sc_bringup rpi.launch.py
```

**터미널 2 — PC**
```bash
ros2 launch sc_bringup pc.launch.py
```

**터미널 3 (옵션) — PC: RViz2**
```bash
rviz2
```

---

## 7. OpenCR 하드웨어 배선

### 7.1 OpenCR 1.0 보드 핀 맵 (TurtleBot3 Waffle Pi 표준)

OpenCR 1.0 은 **STM32F746ZGT6 (ARM Cortex-M7, 216MHz)** + Arduino 호환 핀 헤더 + 5V 외부 전원 입력.

```
┌──────────────────────────────────────────────────────┐
│                   OpenCR 1.0 (윗면)                  │
│                                                      │
│  [LIPO 11.1V 입력]              [USB-C / micro-USB]   │
│                                                      │
│  ┌──────────┐                  ┌──────────────┐      │
│  │ DXL Bus  │                  │ Arduino 핀   │      │
│  │ (모터)    │                  │ 헤더 (D0~D54)│      │
│  └──────────┘                  └──────────────┘      │
│                                                      │
│  ┌─────────────────────────────────────────────┐     │
│  │ User Pin Header (5V/3.3V/GND/GPIO 다수)      │     │
│  └─────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────┘
```

### 7.2 ★ Pan/Tilt/Lid 서보 배선 (MG996R x 3)

#### 권장 핀 배정 (Servo.h 라이브러리 기준)

| 서보 | 역할 | OpenCR 핀 | Arduino 매핑 | 신호선 색 (일반) |
|---|---|---|---|---|
| Servo 0 | **Pan (좌우)** | `BDPIN_GPIO_4` | **D6** | 노란색/주황색 |
| Servo 1 | **Tilt (상하)** | `BDPIN_GPIO_6` | **D9** | 노란색/주황색 |
| Servo 2 | **Lid (뚜껑)** | `BDPIN_GPIO_8` | **D10** | 노란색/주황색 |

**※ `firmware/opencr_pan_tilt/opencr_pan_tilt.ino` 의 `PAN_PIN`, `TILT_PIN` 상수가 이 값과 맞아야 합니다.**

```cpp
// 펌웨어 안의 핀 정의
const int PAN_PIN  = 6;   // Pan
const int TILT_PIN = 9;   // Tilt
// (lid 는 firmware/opencr_lid/opencr_lid.ino — LID_PIN = 10)
```

#### MG996R 3선 배선

```
MG996R 서보 (배선 색상):
   ┌─ 빨강(VCC)  ──▶ 외부 전원 +5V/+6V (OpenCR 5V는 약함, 별도 BEC/UBEC 권장)
   ├─ 갈색(GND)  ──▶ 외부 전원 GND  +  OpenCR GND (공통 GND 필수!)
   └─ 노랑(SIG)  ──▶ OpenCR D6 / D9 / D10
```

#### 외부 전원 권장 사양
- 전압: **5.0~6.0 V**
- 전류: **각 서보 stall 2.5A** → 3개 합쳐서 최소 **5A** 이상 BEC 또는 별도 5V 어댑터
- ⚠ **OpenCR 의 5V 출력 직결 금지** — MG996R 의 stall 전류가 OpenCR 보호회로 트립

#### 배선도 (텍스트)

```
                                    External BEC/UBEC
                                    (5V ~ 6V, 5A 이상)
                                    ┌───┬───┐
                                    │ + │ - │
                                    └─┬─┴─┬─┘
                                      │   │
       ┌──────────────────────────────┼───┼─────────────────┐
       │ 모든 서보 VCC(빨강)           │   │  모든 서보 GND   │
       │ ◀──────────── + ─────────────┘   └─── - ──────▶   │
       │                                                    │
       │                  ┌─── OpenCR GND (공통)  ──────────┤
       │                  │                                 │
       │   Pan  Servo:    │                                 │
       │     VCC ●────────┴─┐ (외부 +5V)                    │
       │     GND ●──────────┴─┐                             │
       │     SIG ●─────────────────▶ OpenCR D6              │
       │                                                    │
       │   Tilt Servo:                                      │
       │     VCC ●─────────────                             │
       │     GND ●─────────────                             │
       │     SIG ●─────────────────▶ OpenCR D9              │
       │                                                    │
       │   Lid Servo:                                       │
       │     VCC ●─────────────                             │
       │     GND ●─────────────                             │
       │     SIG ●─────────────────▶ OpenCR D10             │
       └────────────────────────────────────────────────────┘
```

### 7.3 OpenCR ↔ 라즈베리파이 통신 (USB 시리얼)

```
OpenCR 1.0  ──── USB 케이블 ────▶  Raspberry Pi 4 USB 포트
   /dev/ttyACM0 (Pi 측)
        ▲
        │ 시리얼 명령:  "P:120\n"  Pan 목표 90도
        │              "T:80\n"   Tilt 목표 80도
        │              "L:0\n"    Lid 닫기
        │              "L:120\n"  Lid 열기
        │
   pan_tilt_controller (sc_cpp) → /servo_control 발행
                  │
   ┌──────────────┴───────────┐
   │ 옵션 A: 시리얼 브릿지 노드  │  /servo_control 구독 → "P:120\n" 시리얼 송출
   │ 옵션 B: micro-ROS Agent   │  OpenCR 펌웨어가 /servo_control 직접 구독
   └──────────────────────────┘
```

#### 옵션 A — 단순 시리얼 브릿지 (권장 — 펌웨어 단순)

OpenCR 펌웨어는 **Servo.h 만 사용**. 별도 시리얼 브릿지 ROS2 노드(또는 단순 Python 스크립트)가 `/servo_control` 을 구독해서 시리얼로 변환:

```python
# /servo_control 구독 → 시리얼로 "P:120\n" 송출 (간단 예시)
import serial, rclpy
from sc_interfaces.msg import ServoControl
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
def cb(msg):
    prefix = {0:'P', 1:'T', 2:'L'}.get(msg.servo_id)
    if prefix:
        ser.write(f"{prefix}:{int(msg.angle)}\n".encode())
```

#### 옵션 B — micro-ROS (OpenCR 가 직접 ROS 노드)

```bash
sudo apt install ros-humble-micro-ros-agent
# Pi 에서:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```
이 경우 OpenCR 펌웨어를 micro-ROS 라이브러리로 다시 빌드해야 함.

### 7.4 펌웨어 업로드

```bash
# 1) Arduino IDE 에 OpenCR 보드 패키지 추가
#    File > Preferences > Additional Board URLs:
#    https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

# 2) Tools > Board > OpenCR Board > OpenCR
# 3) Tools > Port > /dev/ttyACM0 (Pi 또는 PC USB 연결)

# 4) firmware/opencr_pan_tilt/opencr_pan_tilt.ino 열고 → 업로드
# 5) firmware/opencr_lid/opencr_lid.ino 도 같은 방식
#    (※ Pan/Tilt 와 Lid 를 한 펌웨어로 합치는 것을 권장 —
#     펌웨어 슬롯이 1개라서)
```

### 7.5 동작 검증

```bash
# 시리얼 직접 테스트 (아무 ROS2 없이)
echo "P:90"  > /dev/ttyACM0    # Pan 90도
echo "P:0"   > /dev/ttyACM0    # Pan 0도
echo "T:135" > /dev/ttyACM0    # Tilt 135도
echo "L:120" > /dev/ttyACM0    # Lid 열기
echo "L:0"   > /dev/ttyACM0    # Lid 닫기

# ROS2 토픽으로 테스트
ros2 topic pub /servo_control sc_interfaces/ServoControl \
  "{servo_id: 0, angle: 90.0, speed: 1.0}" --once
```

---

## 8. 디버그 명령어

```bash
# 모든 토픽
ros2 topic list

# 데이터 실시간 확인
ros2 topic echo /follow_status
ros2 topic echo /tracker/state
ros2 topic echo /smart_cart/learn_status
ros2 topic echo /lid_state
ros2 topic echo /basket/event
ros2 topic echo /cart_status

# 발행 주기
ros2 topic hz /webcam/image_raw                # ~30Hz 정상
ros2 topic hz /yolo/follow_image/compressed    # ~30Hz 정상
ros2 topic hz /person_bbox                     # ~30Hz 정상
ros2 topic hz /cmd_vel                         # ~20Hz (control_freq_hz)

# 노드 정보
ros2 node info /follow_controller
ros2 node info /person_tracker
ros2 node info /lid_controller
ros2 node info /frictionless_gui

# 파라미터 실시간 변경
ros2 param set /follow_controller target_distance_m 0.8
ros2 param set /follow_controller bbox_height_at_target 142.0
ros2 param set /person_tracker similarity_threshold 0.65

# 토픽 직접 발행 (테스트)
ros2 topic pub /smart_cart/mode std_msgs/String "data: 'follow'" --once
ros2 topic pub /smart_cart/learn std_msgs/String "data: 'reset'" --once
ros2 topic pub /smart_cart/destination std_msgs/String "data: 'toilet'" --once
ros2 topic pub /safety_stop std_msgs/Bool "data: true" --once
```

---

## 9. 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| GUI 카메라 검은색 | Pi `/webcam/image_raw` 발행 안 됨. `ros2 topic hz /webcam/image_raw` 확인. `ls /dev/video*` 로 디바이스 확인 후 `webcam_params.yaml` `video_device` 수정 |
| YOLO 화면 안 뜸 | person_tracker (PC) 다운 또는 모델 로드 실패. 로그 확인, `pip install ultralytics` |
| 사람 추종 시작 안 됨 | 학습 데이터 없음. "사람 추종 시작" → 15초 정면 응시. `/smart_cart/learn_status` 가 `learning:N → wait:N → ready` 흐름인지 확인 |
| Kalman 예측 후 정지 안 풀림 | similarity_threshold 너무 높음 → 0.6 으로 낮추거나 학습 다시 |
| `/cmd_vel` 발행되는데 로봇 안 움직임 | turtlebot3_bringup 안 떴거나 ROS_DOMAIN_ID 불일치. `ros2 node list` 확인 |
| RViz2 맵 안 보임 | Nav2 의 `/map` 발행 확인 → `ros2 topic echo /map_metadata` |
| 추종이 너무 빠름/덜커덕 | `linear_kp` 낮추기 (1.5 → 1.0) 또는 `max_linear_vel` 낮추기 |
| 서보 안 움직임 | OpenCR 외부 전원 미연결 / 공통 GND 누락 / 펌웨어 미업로드. `echo "P:90" > /dev/ttyACM0` 로 직접 테스트 |
| 서보 떨림 / 발열 | 외부 전원 부족. 5V/5A 이상 BEC 사용 |
| WSL2 에서 빌드 느림 | `/mnt/c/...` 가 아닌 WSL 리눅스 홈 (`~/`)에 워크스페이스 두고 빌드 |

---

## 부록 A: 환경 설정 영구화 (~/.bashrc)

```bash
# ROS2
source /opt/ros/humble/setup.bash
source ~/smart_cart_ws/smart_cart_ws/install/setup.bash

# 분산 통신
export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0

# (옵션) Cyclone DDS — LAN 안정성 ↑
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 부록 B: GitHub 동기화 워크플로우

```bash
# PC 에서 작업 후 push
cd ~/smart_cart_ws/smart_cart_ws
git add . && git commit -m "Update" && git push origin main

# Pi 에서 받기
ssh pi@<IP>
cd ~/smart_cart_ws/smart_cart_ws
git pull origin main
colcon build --symlink-install --packages-select sc_interfaces sc_cpp sc_python sc_bringup
source install/setup.bash
```

---

문의 / 캘리브레이션 / 추가 기능 — 팀 채널에서 다이렉트 메시지 부탁드립니다.
