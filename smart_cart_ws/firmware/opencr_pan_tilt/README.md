# OpenCR Pan/Tilt 펌웨어

USB 웹캠 1번 (사람 추종) 의 Pan/Tilt 서보(MG996R x 2) 를 OpenCR 보드에서
구동하기 위한 펌웨어 폴더.

## 통신
- ROS2 노드 `pan_tilt_controller` (sc_cpp) 가 `/servo_control` 토픽 발행
  (servo_id=0 → Pan, servo_id=1 → Tilt)
- 본 펌웨어는 시리얼(USB-CDC) 또는 micro-ROS 로 명령을 받아서 PWM 출력

## 폴더 구조
- `opencr_pan_tilt.ino` — Arduino IDE / OpenCR IDE 용 메인 스케치
  (현재는 Uno 코드를 주석으로 보존, OpenCR 마이그레이션 placeholder)

## 빌드 절차 (OpenCR)
1. Arduino IDE 에 OpenCR 보드 매니저 추가
   https://emanual.robotis.com/docs/en/parts/controller/opencr10/#install-on-linux
2. 보드 → OpenCR Board 선택, 포트 설정
3. 이 폴더의 `.ino` 열고 업로드
