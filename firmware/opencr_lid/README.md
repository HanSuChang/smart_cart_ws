# OpenCR Lid (바구니 뚜껑) 펌웨어

바구니 뚜껑 서보(MG996R) 를 OpenCR 에서 구동하기 위한 펌웨어 폴더.

## 통신
- ROS2 노드 `lid_controller` (sc_cpp, Pi) 가 `/servo_control` 토픽 발행 (servo_id=2)
- 본 펌웨어 는 시리얼(USB-CDC) 또는 micro-ROS 로 명령 받아 PWM 출력

## 구현 상태
※ 팀에서 하드웨어 미완성 — 본 코드는 pub/sub 틀만 잡힌 placeholder 입니다.
※ 실제 서보 배선 / PWM 핀 / 각도 캘리브레이션은 하드웨어 완성 후 적용.

## 빌드 절차 (OpenCR)
1. Arduino IDE 에 OpenCR 보드 매니저 추가
   https://emanual.robotis.com/docs/en/parts/controller/opencr10/#install-on-linux
2. 보드 → OpenCR Board 선택, 포트 설정
3. 이 폴더의 `.ino` 열고 업로드
