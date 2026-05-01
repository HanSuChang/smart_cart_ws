// ================================================================
// main.cpp (OpenCR 서보 제어 펌웨어)
//
// [하드웨어]
//   OpenCR 1.0 GPIO → MG996R 서보모터 3개
//   - PIN_PAN  (servo_id=0): 웹캠 좌우 회전
//   - PIN_TILT (servo_id=1): 웹캠 상하 회전
//   - PIN_LID  (servo_id=2): 바구니 뚜껑 개폐
//
// [통신 프로토콜]
//   ROS2(RPi4) → USB Serial (115200bps) → OpenCR
//   수신 포맷: "S<servo_id>,<angle>\n"
//   예시: "S0,90\n"  → Pan 서보를 90도로
//         "S2,120\n" → 뚜껑 서보를 120도로 (열기)
//         "S2,0\n"   → 뚜껑 서보를 0도로 (닫기)
//
//   응답 포맷: "OK<servo_id>,<angle>\n"
//
// ※ OpenCR에서 Arduino IDE로 빌드하는 경우:
//    이 파일을 main.ino로 이름 바꿔서 사용
//
// ※ PlatformIO에서 OpenCR 보드가 지원 안 되면
//    Arduino IDE(OpenCR 보드 매니저 설치)에서 빌드하고
//    이 폴더는 소스 버전 관리용으로만 사용
// ================================================================

#include <Arduino.h>
#include <Servo.h>

// TODO: OpenCR GPIO 핀 번호 확정 후 수정
// OpenCR 핀맵: https://emanual.robotis.com/docs/en/parts/controller/opencr10/
#define PIN_PAN   9    // TODO: 실제 핀 번호로 변경
#define PIN_TILT  10   // TODO: 실제 핀 번호로 변경
#define PIN_LID   11   // TODO: 실제 핀 번호로 변경

// TODO: Servo 객체 선언
// TODO: setup() — Serial 초기화 (115200), Servo attach
// TODO: loop() — Serial 수신 파싱 → 서보 각도 제어
// TODO: parseCommand() — "S<id>,<angle>\n" 파싱 함수

void setup()
{
  // TODO: 구현 예정
}

void loop()
{
  // TODO: 구현 예정
}
