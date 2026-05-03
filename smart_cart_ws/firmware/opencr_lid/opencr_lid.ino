// =====================================================================
// opencr_lid.ino
// 바구니 뚜껑 서보(MG996R) — OpenCR 펌웨어 placeholder
//
// ※ Uno 코드 → OpenCR 코드 옮겨적기 ※
// ※ 팀에서 하드웨어 미구현 — pub/sub 통신 틀만 (시리얼 프로토콜) 잡아둠 ※
//
// [통신 프로토콜 (시리얼)]
//   ROS2 (Pi) → lid_controller → /servo_control (servo_id=2, angle, speed)
//   별도 시리얼 브릿지 노드 또는 micro-ROS 가 OpenCR 로 전달:
//     "L:0\n"     뚜껑 닫기 (CLOSE = 0도)
//     "L:120\n"   뚜껑 열기 (OPEN = 120도)
//     "L:?\n"     현재 각도 질의
//
// [현재 미구현 항목 — TODO]
//   - PWM 핀 결정 (LID_PIN)
//   - 닫힘/열림 각도 캘리브레이션 (close_angle, open_angle)
//   - 자동 닫기 타이머 (현재는 ROS 측 lid_controller 가 5초 후 close 명령 재발행)
//   - 잠금 메커니즘 (결제 후 풀림 방지)
// =====================================================================

#include <Arduino.h>
#include <Servo.h>

// MG996R 뚜껑 서보
Servo lid_servo;

// OpenCR 핀 (예시 — 실제 배선 시 조정)
const int LID_PIN = 10;

// 각도 (캘리브레이션 필요)
const float CLOSE_ANGLE = 0.0;
const float OPEN_ANGLE  = 120.0;
const float Kp = 0.1;

float target_angle  = CLOSE_ANGLE;
float current_angle = CLOSE_ANGLE;

String input_buffer;

void parse_command(const String & cmd) {
  // 형식: "L:120" / "L:0" / "L:?"
  int colon = cmd.indexOf(':');
  if (colon < 0) return;
  char ch = cmd.charAt(0);
  if (ch != 'L' && ch != 'l') return;

  String arg = cmd.substring(colon + 1);
  if (arg == "?") {
    Serial.print("LID:");
    Serial.println((int)current_angle);
    return;
  }
  int v = arg.toInt();
  if (v < 0)   v = 0;
  if (v > 180) v = 180;
  target_angle = (float)v;
}

void setup() {
  Serial.begin(115200);
  lid_servo.attach(LID_PIN);
  lid_servo.write((int)current_angle);
  Serial.println("OpenCR Lid Firmware Ready");
}

void loop() {
  // [STEP 1] 시리얼 명령 수신
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parse_command(input_buffer);
      input_buffer = "";
    } else {
      input_buffer += c;
    }
  }

  // [STEP 2] P 제어 (목표를 향해 점진적 이동)
  current_angle += (target_angle - current_angle) * Kp;

  // [STEP 3] 서보 출력
  lid_servo.write((int)current_angle);

  delay(10);
}

// =====================================================================
// [원본 — Uno 코드 참고]
// (pan_tilt 와 동일한 P 제어 로직.
//  뚜껑은 0~120도 범위 / 일정 속도로 닫고/여는 동작이 핵심)
// =====================================================================
