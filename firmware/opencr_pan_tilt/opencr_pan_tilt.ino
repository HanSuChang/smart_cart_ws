// =====================================================================
// opencr_pan_tilt.ino
// USB 웹캠 Pan/Tilt 서보 (MG996R) — OpenCR 펌웨어
//
// ※ Uno 코드 → OpenCR 코드 옮겨적기 ※
//   - Uno: ATmega328P, Timer1 레지스터 직접 조작 (DDRB / TCCR1A / OCR1A)
//   - OpenCR: STM32F746, Servo.h 라이브러리로 동일한 50Hz PWM 제공
//   - P 제어 로직, Kp=0.1, 0~180도 범위, 10ms 루프 — 모두 그대로 보존
//
// [통신 프로토콜 (시리얼, USB-CDC 115200)]
//   Pi 의 ROS2 노드 pan_tilt_controller 가 /servo_control (servo_id=0 Pan, =1 Tilt)
//   를 발행하면, 별도 시리얼 브릿지 노드가 다음 형식으로 OpenCR 에 전달:
//     "P:120\n"   Pan 목표 90도
//     "T:80\n"    Tilt 목표 80도
//     숫자만 (예: "120\n")  → Pan 으로 처리 (Uno 호환 모드)
// =====================================================================


// =====================================================================
// [원본 — Arduino Uno 펌웨어 (참고용 보존)]
// =====================================================================
/*
#include <Arduino.h>
// 전역 변수 설정
float target_angle = 90.0;  // 파이썬에서 보내주는 목표 각도
float current_angle = 90.0; // 모터가 현재 위치한 실제 각도
float Kp = 0.1;             // 비례 상수 (0.05 ~ 0.2 사이에서 조절해 보세요)
void setup() {
    Serial.begin(115200);
    // 1. 9번 핀(PB1) 출력 설정
    DDRB |= (1 << DDB1);
    // 2. Timer1 설정 (Fast PWM, TOP=ICR1)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // 분주비 8
    // 3. 주파수 50Hz 설정 (16MHz / (8 * 50) = 40,000)
    ICR1 = 39999;
    // 초기값 90도 세팅
    OCR1A = 3000;
}
void loop() {
    // [STEP 1] 시리얼로 새로운 목표 각도 받기
    if (Serial.available() > 0) {
        int input = Serial.parseInt();
        if (input >= 0 && input <= 180) {
            target_angle = input; // 목표치만 갱신
        }
    }
    // [STEP 2] P 제어
    float error = target_angle - current_angle;
    current_angle += error * Kp;
    // [STEP 3] OCR1A 갱신 (0~180도 → 1000~5000)
    unsigned int ocrValue = 1000 + (current_angle * (4000.0 / 180.0));
    OCR1A = ocrValue;
    delay(10);
}
*/


// =====================================================================
// [OpenCR 포팅 — STM32F746 + Servo.h]
// =====================================================================
#include <Arduino.h>
#include <Servo.h>

// ── 핀 배정 (OpenCR PWM 가능 핀: 6, 9, 10, 11 — 데이터시트 확인) ──
const int PAN_PIN  = 6;   // Pan 서보 (좌우)
const int TILT_PIN = 9;   // Tilt 서보 (상하)

// ── 서보 객체 (Robotis OpenCR Servo.h 가 50Hz PWM 자동 생성) ──
Servo pan_servo;
Servo tilt_servo;

// ── 전역 변수 (Uno 코드와 동일한 의미) ──
float pan_target  = 90.0;     // Pi 가 보내주는 Pan 목표 각도
float pan_current = 90.0;     // Pan 모터의 실제 위치
float tilt_target  = 90.0;
float tilt_current = 90.0;

const float Kp = 0.1;          // 비례 상수 (0.05 ~ 0.2)

String input_buffer;

// ─────────────────────────────────────────────────────────
// 시리얼 명령 파싱
// 지원 형식:
//   "P:120\n" / "p:120\n"  → Pan 목표
//   "T:80\n"  / "t:80\n"   → Tilt 목표
//   "120\n"                → Pan 으로 처리 (Uno 1:1 호환)
// ─────────────────────────────────────────────────────────
void parse_command(const String & cmd_in) {
  String cmd = cmd_in;
  cmd.trim();
  if (cmd.length() == 0) return;

  // 접두사 없는 단순 정수 → Pan 으로 처리 (Uno 호환)
  if (cmd.charAt(0) >= '0' && cmd.charAt(0) <= '9') {
    int v = cmd.toInt();
    if (v >= 0 && v <= 180) pan_target = (float)v;
    return;
  }

  // "X:val" 형식
  int colon = cmd.indexOf(':');
  if (colon < 0) return;
  char ch = cmd.charAt(0);
  int v = cmd.substring(colon + 1).toInt();
  if (v < 0)   v = 0;
  if (v > 180) v = 180;

  if (ch == 'P' || ch == 'p') pan_target  = (float)v;
  if (ch == 'T' || ch == 't') tilt_target = (float)v;
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // OpenCR 의 PWM 핀 → 서보 부착 (Servo.h 가 내부적으로 50Hz 설정)
  pan_servo.attach(PAN_PIN);
  tilt_servo.attach(TILT_PIN);

  // 초기 각도 90도 (정면)
  pan_servo.write((int)pan_current);
  tilt_servo.write((int)tilt_current);

  Serial.println("OpenCR PanTilt Firmware Ready");
}

void loop() {
  // [STEP 1] 시리얼로 새로운 목표 각도 받기 (Uno 와 동일)
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (input_buffer.length() > 0) {
        parse_command(input_buffer);
        input_buffer = "";
      }
    } else {
      input_buffer += c;
    }
  }

  // [STEP 2] P 제어 — Uno 코드와 동일한 수식
  //   error = target - current
  //   current += error * Kp
  float pan_error  = pan_target  - pan_current;
  float tilt_error = tilt_target - tilt_current;
  pan_current  += pan_error  * Kp;
  tilt_current += tilt_error * Kp;

  // [STEP 3] 계산된 각도를 서보에 출력
  //   Uno: OCR1A = 1000 + angle * (4000/180)  (Timer1 직접 조작)
  //   OpenCR: Servo.write(angle)              (라이브러리가 내부에서 PWM 변환)
  pan_servo.write((int)pan_current);
  tilt_servo.write((int)tilt_current);

  // [STEP 4] 너무 빨리 돌면 연산이 씹힐 수 있으니 짧은 대기 (Uno 와 동일)
  delay(10);
}

// =====================================================================
// [차이점 요약 — Uno → OpenCR]
//   1) 핀: Uno PB1 (D9)         → OpenCR Servo.attach(6/9 등)
//   2) PWM: Timer1 직접 설정     → Servo.h 가 자동으로 50Hz
//   3) 출력: OCR1A 레지스터      → Servo.write(angle)
//   4) MCU: ATmega328P (16MHz)   → STM32F746 (216MHz, ARM Cortex-M7)
//
// 동작/제어 로직은 100% 동일 — Kp=0.1, P 제어, 10ms 루프
// =====================================================================
