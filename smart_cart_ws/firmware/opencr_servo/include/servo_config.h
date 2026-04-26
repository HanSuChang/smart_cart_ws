// ================================================================
// servo_config.h
// OpenCR 서보 핀맵 및 기본값 정의
// TODO: 실제 OpenCR 핀 번호 확정 후 수정
// ================================================================

#pragma once

// ── 서보 GPIO 핀 (OpenCR) ──
#define PIN_PAN   9    // TODO: 확정 필요
#define PIN_TILT  10   // TODO: 확정 필요
#define PIN_LID   11   // TODO: 확정 필요

// ── 서보 ID (ROS2 토픽과 매칭) ──
#define SERVO_ID_PAN   0
#define SERVO_ID_TILT  1
#define SERVO_ID_LID   2

// ── 기본 각도 ──
#define PAN_CENTER   90
#define TILT_CENTER  90
#define LID_CLOSE    0
#define LID_OPEN     120

// ── 시리얼 통신 ──
#define SERIAL_BAUD  115200
