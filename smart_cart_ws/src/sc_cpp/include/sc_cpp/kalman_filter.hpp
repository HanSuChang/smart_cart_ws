#pragma once
// ================================================================
// kalman_filter.hpp
// [C++ 담당] 2D Bounding Box용 Kalman Filter
//
// state: [cx, cy, w, h, vx, vy, vw, vh] (8-state)
// measurement: [cx, cy, w, h] (4-measurement)
//
// 용도:
//   YOLO bbox가 1~2프레임 사라져도 예측값으로 추종 유지
//   갑자기 bbox가 튀는 노이즈 제거
// ================================================================

// TODO: 나중에 구현 예정

namespace sc_cpp
{

class KalmanFilter2D
{
public:
  KalmanFilter2D() = default;

  // TODO: init(), predict(), update(), getState()
};

}  // namespace sc_cpp
