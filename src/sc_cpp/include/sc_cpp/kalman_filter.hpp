#pragma once

// ================================================================
// kalman_filter.hpp
// bbox (x, y, w, h) 노이즈 제거 + 사각지대 예측용 칼만 필터
//
// 현상: 로봇이 사람을 따라가는데, 카메라가 사람을 1~2프레임 놓치거나
//       bbox가 갑자기 튀는 경우 로봇이 멈추거나 흔들리는 문제
//
// 해결: 속도 항(Velocity)이 포함된 8-state 모델
//       state = [x, y, w, h, vx, vy, vw, vh]
//
// 동작:
//   - YOLO가 사람을 인식하면 → update()로 위치 보정
//   - YOLO가 사람을 놓치면  → predict()만 호출 → 이전 속도로 위치 예측
//   - 사각지대에 가려져도 최대 prediction_timeout_sec 동안 추격 유지
// ================================================================

#include <vector>

namespace sc_cpp
{

class KalmanFilter
{
public:
    KalmanFilter();

    /**
     * @brief 필터 초기 상태 설정
     * @param x 초기 X 좌표
     * @param y 초기 Y 좌표
     * @param w 초기 너비
     * @param h 초기 높이
     */
    void init(double x, double y, double w, double h);

    /**
     * @brief 예측 단계: 현재 속도로 다음 위치 예측
     *        YOLO가 사람 못 찾을 때 이것만 호출 → 사각지대 추격
     */
    void predict();

    /**
     * @brief 보정 단계: YOLO 측정값으로 예측 보정 + 속도 업데이트
     * @param z_x 측정된 X 좌표
     * @param z_y 측정된 Y 좌표
     * @param z_w 측정된 너비
     * @param z_h 측정된 높이
     */
    void update(double z_x, double z_y, double z_w, double z_h);

    /**
     * @brief 현재 보정/예측된 상태 반환
     * @return [x, y, w, h, vx, vy, vw, vh] 8개 값
     *         follow_controller에서는 [0]~[3] (x, y, w, h)만 사용
     */
    std::vector<double> getState();

    /**
     * @brief 초기화 여부 확인
     * @return true면 사용 가능 상태
     */
    bool isInitialized() const;

private:
    bool is_initialized_;
    std::vector<double> state_;  // [x, y, w, h, vx, vy, vw, vh]
    std::vector<double> P_;      // 오차 공분산 (8개)

    double R_;  // 측정 노이즈 (YOLO bbox 신뢰도)
    double Q_;  // 프로세스 노이즈 (사람 움직임 불규칙성)
};

}  // namespace sc_cpp