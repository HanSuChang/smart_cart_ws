#include "sc_cpp/kalman_filter.hpp"

namespace sc_cpp
{

KalmanFilter::KalmanFilter()
{
    is_initialized_ = false;

    // 측정 노이즈 (R): 낮을수록 YOLO bbox를 믿고, 높을수록 내 예측을 믿음
    // R이 너무 낮으면 노이즈에 민감, 너무 높으면 반응이 느림
    R_ = 5.0;

    // 프로세스 노이즈 (Q): 사람이 얼마나 불규칙하게 움직이는지
    // Q가 높으면 빠른 움직임에 민감하게 반응, 낮으면 안정적이지만 뒤처짐
    Q_ = 2.0;
}

void KalmanFilter::init(double x, double y, double w, double h)
{
    // state = [x, y, w, h, vx, vy, vw, vh]
    // ★ 속도 항(vx, vy, vw, vh) 추가 → 빠르게 움직이는 사람도 예측 가능
    state_ = {x, y, w, h, 0.0, 0.0, 0.0, 0.0};

    // 초기 오차 공분산
    // 위치는 어느 정도 알지만 초기 속도는 완전 불확실
    P_ = {10.0, 10.0, 10.0, 10.0,   // 위치 불확실도
          50.0, 50.0, 50.0, 50.0};   // 속도 불확실도

    is_initialized_ = true;
}

void KalmanFilter::predict()
{
    if (!is_initialized_) return;

    // ★ 등속도 모델: 현재 속도로 다음 위치 예측 (dt=1 프레임 기준)
    // 사람이 사각지대에 가려져도 이전 속도로 위치를 예측해서 추격 유지
    state_[0] += state_[4];  // x  += vx
    state_[1] += state_[5];  // y  += vy
    state_[2] += state_[6];  // w  += vw
    state_[3] += state_[7];  // h  += vh

    // 오차 공분산 증가 (예측할수록 불확실해짐)
    for (int i = 0; i < 8; ++i) {
        P_[i] += Q_;
    }
}

void KalmanFilter::update(double z_x, double z_y, double z_w, double z_h)
{
    if (!is_initialized_) {
        init(z_x, z_y, z_w, z_h);
        return;
    }

    double z[4] = {z_x, z_y, z_w, z_h};

    for (int i = 0; i < 4; ++i) {
        // 1. 칼만 이득 계산
        // K → 1: 측정값(YOLO) 신뢰
        // K → 0: 예측값 신뢰
        double K = P_[i] / (P_[i] + R_);

        // 2. 속도 업데이트 (측정값 - 예측값 = 이번 프레임 이동량)
        state_[i + 4] += K * (z[i] - state_[i]);

        // 3. 위치 보정
        state_[i] += K * (z[i] - state_[i]);

        // 4. 오차 공분산 업데이트
        P_[i]     = (1.0 - K) * P_[i];
        P_[i + 4] = (1.0 - K) * P_[i + 4];
    }
}

std::vector<double> KalmanFilter::getState()
{
    // ★ 초기화 전 호출 시 0 벡터 반환 (크래시 방지)
    if (!is_initialized_) {
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    return state_;
}

bool KalmanFilter::isInitialized() const
{
    return is_initialized_;
}

}  // namespace sc_cpp