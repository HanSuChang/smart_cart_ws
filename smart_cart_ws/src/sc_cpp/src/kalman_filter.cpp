#include "sc_cpp/kalman_filter.hpp"

namespace sc_cpp
{

KalmanFilter::KalmanFilter()
{
    // 초기화 여부 플래그
    is_initialized_ = false;

    // 측정 노이즈 (R): 낮을수록 센서(YOLO)를 믿고, 높을수록 내 예측을 믿음
    R_ = 1.0; 
    // 프로세스 노이즈 (Q): 로봇의 움직임이 얼마나 불규칙한지
    Q_ = 0.1; 
}

void KalmanFilter::init(double x, double y, double w, double h)
{
    state_ = {x, y, w, h};
    P_ = {1.0, 1.0, 1.0, 1.0}; // 초기 오차 공분산
    is_initialized_ = true;
}

void KalmanFilter::predict()
{
    if (!is_initialized_) return;

    // 등속도 모델 가정 (단순 예측)
    // 다음 상태 예측: x_new = x_old (정지 상태 혹은 미세 움직임 가정)
    // 실제 주행 환경에 따라 속도 항을 추가할 수 있지만, 지금은 보정 위주로 구성
    for(int i=0; i<4; ++i) {
        P_[i] = P_[i] + Q_; // 예측 오차 증가
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
        // 1. 칼만 이득 (Kalman Gain) 계산
        double K = P_[i] / (P_[i] + R_);

        // 2. 상태 보정 (Update State)
        state_[i] = state_[i] + K * (z[i] - state_[i]);

        // 3. 오차 공분산 업데이트 (Update P)
        P_[i] = (1.0 - K) * P_[i];
    }
}

std::vector<double> KalmanFilter::getState()
{
    return state_;
}

}  // namespace sc_cpp