#pragma once

// 현상: 로봇이 사람을 따라가는데, 카메라가 사람의 실제 속도를 전혀 못 따라오고 심하게 뒤처질 때.

// 해결: 그때는 제가 속도 항(Velocity)이 포함된 행렬 버전으로 코드를 다시 (8-state" 모델)

#include <vector>

namespace sc_cpp
{

/**
 * @brief 2D 좌표 및 크기(x, y, w, h) 보정을 위한 스칼라 칼만 필터 클래스
 */
class KalmanFilter
{
public:
    /**
     * @brief 생성자: 초기 노이즈 파라미터 설정
     */
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
     * @brief 예측 단계 (Predict): 이전 상태를 기반으로 다음 위치 추측
     */
    void predict();

    /**
     * @brief 보정 단계 (Update): 센서 데이터(YOLO 등)를 받아 상태 업데이트
     * @param z_x 측정된 X 좌표
     * @param z_y 측정된 Y 좌표
     * @param z_w 측정된 너비
     * @param z_h 측정된 높이
     */
    void update(double z_x, double z_y, double z_w, double z_h);

    /**
     * @brief 현재 보정된 상태 값 반환
     * @return {x, y, w, h} 벡터
     */
    std::vector<double> getState();

private:
    bool is_initialized_;      // 초기화 여부 플래그
    std::vector<double> state_; // 현재 상태 변수 [x, y, w, h]
    std::vector<double> P_;     // 오차 공분산 (Error Covariance)
    
    double R_;                 // 측정 노이즈 (Measurement Noise)
    double Q_;                 // 프로세스 노이즈 (Process Noise)
};

}  // namespace sc_cpp