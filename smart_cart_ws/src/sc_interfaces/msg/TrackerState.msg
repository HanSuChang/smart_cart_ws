# ================================================================
# TrackerState.msg
# C++(follow_controller)에서 publish → GUI(sc_gui)에서 subscribe
# 사람 추종 시 칼만필터/BoT-SORT 추적 상태를 GUI에 전달
# ================================================================

std_msgs/Header header

# 0: idle, 1: following, 2: predicting(칼만), 3: lost, 4: recovering
uint8   state

# 칼만필터 8-state: x, y, w, h, vx, vy, vw, vh
float32[8] kalman_state

# 현재 거리 (50cm 유지 목표 대비, m)
float32 current_distance
float32 target_distance

# 현재 추적 ID
int32   track_id

# 마지막 측정 후 경과시간 (sec)
float32 time_since_measurement
