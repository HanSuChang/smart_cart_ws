# ================================================================
# CartStatus.msg
# C++ 노드들에서 publish → GUI(sc_gui)에서 subscribe
# 카트 전체 상태를 GUI 화면에 표시하기 위한 메시지
# ================================================================

std_msgs/Header header

# 추종 상태
bool    is_following        # 사람 추종 중인지
int32   tracked_person_id   # 현재 추적 중인 사람 ID
float32 person_distance     # 추정 거리 (bbox 높이 기반, 미터 단위 근사)

# 바구니 상태
bool    lid_open            # 뚜껑 열림 여부
string  last_item_name      # 마지막 인식된 물체 이름
int32   item_count          # 현재까지 담긴 물체 수

# 안전 상태
bool    emergency_stop      # 긴급정지 상태
float32 obstacle_distance   # 전방 최소 장애물 거리 (m)

# 주행 상태
float32 linear_vel          # 현재 선속도 (m/s)
float32 angular_vel         # 현재 각속도 (rad/s)
