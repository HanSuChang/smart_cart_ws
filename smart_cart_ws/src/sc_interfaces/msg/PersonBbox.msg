# ================================================================
# PersonBbox.msg
# Python(person_tracker)에서 publish → C++(follow_controller)에서 subscribe
# USB 웹캠이 찍은 영상에서 YOLO+DeepSORT로 검출한 사람 바운딩박스
# ================================================================

std_msgs/Header header

int32   x              # bbox 좌상단 x (픽셀)
int32   y              # bbox 좌상단 y (픽셀)
int32   width          # bbox 너비 (픽셀)
int32   height         # bbox 높이 (픽셀) — 거리 추정의 핵심 지표
float32 confidence     # YOLO 검출 신뢰도 (0.0~1.0)
int32   track_id       # DeepSORT가 부여한 추적 ID (-1이면 미추적)
bool    is_valid       # 유효한 검출인지 여부 (false면 사람 못 찾음)
