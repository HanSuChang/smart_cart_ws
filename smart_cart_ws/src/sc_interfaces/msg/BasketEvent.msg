# ================================================================
# BasketEvent.msg
# 바구니(터틀봇 후방) OpenCV 비전 노드 → GUI/Flask 결제 서버
#
# 사용 시나리오:
#   대형마트에서 사용자가 바구니에 물건을 넣으면
#   /webcam2 (바구니에 고정된 카메라) → basket_vision 노드가
#   배경 차분 + 모션 감지로 "삽입 이벤트" 발생 → 본 메시지 발행
#   분류 모델(YOLO/Roboflow)이 정해지면 그 결과를 같이 채워서 발행
# ================================================================

std_msgs/Header header

# "insert"  : 새 물체가 바구니에 들어옴
# "remove"  : 물체가 빠짐
# "stable"  : 변화 없음 (디버그용)
string  event_type

# 분류기가 미정이라 일단 빈 값으로 둠 (나중에 채울 것)
string  item_name        # 분류된 클래스명 (e.g. "milk", "bread")
float32 confidence       # 분류 신뢰도 0.0~1.0

# 모션 감지 정보
int32   motion_pixels    # 움직임 픽셀 수
int32   bbox_x
int32   bbox_y
int32   bbox_width
int32   bbox_height

# 스냅샷 저장 경로 (빈 문자열이면 저장 안 됨)
string  snapshot_path
