# ================================================================
# ItemDetected.msg
# Python(item_classifier)에서 publish → C++(lid_controller)에서 subscribe
# RPi 카메라가 Roboflow 학습 모델로 인식한 물체 정보
# ================================================================

std_msgs/Header header

string  item_name       # 인식된 물체 이름 (Roboflow 학습 클래스명)
float32 confidence      # 인식 신뢰도 (0.0~1.0)
int32   bbox_x          # 물체 bbox 좌상단 x
int32   bbox_y          # 물체 bbox 좌상단 y
int32   bbox_width      # 물체 bbox 너비
int32   bbox_height     # 물체 bbox 높이
bool    in_basket_zone  # 바구니 ROI 영역 안에서 감지되었는지
