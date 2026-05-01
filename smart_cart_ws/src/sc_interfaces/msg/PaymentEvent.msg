# ================================================================
# PaymentEvent.msg
# Flask 결제 서버(cart_gui) ↔ PyQt5 GUI ↔ C++ 노드 결제 이벤트 공유
# ================================================================

std_msgs/Header header

# "request" : GUI → 결제 시작 요청 (Flask 페이지 표시)
# "pending" : QR 스캔 대기 중
# "paid"    : 결제 완료
# "cancel"  : 결제 취소
string  event

int32   total_price
int32   item_count
string  message
