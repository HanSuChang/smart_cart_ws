# ================================================================
# Waypoint.msg
# 단일 웨이포인트 (화장실 / 충전소 / 신선식품 등)
# ================================================================

string  name        # "toilet", "charger", "결제" 등 식별자
float32 x           # map frame x (m)
float32 y           # map frame y (m)
string  category    # "toilet" | "charger" | "payment" | "custom"
