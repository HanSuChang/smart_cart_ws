# ================================================================
# WaypointList.msg
# GUI(sc_gui) → 전체 노드들에 현재 등록된 웨이포인트 목록 발행
# 저장(launch 재시작) 시에도 GUI가 waypoints.json에서 로드 후 발행
# ================================================================

std_msgs/Header header
sc_interfaces/Waypoint[] waypoints
