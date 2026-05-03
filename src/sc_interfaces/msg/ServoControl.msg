# ================================================================
# ServoControl.msg
# C++(pan_tilt_controller / lid_controller)에서 publish
# → OpenCR(또는 시리얼 브릿지)에서 subscribe하여 MG996R 서보 구동
#
# [하드웨어 구성]
# - servo_id 0: 웹캠 Pan (좌우 회전) — MG996R
# - servo_id 1: 웹캠 Tilt (상하 회전) — MG996R
# - servo_id 2: 바구니 뚜껑 개폐 — MG996R
#
# ※ 이 구성은 변경될 수 있음. servo_id 매핑은 config/servo_params.yaml 참고
# ================================================================

std_msgs/Header header

uint8   servo_id       # 서보 번호 (0: pan, 1: tilt, 2: lid)
float32 angle          # 목표 각도 (0.0 ~ 180.0도)
float32 speed          # 회전 속도 (0.0~1.0, 1.0이 최대속도)
