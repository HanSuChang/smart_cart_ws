#!/usr/bin/env python3
# ================================================================
# deepsort_tracker.py
# [Python 담당] DeepSORT 래퍼 유틸리티
#
# 역할:
#   YOLOv8n이 프레임마다 독립적으로 검출한 사람 bbox에
#   일관된 ID를 부여하고 프레임 간 추적을 유지합니다.
#
# 카메라: 탑싱크 TS-B7WQ30 1440p
#         v4l2_camera → 640x480 리사이즈 → /webcam/image_raw
#
# 추적 대상: 사람(class_id=0)만 추적
#            물체 인식은 item_classifier.py가 별도 담당
#
# 사각지대 대응:
#   max_age=30 → 사람이 30프레임 동안 안 보여도 ID 기억 유지
#   C++의 Kalman Filter가 이 시간 동안 위치를 예측해서 추격 유지
# ================================================================

# from deep_sort_realtime.deepsort_tracker import DeepSort


# class DeepSortTracker:
#     def __init__(self, max_age=30, n_init=3):
#         """
#         max_age: 사람이 사라진 후 몇 프레임까지 ID를 기억할 것인가
#                  (사각지대 대응 핵심, C++ Kalman Filter와 연동)
#         n_init:  최소 몇 프레임 연속으로 보여야 '확정 타겟'으로 인정할 것인가
#                  낮출수록 빠르게 잠금, 높일수록 오탐 감소
#         """
#         self.tracker = DeepSort(
#             max_age=max_age,
#             n_init=n_init,
#             nms_max_overlap=1.0,        # NMS 겹침 허용 비율
#             max_cosine_distance=0.2,    # 외형 유사도 임계값 (낮을수록 엄격)
#             nn_budget=None,
#             override_track_class=None,
#             embedder="mobilenet",       # RPi4 부하 고려 → 가벼운 모델 사용
#             half=True                   # FP16 연산 → 속도 향상
#         )

#     def update(self, detections, frame):
#         """
#         YOLOv8n 검출 결과를 DeepSORT에 입력해서 추적 결과 반환

#         Args:
#             detections: [[x1, y1, w, h, conf, class_id], ...] 리스트
#                         person_tracker.py에서 사람(0)만 필터링해서 넘김
#             frame:      현재 카메라 프레임 (외형 특징 추출용)

#         Returns:
#             [[x, y, w, h, track_id], ...] 리스트
#             확정(Confirmed)된 트랙만 반환
#             빈 리스트이면 추적 중인 사람 없음
#         """
#         # DeepSORT 입력 형식으로 변환
#         # ([x, y, w, h], confidence, detection_class) 튜플 리스트
#         raw_detections = []
#         for det in detections:
#             x1, y1, w, h, conf, cls = det
#             # ★ 사람(class_id=0)만 추적
#             # 물체 인식은 item_classifier.py가 별도 담당하므로 여기선 제외
#             if int(cls) == 0:
#                 raw_detections.append(([x1, y1, w, h], conf, "person"))

#         # DeepSORT 트래커 업데이트
#         tracks = self.tracker.update_tracks(raw_detections, frame=frame)

#         results = []
#         for track in tracks:
#             # 확정(Confirmed)된 트랙만 반환
#             # is_confirmed()=False: 아직 n_init 프레임 미달 → 제외
#             if not track.is_confirmed():
#                 continue

#             track_id = track.track_id
#             ltrb = track.to_ltrb()  # [left, top, right, bottom]

#             x = int(ltrb[0])
#             y = int(ltrb[1])
#             w = int(ltrb[2] - ltrb[0])
#             h = int(ltrb[3] - ltrb[1])

#             # 유효하지 않은 bbox 필터링 (음수 또는 0 크기)
#             if w <= 0 or h <= 0:
#                 continue

#             results.append([x, y, w, h, track_id])

#         return results