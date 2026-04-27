#!/usr/bin/env python3
# ================================================================
# deepsort_tracker.py
# [Python 담당] DeepSORT 래퍼 유틸
# ================================================================

import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort

class DeepSortTracker:
    def __init__(self, max_age=30, n_init=3):
        """
        max_age: 사람이 사라진 후 몇 프레임까지 기억할 것인가 (사각지대 대응 핵심)
        n_init: 최소 몇 프레임 이상 연속으로 보여야 '확정된 타겟'으로 인정할 것인가
        """
        self.tracker = DeepSort(
            max_age=max_age,
            n_init=n_init,
            nms_max_overlap=1.0,
            max_cosine_distance=0.2,
            nn_budget=None,
            override_track_class=None,
            embedder="mobilenet",  # 라즈베리파이 연산을 고려해 가벼운 모델 사용
            half=True
        )

    def update(self, detections, frame):
        """
        detections: YOLO에서 나온 [[x1, y1, w, h, conf, class_id], ...] 리스트
        frame: 현재 영상 프레임
        return: 추적 중인 객체 리스트 [[x1, y1, w, h, track_id], ...]
        """
        # DeepSORT 형식에 맞게 데이터 변환 ([left, top, w, h], confidence, detection_class)
        raw_detections = []
        for det in detections:
            x1, y1, w, h, conf, cls = det
            # 사람(class_id=0)만 필터링해서 추적 대상에 추가
            if int(cls) == 0:
                raw_detections.append(([x1, y1, w, h], conf, "person"))

        # 트래커 업데이트
        tracks = self.tracker.update_tracks(raw_detections, frame=frame)

        results = []
        for track in tracks:
            # 확정된(Confirmed) 상태이고, 현재 프레임에서 감지된 경우만 반환
            if not track.is_confirmed():
                continue
            
            track_id = track.track_id
            ltrb = track.to_ltrb() # [left, top, right, bottom]
            
            # C++ 노드(FollowController)가 쓰기 편하게 [x, y, w, h, id]로 변환
            x = int(ltrb[0])
            y = int(ltrb[1])
            w = int(ltrb[2] - ltrb[0])
            h = int(ltrb[3] - ltrb[1])
            
            results.append([x, y, w, h, track_id])

        return results