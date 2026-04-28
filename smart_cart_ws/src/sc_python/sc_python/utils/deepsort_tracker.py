#!/usr/bin/env python3
# ================================================================
# deepsort_tracker.py
# [Python 담당] DeepSORT 래퍼 유틸리티 (최종 수정본)
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
        return: 추적 중인 객체 리스트 [[x, y, w, h, track_id], ...]
        """
        
        raw_detections = []
        for det in detections:
            x1, y1, w, h, conf, cls = det
            
            # 클래스 번호를 정수형으로 변환하여 판단
            class_id = int(cls)
            
            # 사람(0) 또는 물체(2)를 필터링하여 DeepSORT 입력으로 변환
            if class_id == 0:
                raw_detections.append(([x1, y1, w, h], conf, "person"))
            elif class_id == 2:
                raw_detections.append(([x1, y1, w, h], conf, "object"))

        # 트래커 업데이트 (필터링된 데이터만 입력)
        tracks = self.tracker.update_tracks(raw_detections, frame=frame)

        results = []
        for track in tracks:
            # 확정된(Confirmed) 상태의 트랙만 반환
            if not track.is_confirmed():
                continue
            
            track_id = track.track_id
            ltrb = track.to_ltrb() # [left, top, right, bottom]
            
            # 정수 좌표 계산
            x = int(ltrb[0])
            y = int(ltrb[1])
            w = int(ltrb[2] - ltrb[0])
            h = int(ltrb[3] - ltrb[1])
            
            results.append([x, y, w, h, track_id])

        return results