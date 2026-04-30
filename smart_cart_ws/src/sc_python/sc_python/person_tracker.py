#!/usr/bin/env python3
# =====================================================================
# person_tracker.py
# YOLOv8n + BoT-SORT (내장 추적) + HSV 히스토그램 (Re-ID)
#
# [동작 흐름]
#   1. /webcam/image_raw 구독 → 프레임 받기
#   2. YOLO model.track(persist=True) → BoT-SORT가 ID 자동 부여
#   3. 학습 모드 (/smart_cart/learn = "start"):
#      - 20초 동안 가장 큰 사람의 HSV 히스토그램을 master_db에 수집
#      - 완료되면 자동으로 추종 모드 진입
#   4. 추종 모드:
#      - 화면의 모든 사람 중 master_db와 가장 유사한 사람을 찾음
#      - 유사도 > 0.7 이면 "Master"로 인식 → /person_bbox 발행
#      - 사람 없거나 Unknown만 있으면 is_valid=False 발행
#
# [GUI 통신]
#   /smart_cart/learn (String):
#     "start"  → 학습 시작
#     "save"   → master_db를 ~/.smart_cart/master_hist.json에 저장
#     "load"   → 저장된 파일에서 로드
#     "reset"  → master_db 초기화
#
#   /smart_cart/learn_status (String):
#     "learning:15"  → 학습 중 (남은 초)
#     "ready"        → 추종 가능 상태
#     "idle"         → 학습 안 됨
# =====================================================================

import os
import json
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

from sc_interfaces.msg import PersonBbox

HIST_BINS_H = 180
HIST_BINS_S = 256
SIMILARITY_THRESHOLD = 0.7   # Master 인식 유사도 임계값
SAVE_DIR = os.path.expanduser("~/.smart_cart")
SAVE_PATH = os.path.join(SAVE_DIR, "master_hist.json")


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')

        # 1. 파라미터
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('img_size', 640)
        self.declare_parameter('image_topic', '/webcam/image_raw')
        self.declare_parameter('learning_duration', 20.0)
        self.declare_parameter('similarity_threshold', SIMILARITY_THRESHOLD)
        self.declare_parameter('auto_load', True)  # 시작 시 저장된 master 자동 로드

        model_path = self.get_parameter('model_path').value
        image_topic = self.get_parameter('image_topic').value
        self.learning_duration = self.get_parameter('learning_duration').value
        self.similarity_threshold = self.get_parameter('similarity_threshold').value

        # 2. 모델 로드
        self.model = YOLO(self._resolve_model_path(model_path))
        self.bridge = CvBridge()

        # 3. ROS2 인터페이스
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.bbox_pub = self.create_publisher(PersonBbox, '/person_bbox', 10)

        # 학습 제어 토픽
        self.learn_sub = self.create_subscription(
            String, '/smart_cart/learn', self.learn_callback, 10)
        self.learn_status_pub = self.create_publisher(
            String, '/smart_cart/learn_status', 10)

        # 4. 학습 상태
        self.master_db = []          # HSV 히스토그램 리스트
        self.is_learning = False
        self.learning_start_time = None
        self.last_status_time = 0.0  # 학습 진행 상태 발행 주기 제어

        # 5. 자동 로드
        if self.get_parameter('auto_load').value:
            self._load_master_db()

        self.get_logger().info(
            f'PersonTracker 시작 — model={model_path}, topic={image_topic}, '
            f'master_count={len(self.master_db)}'
        )
        self._publish_status()

    # =====================================================================
    # 모델 경로 자동 탐색
    # =====================================================================
    def _resolve_model_path(self, model_path: str) -> str:
        if os.path.isabs(model_path):
            return model_path

        candidate_paths = [
            model_path,
            os.path.join(get_package_share_directory('sc_python'), 'models', model_path),
            os.path.join(os.getcwd(), model_path),
            os.path.join(os.getcwd(), 'src', 'sc_python', 'models', model_path),
        ]
        for path in candidate_paths:
            if os.path.exists(path):
                return path

        self.get_logger().warn(
            f'모델 파일을 찾지 못했습니다. ultralytics가 자동 다운로드합니다: {model_path}')
        return model_path

    # =====================================================================
    # GUI 학습 제어 콜백
    # =====================================================================
    def learn_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd == 'start':
            self.master_db = []
            self.is_learning = True
            self.learning_start_time = time.time()
            self.get_logger().info(
                f'>>> 학습 시작! {self.learning_duration}초 동안 카메라 앞에서 움직여 주세요')

        elif cmd == 'save':
            self._save_master_db()

        elif cmd == 'load':
            self._load_master_db()

        elif cmd == 'reset':
            self.master_db = []
            self.is_learning = False
            self.get_logger().info('>>> 학습 데이터 초기화 완료')

        else:
            self.get_logger().warn(f'알 수 없는 학습 명령: {cmd}')

        self._publish_status()

    # =====================================================================
    # HSV 데이터 저장/로드
    # =====================================================================
    def _save_master_db(self):
        if not self.master_db:
            self.get_logger().warn('저장할 학습 데이터가 없습니다.')
            return
        try:
            os.makedirs(SAVE_DIR, exist_ok=True)
            data = [hist.tolist() for hist in self.master_db]
            with open(SAVE_PATH, 'w') as f:
                json.dump(data, f)
            self.get_logger().info(
                f'>>> 학습 데이터 저장 완료 ({len(self.master_db)}개) → {SAVE_PATH}')
        except Exception as e:
            self.get_logger().error(f'저장 실패: {e}')

    def _load_master_db(self):
        if not os.path.exists(SAVE_PATH):
            self.get_logger().info('저장된 학습 데이터 없음 (정상)')
            return
        try:
            with open(SAVE_PATH, 'r') as f:
                data = json.load(f)
            self.master_db = [np.array(d, dtype=np.float32) for d in data]
            self.get_logger().info(
                f'>>> 저장된 학습 데이터 로드 완료 ({len(self.master_db)}개)')
        except Exception as e:
            self.get_logger().error(f'로드 실패: {e}')

    # =====================================================================
    # 상태 발행 (GUI에 학습 진행 상황 알림)
    # =====================================================================
    def _publish_status(self):
        msg = String()
        if self.is_learning:
            elapsed = time.time() - self.learning_start_time
            remaining = max(0, int(self.learning_duration - elapsed))
            msg.data = f'learning:{remaining}'
        elif len(self.master_db) > 0:
            msg.data = 'ready'
        else:
            msg.data = 'idle'
        self.learn_status_pub.publish(msg)

    # =====================================================================
    # HSV 히스토그램 추출 (사람 영역에서)
    # =====================================================================
    def _extract_hist(self, frame, x1, y1, x2, y2):
        person_img = frame[y1:y2, x1:x2]
        if person_img.size == 0:
            return None
        hsv = cv2.cvtColor(person_img, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist(
            [hsv], [0, 1], None, [HIST_BINS_H, HIST_BINS_S], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist

    # =====================================================================
    # 메인 콜백 (이미지 받을 때마다 호출)
    # =====================================================================
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        # YOLOv8 + BoT-SORT 추적 (사람만)
        conf = self.get_parameter('conf_threshold').value
        imgsz = self.get_parameter('img_size').value
        try:
            results = self.model.track(
                frame, persist=True, classes=[0],
                conf=conf, imgsz=imgsz, verbose=False, tracker='botsort.yaml')
        except Exception as e:
            self.get_logger().error(f'YOLO 추적 실패: {e}')
            return

        # 결과 파싱
        detections = []  # [(x1, y1, x2, y2, track_id, hist), ...]
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                tid = int(box.id) if box.id is not None else -1
                hist = self._extract_hist(frame, x1, y1, x2, y2)
                if hist is None:
                    continue
                detections.append((x1, y1, x2, y2, tid, hist))

        # ── 학습 모드 ──
        if self.is_learning:
            elapsed = time.time() - self.learning_start_time
            if elapsed < self.learning_duration:
                # 가장 큰 bbox(가장 가까운 사람)의 히스토그램 수집
                if detections:
                    largest = max(detections,
                                  key=lambda d: (d[2] - d[0]) * (d[3] - d[1]))
                    self.master_db.append(largest[5])
            else:
                # 학습 종료
                self.is_learning = False
                self.get_logger().info(
                    f'>>> 학습 완료! 총 {len(self.master_db)}개 특징 수집됨')

            # 학습 중에는 사람 추종 안 함
            self._publish_invalid_bbox()
            # 진행 상태 1초마다 발행
            if time.time() - self.last_status_time > 1.0:
                self._publish_status()
                self.last_status_time = time.time()
            return

        # ── 추종 모드 ──
        if not self.master_db or not detections:
            self._publish_invalid_bbox()
            return

        # 모든 검출 사람 중 master와 가장 유사한 사람 찾기
        best_match = None
        best_sim = 0.0

        for det in detections:
            hist = det[5]
            max_sim = 0.0
            for db_hist in self.master_db:
                sim = cv2.compareHist(db_hist, hist, cv2.HISTCMP_CORREL)
                if sim > max_sim:
                    max_sim = sim
            if max_sim > best_sim:
                best_sim = max_sim
                best_match = det

        # 유사도 임계값 확인
        if best_match and best_sim > self.similarity_threshold:
            x1, y1, x2, y2, tid, _ = best_match
            self._publish_bbox(x1, y1, x2, y2, tid, best_sim, msg.header.stamp)
        else:
            # Master 못 찾음 (Unknown만 있거나 너무 다름)
            self._publish_invalid_bbox()

    # =====================================================================
    # PersonBbox 발행 헬퍼
    # =====================================================================
    def _publish_bbox(self, x1, y1, x2, y2, tid, conf, stamp):
        bbox_msg = PersonBbox()
        bbox_msg.header.stamp = stamp
        bbox_msg.header.frame_id = 'camera_link'
        bbox_msg.x = int(x1)
        bbox_msg.y = int(y1)
        bbox_msg.width = int(x2 - x1)
        bbox_msg.height = int(y2 - y1)
        bbox_msg.track_id = int(tid)
        bbox_msg.confidence = float(conf)
        bbox_msg.is_valid = True
        self.bbox_pub.publish(bbox_msg)

    def _publish_invalid_bbox(self):
        bbox_msg = PersonBbox()
        bbox_msg.header.stamp = self.get_clock().now().to_msg()
        bbox_msg.header.frame_id = 'camera_link'
        bbox_msg.is_valid = False
        self.bbox_pub.publish(bbox_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()