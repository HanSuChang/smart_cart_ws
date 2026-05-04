#!/usr/bin/env python3
# =====================================================================
# person_tracker.py
# YOLOv8n + BoT-SORT (내장 추적) + HSV 히스토그램 (Re-ID)
#
# [본 노드의 책임]
#   - YOLO 모델 inference (사람 클래스만)
#   - BoT-SORT 트래킹 ID 부여
#   - HSV 히스토그램으로 학습된 사용자 Re-ID
#   - 학습 모드 (15초) + 5초 대기 후 follow 모드 자동 전환
#   - ★ /yolo/follow_image/compressed 로 YOLO 적용 프레임 발행 (GUI용)
#
# [Topic — Pub/Sub]
#   Subscribe:
#     /webcam/image_raw         (sensor_msgs/Image)         웹캠 1번
#     /smart_cart/learn         (std_msgs/String)           학습 명령
#     /smart_cart/mode          (std_msgs/String)           시스템 모드
#
#   Publish:
#     /person_bbox              (sc_interfaces/PersonBbox)  사람 좌표
#     /smart_cart/learn_status  (std_msgs/String)           학습 진행상태
#     /yolo/follow_image/compressed (sensor_msgs/CompressedImage)
#                                YOLO 박스/ID 그려진 프레임 → GUI
#     /smart_cart/mode          (std_msgs/String)           5초 대기 종료 시 follow 자동 발행
#
# [학습 명령 (/smart_cart/learn)]
#   "start"             → 15초 학습만 (자동 follow 전환 X)
#   "start_with_delay"  → 15초 학습 + 5초 카운트다운 → /smart_cart/mode follow 자동 발행
#   "save"  / "load"  / "reset"
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
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO

from sc_interfaces.msg import PersonBbox

HIST_BINS_H = 180
HIST_BINS_S = 256
SIMILARITY_THRESHOLD = 0.7
SAVE_DIR = os.path.expanduser("~/.smart_cart")
SAVE_PATH = os.path.join(SAVE_DIR, "master_hist.json")


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')

        # ── 1. 파라미터 ──
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('img_size', 320)
        self.declare_parameter('image_topic', '/webcam/image_raw')
        self.declare_parameter('learning_duration', 15.0)
        self.declare_parameter('post_learn_delay', 5.0)
        self.declare_parameter('similarity_threshold', SIMILARITY_THRESHOLD)
        self.declare_parameter('auto_load', True)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('annotated_jpeg_quality', 60)

        model_path = self.get_parameter('model_path').value
        image_topic = self.get_parameter('image_topic').value
        self.learning_duration = float(self.get_parameter('learning_duration').value)
        self.post_learn_delay = float(self.get_parameter('post_learn_delay').value)
        self.similarity_threshold = float(self.get_parameter('similarity_threshold').value)

        # ── 2. 모델 ──
        self.model = YOLO(self._resolve_model_path(model_path))
        self.bridge = CvBridge()

        # ── 3. Pub/Sub ──
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.bbox_pub = self.create_publisher(PersonBbox, '/person_bbox', 10)
        self.learn_sub = self.create_subscription(
            String, '/smart_cart/learn', self.learn_callback, 10)
        self.mode_sub = self.create_subscription(
            String, '/smart_cart/mode', self.mode_callback, 10)
        self.learn_status_pub = self.create_publisher(
            String, '/smart_cart/learn_status', 10)
        self.mode_pub = self.create_publisher(String, '/smart_cart/mode', 10)
        self.annotated_pub = self.create_publisher(
            CompressedImage, '/yolo/follow_image/compressed', 10)

        # ── 4. 상태 ──
        self.master_db = []
        self.is_learning = False
        self.learning_start_time = None
        self.is_post_delay = False
        self.post_delay_start_time = None
        self.auto_follow_after_learn = False
        self.last_status_time = 0.0
        self.current_mode = 'idle'

        # ── 5. 자동 로드 ──
        if self.get_parameter('auto_load').value:
            self._load_master_db()

        # 파라미터 콜백 (rqt_reconfigure / ros2 param set 실시간 적용)
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'PersonTracker 시작 — model={model_path}, topic={image_topic}, '
            f'master_count={len(self.master_db)}, '
            f'learn={self.learning_duration}s + delay={self.post_learn_delay}s'
        )
        self._publish_status()

    # ─────────────────────────────────────────────────────────
    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'learning_duration':
                self.learning_duration = float(p.value)
            elif p.name == 'post_learn_delay':
                self.post_learn_delay = float(p.value)
            elif p.name == 'similarity_threshold':
                self.similarity_threshold = float(p.value)
        return SetParametersResult(successful=True)

    # ─────────────────────────────────────────────────────────
    def _resolve_model_path(self, model_path: str) -> str:
        if os.path.isabs(model_path):
            return model_path
        try:
            share = get_package_share_directory('sc_python')
        except Exception:
            share = ''
        candidates = [
            model_path,
            os.path.join(share, 'models', model_path) if share else '',
            os.path.join(os.getcwd(), model_path),
            os.path.join(os.getcwd(), 'src', 'sc_python', 'sc_python', 'models', model_path),
        ]
        for path in candidates:
            if path and os.path.exists(path):
                return path
        self.get_logger().warn(f'모델 미발견 → ultralytics 자동 다운로드: {model_path}')
        return model_path

    # ─────────────────────────────────────────────────────────
    def mode_callback(self, msg):
        self.current_mode = msg.data.strip().lower()

    # ─────────────────────────────────────────────────────────
    def learn_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('start', 'start_with_delay'):
            self.master_db = []
            self.is_learning = True
            self.is_post_delay = False
            self.learning_start_time = time.time()
            self.auto_follow_after_learn = (cmd == 'start_with_delay')
            self.get_logger().info(
                f'>>> 학습 시작 ({self.learning_duration:.0f}초). '
                f'auto_follow={self.auto_follow_after_learn}')
        elif cmd == 'save':
            self._save_master_db()
        elif cmd == 'load':
            self._load_master_db()
        elif cmd == 'reset':
            self.master_db = []
            self.is_learning = False
            self.is_post_delay = False
            self.get_logger().info('>>> 학습 데이터 초기화')
        else:
            self.get_logger().warn(f'알 수 없는 학습 명령: {cmd}')
        self._publish_status()

    # ─────────────────────────────────────────────────────────
    def _save_master_db(self):
        if not self.master_db:
            self.get_logger().warn('저장할 학습 데이터 없음')
            return
        try:
            os.makedirs(SAVE_DIR, exist_ok=True)
            with open(SAVE_PATH, 'w') as f:
                json.dump([h.tolist() for h in self.master_db], f)
            self.get_logger().info(
                f'>>> 학습 데이터 저장 ({len(self.master_db)}개) → {SAVE_PATH}')
        except Exception as e:
            self.get_logger().error(f'저장 실패: {e}')

    def _load_master_db(self):
        if not os.path.exists(SAVE_PATH):
            return
        try:
            with open(SAVE_PATH, 'r') as f:
                data = json.load(f)
            self.master_db = [np.array(d, dtype=np.float32) for d in data]
            self.get_logger().info(f'>>> 학습 데이터 로드 ({len(self.master_db)}개)')
        except Exception as e:
            self.get_logger().error(f'로드 실패: {e}')

    # ─────────────────────────────────────────────────────────
    def _publish_status(self):
        msg = String()
        if self.is_learning:
            elapsed = time.time() - self.learning_start_time
            remaining = max(0, int(self.learning_duration - elapsed))
            msg.data = f'learning:{remaining}'
        elif self.is_post_delay:
            elapsed = time.time() - self.post_delay_start_time
            remaining = max(0, int(self.post_learn_delay - elapsed))
            msg.data = f'wait:{remaining}'
        elif len(self.master_db) > 0:
            msg.data = 'ready'
        else:
            msg.data = 'idle'
        self.learn_status_pub.publish(msg)

    # ─────────────────────────────────────────────────────────
    def _extract_hist(self, frame, x1, y1, x2, y2):
        person_img = frame[y1:y2, x1:x2]
        if person_img.size == 0:
            return None
        hsv = cv2.cvtColor(person_img, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist(
            [hsv], [0, 1], None,
            [HIST_BINS_H, HIST_BINS_S], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist

    # ─────────────────────────────────────────────────────────
    def _publish_annotated(self, frame, master_box=None, all_dets=None,
                           status_text=''):
        if not self.get_parameter('publish_annotated').value:
            return
        try:
            view = frame.copy()
            if all_dets:
                for (x1, y1, x2, y2, tid, _) in all_dets:
                    cv2.rectangle(view, (x1, y1), (x2, y2),
                                  (160, 160, 160), 1)
                    cv2.putText(view, f'ID:{tid}', (x1, max(0, y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (200, 200, 200), 1)
            if master_box is not None:
                x1, y1, x2, y2, tid, sim = master_box
                cv2.rectangle(view, (x1, y1), (x2, y2), (255, 200, 0), 3)
                cv2.putText(view, f'MASTER ID:{tid} sim={sim:.2f}',
                            (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 200, 0), 2)
            if status_text:
                cv2.putText(view, status_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 255), 2)
            jpeg_q = int(self.get_parameter('annotated_jpeg_quality').value)
            ok, buf = cv2.imencode(
                '.jpg', view, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_q])
            if not ok:
                return
            cmsg = CompressedImage()
            cmsg.header.stamp = self.get_clock().now().to_msg()
            cmsg.header.frame_id = 'webcam_link'
            cmsg.format = 'jpeg'
            cmsg.data = buf.tobytes()
            self.annotated_pub.publish(cmsg)
        except Exception as e:
            self.get_logger().error(f'annotated publish 실패: {e}')

    # ─────────────────────────────────────────────────────────
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'camera error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()  # 클래스 이름과 동일해야 함
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()