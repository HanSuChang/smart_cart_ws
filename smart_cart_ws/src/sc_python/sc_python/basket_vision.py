#!/usr/bin/env python3
# =====================================================================
# basket_vision.py
# 터틀봇 후방 바구니 카메라 (USB 웹캠 #2) — OpenCV 영상처리
#
# [시나리오]
#   - 대형마트에서 사용자가 로봇 뒤 바구니에 물건을 넣음
#   - /webcam2/image_raw 를 구독해 ROI(바구니 입구) 안에서 변화 감지
#   - 모션/배경차분 + 컨투어 분석으로 "물체 삽입" 이벤트 판정
#   - 삽입 시점에 스냅샷 저장 (~/.smart_cart/basket_snapshots/)
#   - /basket/event 로 BasketEvent 메시지 발행
#       → GUI 가 구독해 화면에 알림 + Flask 장바구니에 자동 추가
#       → lid_controller 가 구독해서 뚜껑 동작 가능
#       → 분류 모델(미정)이 결정되면 이 노드에서 model.predict 추가
#
# [Topic — Pub/Sub]
#   Subscribe:
#     /webcam2/image_raw     (sensor_msgs/Image)
#
#   Publish:
#     /basket/event          (sc_interfaces/BasketEvent)
#     /basket/annotated/compressed (sensor_msgs/CompressedImage)
#         GUI 의 바구니 카메라 화면에 ROI/모션 박스 오버레이
# =====================================================================

import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage

from sc_interfaces.msg import BasketEvent

SNAPSHOT_DIR = os.path.expanduser('~/.smart_cart/basket_snapshots')


class BasketVision(Node):
    def __init__(self):
        super().__init__('basket_vision')

        # ── 파라미터 ──
        self.declare_parameter('image_topic', '/webcam2/image_raw')
        self.declare_parameter('roi_x', 0.1)
        self.declare_parameter('roi_y', 0.1)
        self.declare_parameter('roi_w', 0.8)
        self.declare_parameter('roi_h', 0.8)
        # 모션 임계값 (픽셀 수)
        self.declare_parameter('motion_threshold', 1500)
        # 모션이 잠잠해지는 데 걸리는 시간 (초) — 안정화 후 스냅샷
        self.declare_parameter('settle_time_sec', 0.5)
        # 같은 이벤트 중복 방지 쿨다운
        self.declare_parameter('cooldown_sec', 2.0)
        # 스냅샷 저장 활성
        self.declare_parameter('save_snapshot', True)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('annotated_jpeg_quality', 60)

        self.bridge = CvBridge()

        # 배경 차분 (MOG2)
        self.bg = cv2.createBackgroundSubtractorMOG2(
            history=400, varThreshold=30, detectShadows=False)

        # 상태
        self.in_motion = False
        self.motion_start_time = 0.0
        self.last_event_time = 0.0
        self.last_motion_time = 0.0

        # ── Pub/Sub ──
        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.event_pub = self.create_publisher(
            BasketEvent, '/basket/event', 10)
        self.annotated_pub = self.create_publisher(
            CompressedImage, '/basket/annotated/compressed', 10)

        if self.get_parameter('save_snapshot').value:
            os.makedirs(SNAPSHOT_DIR, exist_ok=True)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'BasketVision 시작 — topic={image_topic}, '
            f'snapshot_dir={SNAPSHOT_DIR}')

    # ─────────────────────────────────────────────────────────
    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def _get_roi(self, frame):
        h, w = frame.shape[:2]
        rx = int(w * float(self.get_parameter('roi_x').value))
        ry = int(h * float(self.get_parameter('roi_y').value))
        rw = int(w * float(self.get_parameter('roi_w').value))
        rh = int(h * float(self.get_parameter('roi_h').value))
        return rx, ry, rw, rh

    # ─────────────────────────────────────────────────────────
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        rx, ry, rw, rh = self._get_roi(frame)
        roi = frame[ry:ry + rh, rx:rx + rw]
        if roi.size == 0:
            return

        # ── 1) 배경 차분 → 모션 마스크 ──
        fg_mask = self.bg.apply(roi)
        # 노이즈 제거
        fg_mask = cv2.morphologyEx(
            fg_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        fg_mask = cv2.dilate(fg_mask, np.ones((5, 5), np.uint8), iterations=1)
        motion_pixels = int(np.count_nonzero(fg_mask))
        threshold = int(self.get_parameter('motion_threshold').value)
        is_motion = motion_pixels > threshold

        # ── 2) 컨투어 → 가장 큰 변화 영역 찾기 (bbox) ──
        bx = by = bw = bh = 0
        if is_motion:
            contours, _ = cv2.findContours(
                fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                bx, by, bw, bh = cv2.boundingRect(largest)
                # 전체 frame 좌표계로 보정
                bx += rx
                by += ry

        # ── 3) 이벤트 판정 ──
        now = time.time()
        cooldown = float(self.get_parameter('cooldown_sec').value)
        settle = float(self.get_parameter('settle_time_sec').value)

        event_type = 'stable'
        snapshot_path = ''

        if is_motion:
            self.last_motion_time = now
            if not self.in_motion:
                self.in_motion = True
                self.motion_start_time = now
        else:
            # 모션 종료 후 settle_time 만큼 안정화되면 'insert' 이벤트 확정
            if self.in_motion and (now - self.last_motion_time) > settle:
                self.in_motion = False
                if (now - self.last_event_time) > cooldown:
                    event_type = 'insert'
                    self.last_event_time = now
                    if self.get_parameter('save_snapshot').value:
                        snapshot_path = self._save_snapshot(frame)

        # ── 4) Publish ──
        self._publish_event(
            event_type, motion_pixels, bx, by, bw, bh, snapshot_path)
        self._publish_annotated(
            frame, (rx, ry, rw, rh), is_motion,
            (bx, by, bw, bh) if is_motion else None,
            motion_pixels, event_type)

    # ─────────────────────────────────────────────────────────
    def _save_snapshot(self, frame) -> str:
        try:
            ts = time.strftime('%Y%m%d_%H%M%S')
            path = os.path.join(SNAPSHOT_DIR, f'basket_{ts}.jpg')
            cv2.imwrite(path, frame)
            self.get_logger().info(f'>>> 스냅샷 저장: {path}')
            return path
        except Exception as e:
            self.get_logger().error(f'스냅샷 저장 실패: {e}')
            return ''

    def _publish_event(self, event_type, motion_pixels,
                       bx, by, bw, bh, snapshot_path):
        # 분류기 미정 — item_name/confidence 는 빈 값
        m = BasketEvent()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'basket_cam'
        m.event_type = event_type
        m.item_name = ''
        m.confidence = 0.0
        m.motion_pixels = int(motion_pixels)
        m.bbox_x = int(bx)
        m.bbox_y = int(by)
        m.bbox_width = int(bw)
        m.bbox_height = int(bh)
        m.snapshot_path = snapshot_path
        self.event_pub.publish(m)

    def _publish_annotated(self, frame, roi, is_motion, bbox,
                           motion_pixels, event_type):
        if not self.get_parameter('publish_annotated').value:
            return
        view = frame.copy()
        rx, ry, rw, rh = roi
        color = (0, 255, 0) if is_motion else (140, 140, 140)
        cv2.rectangle(view, (rx, ry), (rx + rw, ry + rh), color, 2)
        cv2.putText(view, 'BASKET', (rx, max(0, ry - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if bbox is not None:
            bx, by, bw, bh = bbox
            cv2.rectangle(view, (bx, by), (bx + bw, by + bh),
                          (0, 200, 255), 2)
            cv2.putText(view, 'MOTION', (bx, max(0, by - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        cv2.putText(view,
                    f'pix={motion_pixels} state={event_type}',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 2)

        try:
            jpeg_q = int(self.get_parameter('annotated_jpeg_quality').value)
            ok, buf = cv2.imencode(
                '.jpg', view, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_q])
            if not ok:
                return
            cmsg = CompressedImage()
            cmsg.header.stamp = self.get_clock().now().to_msg()
            cmsg.header.frame_id = 'basket_cam'
            cmsg.format = 'jpeg'
            cmsg.data = buf.tobytes()
            self.annotated_pub.publish(cmsg)
        except Exception as e:
            self.get_logger().error(f'basket annotated publish 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BasketVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
