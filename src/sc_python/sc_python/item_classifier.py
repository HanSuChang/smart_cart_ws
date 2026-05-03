#!/usr/bin/env python3
# =====================================================================
# item_classifier.py
# 두 번째 USB 웹캠(/webcam2/image_raw) → YOLOv8n → /item_detected
#
# ※ 본 노드는 분류기 (Roboflow 학습 모델 등) 가 정해진 후 사용
#    분류기 결정 전이라도 yolov8n.pt (COCO) 로 동작 가능
#    바구니 카메라의 OpenCV 영상처리 (모션, 배경차분, 스냅샷)는
#    별도 노드 basket_vision.py 에서 담당 → /basket/event 발행
#
# [Topic — Pub/Sub]
#   Subscribe:
#     /webcam2/image_raw                (sensor_msgs/Image)
#
#   Publish:
#     /item_detected                    (sc_interfaces/ItemDetected)
#     /yolo/item_image/compressed       (sensor_msgs/CompressedImage)
# =====================================================================

import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from ultralytics import YOLO

from sc_interfaces.msg import ItemDetected


class ItemClassifier(Node):
    def __init__(self):
        super().__init__('item_classifier')

        # ── 파라미터 ──
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.6)
        self.declare_parameter('image_topic', '/webcam2/image_raw')
        self.declare_parameter('img_size', 320)
        self.declare_parameter('roi_x', 0.1)
        self.declare_parameter('roi_y', 0.1)
        self.declare_parameter('roi_w', 0.8)
        self.declare_parameter('roi_h', 0.5)
        self.declare_parameter('motion_threshold', 500)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('annotated_jpeg_quality', 60)
        self.declare_parameter('show_debug', False)

        model_path = self.get_parameter('model_path').value
        image_topic = self.get_parameter('image_topic').value

        self.bridge = CvBridge()
        self.model = YOLO(self._resolve_model_path(model_path))
        self.class_names = self.model.names if hasattr(self.model, 'names') else {}

        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=300, varThreshold=25, detectShadows=False)

        # ── Pub/Sub ──
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.item_pub = self.create_publisher(ItemDetected, '/item_detected', 10)
        self.annotated_pub = self.create_publisher(
            CompressedImage, '/yolo/item_image/compressed', 10)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'ItemClassifier 시작 — model={model_path}, topic={image_topic}, '
            f'classes={len(self.class_names)}')

    # ─────────────────────────────────────────────────────────
    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

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

    def _get_roi(self, frame):
        h, w = frame.shape[:2]
        rx = int(w * float(self.get_parameter('roi_x').value))
        ry = int(h * float(self.get_parameter('roi_y').value))
        rw = int(w * float(self.get_parameter('roi_w').value))
        rh = int(h * float(self.get_parameter('roi_h').value))
        return rx, ry, rw, rh

    def _publish_annotated(self, frame, items, roi, motion):
        if not self.get_parameter('publish_annotated').value:
            return
        view = frame.copy()
        rx, ry, rw, rh = roi
        color = (0, 255, 0) if motion else (80, 80, 80)
        cv2.rectangle(view, (rx, ry), (rx + rw, ry + rh), color, 2)
        cv2.putText(view, 'BASKET ROI', (rx, max(0, ry - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        for (cls, conf, x1, y1, x2, y2) in items:
            cv2.rectangle(view, (x1, y1), (x2, y2), (0, 200, 255), 2)
            cv2.putText(view, f'{cls} {conf:.2f}', (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        try:
            jpeg_q = int(self.get_parameter('annotated_jpeg_quality').value)
            ok, buf = cv2.imencode(
                '.jpg', view, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_q])
            if not ok:
                return
            cmsg = CompressedImage()
            cmsg.header.stamp = self.get_clock().now().to_msg()
            cmsg.header.frame_id = 'webcam2_link'
            cmsg.format = 'jpeg'
            cmsg.data = buf.tobytes()
            self.annotated_pub.publish(cmsg)
        except Exception as e:
            self.get_logger().error(f'item annotated publish 실패: {e}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        rx, ry, rw, rh = self._get_roi(frame)
        roi_img = frame[ry:ry + rh, rx:rx + rw]

        motion = False
        if roi_img.size > 0:
            fg_mask = self.bg_subtractor.apply(roi_img)
            motion_pixels = int(np.count_nonzero(fg_mask))
            motion = motion_pixels > int(
                self.get_parameter('motion_threshold').value)

        items = []
        if motion:
            conf = float(self.get_parameter('confidence').value)
            imgsz = int(self.get_parameter('img_size').value)
            try:
                results = self.model.predict(
                    frame, conf=conf, imgsz=imgsz, verbose=False)
            except Exception as e:
                self.get_logger().error(f'YOLO 추론 실패: {e}')
                results = []

            for r in results:
                if r.boxes is None:
                    continue
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0]) if box.cls is not None else -1
                    conf_v = float(box.conf[0]) if box.conf is not None else 0.0
                    cls_name = self.class_names.get(cls_id, f'cls{cls_id}')
                    bbox_cx = (x1 + x2) / 2.0
                    bbox_cy = (y1 + y2) / 2.0
                    in_roi = (rx <= bbox_cx <= rx + rw) and \
                             (ry <= bbox_cy <= ry + rh)
                    items.append((cls_name, conf_v, x1, y1, x2, y2))
                    self._publish_item(
                        cls_name, conf_v, x1, y1, x2 - x1, y2 - y1, in_roi)

        self._publish_annotated(frame, items, (rx, ry, rw, rh), motion)

        if self.get_parameter('show_debug').value:
            try:
                cv2.imshow('item_classifier', frame)
                cv2.waitKey(1)
            except Exception:
                pass

    def _publish_item(self, name, conf, x, y, w, h, in_roi):
        m = ItemDetected()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'webcam2_link'
        m.item_name = str(name)
        m.confidence = float(conf)
        m.bbox_x = int(x)
        m.bbox_y = int(y)
        m.bbox_width = int(w)
        m.bbox_height = int(h)
        m.in_basket_zone = bool(in_roi)
        self.item_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ItemClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
