#!/usr/bin/env python3
# ================================================================
# person_tracker.py
# [Python 담당] USB 웹캠 → YOLOv8 + DeepSORT → /person_bbox
#
# 동작:
#   1. USB 웹캠(v4l2_camera)에서 /webcam/image_raw 수신
#   2. YOLOv8로 사람(class 0) 검출
#   3. DeepSORT로 ID 부여 + 추적 유지
#   4. 추적 대상의 bbox를 /person_bbox로 publish
#      → C++(follow_controller)가 PID로 /cmd_vel 생성
#      → C++(pan_tilt_controller)가 서보 각도 계산
#
# 카메라: USB 웹캠 (/dev/video0)
# 모델: YOLOv8n (RPi4 경량화)
# 추적: DeepSORT (pip install deep-sort-realtime)
#
# Subscribe: /webcam/image_raw (sensor_msgs/msg/Image)
# Publish:   /person_bbox (sc_interfaces/msg/PersonBbox)
# ================================================================

import rclpy
from rclpy.node import Node


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')
        # TODO: 파라미터 선언 (model_path, confidence, image_topic 등)
        # TODO: /webcam/image_raw subscriber
        # TODO: /person_bbox publisher
        # TODO: YOLO 모델 로드
        # TODO: DeepSORT 초기화
        self.get_logger().info('PersonTracker 노드 생성됨 (TODO: 구현 필요)')


def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
