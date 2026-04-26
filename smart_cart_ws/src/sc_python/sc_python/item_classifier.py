#!/usr/bin/env python3
# ================================================================
# item_classifier.py
# [Python 담당] RPi 카메라 → Roboflow 학습 모델 → /item_detected
#
# 동작:
#   1. RPi 카메라에서 /rpi_camera/image_raw 수신
#   2. 바구니 입구 ROI 영역에서 배경 차분으로 모션 감지
#   3. 모션 감지 시 Roboflow 학습 YOLO 모델로 물체 분류
#   4. /item_detected publish
#      → C++(lid_controller)가 연속 감지 확정 → 뚜껑 열기
#
# 카메라: 라즈베리파이 카메라 (CSI)
# 모델: Roboflow에서 학습 → YOLOv8n export
#       models/ 폴더에 .pt 파일 배치
#
# Subscribe: /rpi_camera/image_raw (sensor_msgs/msg/Image)
# Publish:   /item_detected (sc_interfaces/msg/ItemDetected)
# ================================================================

import rclpy
from rclpy.node import Node


class ItemClassifier(Node):
    def __init__(self):
        super().__init__('item_classifier')
        # TODO: 파라미터 선언 (model_path, confidence, roi 영역 등)
        # TODO: /rpi_camera/image_raw subscriber
        # TODO: /item_detected publisher
        # TODO: Roboflow 학습 모델 로드
        # TODO: 배경 차분 모션 감지 초기화
        self.get_logger().info('ItemClassifier 노드 생성됨 (TODO: 구현 필요)')


def main(args=None):
    rclpy.init(args=args)
    node = ItemClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
