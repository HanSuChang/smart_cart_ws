#!/usr/bin/env python3
# ================================================================
# cart_gui.py
# [GUI 담당] 카트 상태 실시간 표시
#
# Subscribe: /cart_status (sc_interfaces/msg/CartStatus)
#            /webcam/image_raw (카메라 영상 표시)
#            /rpi_camera/image_raw (물체 인식 영상 표시)
#
# GUI 프레임워크: PyQt5 / rqt / 등 GUI 담당 팀원이 결정
# ================================================================

import rclpy
from rclpy.node import Node


class CartGui(Node):
    def __init__(self):
        super().__init__('cart_gui')
        # TODO: GUI 담당 팀원이 구현
        self.get_logger().info('CartGui 노드 생성됨 (TODO: 구현 필요)')


def main(args=None):
    rclpy.init(args=args)
    node = CartGui()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
