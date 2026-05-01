# =====================================================================
# main.py
# Smart Cart GUI 진입점
#   - Flask 결제 서버 (localhost:5000)
#   - PyQt5 관제 GUI
#   - ROS2 노드 (Flask 결제 ↔ 토픽 브릿지)
#
# launch에서 sc_gui frictionless_gui 실행 시 위 3가지 동시 가동
# =====================================================================

import os
import sys
import threading
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QApplication

import rclpy

from sc_gui.cart_gui import SmartCartServer, CartRosBridgeNode
from sc_gui.robot_gui import FrictionlessStoreGUI


def run_flask_server(server: SmartCartServer):
    server.run()


def run_rclpy_node(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def _resolve_map_yaml():
    """sc_bringup/maps/map_cleaned.yaml 경로 자동 해석"""
    # 1) 환경변수
    env = os.environ.get('SC_MAP_YAML')
    if env and os.path.exists(env):
        return env
    # 2) ament share
    try:
        share = get_package_share_directory('sc_bringup')
        cand = os.path.join(share, 'maps', 'map_cleaned.yaml')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    # 3) 워크스페이스 src
    here = os.getcwd()
    cand = os.path.join(
        here, 'src', 'sc_bringup', 'maps', 'map_cleaned.yaml')
    if os.path.exists(cand):
        return cand
    return 'src/sc_bringup/maps/map_cleaned.yaml'


def main():
    # ── ROS 초기화 ──
    rclpy.init()

    # ── 1) Flask 결제 서버 + ROS 브릿지 노드 ──
    bridge_node = CartRosBridgeNode()
    server = SmartCartServer(ros_bridge=bridge_node)
    threading.Thread(
        target=run_flask_server, args=(server,), daemon=True).start()
    threading.Thread(
        target=run_rclpy_node, args=(bridge_node,), daemon=True).start()

    # ── 2) PyQt5 관제 GUI ──
    app = QApplication(sys.argv)

    # 로봇 연동 설정
    ROBOT_IP = os.environ.get('SC_ROBOT_IP', '127.0.0.1')
    ROBOT_USER = os.environ.get('SC_ROBOT_USER', '')
    ROBOT_PASS = os.environ.get('SC_ROBOT_PASS', '')
    WORKSPACE_PATH = os.environ.get(
        'SC_WORKSPACE_PATH', '~/smart_cart_ws/smart_cart_ws')

    map_yaml = _resolve_map_yaml()
    print(f'[main] map yaml = {map_yaml}')

    window = FrictionlessStoreGUI(
        yaml_path=map_yaml,
        robot_ip=ROBOT_IP,
        robot_user=ROBOT_USER,
        robot_pw=ROBOT_PASS,
        workspace_path=WORKSPACE_PATH,
        flask_url='http://127.0.0.1:5000',
    )
    window.show()
    rc = app.exec_()

    try:
        bridge_node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(rc)


if __name__ == '__main__':
    main()
