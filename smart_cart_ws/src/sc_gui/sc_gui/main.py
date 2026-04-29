main.py

import sys
import threading
from PyQt6.QtWidgets import QApplication
from cart_gui import SmartCartServer
from robot_gui import FrictionlessStoreGUI


def run_flask_server():
    """백그라운드에서 Flask 웹 장바구니 서버를 실행하는 함수"""
    server = SmartCartServer()
    server.run()


def main():
    # =========================================================
    # [Step 1] 웹 서버(Flask) 스레드 분리 실행
    # =========================================================
    flask_thread = threading.Thread(target=run_flask_server, daemon=True)
    flask_thread.start()

    # =========================================================
    # [Step 2] 로봇 관제 GUI(PyQt6) 메인 스레드 실행
    # =========================================================
    app = QApplication(sys.argv)

    # 로봇 연동 및 시스템 기본 설정값
    ROBOT_IP = "192.168.0.222"
    ROBOT_USER = "song"
    ROBOT_PASS = "1234"
    WORKSPACE_PATH = "~/smart_cart_ws"
    MAP_YAML_FILE = "map_cleaned.yaml"

    window = FrictionlessStoreGUI(yaml_path=MAP_YAML_FILE,
                                  robot_ip=ROBOT_IP,
                                  robot_user=ROBOT_USER,
                                  robot_pw=ROBOT_PASS,
                                  workspace_path=WORKSPACE_PATH)


    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()

