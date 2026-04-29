robot_gui.py

import sys
import yaml
import base64
import roslibpy
import requests
from PyQt6.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout,
                             QHBoxLayout, QWidget, QMessageBox, QTextEdit, QFrame, QPushButton)
from PyQt6.QtGui import QPixmap, QPainter, QPen, QColor, QFont
from PyQt6.QtCore import Qt, pyqtSignal, QObject, QDateTime, QThread
from waypoint_manager import InteractiveMapPanel

try:
    import paramiko
except ImportError:
    print("paramiko 라이브러리가 필요합니다. CMD에서 'pip install paramiko'를 실행하세요.")
    sys.exit(1)


# =====================================================================
# [0] 원격 런치 매니저: SSH Thread 처리 클래스
# =====================================================================
class SshWorker(QThread):
    log_signal = pyqtSignal(str)

    def __init__(self, ip, username, password, command):
        super().__init__()
        self.ip = ip
        self.username = username
        self.password = password
        self.command = command

    def run(self):
        try:
            self.log_signal.emit(f"> [SSH] ATTEMPTING SECURE CONNECTION TO {self.username}@{self.ip}...")
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.ip, username=self.username, password=self.password, timeout=5)
            self.log_signal.emit("> [SSH] ACCESS GRANTED. EXECUTING LAUNCH SEQUENCE...")

            stdin, stdout, stderr = ssh.exec_command(self.command)
            err = stderr.read().decode().strip()
            if err and "command not found" in err.lower():
                self.log_signal.emit(f"> [SSH ERROR] {err}")
            else:
                self.log_signal.emit("> [SSH] REMOTE LAUNCH SIGNAL TRANSMITTED SUCCESSFULLY.")
            ssh.close()
        except Exception as e:
            self.log_signal.emit(f"> [SSH FATAL ERROR] {e}")


# =====================================================================
# [1] 통신 매니저: ROS2 Websocket 처리 클래스
# =====================================================================
class RosManager(QObject):
    pose_signal = pyqtSignal(float, float)
    status_signal = pyqtSignal(str)
    log_signal = pyqtSignal(str)
    camera1_signal = pyqtSignal(bytes)
    camera2_signal = pyqtSignal(bytes)

    def __init__(self, ip_address):
        super().__init__()
        self.ip_address = ip_address
        self.ros = None
        self.is_moving = False

    def connect_to_robot(self):
        self.log_signal.emit(f"> ESTABLISHING UPLINK TO {self.ip_address}:9090...")
        try:
            self.ros = roslibpy.Ros(host=self.ip_address, port=9090)
            self.ros.run()

            if self.ros.is_connected:
                self.log_signal.emit("> UPLINK SECURED. TELEMETRY ACTIVE.")
                self.status_signal.emit("UPDATE")

                self.odom_listener = roslibpy.Topic(self.ros, '/odom', 'nav_msgs/Odometry')
                self.odom_listener.subscribe(self.odom_callback)

                self.cmd_vel_listener = roslibpy.Topic(self.ros, '/cmd_vel', 'geometry_msgs/Twist')
                self.cmd_vel_listener.subscribe(self.cmd_vel_callback)

                self.cam1_listener = roslibpy.Topic(self.ros, '/webcam/image_raw/compressed',
                                                    'sensor_msgs/CompressedImage')
                self.cam1_listener.subscribe(self.camera1_callback)

                self.cam2_listener = roslibpy.Topic(self.ros, '/rpi_camera/image_raw/compressed',
                                                    'sensor_msgs/CompressedImage')
                self.cam2_listener.subscribe(self.camera2_callback)

                self.item_listener = roslibpy.Topic(self.ros, '/item_detected', 'sc_interfaces/ItemDetected')
                self.item_listener.subscribe(self.item_callback)
            else:
                self.log_signal.emit("> [WARNING] WS REFUSED. (Launch가 아직 실행되지 않았을 수 있습니다)")
        except Exception as e:
            self.log_signal.emit(f"> [ERROR] NETWORK FAILURE: {e}")

    def send_mission(self, mission_id):
        if self.ros and self.ros.is_connected:
            topic = roslibpy.Topic(self.ros, '/mission_control', 'std_msgs/String')
            topic.publish(roslibpy.Message({'data': mission_id}))
            topic.unadvertise()

    def send_estop(self):
        if self.ros and self.ros.is_connected:
            topic = roslibpy.Topic(self.ros, '/safety_stop', 'std_msgs/Bool')
            topic.publish(roslibpy.Message({'data': True}))
            topic.unadvertise()

    def disconnect(self):
        if self.ros and self.ros.is_connected:
            if hasattr(self, 'odom_listener'): self.odom_listener.unsubscribe()
            if hasattr(self, 'cmd_vel_listener'): self.cmd_vel_listener.unsubscribe()
            if hasattr(self, 'cam1_listener'): self.cam1_listener.unsubscribe()
            if hasattr(self, 'cam2_listener'): self.cam2_listener.unsubscribe()
            if hasattr(self, 'item_listener'): self.item_listener.unsubscribe()
            self.ros.terminate()

    def odom_callback(self, message):
        try:
            x, y = message['pose']['pose']['position']['x'], message['pose']['pose']['position']['y']
            self.pose_signal.emit(x, y)
        except Exception:
            pass

    def cmd_vel_callback(self, message):
        try:
            moving = abs(message['linear']['x']) > 0.01 or abs(message['angular']['z']) > 0.01
            if moving != self.is_moving:
                self.is_moving = moving
                self.status_signal.emit("UPDATE")
        except Exception:
            pass

    def camera1_callback(self, message):
        try:
            self.camera1_signal.emit(base64.b64decode(message['data']))
        except Exception:
            pass

    def camera2_callback(self, message):
        try:
            self.camera2_signal.emit(base64.b64decode(message['data']))
        except Exception:
            pass

    def item_callback(self, message):
        try:
            item_name = message.get('item_name', '')
            in_basket = message.get('in_basket_zone', True)

            if in_basket and item_name:
                res = requests.post('http://127.0.0.1:5000/api/add_item',
                                    json={'class_name': item_name},
                                    timeout=1.0)

                if res.status_code == 200:
                    self.log_signal.emit(f"> [WEB 연동] '{item_name}' 장바구니 갱신 완료")
        except requests.exceptions.RequestException:
            pass
        except Exception as e:
            self.log_signal.emit(f"> [WEB 연동 오류] {e}")


# =====================================================================
# [2] UI 컴포넌트
# =====================================================================
class BasePanel(QFrame):
    def __init__(self, title_text=""):
        super().__init__()
        self.setProperty("class", "Panel")
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)

        if title_text:
            self.title_label = QLabel(title_text)
            self.title_label.setProperty("class", "Title")
            self.layout.addWidget(self.title_label)


class LogPanel(BasePanel):
    def __init__(self):
        super().__init__(" SYSTEM DIAGNOSTICS & EVENT LOG")
        self.log_window = QTextEdit()
        self.log_window.setReadOnly(True)
        self.log_window.setProperty("class", "Terminal")
        self.layout.addWidget(self.log_window, stretch=1)

    def append_log(self, text):
        time_str = QDateTime.currentDateTime().toString("HH:mm:ss")
        error_keywords = ["ERROR", "CRITICAL", "WARNING", "FATAL", "오류", "실패", "경고"]

        if any(keyword in text.upper() for keyword in error_keywords):
            formatted_text = f"<span style='color:#FF3366; font-weight:bold;'>{text}</span>"
        else:
            formatted_text = text

        self.log_window.append(f"<span style='color:#555;'>[{time_str}]</span> {formatted_text}")
        scrollbar = self.log_window.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


class CameraPanel(BasePanel):
    def __init__(self, title, default_text):
        super().__init__(title)
        self.camera_label = QLabel(default_text)
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setStyleSheet(
            "background-color: transparent; color: #555555; font-weight: bold; font-size: 11px;")
        self.layout.addWidget(self.camera_label, stretch=1)

    def render_base_image(self, img_bytes):
        pixmap = QPixmap()
        pixmap.loadFromData(img_bytes)
        return pixmap.scaled(self.camera_label.size(), Qt.AspectRatioMode.KeepAspectRatio,
                             Qt.TransformationMode.SmoothTransformation)


class MainCameraPanel(CameraPanel):
    def __init__(self):
        super().__init__(" FPV TARGET TRACKING", "NO SIGNAL /webcam/image_raw")

    def update_view(self, img_bytes):
        scaled_pixmap = self.render_base_image(img_bytes)
        painter = QPainter(scaled_pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = scaled_pixmap.width(), scaled_pixmap.height()
        cx, cy = w // 2, h // 2

        painter.setPen(QPen(QColor(0, 229, 255, 150), 1))
        painter.drawEllipse(cx - 4, cy - 4, 8, 8)
        painter.drawLine(cx, cy - 20, cx, cy - 8)
        painter.drawLine(cx, cy + 8, cx, cy + 20)
        painter.drawLine(cx - 20, cy, cx - 8, cy)
        painter.drawLine(cx + 8, cy, cx + 20, cy)
        painter.end()
        self.camera_label.setPixmap(scaled_pixmap)


class BasketCameraPanel(CameraPanel):
    def __init__(self):
        super().__init__(" ITEM RECOGNITION VISION", "NO SIGNAL /rpi_camera")

    def update_view(self, img_bytes):
        self.camera_label.setPixmap(self.render_base_image(img_bytes))


class SidebarPanel(QFrame):
    launch_requested = pyqtSignal()
    mission_requested = pyqtSignal(str, str)
    estop_requested = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setProperty("class", "Panel")
        self.setFixedWidth(280)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        title = QLabel("FRICTIONLESS\nCOMMAND")
        title.setStyleSheet("color: #FFFFFF; font-size: 22px; font-weight: 900; letter-spacing: 1px;")
        layout.addWidget(title)

        self.status_label = QLabel("STANDBY")
        self.status_label.setProperty("class", "Status")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        layout.addSpacing(20)

        lbl_sys = QLabel("SYSTEM CONTROL")
        lbl_sys.setStyleSheet("color: #666; font-size: 11px; font-weight: bold;")
        layout.addWidget(lbl_sys)

        btn_launch = QPushButton("SYSTEM LAUNCH")
        btn_launch.setProperty("class", "LaunchBtn")
        btn_launch.clicked.connect(self.launch_requested.emit)
        layout.addWidget(btn_launch)

        layout.addSpacing(10)

        lbl_mis = QLabel("MISSION CONTROL")
        lbl_mis.setStyleSheet("color: #666; font-size: 11px; font-weight: bold;")
        layout.addWidget(lbl_mis)

        btn_follow = QPushButton("사람 유동 추종")
        btn_follow.setProperty("class", "MissionBtn")
        btn_follow.clicked.connect(lambda: self.mission_requested.emit("FOLLOW", "사람 추종"))
        layout.addWidget(btn_follow)

        btn_guide_a = QPushButton("A구역 안내")
        btn_guide_a.setProperty("class", "MissionBtn")
        btn_guide_a.clicked.connect(lambda: self.mission_requested.emit("GUIDE_A", "A구역 안내"))
        layout.addWidget(btn_guide_a)

        btn_guide_b = QPushButton("B구역 안내")
        btn_guide_b.setProperty("class", "MissionBtn")
        btn_guide_b.clicked.connect(lambda: self.mission_requested.emit("GUIDE_B", "B구역 안내"))
        layout.addWidget(btn_guide_b)

        btn_return = QPushButton("복귀 및 도킹")
        btn_return.setProperty("class", "MissionBtn")
        btn_return.clicked.connect(lambda: self.mission_requested.emit("RETURN", "복귀 및 주차"))
        layout.addWidget(btn_return)

        layout.addStretch()

        btn_estop = QPushButton("EMERGENCY STOP")
        btn_estop.setObjectName("EStopBtn")
        btn_estop.clicked.connect(self.estop_requested.emit)
        layout.addWidget(btn_estop)

    def update_status(self, text, color):
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color};")


# =====================================================================
# [3] 메인 윈도우: 시스템 통합
# =====================================================================
class FrictionlessStoreGUI(QMainWindow):
    def __init__(self, yaml_path, robot_ip, robot_user, robot_pw, workspace_path):
        super().__init__()
        self.setWindowTitle("Project Frictionless Store 2026 - SCOUT COMMAND CENTER")
        self.resize(1600, 900)
        self.setup_stylesheet()

        self.robot_ip = robot_ip
        self.robot_user = robot_user
        self.robot_pw = robot_pw
        self.workspace_path = workspace_path
        self.current_mission = "IDLE"
        self.ssh_worker = None

        self.ros_manager = RosManager(robot_ip)
        self.sidebar_panel = SidebarPanel()

        # 💡 독립된 대화형 맵 에디터 패널 연동!
        self.map_panel = InteractiveMapPanel(yaml_path)

        self.log_panel = LogPanel()
        self.cam1_panel = MainCameraPanel()
        self.cam2_panel = BasketCameraPanel()

        self.build_layout()
        self.connect_signals()

        self.log_panel.append_log("SYSTEM BOOT SEQUENCE INITIATED.")
        self.ros_manager.connect_to_robot()

    def build_layout(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        main_layout.addWidget(self.sidebar_panel)

        vision_layout = QVBoxLayout()
        vision_layout.setSpacing(20)

        top_vision = QHBoxLayout()
        top_vision.setSpacing(20)

        top_vision.addWidget(self.map_panel, stretch=6)

        cam_col = QVBoxLayout()
        cam_col.setSpacing(20)
        cam_col.addWidget(self.cam1_panel)
        cam_col.addWidget(self.cam2_panel)
        top_vision.addLayout(cam_col, stretch=4)

        vision_layout.addLayout(top_vision, stretch=7)
        vision_layout.addWidget(self.log_panel, stretch=3)

        main_layout.addLayout(vision_layout, stretch=1)

    def connect_signals(self):
        self.map_panel.log_event.connect(self.log_panel.append_log)
        self.ros_manager.log_signal.connect(self.log_panel.append_log)

        self.ros_manager.pose_signal.connect(self.map_panel.update_pose)
        self.ros_manager.status_signal.connect(self.update_status_ui)
        self.ros_manager.camera1_signal.connect(self.cam1_panel.update_view)
        self.ros_manager.camera2_signal.connect(self.cam2_panel.update_view)

        self.sidebar_panel.launch_requested.connect(self.handle_remote_launch)
        self.sidebar_panel.mission_requested.connect(self.handle_mission)
        self.sidebar_panel.estop_requested.connect(self.handle_estop)

    def handle_remote_launch(self):
        launch_cmd = (f"bash -c 'source /opt/ros/humble/setup.bash && "
                      f"source {self.workspace_path}/install/setup.bash && "
                      f"nohup ros2 launch sc_bringup robot.launch.py > ~/gui_launch.log 2>&1 &'")

        self.ssh_worker = SshWorker(self.robot_ip, self.robot_user, self.robot_pw, launch_cmd)
        self.ssh_worker.log_signal.connect(self.log_panel.append_log)
        self.ssh_worker.start()
        self.log_panel.append_log("[INFO] ROS2 노드 부팅에 5~10초가 소요됩니다.")

    def handle_mission(self, mission_id, mission_name):
        if not self.ros_manager.ros or not self.ros_manager.ros.is_connected:
            msg_box = QMessageBox(self)
            msg_box.setWindowTitle("경고")
            msg_box.setText("먼저 SYSTEM LAUNCH를 눌러 로봇을 가동하거나 통신을 점검하세요.")
            msg_box.setIcon(QMessageBox.Icon.Warning)
            msg_box.addButton("확인", QMessageBox.ButtonRole.AcceptRole)
            msg_box.exec()

            self.ros_manager.connect_to_robot()
            return

        self.current_mission = mission_id
        self.log_panel.append_log(f"COMMAND TRANSMITTED: EXECUTE MISSION [{mission_name}]")
        self.ros_manager.send_mission(mission_id)

        # 💡 향후 Nav2 연동을 위한 좌표 로드 로직 준비 (출력만 수행)
        wp_coords = self.map_panel.wp_db.get_waypoint(mission_name)
        if wp_coords:
            self.log_panel.append_log(f"> [NAV2 대기] 목적지 좌표 로드 완료: X:{wp_coords['x']:.2f}, Y:{wp_coords['y']:.2f}")

        self.update_status_ui()

    def handle_estop(self):
        if self.ros_manager.ros and self.ros_manager.ros.is_connected:
            self.current_mission = "ESTOP"
            self.log_panel.append_log("[CRITICAL] EMERGENCY STOP INITIATED.")
            self.ros_manager.send_estop()
            self.update_status_ui()

    def update_status_ui(self):
        move_str = "[IN MOTION]" if self.ros_manager.is_moving else "[STANDBY]"
        color = "#00E5FF"

        if self.current_mission == "ESTOP":
            text, color = "E-STOP ENGAGED\nMOTORS LOCKED", "#FF3366"
        elif self.current_mission == "FOLLOW":
            text = f"TRACKING TARGET\n{move_str}"
        elif self.current_mission == "GUIDE_A":
            text = f"NAVIGATING: ZONE A\n{move_str}"
        elif self.current_mission == "GUIDE_B":
            text = f"NAVIGATING: ZONE B\n{move_str}"
        elif self.current_mission == "RETURN":
            text = "RTB (RETURN TO BASE)" if self.ros_manager.is_moving else "ARUCO DOCKING..."
            color = "#FFCC00"
        else:
            text = "SYSTEM STANDBY\nREADY FOR COMMAND"

        self.sidebar_panel.update_status(text, color)

    def closeEvent(self, event):
        self.log_panel.append_log("SHUTDOWN SEQUENCE INITIATED.")
        self.ros_manager.disconnect()
        event.accept()

    def setup_stylesheet(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #0A0A0C; }

            QFrame.Panel { 
                background-color: #121214; 
                border-radius: 12px; 
            }

            QLabel { color: #D0D0D0; font-family: 'Malgun Gothic', sans-serif; }
            QLabel.Title { 
                color: #888888; 
                font-weight: bold; 
                font-size: 11px; 
                padding: 12px 15px; 
                letter-spacing: 1px; 
                background-color: transparent;
            }
            QLabel.Status { font-weight: bold; font-size: 15px; letter-spacing: 1px;}

            QTextEdit.Terminal { 
                background-color: transparent; 
                border: none; 
                color: #00E5FF; 
                font-family: 'Consolas', monospace; 
                font-size: 13px; 
                padding: 0px 15px 15px 15px;
            }

            QPushButton {
                font-family: 'Malgun Gothic';
                border-radius: 6px;
                padding: 14px;
                font-size: 13px;
                font-weight: bold;
                letter-spacing: 1px;
            }

            QPushButton.MissionBtn { 
                background-color: #1A1A1E; 
                color: #D0D0D0; 
                border: 1px solid #2A2A2E; 
                text-align: left;
                padding-left: 20px;
            }
            QPushButton.MissionBtn:hover { background-color: #2A2A2E; color: #FFFFFF; }
            QPushButton.MissionBtn:pressed { background-color: #00E5FF; color: #000000; }

            QPushButton.LaunchBtn { 
                background-color: #004455; 
                color: #00E5FF; 
                border: none;
            }
            QPushButton.LaunchBtn:hover { background-color: #006677; color: #FFFFFF; }
            QPushButton.LaunchBtn:pressed { background-color: #008899; }

            QPushButton#EStopBtn { 
                background-color: #330A0A; 
                color: #FF3366; 
                border: 1px solid #FF3366; 
            }
            QPushButton#EStopBtn:hover { background-color: #FF3366; color: #FFFFFF; }

            QMessageBox {
                background-color: #121214;
            }
            QMessageBox QLabel {
                color: #E0E0E0;
                font-size: 14px;
                background-color: transparent;
            }
            QMessageBox QPushButton {
                background-color: #2A2A2E;
                color: #FFFFFF;
                border: 1px solid #444;
                min-width: 80px;
                padding: 8px 16px;
            }
            QMessageBox QPushButton:hover {
                background-color: #3A3A3E;
            }
            QMessageBox QPushButton:pressed {
                background-color: #00E5FF;
                color: #000000;
            }
        """)

