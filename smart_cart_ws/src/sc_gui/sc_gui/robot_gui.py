# =====================================================================
# robot_gui.py
# Smart Cart 메인 관제 GUI (PyQt6)
#
# 기능:
#   1. 맵 표시 + 웨이포인트(노드) 찍기/저장
#   2. 사람 추종 모드 / 자동 주행 모드 전환
#   3. 저장된 웨이포인트로 Nav2 자동 주행 (send_goal)
#   4. USB 웹캠 2대 실시간 표시
#   5. 비상 정지
# =====================================================================

import sys
import base64
import roslibpy
import requests
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget,
    QMessageBox, QTextEdit, QFrame, QPushButton, QScrollArea
)
from PyQt6.QtGui import QPixmap, QPainter, QPen, QColor
from PyQt6.QtCore import Qt, pyqtSignal, QObject, QDateTime, QThread

from sc_gui.waypoint_manager import InteractiveMapPanel

try:
    import paramiko
except ImportError:
    print("paramiko 라이브러리가 필요합니다. 'pip install paramiko'를 실행하세요.")
    sys.exit(1)


# =====================================================================
# [0] 원격 런치 매니저: SSH로 터틀봇에 launch 명령 전송
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
            self.log_signal.emit(f"> [SSH] CONNECTING TO {self.username}@{self.ip}...")
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.ip, username=self.username, password=self.password, timeout=5)
            self.log_signal.emit("> [SSH] ACCESS GRANTED. EXECUTING LAUNCH...")

            stdin, stdout, stderr = ssh.exec_command(self.command)
            err = stderr.read().decode().strip()
            if err and "command not found" in err.lower():
                self.log_signal.emit(f"> [SSH ERROR] {err}")
            else:
                self.log_signal.emit("> [SSH] REMOTE LAUNCH TRANSMITTED.")
            ssh.close()
        except Exception as e:
            self.log_signal.emit(f"> [SSH FATAL ERROR] {e}")


# =====================================================================
# [1] ROS Manager: rosbridge_websocket 통신 + Nav2 send_goal
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
        self.log_signal.emit(f"> CONNECTING TO {self.ip_address}:9090...")
        try:
            self.ros = roslibpy.Ros(host=self.ip_address, port=9090)
            self.ros.run()

            if self.ros.is_connected:
                self.log_signal.emit("> CONNECTED. TELEMETRY ACTIVE.")
                self.status_signal.emit("UPDATE")

                # ── /odom (터틀봇 위치) ──
                self.odom_listener = roslibpy.Topic(
                    self.ros, '/odom', 'nav_msgs/Odometry')
                self.odom_listener.subscribe(self.odom_callback)

                # ── /cmd_vel (이동 중 여부 감지) ──
                self.cmd_vel_listener = roslibpy.Topic(
                    self.ros, '/cmd_vel', 'geometry_msgs/Twist')
                self.cmd_vel_listener.subscribe(self.cmd_vel_callback)

                # ── 웹캠 1번 (사람 추종용) ──
                self.cam1_listener = roslibpy.Topic(
                    self.ros, '/webcam/image_raw/compressed',
                    'sensor_msgs/CompressedImage')
                self.cam1_listener.subscribe(self.camera1_callback)

                # ── 웹캠 2번 (물체 인식용) — USB 웹캠으로 변경 ──
                self.cam2_listener = roslibpy.Topic(
                    self.ros, '/webcam2/image_raw/compressed',
                    'sensor_msgs/CompressedImage')
                self.cam2_listener.subscribe(self.camera2_callback)

                # ── /item_detected (물체 인식 → Flask 결제 연동) ──
                self.item_listener = roslibpy.Topic(
                    self.ros, '/item_detected', 'sc_interfaces/ItemDetected')
                self.item_listener.subscribe(self.item_callback)
            else:
                self.log_signal.emit("> [WARNING] WS REFUSED. (Launch가 아직 실행 안 됐을 수 있음)")
        except Exception as e:
            self.log_signal.emit(f"> [ERROR] NETWORK FAILURE: {e}")

    # ── 사람 추종 / 정지 모드 전환 ──
    def send_mode(self, mode):
        """
        mode: 'follow' / 'navigate' / 'idle'
        follow_controller가 이걸 구독해서 모드 전환
        """
        if self.ros and self.ros.is_connected:
            topic = roslibpy.Topic(
                self.ros, '/smart_cart/mode', 'std_msgs/String')
            topic.publish(roslibpy.Message({'data': mode}))
            topic.unadvertise()
            self.log_signal.emit(f"> MODE CHANGED → [{mode.upper()}]")

    # ── Nav2 자동 주행: 목표 좌표로 이동 ──
    def send_nav_goal(self, x, y):
        """
        Nav2 navigate_to_pose 액션에 send_goal
        x, y: 목적지 실제 좌표 (m)
        """
        if not self.ros or not self.ros.is_connected:
            self.log_signal.emit("> [ERROR] ROS 연결 안 됨")
            return False

        # 먼저 navigate 모드로 전환 (follow_controller 정지)
        self.send_mode('navigate')

        # Nav2 액션 클라이언트
        action_client = roslibpy.actionlib.ActionClient(
            self.ros, '/navigate_to_pose', 'nav2_msgs/NavigateToPose')

        goal_msg = {
            'pose': {
                'header': {'frame_id': 'map'},
                'pose': {
                    'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                }
            }
        }

        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message(goal_msg))
        goal.on('result', lambda r: self.log_signal.emit("> [NAV2] 목적지 도착!"))
        goal.on('feedback', lambda f: None)
        goal.send()
        self.log_signal.emit(f"> [NAV2] 목적지 전송: x={x:.2f}, y={y:.2f}")
        return True

    def send_estop(self):
        """비상 정지 (사람 추종 + Nav2 둘 다 멈춤)"""
        if self.ros and self.ros.is_connected:
            # 1. mode를 idle로
            self.send_mode('idle')
            # 2. /safety_stop true 발행
            topic = roslibpy.Topic(self.ros, '/safety_stop', 'std_msgs/Bool')
            topic.publish(roslibpy.Message({'data': True}))
            topic.unadvertise()

    def disconnect(self):
        if self.ros and self.ros.is_connected:
            for attr in ['odom_listener', 'cmd_vel_listener',
                         'cam1_listener', 'cam2_listener', 'item_listener']:
                if hasattr(self, attr):
                    getattr(self, attr).unsubscribe()
            self.ros.terminate()

    # ── 콜백 ──
    def odom_callback(self, message):
        try:
            x = message['pose']['pose']['position']['x']
            y = message['pose']['pose']['position']['y']
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
        """물체 인식 결과 → Flask 결제 서버에 전달"""
        try:
            item_name = message.get('item_name', '')
            in_basket = message.get('in_basket_zone', True)
            if in_basket and item_name:
                res = requests.post(
                    'http://127.0.0.1:5000/api/add_item',
                    json={'class_name': item_name},
                    timeout=1.0)
                if res.status_code == 200:
                    self.log_signal.emit(f"> [WEB 연동] '{item_name}' 장바구니 갱신")
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
        super().__init__(" 시스템 로그")
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
        return pixmap.scaled(self.camera_label.size(),
                             Qt.AspectRatioMode.KeepAspectRatio,
                             Qt.TransformationMode.SmoothTransformation)


class MainCameraPanel(CameraPanel):
    """웹캠 1번 — 사람 추종용 (FPV + 조준점)"""
    def __init__(self):
        super().__init__(" 사람 추종 카메라", "NO SIGNAL /webcam/image_raw")

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
    """웹캠 2번 — 물체 인식용 (USB 웹캠)"""
    def __init__(self):
        super().__init__(" 물체 인식 카메라", "NO SIGNAL /webcam2/image_raw")

    def update_view(self, img_bytes):
        self.camera_label.setPixmap(self.render_base_image(img_bytes))


# =====================================================================
# [3] 사이드바 (시스템 + 미션 버튼)
# =====================================================================
class SidebarPanel(QFrame):
    launch_requested = pyqtSignal()
    follow_requested = pyqtSignal()         # 사람 추종 시작
    stop_follow_requested = pyqtSignal()    # 사람 추종 정지
    waypoint_requested = pyqtSignal(str)    # 웨이포인트 자동 주행 (이름)
    estop_requested = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setProperty("class", "Panel")
        self.setFixedWidth(280)

        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(20, 20, 20, 20)
        outer_layout.setSpacing(15)

        # 타이틀
        title = QLabel("FRICTIONLESS\nCOMMAND")
        title.setStyleSheet(
            "color: #FFFFFF; font-size: 22px; font-weight: 900; letter-spacing: 1px;")
        outer_layout.addWidget(title)

        # 상태 표시
        self.status_label = QLabel("STANDBY")
        self.status_label.setProperty("class", "Status")
        self.status_label.setWordWrap(True)
        outer_layout.addWidget(self.status_label)

        outer_layout.addSpacing(10)

        # ── SYSTEM CONTROL ──
        lbl_sys = QLabel("SYSTEM CONTROL")
        lbl_sys.setStyleSheet("color: #666; font-size: 11px; font-weight: bold;")
        outer_layout.addWidget(lbl_sys)

        btn_launch = QPushButton("SYSTEM LAUNCH")
        btn_launch.setProperty("class", "LaunchBtn")
        btn_launch.clicked.connect(self.launch_requested.emit)
        outer_layout.addWidget(btn_launch)

        outer_layout.addSpacing(5)

        # ── 사람 추종 ──
        lbl_follow = QLabel("FOLLOW MODE")
        lbl_follow.setStyleSheet("color: #666; font-size: 11px; font-weight: bold;")
        outer_layout.addWidget(lbl_follow)

        btn_follow_start = QPushButton("사람 추종 시작")
        btn_follow_start.setProperty("class", "MissionBtn")
        btn_follow_start.clicked.connect(self.follow_requested.emit)
        outer_layout.addWidget(btn_follow_start)

        btn_follow_stop = QPushButton("사람 추종 정지")
        btn_follow_stop.setProperty("class", "MissionBtn")
        btn_follow_stop.clicked.connect(self.stop_follow_requested.emit)
        outer_layout.addWidget(btn_follow_stop)

        outer_layout.addSpacing(5)

        # ── 자동 주행 (웨이포인트 동적 버튼) ──
        lbl_nav = QLabel("AUTO NAVIGATION")
        lbl_nav.setStyleSheet("color: #666; font-size: 11px; font-weight: bold;")
        outer_layout.addWidget(lbl_nav)

        # 스크롤 영역 (웨이포인트가 많아지면 스크롤)
        self.wp_scroll = QScrollArea()
        self.wp_scroll.setWidgetResizable(True)
        self.wp_scroll.setStyleSheet("background-color: transparent; border: none;")
        self.wp_container = QWidget()
        self.wp_layout = QVBoxLayout(self.wp_container)
        self.wp_layout.setContentsMargins(0, 0, 0, 0)
        self.wp_layout.setSpacing(8)
        self.wp_scroll.setWidget(self.wp_container)
        outer_layout.addWidget(self.wp_scroll, stretch=1)

        # 안내 라벨 (웨이포인트 없을 때)
        self.empty_label = QLabel("맵에서 좌클릭으로\n웨이포인트를 추가하세요")
        self.empty_label.setStyleSheet(
            "color: #555; font-size: 11px; padding: 10px; text-align: center;")
        self.empty_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.wp_layout.addWidget(self.empty_label)

        # ── EMERGENCY STOP ──
        btn_estop = QPushButton("EMERGENCY STOP")
        btn_estop.setObjectName("EStopBtn")
        btn_estop.clicked.connect(self.estop_requested.emit)
        outer_layout.addWidget(btn_estop)

    def update_status(self, text, color):
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color};")

    def refresh_waypoints(self, waypoint_names):
        """웨이포인트 목록 변경 시 동적으로 버튼 재생성"""
        # 기존 버튼들 제거
        while self.wp_layout.count():
            item = self.wp_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        if not waypoint_names:
            self.empty_label = QLabel("맵에서 좌클릭으로\n웨이포인트를 추가하세요")
            self.empty_label.setStyleSheet(
                "color: #555; font-size: 11px; padding: 10px; text-align: center;")
            self.empty_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.wp_layout.addWidget(self.empty_label)
            return

        # 웨이포인트별 버튼 동적 생성
        for name in waypoint_names:
            btn = QPushButton(name)
            btn.setProperty("class", "MissionBtn")
            btn.clicked.connect(lambda checked, n=name: self.waypoint_requested.emit(n))
            self.wp_layout.addWidget(btn)
        self.wp_layout.addStretch()


# =====================================================================
# [4] 메인 윈도우
# =====================================================================
class FrictionlessStoreGUI(QMainWindow):
    def __init__(self, yaml_path, robot_ip, robot_user, robot_pw, workspace_path):
        super().__init__()
        self.setWindowTitle("Smart Cart - Frictionless Store Command Center")
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
        self.map_panel = InteractiveMapPanel(yaml_path)
        self.log_panel = LogPanel()
        self.cam1_panel = MainCameraPanel()
        self.cam2_panel = BasketCameraPanel()

        self.build_layout()
        self.connect_signals()

        # 초기 웨이포인트 버튼 표시
        self.sidebar_panel.refresh_waypoints(self.map_panel.wp_db.get_all_names())

        self.log_panel.append_log("SYSTEM BOOT.")
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
        # 맵 패널 → 로그
        self.map_panel.log_event.connect(self.log_panel.append_log)
        # 웨이포인트 추가/삭제 → 사이드바 버튼 갱신
        self.map_panel.waypoints_changed.connect(
            lambda: self.sidebar_panel.refresh_waypoints(self.map_panel.wp_db.get_all_names()))

        # ROS 신호 → UI
        self.ros_manager.log_signal.connect(self.log_panel.append_log)
        self.ros_manager.pose_signal.connect(self.map_panel.update_pose)
        self.ros_manager.status_signal.connect(self.update_status_ui)
        self.ros_manager.camera1_signal.connect(self.cam1_panel.update_view)
        self.ros_manager.camera2_signal.connect(self.cam2_panel.update_view)

        # 사이드바 버튼
        self.sidebar_panel.launch_requested.connect(self.handle_remote_launch)
        self.sidebar_panel.follow_requested.connect(self.handle_follow_start)
        self.sidebar_panel.stop_follow_requested.connect(self.handle_follow_stop)
        self.sidebar_panel.waypoint_requested.connect(self.handle_waypoint_nav)
        self.sidebar_panel.estop_requested.connect(self.handle_estop)

    def handle_remote_launch(self):
        """터틀봇 라즈베리파이에 SSH로 ros2 launch 실행"""
        launch_cmd = (
            f"bash -c 'source /opt/ros/humble/setup.bash && "
            f"source {self.workspace_path}/install/setup.bash && "
            f"nohup ros2 launch sc_bringup robot.launch.py > ~/gui_launch.log 2>&1 &'")

        self.ssh_worker = SshWorker(
            self.robot_ip, self.robot_user, self.robot_pw, launch_cmd)
        self.ssh_worker.log_signal.connect(self.log_panel.append_log)
        self.ssh_worker.start()
        self.log_panel.append_log("[INFO] ROS2 노드 부팅에 5~10초 소요됩니다.")

    def handle_follow_start(self):
        """사람 추종 모드 시작"""
        if not self._check_ros_connection():
            return
        self.current_mission = "FOLLOW"
        self.ros_manager.send_mode('follow')
        self.log_panel.append_log("> [모드] 사람 추종 시작")
        self.update_status_ui()

    def handle_follow_stop(self):
        """사람 추종 정지"""
        if not self._check_ros_connection():
            return
        self.current_mission = "IDLE"
        self.ros_manager.send_mode('idle')
        self.log_panel.append_log("> [모드] 사람 추종 정지")
        self.update_status_ui()

    def handle_waypoint_nav(self, waypoint_name):
        """웨이포인트로 자동 주행"""
        if not self._check_ros_connection():
            return

        wp = self.map_panel.wp_db.get_waypoint(waypoint_name)
        if not wp:
            self.log_panel.append_log(f"> [ERROR] 웨이포인트 없음: {waypoint_name}")
            return

        self.current_mission = f"NAV_{waypoint_name}"
        self.log_panel.append_log(f"> [자동 주행] '{waypoint_name}'으로 이동")
        success = self.ros_manager.send_nav_goal(wp['x'], wp['y'])
        if not success:
            self.log_panel.append_log("> [ERROR] Nav2 send_goal 실패")
        self.update_status_ui()

    def handle_estop(self):
        if not self._check_ros_connection():
            return
        self.current_mission = "ESTOP"
        self.log_panel.append_log("[CRITICAL] EMERGENCY STOP")
        self.ros_manager.send_estop()
        self.update_status_ui()

    def _check_ros_connection(self):
        """ROS 연결 확인 + 안 됐으면 재연결 시도"""
        if not self.ros_manager.ros or not self.ros_manager.ros.is_connected:
            msg_box = QMessageBox(self)
            msg_box.setWindowTitle("경고")
            msg_box.setText("먼저 SYSTEM LAUNCH를 눌러 로봇을 가동하거나 통신을 점검하세요.")
            msg_box.setIcon(QMessageBox.Icon.Warning)
            msg_box.addButton("확인", QMessageBox.ButtonRole.AcceptRole)
            msg_box.exec()
            self.ros_manager.connect_to_robot()
            return False
        return True

    def update_status_ui(self):
        move_str = "[IN MOTION]" if self.ros_manager.is_moving else "[STANDBY]"
        color = "#00E5FF"

        if self.current_mission == "ESTOP":
            text, color = "E-STOP ENGAGED\nMOTORS LOCKED", "#FF3366"
        elif self.current_mission == "FOLLOW":
            text = f"사람 추종 중\n{move_str}"
        elif self.current_mission.startswith("NAV_"):
            wp_name = self.current_mission[4:]
            text = f"자동 주행: {wp_name}\n{move_str}"
            color = "#FFCC00"
        else:
            text = "SYSTEM STANDBY\nREADY FOR COMMAND"

        self.sidebar_panel.update_status(text, color)

    def closeEvent(self, event):
        self.log_panel.append_log("SHUTDOWN.")
        self.ros_manager.disconnect()
        event.accept()

    def setup_stylesheet(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #0A0A0C; }
            QFrame.Panel { background-color: #121214; border-radius: 12px; }
            QLabel { color: #D0D0D0; font-family: 'Malgun Gothic', sans-serif; }
            QLabel.Title {
                color: #888888; font-weight: bold; font-size: 11px;
                padding: 12px 15px; letter-spacing: 1px;
                background-color: transparent;
            }
            QLabel.Status { font-weight: bold; font-size: 15px; letter-spacing: 1px; }
            QTextEdit.Terminal {
                background-color: transparent; border: none; color: #00E5FF;
                font-family: 'Consolas', monospace; font-size: 13px;
                padding: 0px 15px 15px 15px;
            }
            QPushButton {
                font-family: 'Malgun Gothic'; border-radius: 6px;
                padding: 12px; font-size: 13px; font-weight: bold; letter-spacing: 1px;
            }
            QPushButton.MissionBtn {
                background-color: #1A1A1E; color: #D0D0D0;
                border: 1px solid #2A2A2E; text-align: left; padding-left: 20px;
            }
            QPushButton.MissionBtn:hover { background-color: #2A2A2E; color: #FFFFFF; }
            QPushButton.MissionBtn:pressed { background-color: #00E5FF; color: #000000; }
            QPushButton.LaunchBtn {
                background-color: #004455; color: #00E5FF; border: none;
            }
            QPushButton.LaunchBtn:hover { background-color: #006677; color: #FFFFFF; }
            QPushButton.LaunchBtn:pressed { background-color: #008899; }
            QPushButton#EStopBtn {
                background-color: #330A0A; color: #FF3366; border: 1px solid #FF3366;
            }
            QPushButton#EStopBtn:hover { background-color: #FF3366; color: #FFFFFF; }
            QMessageBox { background-color: #121214; }
            QMessageBox QLabel { color: #E0E0E0; font-size: 14px; background-color: transparent; }
            QMessageBox QPushButton {
                background-color: #2A2A2E; color: #FFFFFF; border: 1px solid #444;
                min-width: 80px; padding: 8px 16px;
            }
            QMessageBox QPushButton:hover { background-color: #3A3A3E; }
            QMessageBox QPushButton:pressed { background-color: #00E5FF; color: #000000; }
            QScrollArea { background-color: transparent; border: none; }
        """)