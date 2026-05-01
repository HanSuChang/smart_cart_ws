# =====================================================================
# robot_gui.py — Smart Cart 메인 관제 GUI (PyQt5, 단순 버전)
#
# UI 는 꾸미지 않고 기능만 명확히. 기본 PyQt5 위젯 + 기본 스타일 사용.
#
# [기능]
#   - 사람 추종 카메라 / 바구니 카메라 두 화면
#   - 맵 + 화장실/충전소 노드 1점씩 + 저장
#   - 사람 추종 시작 (15초 학습 + 5초 대기 → follow)
#   - 화장실 이동 (Yes/No 알림창) → /smart_cart/destination "toilet"
#     → lid_controller가 뚜껑 닫기
#   - 충전소 이동 → /smart_cart/destination "charger"
#   - 결제 → Flask GUI 브라우저 오픈 + /payment/event 발행
#   - 비상 정지
#
# [Pub/Sub]
#   Subscribe (roslibpy):
#     /yolo/follow_image/compressed   → 사람 추종 카메라
#     /yolo/item_image/compressed     → 물체 인식 카메라(YOLO)
#     /basket/annotated/compressed    → 바구니 OpenCV 카메라
#     /webcam/image_raw/compressed    → 백업 raw 1
#     /webcam2/image_raw/compressed   → 백업 raw 2
#     /odom, /cmd_vel, /smart_cart/learn_status, /follow_status,
#     /tracker/state, /cart_status, /item_detected, /item_confirm,
#     /payment/event, /basket/event, /lid_state
#
#   Publish:
#     /smart_cart/mode, /smart_cart/learn, /smart_cart/destination,
#     /safety_stop, /payment/event, /waypoint_list, /navigate_to_pose
# =====================================================================

import sys
import os
import base64
import webbrowser

import roslibpy
import requests

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout,
    QWidget, QMessageBox, QTextEdit, QPushButton, QGroupBox, QSplitter
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QDateTime, QTimer

from sc_gui.waypoint_manager import (
    InteractiveMapPanel, CATEGORY_LABELS)


# =====================================================================
# RosManager — roslibpy 로 모든 토픽 pub/sub
# =====================================================================
class RosManager(QObject):
    pose_signal           = pyqtSignal(float, float)
    cmd_vel_signal        = pyqtSignal(float, float)
    log_signal            = pyqtSignal(str)
    follow_cam_signal     = pyqtSignal(bytes)
    item_cam_signal       = pyqtSignal(bytes)
    basket_cam_signal     = pyqtSignal(bytes)
    learn_status_signal   = pyqtSignal(str)
    follow_status_signal  = pyqtSignal(str)
    cart_status_signal    = pyqtSignal(dict)
    item_confirm_signal   = pyqtSignal(str)
    basket_event_signal   = pyqtSignal(dict)
    payment_event_signal  = pyqtSignal(dict)
    lid_state_signal      = pyqtSignal(str)
    connected_signal      = pyqtSignal(bool)

    def __init__(self, ip='127.0.0.1', port=9090):
        super().__init__()
        self.ip = ip
        self.port = port
        self.ros = None
        self.is_moving = False
        self._listeners = []

    def connect(self):
        self.log_signal.emit(f"connecting {self.ip}:{self.port} ...")
        try:
            self.ros = roslibpy.Ros(host=self.ip, port=self.port)
            self.ros.run()
            if not self.ros.is_connected:
                self.log_signal.emit("[WARN] rosbridge 연결 실패")
                self.connected_signal.emit(False)
                return
            self.log_signal.emit("connected.")
            self.connected_signal.emit(True)
            self._setup_subs()
        except Exception as e:
            self.log_signal.emit(f"[ERR] {e}")
            self.connected_signal.emit(False)

    def _add(self, topic, mtype, cb):
        t = roslibpy.Topic(self.ros, topic, mtype)
        t.subscribe(cb)
        self._listeners.append(t)

    def _setup_subs(self):
        # 카메라
        self._add('/yolo/follow_image/compressed', 'sensor_msgs/CompressedImage',
                  lambda m: self._safe_emit(self.follow_cam_signal, m))
        self._add('/webcam/image_raw/compressed', 'sensor_msgs/CompressedImage',
                  lambda m: self._safe_emit(self.follow_cam_signal, m))
        self._add('/yolo/item_image/compressed', 'sensor_msgs/CompressedImage',
                  lambda m: self._safe_emit(self.item_cam_signal, m))
        self._add('/basket/annotated/compressed', 'sensor_msgs/CompressedImage',
                  lambda m: self._safe_emit(self.basket_cam_signal, m))
        self._add('/webcam2/image_raw/compressed', 'sensor_msgs/CompressedImage',
                  lambda m: self._safe_emit(self.item_cam_signal, m))

        # 위치/속도
        # /amcl_pose 가 있으면 그 값(map frame, AMCL 보정) 우선 사용
        # /odom 은 백업/실시간성 (odom frame)
        self._add('/amcl_pose',
                  'geometry_msgs/PoseWithCovarianceStamped',
                  self._cb_amcl)
        self._add('/odom', 'nav_msgs/Odometry', self._cb_odom)
        self._add('/cmd_vel', 'geometry_msgs/Twist', self._cb_cmd_vel)
        # RViz2 의 "2D Pose Estimate" 도구로 publish 되는 토픽
        self._add('/initialpose',
                  'geometry_msgs/PoseWithCovarianceStamped',
                  self._cb_initialpose)

        # 상태
        self._add('/smart_cart/learn_status', 'std_msgs/String',
                  lambda m: self.learn_status_signal.emit(m.get('data', '')))
        self._add('/follow_status', 'std_msgs/String',
                  lambda m: self.follow_status_signal.emit(m.get('data', '')))
        self._add('/cart_status', 'sc_interfaces/CartStatus',
                  lambda m: self.cart_status_signal.emit(m))
        self._add('/item_confirm', 'std_msgs/String',
                  lambda m: self.item_confirm_signal.emit(m.get('data', '')))
        self._add('/item_detected', 'sc_interfaces/ItemDetected',
                  self._cb_item_detected)
        self._add('/basket/event', 'sc_interfaces/BasketEvent',
                  lambda m: self.basket_event_signal.emit(m))
        self._add('/payment/event', 'sc_interfaces/PaymentEvent',
                  lambda m: self.payment_event_signal.emit(m))
        self._add('/lid_state', 'std_msgs/String',
                  lambda m: self.lid_state_signal.emit(m.get('data', '')))

    def _safe_emit(self, sig, m):
        try:
            sig.emit(base64.b64decode(m['data']))
        except Exception:
            pass

    def _cb_odom(self, m):
        # /amcl_pose 가 들어오기 전(또는 AMCL 미가동) 백업
        if getattr(self, '_amcl_seen', False):
            return
        try:
            x = m['pose']['pose']['position']['x']
            y = m['pose']['pose']['position']['y']
            self.pose_signal.emit(x, y)
        except Exception:
            pass

    def _cb_amcl(self, m):
        # AMCL 이 발행하는 map frame 위치 (RViz2 2D Pose Estimate 후 활성화됨)
        # 이 값이 더 정확하므로 /odom 보다 우선 사용
        self._amcl_seen = True
        try:
            x = m['pose']['pose']['position']['x']
            y = m['pose']['pose']['position']['y']
            self.pose_signal.emit(x, y)
        except Exception:
            pass

    def _cb_initialpose(self, m):
        # RViz2 의 "2D Pose Estimate" 클릭 시 발행되는 토픽 — GUI 로그에 표시
        try:
            x = m['pose']['pose']['position']['x']
            y = m['pose']['pose']['position']['y']
            self.log_signal.emit(
                f"[RViz2] 2D Pose Estimate: ({x:.2f}, {y:.2f}) — AMCL 초기화")
        except Exception:
            pass

    def _cb_cmd_vel(self, m):
        try:
            lx = float(m['linear']['x'])
            az = float(m['angular']['z'])
            self.cmd_vel_signal.emit(lx, az)
            self.is_moving = abs(lx) > 0.01 or abs(az) > 0.01
        except Exception:
            pass

    def _cb_item_detected(self, m):
        # Flask 장바구니 자동 추가
        try:
            if m.get('in_basket_zone') and m.get('item_name'):
                requests.post('http://127.0.0.1:5000/api/add_item',
                              json={'class_name': m['item_name']},
                              timeout=1.0)
        except Exception:
            pass

    # ── 발행 ──
    def _pub(self, topic, mtype, data):
        if not self.ros or not self.ros.is_connected:
            self.log_signal.emit("[ERR] ROS 미연결")
            return False
        t = roslibpy.Topic(self.ros, topic, mtype)
        t.publish(roslibpy.Message(data))
        t.unadvertise()
        return True

    def send_mode(self, mode):
        if self._pub('/smart_cart/mode', 'std_msgs/String', {'data': mode}):
            self.log_signal.emit(f"mode={mode}")

    def send_learn(self, cmd):
        if self._pub('/smart_cart/learn', 'std_msgs/String', {'data': cmd}):
            self.log_signal.emit(f"learn={cmd}")

    def send_destination(self, dest):
        if self._pub('/smart_cart/destination', 'std_msgs/String', {'data': dest}):
            self.log_signal.emit(f"destination={dest}")

    def send_estop(self):
        self.send_mode('idle')
        self._pub('/safety_stop', 'std_msgs/Bool', {'data': True})

    def send_payment(self, event, total=0, count=0, msg=''):
        self._pub('/payment/event', 'sc_interfaces/PaymentEvent', {
            'header': {'frame_id': 'gui'},
            'event': event,
            'total_price': int(total),
            'item_count': int(count),
            'message': msg,
        })

    def send_waypoints(self, wps):
        wp_msgs = []
        for cat, c in wps.items():
            wp_msgs.append({
                'name': CATEGORY_LABELS.get(cat, cat),
                'x': float(c.get('x', 0.0)),
                'y': float(c.get('y', 0.0)),
                'category': cat,
            })
        self._pub('/waypoint_list', 'sc_interfaces/WaypointList', {
            'header': {'frame_id': 'map'},
            'waypoints': wp_msgs,
        })

    def send_nav_goal(self, x, y):
        if not self.ros or not self.ros.is_connected:
            self.log_signal.emit("[ERR] ROS 미연결")
            return False
        self.send_mode('navigate')
        ac = roslibpy.actionlib.ActionClient(
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
        g = roslibpy.actionlib.Goal(ac, roslibpy.Message(goal_msg))
        g.on('result', lambda r: self.log_signal.emit("[NAV2] arrived"))
        g.send()
        self.log_signal.emit(f"[NAV2] goal=({x:.2f},{y:.2f})")
        return True

    def disconnect(self):
        for t in self._listeners:
            try:
                t.unsubscribe()
            except Exception:
                pass
        self._listeners.clear()
        if self.ros and self.ros.is_connected:
            self.ros.terminate()


# =====================================================================
# 카메라 위젯
# =====================================================================
class CamView(QLabel):
    def __init__(self, default_text):
        super().__init__(default_text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setFrameStyle(0x10 | 0x01)  # Box | Plain

    def update_view(self, img_bytes):
        pix = QPixmap()
        pix.loadFromData(img_bytes)
        self.setPixmap(pix.scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


# =====================================================================
# Main Window
# =====================================================================
class FrictionlessStoreGUI(QMainWindow):
    def __init__(self, yaml_path, robot_ip='127.0.0.1',
                 robot_user='', robot_pw='',
                 workspace_path='~/smart_cart_ws/smart_cart_ws',
                 flask_url='http://127.0.0.1:5000'):
        super().__init__()
        self.setWindowTitle("Smart Cart")
        self.resize(1280, 800)

        self.flask_url = flask_url
        self.last_cart_status = {}

        self.ros = RosManager(robot_ip)

        # ── 위젯 ──
        self.map_panel = InteractiveMapPanel(yaml_path)
        self.cam_follow = CamView("사람 추종 카메라\n/yolo/follow_image/compressed")
        self.cam_basket = CamView("바구니/물체 카메라\n/basket/annotated/compressed\n/yolo/item_image/compressed")

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        self.lbl_status = QLabel("STANDBY")
        self.lbl_learn = QLabel("학습 상태: idle")
        self.lbl_follow = QLabel("추종 상태: idle")
        self.lbl_lid = QLabel("뚜껑: closed")
        self.lbl_pose = QLabel("위치: -")
        self.lbl_vel = QLabel("속도: 0.00 / 0.00")

        # ── 버튼 ──
        btn_launch     = QPushButton("SYSTEM LAUNCH (rosbridge 재연결)")
        btn_follow     = QPushButton("사람 추종 시작 (15s 학습 + 5s 대기)")
        btn_follow_off = QPushButton("사람 추종 정지")
        btn_save       = QPushButton("학습 저장")
        btn_reset      = QPushButton("학습 초기화")

        btn_toilet     = QPushButton("화장실 이동")
        btn_charger    = QPushButton("충전소 이동")
        btn_payment    = QPushButton("결제")

        btn_estop      = QPushButton("EMERGENCY STOP")
        btn_estop.setStyleSheet("background-color: #c33; color: white; font-weight: bold;")

        # 클릭 핸들러
        btn_launch.clicked.connect(self._on_launch)
        btn_follow.clicked.connect(self._on_follow_start)
        btn_follow_off.clicked.connect(self._on_follow_stop)
        btn_save.clicked.connect(lambda: self._on_check() and self.ros.send_learn('save'))
        btn_reset.clicked.connect(self._on_reset)
        btn_toilet.clicked.connect(self._on_toilet)
        btn_charger.clicked.connect(self._on_charger)
        btn_payment.clicked.connect(self._on_payment)
        btn_estop.clicked.connect(self._on_estop)

        # ── 좌측 컨트롤 패널 ──
        ctrl_box = QGroupBox("CONTROL")
        cv = QVBoxLayout(ctrl_box)
        for w in (self.lbl_status, self.lbl_learn, self.lbl_follow,
                  self.lbl_lid, self.lbl_pose, self.lbl_vel):
            cv.addWidget(w)
        cv.addWidget(btn_launch)
        cv.addWidget(btn_follow)
        cv.addWidget(btn_follow_off)
        h = QHBoxLayout()
        h.addWidget(btn_save)
        h.addWidget(btn_reset)
        cv.addLayout(h)
        cv.addWidget(btn_toilet)
        cv.addWidget(btn_charger)
        cv.addWidget(btn_payment)
        cv.addStretch()
        cv.addWidget(btn_estop)

        # ── 우측 비전 패널 ──
        vis_box = QGroupBox("VISION")
        gv = QGridLayout(vis_box)
        gv.addWidget(QLabel("MAP"), 0, 0)
        gv.addWidget(self.map_panel, 1, 0, 2, 1)
        gv.addWidget(QLabel("사람 추종 카메라"), 0, 1)
        gv.addWidget(self.cam_follow, 1, 1)
        gv.addWidget(QLabel("바구니/물체 카메라"), 2, 1, alignment=Qt.AlignTop)
        gv.addWidget(self.cam_basket, 3, 1)
        gv.setColumnStretch(0, 2)
        gv.setColumnStretch(1, 1)

        # ── 하단 로그 ──
        log_box = QGroupBox("LOG")
        lv = QVBoxLayout(log_box)
        lv.addWidget(self.log)

        # ── 전체 레이아웃 (Splitter 로 사이즈 자유 조절) ──
        top = QSplitter(Qt.Horizontal)
        top.addWidget(ctrl_box)
        top.addWidget(vis_box)
        top.setStretchFactor(0, 0)
        top.setStretchFactor(1, 1)

        outer = QSplitter(Qt.Vertical)
        outer.addWidget(top)
        outer.addWidget(log_box)
        outer.setStretchFactor(0, 4)
        outer.setStretchFactor(1, 1)

        central = QWidget()
        cen_layout = QVBoxLayout(central)
        cen_layout.setContentsMargins(8, 8, 8, 8)
        cen_layout.addWidget(outer)
        self.setCentralWidget(central)

        # ── 시그널 연결 ──
        self._wire_signals()
        self._wp_timer = QTimer()
        self._wp_timer.setSingleShot(True)
        self._wp_timer.timeout.connect(self._publish_waypoints)

        self._append_log("BOOT")
        self.ros.connect()

    # ─────────────────────────────────────────────────────────
    def _wire_signals(self):
        self.map_panel.log_event.connect(self._append_log)
        self.map_panel.waypoints_changed.connect(self._publish_waypoints)

        self.ros.log_signal.connect(self._append_log)
        self.ros.pose_signal.connect(self.map_panel.update_pose)
        self.ros.pose_signal.connect(
            lambda x, y: self.lbl_pose.setText(f"위치: ({x:.2f}, {y:.2f})"))
        self.ros.cmd_vel_signal.connect(
            lambda lx, az: self.lbl_vel.setText(f"속도: {lx:.2f} / {az:.2f}"))
        self.ros.follow_cam_signal.connect(self.cam_follow.update_view)
        self.ros.item_cam_signal.connect(self.cam_basket.update_view)
        self.ros.basket_cam_signal.connect(self.cam_basket.update_view)
        self.ros.learn_status_signal.connect(
            lambda s: self.lbl_learn.setText(f"학습 상태: {s}"))
        self.ros.follow_status_signal.connect(
            lambda s: self.lbl_follow.setText(f"추종 상태: {s}"))
        self.ros.lid_state_signal.connect(
            lambda s: self.lbl_lid.setText(f"뚜껑: {s}"))
        self.ros.cart_status_signal.connect(
            lambda m: setattr(self, 'last_cart_status', m))
        self.ros.item_confirm_signal.connect(
            lambda n: self._append_log(f"[LID] OPEN — {n}"))
        self.ros.basket_event_signal.connect(self._on_basket_event)
        self.ros.payment_event_signal.connect(self._on_payment_event)
        self.ros.connected_signal.connect(
            lambda ok: self._wp_timer.start(1000) if ok else None)

    # ─────────────────────────────────────────────────────────
    def _append_log(self, text):
        ts = QDateTime.currentDateTime().toString("HH:mm:ss")
        self.log.append(f"[{ts}] {text}")
        sb = self.log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _publish_waypoints(self):
        wps = self.map_panel.wp_db.data
        self.ros.send_waypoints(wps)
        self._append_log(f"[WAYPOINT] {len(wps)}개 발행")

    def _on_check(self):
        if not self.ros.ros or not self.ros.ros.is_connected:
            QMessageBox.warning(self, '경고', 'ROS 미연결 — SYSTEM LAUNCH 후 재시도')
            self.ros.connect()
            return False
        return True

    # ── 버튼 핸들러 ──
    def _on_launch(self):
        self.ros.connect()

    def _on_follow_start(self):
        if not self._on_check():
            return
        self.lbl_status.setText("FOLLOW (학습+대기)")
        self._append_log("[FOLLOW] 15s 학습 + 5s 대기 시작")
        self.ros.send_learn('start_with_delay')

    def _on_follow_stop(self):
        if not self._on_check():
            return
        self.ros.send_mode('idle')
        self.lbl_status.setText("STANDBY")
        self._append_log("[FOLLOW] 정지")

    def _on_reset(self):
        if not self._on_check():
            return
        if QMessageBox.question(
                self, '초기화', '학습 데이터를 초기화할까요?',
                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.ros.send_learn('reset')

    def _on_toilet(self):
        if not self._on_check():
            return
        wp = self.map_panel.get_waypoint('toilet')
        if not wp:
            QMessageBox.warning(self, '경고',
                                '화장실 노드가 설정되지 않았습니다.')
            return
        if QMessageBox.question(
                self, '화장실 이동', '화장실로 이동하시겠습니까?',
                QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return
        # ★ destination 발행 → lid_controller가 뚜껑 닫기
        self.ros.send_destination('toilet')
        self.ros.send_nav_goal(wp['x'], wp['y'])
        self.lbl_status.setText("NAV: 화장실")

    def _on_charger(self):
        if not self._on_check():
            return
        wp = self.map_panel.get_waypoint('charger')
        if not wp:
            QMessageBox.warning(self, '경고',
                                '충전소 노드가 설정되지 않았습니다.')
            return
        self.ros.send_destination('charger')
       