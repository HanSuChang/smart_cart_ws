# =====================================================================
# waypoint_manager.py
# 맵 표시 + 노드(웨이포인트) 찍기/저장/삭제
#
# - 좌클릭: 새 노드 추가 (이름 자유 입력)
# - 우클릭: 가까운 노드 삭제
# - 저장: waypoints.json 자동 저장
# - 터틀봇 위치 실시간 표시 (/odom)
# =====================================================================

import json
import os
import math
import yaml
from PyQt6.QtWidgets import QFrame, QVBoxLayout, QLabel, QInputDialog, QMessageBox
from PyQt6.QtGui import QPixmap, QPainter, QPen, QColor, QFont
from PyQt6.QtCore import Qt, pyqtSignal


class WaypointDB:
    """웨이포인트(노드) 정보를 JSON 파일로 영구 저장/로드"""

    def __init__(self, filename="waypoints.json"):
        self.filename = filename
        self.data = self.load()

    def load(self):
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                print(f"Waypoint 로드 실패: {e}")
        return {}

    def save(self):
        try:
            with open(self.filename, 'w', encoding='utf-8') as f:
                json.dump(self.data, f, ensure_ascii=False, indent=4)
        except Exception as e:
            print(f"Waypoint 저장 실패: {e}")

    def set_waypoint(self, name, x, y):
        self.data[name] = {"x": x, "y": y}
        self.save()

    def get_waypoint(self, name):
        return self.data.get(name)

    def delete_waypoint(self, name):
        if name in self.data:
            del self.data[name]
            self.save()
            return True
        return False

    def get_all_names(self):
        """저장된 모든 웨이포인트 이름 반환"""
        return list(self.data.keys())


class ClickableMapLabel(QLabel):
    """맵 클릭 이벤트 처리 (픽셀 좌표 → 실제 좌표 변환)"""
    map_clicked = pyqtSignal(float, float)
    map_right_clicked = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet("background-color: transparent;")
        self.original_w = 1
        self.original_h = 1

    def set_original_size(self, w, h):
        self.original_w = w
        self.original_h = h

    def _get_pixel_pos(self, pos):
        lbl_w, lbl_h = self.width(), self.height()
        if self.original_w <= 0 or self.original_h <= 0:
            return None, None
        ratio = min(lbl_w / self.original_w, lbl_h / self.original_h)
        act_w = self.original_w * ratio
        act_h = self.original_h * ratio
        offset_x = (lbl_w - act_w) / 2.0
        offset_y = (lbl_h - act_h) / 2.0
        click_x = pos.x() - offset_x
        click_y = pos.y() - offset_y
        if 0 <= click_x <= act_w and 0 <= click_y <= act_h:
            return click_x / ratio, click_y / ratio
        return None, None

    def mousePressEvent(self, event):
        try:
            px, py = self._get_pixel_pos(event.position())
            if px is not None and py is not None:
                if event.button() == Qt.MouseButton.LeftButton:
                    self.map_clicked.emit(float(px), float(py))
                elif event.button() == Qt.MouseButton.RightButton:
                    self.map_right_clicked.emit(float(px), float(py))
        except Exception as e:
            print(f"Map click math error: {e}")


class InteractiveMapPanel(QFrame):
    """대화형 맵 패널 — 맵 표시 + 노드 찍기 + 터틀봇 위치"""
    log_event = pyqtSignal(str)
    waypoints_changed = pyqtSignal()  # 웨이포인트 추가/삭제 시 사이드바 갱신용

    def __init__(self, yaml_path):
        super().__init__()
        self.setProperty("class", "Panel")
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)

        title_label = QLabel(" 맵 + 웨이포인트 에디터 (좌클릭: 추가 / 우클릭: 삭제)")
        title_label.setProperty("class", "Title")
        self.layout.addWidget(title_label)

        self.map_label = ClickableMapLabel()
        self.map_label.map_clicked.connect(self.handle_map_click)
        self.map_label.map_right_clicked.connect(self.handle_map_right_click)
        self.layout.addWidget(self.map_label, stretch=1)

        self.map_res, self.origin_x, self.origin_y = 0.05, 0.0, 0.0
        self.original_pixmap = None
        self.wp_db = WaypointDB()
        self.last_robot_x = None
        self.last_robot_y = None

        self.load_map_yaml(yaml_path)

    def load_map_yaml(self, yaml_path):
        """yaml 파일 로드 후 같은 폴더에서 pgm 이미지 자동 탐색"""
        if not os.path.exists(yaml_path):
            self.log_event.emit(f"> [ERROR] 파일을 찾을 수 없음: {yaml_path}")
            return

        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                map_data = yaml.safe_load(f)

            self.map_res = map_data['resolution']
            self.origin_x, self.origin_y = map_data['origin'][0], map_data['origin'][1]

            # yaml에 적힌 경로가 절대/상대든 관계없이 같은 폴더에서 이미지 탐색
            img_filename = os.path.basename(map_data['image'])
            yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
            img_path = os.path.join(yaml_dir, img_filename)

            if os.path.exists(img_path):
                self.original_pixmap = QPixmap(img_path)
                self.map_label.set_original_size(
                    self.original_pixmap.width(), self.original_pixmap.height())
                self.log_event.emit(f"> MAP LOADED: {img_filename} (res={self.map_res})")
                self.redraw_map()
            else:
                self.log_event.emit(f"> [ERROR] 이미지 파일을 찾을 수 없음: {img_path}")

        except Exception as e:
            self.log_event.emit(f"> [ERROR] MAP LOAD FAILED: {e}")

    def _px_to_real(self, px, py):
        """픽셀 좌표 → ROS 실제 좌표 (m)"""
        real_x = self.origin_x + (px * self.map_res)
        real_y = self.origin_y + ((self.original_pixmap.height() - py) * self.map_res)
        return real_x, real_y

    def handle_map_click(self, px, py):
        """좌클릭 → 새 웨이포인트 추가 (이름 자유 입력)"""
        if not self.original_pixmap or self.original_pixmap.isNull():
            QMessageBox.warning(self, "경고", "지도가 로드되지 않았습니다.")
            return

        try:
            real_x, real_y = self._px_to_real(px, py)
            # ★ 자유 입력: "화장실", "충전소", "신선식품" 등 사용자가 직접 입력
            name, ok = QInputDialog.getText(
                self, "웨이포인트 저장",
                "노드 이름을 입력하세요 (예: 화장실, 충전소, 신선식품):"
            )
            if ok and name.strip():
                name = name.strip()
                self.wp_db.set_waypoint(name, real_x, real_y)
                self.log_event.emit(f"> WAYPOINT SAVED: {name} ({real_x:.2f}, {real_y:.2f})")
                self.redraw_map()
                self.waypoints_changed.emit()
        except Exception as e:
            self.log_event.emit(f"> [ERROR] 저장 오류: {e}")

    def handle_map_right_click(self, px, py):
        """우클릭 → 가까운 웨이포인트 삭제"""
        if not self.original_pixmap or not self.wp_db.data:
            return
        try:
            real_x, real_y = self._px_to_real(px, py)
            closest_name, min_dist = None, float('inf')
            threshold_m = 20 * self.map_res  # 픽셀 20개 이내
            for name, coords in self.wp_db.data.items():
                dist = math.hypot(coords['x'] - real_x, coords['y'] - real_y)
                if dist < min_dist:
                    min_dist, closest_name = dist, name
            if closest_name and min_dist <= threshold_m:
                if QMessageBox.question(self, "삭제", f"'{closest_name}' 삭제?") == QMessageBox.StandardButton.Yes:
                    self.wp_db.delete_waypoint(closest_name)
                    self.log_event.emit(f"> WAYPOINT DELETED: {closest_name}")
                    self.redraw_map()
                    self.waypoints_changed.emit()
        except Exception as e:
            print(f"Delete error: {e}")

    def update_pose(self, x, y):
        """/odom 콜백 → 터틀봇 위치 업데이트"""
        self.last_robot_x, self.last_robot_y = x, y
        self.redraw_map()

    def redraw_map(self):
        """맵 다시 그리기 (웨이포인트 + 터틀봇 위치)"""
        if not self.original_pixmap or self.original_pixmap.isNull():
            return
        canvas = self.original_pixmap.copy()
        painter = QPainter(canvas)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 웨이포인트 (노란 점 + 이름)
        for wp_name, coords in self.wp_db.data.items():
            px = int((coords["x"] - self.origin_x) / self.map_res)
            py = self.original_pixmap.height() - int((coords["y"] - self.origin_y) / self.map_res)
            painter.setBrush(QColor("#FFCC00"))
            painter.setPen(QPen(QColor("#000000"), 1))
            painter.drawEllipse(px - 6, py - 6, 12, 12)
            painter.setPen(QColor("#FFFFFF"))
            painter.drawText(px + 10, py + 5, wp_name)

        # 터틀봇 위치 (시안 점)
        if self.last_robot_x is not None:
            rx = int((self.last_robot_x - self.origin_x) / self.map_res)
            ry = self.original_pixmap.height() - int((self.last_robot_y - self.origin_y) / self.map_res)
            painter.setBrush(QColor("#00E5FF"))
            painter.setPen(QPen(QColor("#000000"), 1))
            painter.drawEllipse(rx - 7, ry - 7, 14, 14)

        painter.end()
        self.map_label.setPixmap(
            canvas.scaled(self.map_label.size(),
                          Qt.AspectRatioMode.KeepAspectRatio,
                          Qt.TransformationMode.SmoothTransformation))