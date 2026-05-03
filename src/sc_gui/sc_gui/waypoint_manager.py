# =====================================================================
# waypoint_manager.py — 단순 버전
# 맵 표시 + 화장실/충전소 노드 1점씩 + 저장 (꾸미지 않음)
#
# 동작:
#   - 상단 "화장실 노드 정하기" 버튼 클릭 → toilet 배치 모드
#     다음 좌클릭 위치에 파란점 1개만 (덮어씀)
#   - 상단 "충전소 노드 정하기" 버튼 클릭 → charger 배치 모드
#     다음 좌클릭 위치에 빨간점 1개만 (덮어씀)
#   - "노드 저장" → ~/.smart_cart/waypoints.json (launch 재시작 후에도 유지)
# =====================================================================

import json
import os
import yaml
from PyQt5.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QMessageBox)
from PyQt5.QtGui import QPixmap, QPainter, QPen, QColor, QBrush
from PyQt5.QtCore import Qt, pyqtSignal


CATEGORY_COLORS = {
    'toilet':  QColor('#1E88FF'),
    'charger': QColor('#FF3344'),
}
CATEGORY_LABELS = {
    'toilet':  '화장실',
    'charger': '충전소',
}


class WaypointDB:
    """카테고리당 1점만 저장 — ~/.smart_cart/waypoints.json"""

    def __init__(self, filename=None):
        if filename is None:
            home = os.path.expanduser('~/.smart_cart')
            os.makedirs(home, exist_ok=True)
            filename = os.path.join(home, 'waypoints.json')
        self.filename = filename
        self.data = self.load()

    def load(self):
        if not os.path.exists(self.filename):
            return {}
        try:
            with open(self.filename, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return {}

    def save(self):
        try:
            with open(self.filename, 'w', encoding='utf-8') as f:
                json.dump(self.data, f, ensure_ascii=False, indent=4)
        except Exception as e:
            print(f'WaypointDB save error: {e}')

    def set_waypoint(self, category, x, y):
        self.data[category] = {'x': float(x), 'y': float(y), 'category': category}
        self.save()

    def get_waypoint(self, category):
        return self.data.get(category)

    def delete_waypoint(self, category):
        if category in self.data:
            del self.data[category]
            self.save()


class _ClickableMapLabel(QLabel):
    map_clicked = pyqtSignal(float, float)
    map_right_clicked = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.original_w = 1
        self.original_h = 1

    def set_original_size(self, w, h):
        self.original_w = w
        self.original_h = h

    def _to_pixel(self, pos):
        lw, lh = self.width(), self.height()
        if self.original_w <= 0 or self.original_h <= 0:
            return None, None
        ratio = min(lw / self.original_w, lh / self.original_h)
        aw = self.original_w * ratio
        ah = self.original_h * ratio
        ox = (lw - aw) / 2.0
        oy = (lh - ah) / 2.0
        cx = pos.x() - ox
        cy = pos.y() - oy
        if 0 <= cx <= aw and 0 <= cy <= ah:
            return cx / ratio, cy / ratio
        return None, None

    def mousePressEvent(self, ev):
        try:
            px, py = self._to_pixel(ev.pos())
            if px is None:
                return
            if ev.button() == Qt.LeftButton:
                self.map_clicked.emit(float(px), float(py))
            elif ev.button() == Qt.RightButton:
                self.map_right_clicked.emit(float(px), float(py))
        except Exception as e:
            print(f'map click error: {e}')


class InteractiveMapPanel(QFrame):
    log_event = pyqtSignal(str)
    waypoints_changed = pyqtSignal()

    def __init__(self, yaml_path):
        super().__init__()
        layout = QVBoxLayout(self)

        # 상단 버튼 행 (꾸미지 않음)
        bar = QHBoxLayout()
        self.btn_toilet = QPushButton("화장실 노드 정하기")
        self.btn_toilet.setCheckable(True)
        self.btn_toilet.toggled.connect(
            lambda c: self._set_mode('toilet' if c else None))
        bar.addWidget(self.btn_toilet)

        self.btn_charger = QPushButton("충전소 노드 정하기")
        self.btn_charger.setCheckable(True)
        self.btn_charger.toggled.connect(
            lambda c: self._set_mode('charger' if c else None))
        bar.addWidget(self.btn_charger)

        self.btn_save = QPushButton("노드 저장")
        self.btn_save.clicked.connect(self._on_save)
        bar.addWidget(self.btn_save)

        bar.addStretch()
        layout.addLayout(bar)

        self.map_label = _ClickableMapLabel()
        self.map_label.map_clicked.connect(self._on_left_click)
        self.map_label.map_right_clicked.connect(self._on_right_click)
        layout.addWidget(self.map_label, stretch=1)

        # 상태
        self.map_res = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.original_pixmap = None
        self.wp_db = WaypointDB()
        self.last_robot_x = None
        self.last_robot_y = None
        self.placement_mode = None

        self.load_map_yaml(yaml_path)

    def _set_mode(self, mode):
        self.placement_mode = mode
        # 양쪽 버튼이 동시에 켜지지 않도록
        self.btn_toilet.blockSignals(True)
        self.btn_charger.blockSignals(True)
        self.btn_toilet.setChecked(mode == 'toilet')
        self.btn_charger.setChecked(mode == 'charger')
        self.btn_toilet.blockSignals(False)
        self.btn_charger.blockSignals(False)
        if mode:
            self.log_event.emit(
                f"[MODE] {CATEGORY_LABELS.get(mode, mode)} 배치 — 맵 좌클릭")
        else:
            self.log_event.emit("[MODE] 해제")

    def load_map_yaml(self, yaml_path):
        if not os.path.exists(yaml_path):
            self.log_event.emit(f"[ERR] yaml 없음: {yaml_path}")
            return
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                m = yaml.safe_load(f)
            self.map_res = m['resolution']
            self.origin_x = m['origin'][0]
            self.origin_y = m['origin'][1]
            img_name = os.path.basename(m['image'])
            img_path = os.path.join(
                os.path.dirname(os.path.abspath(yaml_path)), img_name)
            if not os.path.exists(img_path):
                self.log_event.emit(f"[ERR] 이미지 없음: {img_path}")
                return
            self.original_pixmap = QPixmap(img_path)
            self.map_label.set_original_size(
                self.original_pixmap.width(), self.original_pixmap.height())
            self.log_event.emit(f"MAP LOADED: {img_name}")
            self.redraw_map()
        except Exception as e:
            self.log_event.emit(f"[ERR] map load: {e}")

    def _px_to_real(self, px, py):
        rx = self.origin_x + (px * self.map_res)
        ry = self.origin_y + ((self.original_pixmap.height() - py) * self.map_res)
        return rx, ry

    def _on_left_click(self, px, py):
        if not self.original_pixmap or self.original_pixmap.isNull():
            return
        if self.placement_mode is None:
            self.log_event.emit("[HINT] 상단 노드 정하기 버튼 먼저 누르세요")
            return
        rx, ry = self._px_to_real(px, py)
        cat = self.placement_mode
        self.wp_db.set_waypoint(cat, rx, ry)
        self.log_event.emit(
            f"WP SET: {CATEGORY_LABELS[cat]} ({rx:.2f}, {ry:.2f})")
        self._set_mode(None)
        self.redraw_map()
        self.waypoints_changed.emit()

    def _on_right_click(self, px, py):
        if not self.original_pixmap or not self.wp_db.data:
            return
        rx, ry = self._px_to_real(px, py)
        best, mind = None, float('inf')
        thr = 20 * self.map_res
        for cat, c in self.wp_db.data.items():
            d = ((c['x'] - rx) ** 2 + (c['y'] - ry) ** 2) ** 0.5
            if d < mind:
                mind, best = d, cat
        if best and mind <= thr:
            if QMessageBox.question(
                    self, '삭제',
                    f"'{CATEGORY_LABELS.get(best, best)}' 삭제?",
                    QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                self.wp_db.delete_waypoint(best)
                self.log_event.emit(f"WP DELETED: {best}")
                self.redraw_map()
                self.waypoints_changed.emit()

    def _on_save(self):
        self.wp_db.save()
        cnt = len(self.wp_db.data)
        self.log_event.emit(f"[SAVE] {cnt}개 → {self.wp_db.filename}")
        QMessageBox.information(
            self, '저장',
            f"{cnt}개 노드 저장 완료\n다음 launch 시 자동 로드")

    def update_pose(self, x, y):
        self.last_robot_x = x
        self.last_robot_y = y
        self.redraw_map()

    def redraw_map(self):
        if not self.original_pixmap or self.original_pixmap.isNull():
            return
        canvas = self.original_pixmap.copy()
        p = QPainter(canvas)
        p.setRenderHint(QPainter.Antialiasing)

        for cat, c in self.wp_db.data.items():
            color = CATEGORY_COLORS.get(cat, QColor('#FFCC00'))
            px = int((c['x'] - self.origin_x) / self.map_res)
            py = self.original_pixmap.height() \
                - int((c['y'] - self.origin_y) / self.map_res)
            p.setBrush(QBrush(color))
            p.setPen(QPen(QColor('#000'), 1))
            p.drawEllipse(px - 7, py - 7, 14, 14)
            p.setPen(QColor('#000'))
            p.drawText(px + 10, py + 5, CATEGORY_LABELS.get(cat, cat))

        if self.last_robot_x is not None:
            rx = int((self.last_robot_x - self.origin_x) / self.map_res)
            ry = self.original_pixmap.height() \
                - int((self.last_robot_y - self.origin_y) / self.map_res)
            p.setBrush(QBrush(QColor('#00CC66')))
            p.setPen(QPen(QColor('#000'), 1))
            p.drawEllipse(rx - 8, ry - 8, 16, 16)

        p.end()
        self.map_label.setPixmap(
            canvas.scaled(self.map_label.size(),
                          Qt.KeepAspectRatio,
                          Qt.SmoothTransformation))

    def get_waypoint(self, category):
        return self.wp_db.get_waypoint(category)
