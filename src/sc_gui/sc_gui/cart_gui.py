# =====================================================================
# cart_gui.py
# Flask 기반 웹 장바구니 + QR 결제 시스템
# 별도 5000번 포트로 실행 (모바일에서 접속해서 결제)
# =====================================================================

import socket
import io
import qrcode
import time
import csv
import os
from flask import Flask, render_template_string, send_file, jsonify, request

# ROS2 (cart ↔ topic 브릿지)
try:
    import rclpy
    from rclpy.node import Node as RclNode
    from std_msgs.msg import String as StdString
    from sc_interfaces.msg import PaymentEvent, ItemDetected
    _ROS_OK = True
except Exception:
    _ROS_OK = False
    RclNode = object


# =========================================================
# 1. 데이터 관리 클래스 (Database & Cart Manager)
# =========================================================
class CartManager:
    """상품 DB 로드 및 장바구니 상태 관리"""

    def __init__(self, db_path='product_db.csv'):
        self.db_path = db_path
        self.product_db = self._load_db()
        self.cart_items = []
        self.payment_completed = False
        self.last_state = {"length": 0, "qty_sum": 0}

    def _load_db(self):
        db = {}
        if not os.path.exists(self.db_path):
            print(f"[경고] '{self.db_path}' 파일이 없습니다.")
            return db
        try:
            with open(self.db_path, 'r', encoding='utf-8-sig') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row.get('class_name'):
                        db[row['class_name']] = {
                            "name": row.get('name', '이름 없음'),
                            "price": int(row.get('price', 0)),
                            "img_url": row.get('img_url', '')
                        }
            print(f"DB 로드 완료: {len(db)}개 품목")
        except Exception as e:
            print(f"DB 로드 오류: {e}")
        return db

    def add_item(self, class_name):
        db_item = self.product_db.get(class_name, {
            "name": f"미등록 상품 ({class_name})",
            "price": 0,
            "img_url": "https://placehold.co/150x150/EEEEEE/333333?text=Unknown"
        })
        for item in self.cart_items:
            if item['name'] == db_item['name']:
                item['qty'] += 1
                return True
        self.cart_items.append({
            "name": db_item["name"], "price": db_item["price"],
            "qty": 1, "img_url": db_item["img_url"]
        })
        return True

    def reset(self):
        self.cart_items = []
        self.payment_completed = False

    def check_changed(self):
        curr_len = len(self.cart_items)
        curr_qty = sum(item['qty'] for item in self.cart_items)
        changed = (curr_len != self.last_state["length"] or curr_qty != self.last_state["qty_sum"])
        self.last_state = {"length": curr_len, "qty_sum": curr_qty}
        return changed


# =========================================================
# 2. 시스템 유틸리티
# =========================================================
class SystemUtils:
    """네트워크 IP 및 QR 코드 생성"""

    @staticmethod
    def get_local_ip():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('10.255.255.255', 1))
            ip = s.getsockname()[0]
        except Exception:
            ip = '127.0.0.1'
        finally:
            s.close()
        return ip

    @staticmethod
    def create_qr_buffer(url):
        qr = qrcode.QRCode(version=1, box_size=10, border=4)
        qr.add_data(url)
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white")
        buf = io.BytesIO()
        img.save(buf, format='PNG')
        buf.seek(0)
        return buf


# =========================================================
# 3. 템플릿 엔진 클래스 (UI/UX)
# =========================================================
class TemplateManager:

    HEAD_COMMON = """
    <meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css" rel="stylesheet">
    <style>
        body { background-color: #eaeded; font-family: -apple-system, sans-serif; padding-bottom: 120px; }
        .navbar-custom { background-color: #232f3e; color: white; padding: 15px; text-align: center; font-weight: 700; font-size: 1.4rem; }
        .promo-banner { background-color: #e3f2fd; color: #0d47a1; padding: 12px; text-align: center; font-weight: bold; font-size: 0.95rem; }
        .cart-container { background-color: white; margin-top: 15px; padding: 20px; border-radius: 8px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
        .product-card { display: flex; border-bottom: 1px solid #ddd; padding-bottom: 20px; margin-bottom: 20px; }
        .product-img { width: 90px; height: 90px; object-fit: cover; border-radius: 8px; margin-right: 15px; }
        .fixed-bottom-bar { position: fixed; bottom: 0; left: 0; width: 100%; background-color: white; padding: 15px 25px; box-shadow: 0 -4px 20px rgba(0,0,0,0.1); z-index: 1030; display: flex; justify-content: space-between; align-items: center; border-radius: 20px 20px 0 0; }
        .total-price { font-size: 1.8rem; font-weight: 900; color: #B12704; }
        .btn-pay { font-size: 1.2rem; padding: 12px 25px; border-radius: 50px; background-color: #ffd814; border: 1px solid #fcd200; font-weight: bold; }
    </style>
    """

    MAIN = """
    <!DOCTYPE html><html><head>""" + HEAD_COMMON + """<title>Smart Cart</title></head>
    <body>
        <nav class="navbar-custom">Frictionless Store</nav>
        <div class="promo-banner">🎉 [오픈 이벤트] 신선식품 코너 20% 할인 진행 중!</div>
        <div class="container-fluid">
            <div class="cart-container">
                <h5 class="fw-bold border-bottom pb-2 mb-3">장바구니 <small class="text-muted fw-normal">자동 인식 대기중</small></h5>
                {% if items %}
                    {% for item in items %}
                    <div class="product-card">
                        <img src="{{ item.img_url }}" class="product-img" onerror="this.src='https://placehold.co/90x90?text=No+Image'">
                        <div class="flex-grow-1">
                            <div class="fw-bold">{{ item.name }}</div>
                            <div class="text-success small mb-2">바로 결제 가능</div>
                            <div class="d-flex justify-content-between align-items-center">
                                <span class="fs-5 fw-bold">{{ "{:,}".format(item.price * item.qty) }}원</span>
                                <span class="badge bg-light text-dark border">수량: {{ item.qty }}</span>
                            </div>
                        </div>
                    </div>
                    {% endfor %}
                {% else %}
                    <div class="text-center py-5 text-muted">장바구니가 비어 있습니다.</div>
                {% endif %}
            </div>
        </div>

        {% set total_qty = items|sum(attribute='qty') %}
        {% set total_price = namespace(value=0) %}
        {% for item in items %}{% set total_price.value = total_price.value + (item.price * item.qty) %}{% endfor %}

        <div class="fixed-bottom-bar">
            <div><div class="small text-muted">총 수량: {{ total_qty }}개</div><div class="total-price">{{ "{:,}".format(total_price.value) }}원</div></div>
            <button class="btn btn-pay" onclick="new bootstrap.Modal(document.getElementById('qrModal')).show(); document.getElementById('qrImg').src='/qrcode?t='+Date.now()">💳 결제하기</button>
        </div>

        <div class="modal fade" id="qrModal" tabindex="-1" aria-hidden="true">
            <div class="modal-dialog modal-dialog-centered"><div class="modal-content border-0 rounded-4 text-center shadow-lg"><div class="modal-body p-5">
                <h4 class="fw-bold mb-3">결제 QR 코드</h4><p class="text-muted">스마트폰으로 스캔하여 결제하세요.</p>
                <div class="p-3 bg-light rounded-4 d-inline-block border"><img id="qrImg" src="" style="width: 200px;"></div>
                <div class="mt-4"><button class="btn btn-outline-dark rounded-pill px-5" data-bs-dismiss="modal">취소</button></div>
            </div></div></div>
        </div>

        <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/js/bootstrap.bundle.min.js"></script>
        <script>
            setInterval(() => {
                fetch('/api/status').then(r => r.json()).then(data => {
                    if(data.cart_changed) window.location.reload();
                    if(data.paid) window.location.href='/show_success';
                });
            }, 1000);
        </script>
    </body></html>
    """

    RECEIPT = """
    <!DOCTYPE html><html><head>""" + HEAD_COMMON + """<title>Receipt</title>
    <style>.receipt-card { max-width: 400px; margin: 30px auto; border-radius: 12px; border: 1px solid #ddd; background: white; }</style></head>
    <body class="p-3">
        <div class="receipt-card p-4 shadow-sm text-center">
            <div class="display-4 text-success mb-2">✔</div><h3 class="fw-bold">주문 완료</h3>
            <div class="border-top border-bottom py-3 my-3 text-start">
                {% for item in items %}<div class="d-flex justify-content-between"><span>{{item.name}} ({{item.qty}})</span><b>{{ "{:,}".format(item.price*item.qty) }}원</b></div>{% endfor %}
            </div>
            <div class="d-flex justify-content-between fs-4 fw-bold"><span>총액</span><span class="text-danger">{{ "{:,}".format(total) }}원</span></div>
            <button class="btn btn-dark w-100 mt-4 py-3 fw-bold" onclick="closeApp()">확인</button>
        </div>
        <script>
            function closeApp() {
                const ua = navigator.userAgent.toLowerCase();
                if(ua.includes('kakaotalk')) location.href='kakaotalk://inappbrowser/close';
                else if(ua.includes('naver')) location.href='naversearchapp://inappbrowser/close';
                else { window.open('','_self').close(); setTimeout(()=>{ location.href='/thank_you' },500); }
            }
        </script>
    </body></html>
    """

    SUCCESS_POPUP = """
    <!DOCTYPE html><html><head>""" + HEAD_COMMON + """<title>Success</title></head>
    <body>
        <div class="modal fade show" style="display:block; background:rgba(0,0,0,0.5);" tabindex="-1">
            <div class="modal-dialog modal-dialog-centered"><div class="modal-content border-0 rounded-4 text-center shadow-lg"><div class="modal-body p-5">
                <div style="font-size: 50px; color: #007600; margin-bottom: 15px;">✔</div>
                <h4 class="fw-bold mb-3">결제가 완료되었습니다</h4>
                <p class="text-muted mb-4">이용해 주셔서 감사합니다.<br>안전하게 상품을 담아가세요.</p>
                <button class="btn btn-dark rounded-pill px-5 py-2 fw-bold" onclick="window.location.href='/reset_cart'">확인</button>
            </div></div></div>
        </div>
    </body></html>
    """


# =========================================================
# 3.5 ROS Bridge Node — Flask ↔ ROS2 토픽 발행/구독
# =========================================================
class CartRosBridgeNode(RclNode):
    """
    Flask 결제 서버에서 일어난 이벤트를 ROS 토픽으로 발행.
    또한 /item_detected, /item_confirm 을 구독해 장바구니에 자동 추가.

    Publish:
      /payment/event   (sc_interfaces/PaymentEvent)
      /item_confirm    (std_msgs/String)  — 결제 완료 후 cart 변경 알림 등

    Subscribe:
      /item_detected   (sc_interfaces/ItemDetected) — 백업: GUI 외 직접 추가
      /item_confirm    (std_msgs/String)            — 뚜껑 OPEN 확정 시 동기화
    """

    def __init__(self):
        if not _ROS_OK:
            return
        super().__init__('cart_ros_bridge')
        self.cart_ref = None  # SmartCartServer.cart (장바구니 매니저)

        self.payment_pub = self.create_publisher(
            PaymentEvent, '/payment/event', 10)
        self.item_confirm_pub = self.create_publisher(
            StdString, '/item_confirm', 10)

        self.item_sub = self.create_subscription(
            ItemDetected, '/item_detected', self._on_item, 10)
        self.confirm_sub = self.create_subscription(
            StdString, '/item_confirm', self._on_confirm, 10)

        self.get_logger().info('CartRosBridgeNode 시작')

    def attach_cart(self, cart):
        self.cart_ref = cart

    def _on_item(self, msg: 'ItemDetected'):
        if self.cart_ref and msg.in_basket_zone and msg.item_name:
            self.cart_ref.add_item(msg.item_name)

    def _on_confirm(self, msg: 'StdString'):
        # 뚜껑 OPEN 확정 — 추가 로깅용
        try:
            self.get_logger().info(f'[item_confirm] {msg.data}')
        except Exception:
            pass

    def publish_payment(self, event, total_price=0, item_count=0, message=''):
        if not _ROS_OK:
            return
        try:
            m = PaymentEvent()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'flask_cart'
            m.event = event
            m.total_price = int(total_price)
            m.item_count = int(item_count)
            m.message = message
            self.payment_pub.publish(m)
        except Exception as e:
            try:
                self.get_logger().error(f'publish_payment 실패: {e}')
            except Exception:
                pass


# =========================================================
# 4. Flask 서버 메인 엔진
# =========================================================
class SmartCartServer:
    """Flask 서버 구동 + 모든 API 라우팅 + ROS 토픽 발행"""

    def __init__(self, ros_bridge=None):
        self.app = Flask(__name__)
        self.cart = CartManager()
        self.utils = SystemUtils()
        self.ui = TemplateManager()
        self.host_ip = self.utils.get_local_ip()
        self.ros_bridge = ros_bridge
        if self.ros_bridge is not None:
            try:
                self.ros_bridge.attach_cart(self.cart)
            except Exception:
                pass
        self._setup_routes()

    def _setup_routes(self):
        @self.app.after_request
        def add_header(r):
            r.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
            return r

        @self.app.route('/')
        def index():
            self.cart.check_changed()
            return render_template_string(self.ui.MAIN, items=self.cart.cart_items)

        @self.app.route('/qrcode')
        def qrcode_gen():
            url = f"http://{self.host_ip}:5000/process_payment?t={int(time.time())}"
            return send_file(self.utils.create_qr_buffer(url), mimetype='image/png')

        @self.app.route('/process_payment')
        def process():
            total = sum(i['price'] * i['qty'] for i in self.cart.cart_items)
            res = render_template_string(self.ui.RECEIPT, items=self.cart.cart_items, total=total)
            self.cart.payment_completed = True
            # ★ ROS 토픽으로 결제 완료 발행
            if self.ros_bridge is not None:
                try:
                    self.ros_bridge.publish_payment(
                        event='paid',
                        total_price=total,
                        item_count=sum(i['qty'] for i in self.cart.cart_items),
                        message='Flask QR 결제 완료')
                except Exception as e:
                    print(f'publish_payment 실패: {e}')
            return res

        @self.app.route('/api/status')
        def status():
            return jsonify({"paid": self.cart.payment_completed, "cart_changed": self.cart.check_changed()})

        @self.app.route('/api/add_item', methods=['POST'])
        def api_add():
            data = request.json
            if not data:
                return jsonify({"status": "fail"}), 400
            c_name = data.get('class_name')
            if c_name and self.cart.add_item(c_name):
                return jsonify({"status": "ok"})
            return jsonify({"status": "fail"}), 400

        @self.app.route('/show_success')
        def show_success():
            return render_template_string(self.ui.SUCCESS_POPUP)

        @self.app.route('/reset_cart')
        def reset():
            self.cart.reset()
            return """<script>window.location.href='/';</script>"""

        @self.app.route('/thank_you')
        def thanks():
            return "<body style='text-align:center;padding-top:100px;'><h3>결제가 완료되었습니다.</h3><p>화면을 닫아주세요.</p></body>"

    def run(self):
        print(
            f"\n{'=' * 45}\n Smart Cart Server Online\n"
            f" API: http://{self.host_ip}:5000/api/add_item\n"
            f" GUI: http://{self.host_ip}:5000\n{'=' * 45}")
        self.app.run(host='0.0.0.0', port=5000, debug=False)


if __name__ == '__main__':
    server = SmartCartServer()
    server.run()


def main():
    """ros2 run sc_gui cart_server 로 단독 실행할 때 호출"""
    bridge = None
    if _ROS_OK:
        try:
            rclpy.init()
            bridge = CartRosBridgeNode()
            import threading
            threading.Thread(
                target=lambda: rclpy.spin(bridge), daemon=True).start()
        except Exception as e:
            print(f'rclpy 초기화 실패 (Flask only): {e}')
    server = SmartCartServer(ros_bridge=bridge)
    server.run()