#!/usr/bin/env python3
# 폰 웹 주문 서버 — 서빙봇 주방 패널 (1008 v5 · 테이블 6개 · 실물 테스트 전 · Claude 가 쓴 것)
#
# 화면 두 개 (같은 서버)
#   손님 폰   http://<노트북 IP>:5000/order   메뉴를 골라 주문 → 주방 목록에 쌓임 (로봇은 아직 안 움직임)
#   직원 폰   http://<노트북 IP>:5000/        주문 목록을 보고 테이블을 골라 로봇을 보냄
#
# 하는 일
#   폰 화면에서 버튼을 누름  →  이 서버가 /delivery_order 토픽으로 주문 글자를 보냄  →  매니저가 로봇을 움직임
#   매니저가 /delivery_status 로 알려 주는 상태  →  이 서버가 받아 둠  →  폰 화면이 1초에 한두 번 물어봐서 보여 줌
#
# 켜는 법 (노트북)
#   python3 web_order.py          ← ROS 가 있으면 진짜 로봇 모드, 없으면 자동으로 연습 모드
#   python3 web_order.py --demo   ← 로봇 없이 화면만 연습 (가짜 로봇이 움직임)
#   python3 web_order.py --demo --port 5050   ← 포트를 바꿀 때 (맥은 5000 을 다른 프로그램이 씀)
# 폰에서:  http://<노트북 IP>:5000   (노트북 IP 는 hostname -I)
#
# 주문 글자 (order_ui.py 와 같음)
#   '2'          모드 2 · 단일   (도착 후 10초 주문 없으면 매니저가 주방으로 복귀)
#   '3,1,4'      모드 3 · 골라서 (끝나면 주방 복귀)
#   '1,2,3,4,5,6'  모드 1 · 전체
#   '0'          주방 복귀
#   'STOP'       긴급 정지
import json
import os
import sys
import threading
import time

from flask import Flask, jsonify, request, send_from_directory

PORT = 5000
TABLES = ['1', '2', '3', '4', '5', '6']  # 화면에 보여 줄 테이블 번호 (10/8 테이블 6개 맵)
BUSY_STATES = ('moving', 'arrived', 'returning')  # 이 상태일 때는 새 주문을 받지 않음 (매니저도 거절함)
STATUS_FRESH_SEC = 3.0                   # 이 시간 안에 상태가 왔으면 "매니저 켜짐" 으로 봄

# 손님 화면 메뉴 (임시 — 이름 · 값은 여기서 고치면 됨)
MENU = [
    {'id': 'kimchi', 'name': '김치찌개', 'price': 8000},
    {'id': 'bibimbap', 'name': '비빔밥', 'price': 9000},
    {'id': 'donkatsu', 'name': '돈까스', 'price': 10000},
    {'id': 'naengmyeon', 'name': '냉면', 'price': 9000},
    {'id': 'cola', 'name': '콜라', 'price': 2000},
    {'id': 'ade', 'name': '레몬에이드', 'price': 4000},
]

HERE = os.path.dirname(os.path.abspath(__file__))


# ─────────────────────────────────────────────────────────────
# 1. 상태 보관함 — 로봇 상태와 최근 기록을 들고 있음
# ─────────────────────────────────────────────────────────────
class Hub:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = {'state': 'unknown', 'target': '', 'route': [], 'done': [], 'msg': '로봇 상태를 기다리는 중'}
        self.status_time = 0.0   # 마지막으로 상태를 받은 시각
        self.log = []            # 최근 기록 [{'t': '14:03:21', 'kind': 'order', 'text': '...'}]
        self.listeners = []      # 로봇 상태가 바뀔 때 부를 함수들 (주문 장부가 등록함)

    def add_log(self, kind, text):
        with self.lock:
            self.log.insert(0, {'t': time.strftime('%H:%M:%S'), 'kind': kind, 'text': text})
            del self.log[30:]

    # 매니저(또는 연습용 가짜 로봇)가 상태를 알려 줄 때마다 불림
    def on_status(self, data):
        with self.lock:
            old = (self.status.get('state'), self.status.get('target'))
            self.status = data
            self.status_time = time.time()
            changed = old != (data.get('state'), data.get('target'))
        if changed and data.get('msg'):
            kind = {'failed': 'bad', 'stopped': 'bad', 'arrived': 'good', 'hold': 'good'}.get(data.get('state'), 'info')
            self.add_log(kind, data['msg'])
        if changed:
            for fn in self.listeners:
                fn(data)

    def snapshot(self):
        with self.lock:
            return dict(self.status), time.time() - self.status_time, list(self.log)


# ─────────────────────────────────────────────────────────────
# 1-B. 주문 장부 — 손님이 넣은 음식 주문을 쌓아 둠 (서버를 끄면 사라짐)
#   주문 한 건의 흐름:  waiting 주방 준비 중 → sent 로봇이 싣고 출발 → arrived 테이블 도착 → done 손님이 받음
# ─────────────────────────────────────────────────────────────
class Orders:
    def __init__(self, hub):
        self.lock = threading.Lock()
        self.items = []     # [{'id': 1, 'table': '3', 'lines': [...], 'total': 17000, 't': '14:03', 'state': 'waiting'}]
        self.next_id = 1
        self.hub = hub
        hub.listeners.append(self.on_robot)

    # 손님이 주문함
    def add(self, table, cart):
        lines, total = [], 0
        for m in MENU:
            qty = int(cart.get(m['id'], 0) or 0)
            if qty > 0:
                lines.append({'id': m['id'], 'name': m['name'], 'qty': qty})
                total += m['price'] * qty
        if not lines:
            return None
        with self.lock:
            order = {'id': self.next_id, 'table': table, 'lines': lines, 'total': total,
                     't': time.strftime('%H:%M'), 'state': 'waiting'}
            self.next_id += 1
            self.items.append(order)
        text = ' · '.join(f"{x['name']} {x['qty']}" for x in lines)
        self.hub.add_log('order', f'{table}번 테이블 주문 들어옴: {text}')
        return order

    def _set(self, test, new_state):
        with self.lock:
            for o in self.items:
                if test(o):
                    o['state'] = new_state

    # 직원이 로봇을 보냄 → 그 테이블들의 준비 중 주문을 "배달 중" 으로
    def mark_sent(self, tables):
        self._set(lambda o: o['state'] == 'waiting' and o['table'] in tables, 'sent')

    # 로봇 상태가 바뀔 때마다 불림
    def on_robot(self, st):
        state, target = st.get('state'), st.get('target')
        if state in ('arrived', 'hold'):
            self._set(lambda o: o['state'] == 'sent' and o['table'] == target, 'arrived')
        elif state in ('failed', 'stopped'):
            # 못 간 주문은 다시 주방 목록으로 (직원이 다시 보낼 수 있게)
            self._set(lambda o: o['state'] == 'sent', 'waiting')
        elif state == 'returning' and '실패' in (st.get('msg') or ''):
            self._set(lambda o: o['state'] == 'sent', 'waiting')

    def set_state(self, order_id, new_state, only_from=None):
        with self.lock:
            for o in self.items:
                if o['id'] == order_id and (only_from is None or o['state'] in only_from):
                    o['state'] = new_state
                    return o
        return None

    def active(self):       # 직원 화면용 — 아직 안 끝난 주문
        with self.lock:
            return [dict(o) for o in self.items if o['state'] in ('waiting', 'sent', 'arrived')]

    def of_table(self, table):   # 손님 화면용 — 그 테이블의 최근 주문
        with self.lock:
            mine = [dict(o) for o in self.items if o['table'] == table and o['state'] != 'cancel']
        return mine[-4:]


# ─────────────────────────────────────────────────────────────
# 2-A. 진짜 로봇 연결 (ROS 2)
# ─────────────────────────────────────────────────────────────
class RosLink:
    name = 'ros'

    def __init__(self, hub):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        self.String = String
        rclpy.init()
        self.node = Node('web_order_node')
        self.pub = self.node.create_publisher(String, '/delivery_order', 10)
        self.node.create_subscription(String, '/delivery_status', self._on_msg, 10)
        self.hub = hub
        # spin = 들어오는 메시지를 계속 받는 무한 반복 → 웹 서버와 따로 돌도록 다른 실(thread)에서
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _on_msg(self, msg):
        try:
            self.hub.on_status(json.loads(msg.data))
        except ValueError:
            pass  # JSON 이 아니면 무시

    def send(self, text):
        msg = self.String()
        msg.data = text
        self.pub.publish(msg)

    # /delivery_order 를 듣고 있는 노드 수 (1 이상이면 매니저가 켜져 있음)
    def listeners(self):
        return self.pub.get_subscription_count()


# ─────────────────────────────────────────────────────────────
# 2-B. 연습 모드 — 로봇 없이 매니저 흉내 (화면 확인 · 발표 연습용)
# ─────────────────────────────────────────────────────────────
class DemoLink:
    name = 'demo'
    # 10/8 깃허브 robot5 좌표와 같은 값 (연습 모드에서 X · Y 숫자가 실물과 비슷하게 보이도록)
    SPOTS = {'0': (0.25, 0.14), '1': (1.69, 0.05), '2': (2.88, 0.34), '3': (2.90, 1.61),
             '4': (1.93, 1.63), '5': (1.07, 1.71), '6': (0.09, 1.46)}
    SPEED = 0.6          # 가짜 로봇 속도 (m/s) — 실물보다 빠름
    SERVE_WAIT = 3.0     # 테이블 도착 후 대기
    IDLE_RETURN = 10.0   # 모드 2 도착 후 주문 대기

    def __init__(self, hub):
        self.hub = hub
        self.lock = threading.Lock()
        self.busy = False
        self.stop = False
        self.job = 0       # 주문 번호 (새 주문이 오면 옛 자동 복귀를 무시하려고)
        self.xy = list(self.SPOTS['0'])
        self.st = {'state': 'idle', 'target': '', 'route': [], 'done': [], 'msg': '주문 대기 중 (연습 모드)'}
        # 실패 화면을 보고 싶으면:  SERVING_DEMO_FAIL=3 python3 web_order.py --demo
        self.fail = set(os.environ.get('SERVING_DEMO_FAIL', '').split(',')) - {''}
        threading.Thread(target=self._beat, daemon=True).start()

    def listeners(self):
        return 1

    def _emit(self, state=None, target='', msg=''):
        if state is not None:
            self.st.update(state=state, target=target, msg=msg)
        data = dict(self.st, x=round(self.xy[0], 2), y=round(self.xy[1], 2))
        data['route'], data['done'] = list(self.st['route']), list(self.st['done'])
        self.hub.on_status(data)

    def _beat(self):   # 매니저처럼 1초마다 상태를 다시 알림
        while True:
            time.sleep(1.0)
            self._emit()

    def _name(self, t):
        return '주방(0번)' if t == '0' else f'{t}번 테이블'

    def _go(self, t):  # 목적지까지 조금씩 이동. 정지되면 False
        tx, ty = self.SPOTS[t]
        t0 = time.time()
        while True:
            if self.stop:
                return False
            if t in self.fail and time.time() - t0 > 4.0:
                return False
            dx, dy = tx - self.xy[0], ty - self.xy[1]
            d = (dx * dx + dy * dy) ** 0.5
            if d < 0.03:
                return True
            step = min(d, self.SPEED * 0.1)
            self.xy[0] += dx / d * step
            self.xy[1] += dy / d * step
            self._emit()
            time.sleep(0.1)

    def _wait(self, sec):
        end = time.time() + sec
        while time.time() < end:
            if self.stop:
                return False
            time.sleep(0.05)
        return True

    def _home(self, msg):
        self._emit('returning', '0', msg)
        if self._go('0'):
            self.st['route'], self.st['done'] = [], []
            self._emit('idle', '', '주방 복귀 완료 · 주문 대기 중')
        elif not self.stop:
            self._emit('failed', '0', '주방 복귀 실패 · 로봇 위치를 확인하세요')

    def send(self, text):
        if text == 'STOP':
            self.stop = True
            self.job += 1
            self._emit('stopped', self.st['target'], '긴급 정지 · 그 자리에 멈춤')
            return
        with self.lock:
            if self.busy:
                return
            self.busy = True
            self.job += 1
        threading.Thread(target=self._run, args=(text, self.job), daemon=True).start()

    def _run(self, text, job):
        try:
            self.stop = False
            route = [t.strip() for t in text.split(',') if t.strip() in self.SPOTS]
            single = len(route) == 1
            self.st['route'], self.st['done'] = list(route), []
            ok = False
            for t in route:
                self._emit('moving', t, f'{self._name(t)}(으)로 가는 중')
                ok = self._go(t)
                if not ok or self.stop:
                    break
                self.st['done'].append(t)
                self._emit('arrived', t, f'{self._name(t)} 도착 · {self.SERVE_WAIT:.0f}초 대기')
                if not self._wait(self.SERVE_WAIT):
                    break
            if self.stop:
                return
            if single:
                if not ok:
                    self._emit('failed', route[0], f'{self._name(route[0])}에 도착하지 못했습니다')
                elif route[0] != '0':
                    self.st['hold_sec'] = self.IDLE_RETURN
                    self._emit('hold', route[0], f'{self._name(route[0])} 도착 · {self.IDLE_RETURN:.0f}초 안에 주문 없으면 주방 복귀')
                    threading.Thread(target=self._auto_return, args=(job,), daemon=True).start()
                else:
                    self.st['route'], self.st['done'] = [], []
                    self._emit('idle', '', '주방 도착 · 주문 대기 중')
            else:
                self._home('배달 끝 · 주방으로 복귀 중' if ok else '중간에 실패 · 남은 테이블은 건너뛰고 주방으로 복귀 중')
        finally:
            with self.lock:
                self.busy = False

    def _auto_return(self, job):
        time.sleep(self.IDLE_RETURN)
        with self.lock:
            if self.job != job or self.busy:
                return   # 그 사이 새 주문 · 정지가 왔음
            self.busy = True
        try:
            self.stop = False
            self._home('추가 주문 없음 → 주방으로 복귀 중')
        finally:
            with self.lock:
                self.busy = False


# ─────────────────────────────────────────────────────────────
# 3. 웹 서버 (Flask)
# ─────────────────────────────────────────────────────────────
hub = Hub()
orders = Orders(hub)
link = None
app = Flask(__name__, static_folder=os.path.join(HERE, 'static'), static_url_path='/static')


@app.route('/')
def page():
    return send_from_directory(app.static_folder, 'index.html')


# 손님 화면
@app.route('/order')
def page_customer():
    return send_from_directory(app.static_folder, 'customer.html')


# 직원 폰 화면이 계속 물어보는 곳 — 로봇 상태 · 연결 · 주문 목록 · 최근 기록
@app.route('/api/state')
def api_state():
    status, age, log = hub.snapshot()
    return jsonify(
        mode=link.name,                       # 'ros' 진짜 로봇 / 'demo' 연습
        manager=link.listeners() > 0,         # 매니저가 주문을 듣고 있나
        live=age < STATUS_FRESH_SEC,          # 상태가 최근에 왔나 (v4 · v5 매니저여야 옴)
        status=status, tables=TABLES, log=log[:12], orders=orders.active(),
    )


# 손님 화면이 계속 물어보는 곳 — 메뉴 · 내 테이블 주문 · 로봇이 어디로 가는 중인지
@app.route('/api/my')
def api_my():
    table = request.args.get('table', '')
    status, age, _ = hub.snapshot()
    live = age < STATUS_FRESH_SEC
    return jsonify(
        menu=MENU, tables=TABLES, orders=orders.of_table(table),
        robot={'state': status.get('state') if live else 'unknown', 'target': status.get('target', '')},
    )


# 손님이 음식 주문 — {"table": "3", "cart": {"kimchi": 1, "cola": 2}}   ※ 로봇은 아직 안 움직임
@app.route('/api/menu-order', methods=['POST'])
def api_menu_order():
    data = request.get_json(silent=True) or {}
    table = str(data.get('table', '')).strip()
    if table not in TABLES:
        return jsonify(ok=False, msg='테이블 번호를 골라 주세요'), 400
    try:
        order = orders.add(table, data.get('cart') or {})
    except (TypeError, ValueError):
        order = None
    if order is None:
        return jsonify(ok=False, msg='메뉴를 하나 이상 담아 주세요'), 400
    return jsonify(ok=True, msg='주문이 들어갔어요', order=order)


# 손님이 "받았어요" — {"id": 3}
@app.route('/api/received', methods=['POST'])
def api_received():
    data = request.get_json(silent=True) or {}
    o = orders.set_state(data.get('id'), 'done', only_from=('arrived',))
    if o is None:
        return jsonify(ok=False, msg='아직 도착하지 않은 주문이에요'), 400
    hub.add_log('good', f"{o['table']}번 테이블 손님이 음식을 받음")
    return jsonify(ok=True, msg='맛있게 드세요!')


# 직원이 주문을 목록에서 지움 — {"id": 3}
@app.route('/api/order-cancel', methods=['POST'])
def api_order_cancel():
    data = request.get_json(silent=True) or {}
    o = orders.set_state(data.get('id'), 'cancel', only_from=('waiting', 'arrived'))
    if o is None:
        return jsonify(ok=False, msg='지울 수 없는 주문이에요 (배달 중)'), 400
    hub.add_log('info', f"{o['table']}번 테이블 주문을 목록에서 지움")
    return jsonify(ok=True, msg='목록에서 지웠어요')


# 주문 보내기 — {"route": ["3", "1"]}
@app.route('/api/order', methods=['POST'])
def api_order():
    data = request.get_json(silent=True) or {}
    route = [str(t).strip() for t in data.get('route', [])]

    if not route:
        return jsonify(ok=False, msg='테이블을 하나 이상 골라 주세요'), 400
    if any(t not in TABLES + ['0'] for t in route):
        return jsonify(ok=False, msg='없는 테이블 번호가 있어요'), 400
    if len(set(route)) != len(route):
        return jsonify(ok=False, msg='같은 테이블을 두 번 넣었어요'), 400
    if '0' in route and len(route) > 1:
        return jsonify(ok=False, msg='주방(0)은 혼자만 보낼 수 있어요'), 400
    if link.listeners() == 0:
        return jsonify(ok=False, msg='매니저가 꺼져 있어요 (ros2 run … manager 를 먼저 켜세요)'), 503

    status, age, _ = hub.snapshot()
    if age < STATUS_FRESH_SEC and status.get('state') in BUSY_STATES:
        return jsonify(ok=False, msg='로봇이 이동 중이에요. 끝나거나 정지한 뒤에 보내세요'), 409

    text = ','.join(route)
    link.send(text)
    orders.mark_sent(route)   # 그 테이블들의 준비 중 주문을 "배달 중" 으로
    if route == ['0']:
        label = '주방 복귀'
    elif len(route) == 1:
        label = f'모드 2 · 단일 → {route[0]}번'
    elif route == TABLES:
        label = '모드 1 · 전체 → ' + ' → '.join(route)
    else:
        label = '모드 3 · 골라서 → ' + ' → '.join(route)
    hub.add_log('order', f'주문 보냄: {label}')
    return jsonify(ok=True, msg=f'보냈어요 · {label}', sent=text)


# 긴급 정지 — 언제든 받음
@app.route('/api/stop', methods=['POST'])
def api_stop():
    link.send('STOP')
    hub.add_log('bad', '🚨 긴급 정지 보냄')
    return jsonify(ok=True, msg='긴급 정지를 보냈어요')


def main():
    global link
    demo = '--demo' in sys.argv
    port = PORT
    if '--port' in sys.argv:   # 예: python3 web_order.py --port 5050  (맥은 5000 을 다른 프로그램이 씀)
        port = int(sys.argv[sys.argv.index('--port') + 1])
    if not demo:
        try:
            link = RosLink(hub)
        except ImportError:
            print('[알림] rclpy 가 없어서 연습 모드로 켭니다 (로봇과 연결 안 됨)')
            demo = True
    if demo:
        link = DemoLink(hub)

    print(f'\n  서빙봇 주방 패널  —  {"연습 모드 (로봇 없음)" if demo else "진짜 로봇 모드"}')
    print(f'  이 노트북에서:  http://localhost:{port}')
    print(f'  직원 폰:       http://<노트북 IP>:{port}/        (IP 는 hostname -I)')
    print(f'  손님 폰:       http://<노트북 IP>:{port}/order\n')
    # host='0.0.0.0' = 같은 와이파이의 다른 기기(폰)도 들어올 수 있게
    app.run(host='0.0.0.0', port=port, threaded=True, debug=False)


if __name__ == '__main__':
    main()
