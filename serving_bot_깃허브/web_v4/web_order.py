#!/usr/bin/env python3
# 폰 웹 주문 서버 — 서빙봇 주방 패널 (1008 v5 · 테이블 6개 · 실물 테스트 전 · Claude 가 쓴 것)
#
# 화면 두 개 (같은 서버)
#   손님 폰   http://<노트북 IP>:5000/order   메뉴를 골라 주문 → 주방 목록에 쌓임 (로봇은 아직 안 움직임)
#   직원 폰   http://<노트북 IP>:5000/        주문 목록을 보고 테이블을 골라 로봇을 보냄
#
# 하는 일
#   폰 화면에서 버튼을 누름  →  이 서버가 /delivery_order 토픽으로 주문 글자를 보냄  →  매니저가 로봇을 움직임
#   매니저가 /delivery_status 로 알려 주는 상태  →  이 서버가 받아 둠  →  폰 화면이 1초에 두세 번 물어봐서 보여 줌
#   [추가 1013] 실시간 위치 · 진행 상황 — 매니저를 안 고쳐도 보이게
#     로봇 위치   TF(map → base_footprint)를 이 서버가 직접 읽음 (1초에 5번)
#     진행 상황   매니저가 터미널에 찍는 글자(/rosout)를 읽어 '이동 시작 요청' · '도달 완료' 로 상태를 만듦
#                 (매니저가 /delivery_status 를 내보내면 그쪽을 먼저 씀)
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
import heapq
import json
import math
import os
import random
import re
import sys
import threading
import time

from flask import Flask, jsonify, request, send_from_directory

PORT = 5000
TABLES = ['1', '2', '3', '4', '5', '6']  # 화면에 보여 줄 테이블 번호 (10/8 테이블 6개 맵)
BUSY_STATES = ('moving', 'arrived', 'returning')  # 이 상태일 때는 새 주문을 받지 않음 (매니저도 거절함)
STATUS_FRESH_SEC = 3.0                   # 이 시간 안에 상태가 왔으면 "매니저 켜짐" 으로 봄
POSE_FRESH_SEC = 3.0                     # 이 시간 안에 온 좌표만 화면에 보냄

# 자리의 지도 좌표 (m) — 화면이 "지도 좌표 → 맵 그림 위치" 를 맞추는 기준점. '0' = 주방
#   처음 값은 10/8 깃허브 robot5 좌표. 매니저가 '이동 시작 요청: 3번 테이블 (2.90, 1.61)' 을 찍으면 그 값으로 저절로 바뀜
SPOTS_XY = {'0': (0.25, 0.14), '1': (1.69, 0.05), '2': (2.88, 0.34), '3': (2.90, 1.61),
            '4': (1.93, 1.63), '5': (1.07, 1.71), '6': (0.09, 1.46)}

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
        self.mgr_time = 0.0      # [추가 1013] 매니저가 /delivery_status 로 직접 알려 준 마지막 시각
        self.pose = None         # [추가 1013] 로봇 지도 좌표 (x, y)
        self.pose_time = 0.0
        self.spots = dict(SPOTS_XY)  # [추가 1013] 자리의 지도 좌표
        self.mgr_idle = True         # [추가 1013 v4] 매니저 로그의 좌표 줄이 '주문 대기 중' 인가 (도착 후 대기가 끝났나)

    def add_log(self, kind, text):
        with self.lock:
            self.log.insert(0, {'t': time.strftime('%H:%M:%S'), 'kind': kind, 'text': text})
            del self.log[30:]

    # [추가 1013] 매니저가 /delivery_status 를 직접 내보내는 중인가
    def manager_live(self):
        return time.time() - self.mgr_time < STATUS_FRESH_SEC

    # [추가 1013] 로봇 좌표가 올 때마다 불림
    def on_pose(self, x, y):
        with self.lock:
            self.pose = (x, y)
            self.pose_time = time.time()

    # [추가 1013] 매니저가 찍은 목표 좌표로 자리 좌표를 고침
    def learn_spot(self, t, x, y):
        with self.lock:
            self.spots[t] = (x, y)

    # [추가 1013] (최근 좌표 또는 None, 자리 좌표)
    def where(self):
        with self.lock:
            fresh = self.pose is not None and time.time() - self.pose_time < POSE_FRESH_SEC
            return (self.pose if fresh else None), dict(self.spots)

    # 매니저(또는 연습용 가짜 로봇)가 상태를 알려 줄 때마다 불림
    #   from_manager=False → 매니저 로그를 읽어 이 서버가 만든 상태 (LogWatch)
    def on_status(self, data, from_manager=True):
        with self.lock:
            old = (self.status.get('state'), self.status.get('target'))
            self.status = data
            self.status_time = time.time()
            if from_manager:
                self.mgr_time = self.status_time
            changed = old != (data.get('state'), data.get('target'))
            xy = self.pose if time.time() - self.pose_time < POSE_FRESH_SEC else None
        if changed and data.get('msg'):
            kind = {'failed': 'bad', 'stopped': 'bad', 'arrived': 'good', 'hold': 'good'}.get(data.get('state'), 'info')
            if xy is None and data.get('x') is not None:
                xy = (data['x'], data['y'])
            # [추가 1013] 상태가 바뀐 순간의 좌표를 기록에 같이 남김 (1초마다 오는 좌표는 위쪽 X · Y 숫자로만 보여 줌)
            where = f'  (X {xy[0]:.2f} · Y {xy[1]:.2f})' if xy else ''
            self.add_log(kind, data['msg'] + where)
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
# 1-C. [추가 1013] 매니저 로그 읽기 — 매니저가 터미널에 찍는 글자로 진행 상황을 만듦
#   매니저가 /delivery_status 를 안 내보내도(깃허브 robot5 판) 화면에 상태 · 도착이 보이게 하려는 것
#   읽는 글자:  '>> 이동 시작 요청: 3번 테이블 (2.90, 1.61)'   → 가는 중 + 그 자리 좌표를 배움
#              '>> 도달 완료: 3번 테이블'                      → 도착
#              '[… 주행 중] 현재 좌표 -> X: 1.23 m, Y: 0.45 m'  → 좌표 (TF 를 못 읽을 때만 씀)
#   매니저의 이 글자들을 바꾸면 아래 RE_ 세 줄도 같이 고쳐야 함
# ─────────────────────────────────────────────────────────────
class LogWatch:
    NODE = 'delivery_manager_node'   # 매니저 노드 이름
    RE_XY = re.compile(r'X:\s*(-?\d+(?:\.\d+)?)\s*m?\s*,\s*Y:\s*(-?\d+(?:\.\d+)?)')
    RE_GO = re.compile(r'이동 시작 요청:\s*(.+?)\s*\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)')
    RE_OK = re.compile(r'도달 완료:\s*(.+)')

    def __init__(self, hub):
        self.hub = hub
        self.lock = threading.Lock()
        # synth=True → 이 서버가 만든 상태라는 표시 (이때는 "이동 중이라 주문 못 받음" 으로 막지 않음 · 거절은 매니저가 함)
        self.st = {'state': 'idle', 'target': '', 'route': [], 'done': [], 'msg': '주문 대기 중', 'synth': True}
        self.idle_lines = 0   # 가는 중인데 매니저가 '주문 대기 중' 이라고 찍은 횟수

    @staticmethod
    def _spot(name):   # '3번 테이블' → '3' · '주방(kitchen)' → '0'
        if '주방' in name or 'kitchen' in name:
            return '0'
        m = re.search(r'\d+', name)
        return m.group() if m else None

    def _set(self, state, target, msg):
        self.st.update(state=state, target=target, msg=msg)
        self._emit()

    def _emit(self):
        data = dict(self.st)
        data['route'], data['done'] = list(self.st['route']), list(self.st['done'])
        self.hub.on_status(data, from_manager=False)

    # 웹에서 주문을 보낼 때 불림 — 경로 칩(주방 → 3 → 1 → 주방)을 그리려고 기억해 둠
    def set_route(self, route):
        with self.lock:
            self.st['route'] = [] if route == ['0'] else list(route)
            self.st['done'] = []

    # 매니저가 한 줄 찍을 때마다 불림 (level: 20 정보 · 30 주의 · 40 오류)
    def feed(self, name, level, text):
        if name != self.NODE:
            return
        text = text.strip()
        if '현재 좌표' in text:   # 1초마다 오는 좌표 줄 — 기록에는 안 남김
            m = self.RE_XY.search(text)
            if m and time.time() - self.hub.pose_time > 1.5:
                self.hub.on_pose(float(m.group(1)), float(m.group(2)))
            self.hub.mgr_idle = '주문 대기 중' in text   # 매니저가 지금 주문을 받을 수 있나 (자동 모드가 봄)
            if not self.hub.manager_live():
                self._beat('주문 대기 중' in text)
            return

        go, target = self.RE_GO.search(text), None
        if go:
            target = self._spot(go.group(1))
            if target:
                self.hub.learn_spot(target, float(go.group(2)), float(go.group(3)))
        if self.hub.manager_live():
            return   # 매니저가 상태를 직접 알려 주는 중 → 그쪽을 씀

        ok = self.RE_OK.search(text)
        with self.lock:
            st = self.st
            self.idle_lines = 0
            # 주방이 경로 중간에 들어 있고 아직 안 들렀으면 "들르는 곳", 아니면 "복귀"
            via = lambda t: t != '0' or ('0' in st['route'][:-1] and '0' not in st['done'])
            if go and target:
                if target != '0' and target not in st['route']:   # 웹이 아닌 곳(주문 콘솔)에서 온 주문
                    st['route'], st['done'] = [target], []
                where = '주방' if target == '0' else f'{target}번 테이블'
                self._set('moving' if via(target) else 'returning', target, f'{where}(으)로 가는 중')
            elif ok and self._spot(ok.group(1)):
                target = self._spot(ok.group(1))
                if not via(target):
                    st['route'], st['done'] = [], []
                    self._set('idle', '', '주방 도착 · 주문 대기 중')
                else:
                    if target not in st['done']:
                        st['done'].append(target)
                    self._set('arrived', target, '주방 도착' if target == '0' else f'{target}번 테이블 도착')
            elif '긴급 정지' in text:
                self._set('stopped', st['target'], '긴급 정지 · 그 자리에 멈춤')
            elif level >= 40 and st['state'] in ('moving', 'returning'):
                self._set('failed', st['target'], text.lstrip('> '))
            else:
                self.hub.add_log('bad' if level >= 40 else 'info', text.lstrip('> '))

    # 좌표 줄이 올 때마다(1초) 상태를 다시 알림 → 화면이 "연결됨" 으로 유지됨
    def _beat(self, idle_line):
        with self.lock:
            if idle_line and self.st['state'] == 'moving':
                # 매니저는 대기 중이라는데 도달 완료가 안 왔음 → 못 간 것
                self.idle_lines += 1
                if self.idle_lines >= 2:
                    self.idle_lines = 0
                    self._set('failed', self.st['target'], '도착하지 못하고 멈췄어요 (매니저 창을 확인하세요)')
                    return
            else:
                self.idle_lines = 0
            self._emit()


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
        self.watch = LogWatch(hub)
        # [추가 1013] 아래 둘은 실패해도 주문 보내기는 그대로 되게 따로 감쌈
        try:   # 매니저 로그 (/rosout)
            from rcl_interfaces.msg import Log
            self.node.create_subscription(Log, '/rosout', self._on_log, 100)
        except Exception as e:
            print(f'[알림] 매니저 로그를 못 읽습니다 (진행 상황 표시 안 됨): {e}')
        try:   # 로봇 위치 (TF) — 1초에 5번
            from tf2_ros import Buffer, TransformListener
            self.Time = rclpy.time.Time
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
            self.node.create_timer(0.2, self._on_tf)
        except Exception as e:
            print(f'[알림] 로봇 위치(TF)를 못 읽습니다 (매니저 로그의 좌표를 대신 씀): {e}')
        # spin = 들어오는 메시지를 계속 받는 무한 반복 → 웹 서버와 따로 돌도록 다른 실(thread)에서
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _on_msg(self, msg):
        try:
            self.hub.on_status(json.loads(msg.data))
        except ValueError:
            pass  # JSON 이 아니면 무시

    def _on_log(self, msg):
        try:
            self.watch.feed(msg.name, msg.level, msg.msg)
        except Exception:
            pass  # 로그 한 줄 때문에 서버가 멈추지 않게

    def _on_tf(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', self.Time())
            self.hub.on_pose(t.transform.translation.x, t.transform.translation.y)
        except Exception:
            pass  # 아직 위치를 모름 (Nav2 켜기 전 등)

    # 웹에서 주문을 보낸 직후 불림
    def ordered(self, route):
        self.watch.set_route(route)

    # [추가 1013 v4] 매니저의 "도착 후 대기 시간" 을 바꿈 (자동 모드가 0 으로 · 끌 때 원래 값으로). 결과는 done(성공 여부) 로 알려 줌
    #   매니저에 파라미터 serve_wait_sec 가 있어야 됨 (web_v4/매니저_자동모드0초/ 판). 없는 매니저면 실패로 돌아오고 아무 일도 안 생김
    def set_wait(self, sec, done):
        try:
            from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
            from rcl_interfaces.srv import SetParameters
            if not hasattr(self, 'param_cli'):
                self.param_cli = self.node.create_client(SetParameters, f'/{LogWatch.NODE}/set_parameters')
            if not self.param_cli.service_is_ready():
                return done(False)
            req = SetParameters.Request()
            req.parameters = [Parameter(name='serve_wait_sec', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(sec)))]
            fut = self.param_cli.call_async(req)
            fut.add_done_callback(lambda f: done(bool(f.exception() is None and f.result().results and f.result().results[0].successful)))
        except Exception:
            done(False)

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
    SPOTS = SPOTS_XY     # 주문 글자에 쓸 수 있는 자리 번호
    # [수정 1013] 가짜 로봇은 맵 그림 좌표로 움직임 (그림의 벽을 피해 가려고). 아래 세 값은 static/index.html 의 그림과 같아야 함
    PIX = {'0': (440, 720), '1': (490, 401), '2': (440, 170), '3': (170, 160),
           '4': (150, 348), '5': (150, 518), '6': (185, 710)}          # 자리 (index.html 의 SPOT)
    WALLS = [(300, 610, 16, 180), (70, 594, 90, 16), (70, 264, 90, 16), (466, 508, 90, 16), (466, 278, 90, 16),
             (280, 70, 16, 90), (298, 260, 16, 80), (190, 415, 14, 30), (358, 415, 14, 30)]   # 벽 (x, y, 너비, 높이)
    ROOM = (70, 70, 556, 790)   # 방 (왼쪽, 위, 오른쪽, 아래)
    CELL, GAP = 10, 20   # 길 찾기 칸 크기 · 벽에서 띄울 거리 (그림 좌표)
    SPEED_PX = 120       # 가짜 로봇 속도 (그림 좌표/초) — 실물보다 빠름
    SERVE_WAIT = 3.0     # 테이블 도착 후 대기
    IDLE_RETURN = 10.0   # 모드 2 도착 후 주문 대기

    def __init__(self, hub):
        self.hub = hub
        self.lock = threading.Lock()
        self.busy = False
        self.stop = False
        self.job = 0       # 주문 번호 (새 주문이 오면 옛 자동 복귀를 무시하려고)
        self.xy = list(self.PIX['0'])   # 가짜 로봇 위치 (맵 그림 좌표)
        self.st ={'state': 'idle', 'target': '', 'route': [], 'done': [], 'msg': '주문 대기 중 (연습 모드)'}
        # 실패 화면을 보고 싶으면:  SERVING_DEMO_FAIL=3 python3 web_order.py --demo
        self.fail = set(os.environ.get('SERVING_DEMO_FAIL', '').split(',')) - {''}
        threading.Thread(target=self._beat, daemon=True).start()

    def listeners(self):
        return 1

    def ordered(self, route):
        pass

    def set_wait(self, sec, done):   # 가짜 로봇의 도착 후 대기 시간
        self.SERVE_WAIT = float(sec)
        done(True)

    def _emit(self, state=None, target='', msg=''):
        if state is not None:
            self.st.update(state=state, target=target, msg=msg)
        # 화면에 보여 줄 X · Y (m) 는 그림 좌표를 어림으로 바꾼 값. px · py = 그림 좌표 (화면이 그대로 씀)
        x, y = (720 - self.xy[1]) / 209 + 0.25, (440 - self.xy[0]) / 184 + 0.14
        self.hub.on_pose(x, y)
        data = dict(self.st, x=round(x, 2), y=round(y, 2), px=round(self.xy[0], 1), py=round(self.xy[1], 1))
        data['route'], data['done'] = list(self.st['route']), list(self.st['done'])
        self.hub.on_status(data)

    def _beat(self):   # 매니저처럼 1초마다 상태를 다시 알림
        while True:
            time.sleep(1.0)
            self._emit()

    def _name(self, t):
        return '주방(0번)' if t == '0' else f'{t}번 테이블'

    # [추가 1013] 가짜 로봇이 벽을 뚫고 가지 않게 — 맵 그림 위에서 벽을 피해 가장 짧은 길을 찾음 (연습 모드 전용)
    #   진짜 로봇은 Nav2 가 간 길의 좌표가 그대로 화면에 나옴 (이 부분과 상관없음)
    def _free(self, x, y):   # 그 점이 벽 · 바깥에서 GAP 만큼 떨어져 있나
        g = self.GAP
        x0, y0, x1, y1 = self.ROOM
        if not (x0 + g <= x <= x1 - g and y0 + g <= y <= y1 - g):
            return False
        return not any(wx - g < x < wx + w + g and wy - g < y < wy + h + g for wx, wy, w, h in self.WALLS)

    def _clear(self, a, b):  # a 에서 b 까지 곧게 가도 벽에 안 닿나
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 4))
        return all(self._free(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1))

    def _path(self, goal):   # 지금 자리 → goal 까지 꺾이는 점들
        c = self.CELL
        start = (round(self.xy[0] / c), round(self.xy[1] / c))
        end = (round(goal[0] / c), round(goal[1] / c))
        dist, prev, heap = {start: 0.0}, {}, [(0.0, start)]
        while heap:   # 칸 단위로 가장 짧은 길 찾기 (다익스트라)
            d, cur = heapq.heappop(heap)
            if cur == end:
                break
            if d > dist[cur]:
                continue
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nxt = (cur[0] + dx, cur[1] + dy)
                    if nxt == cur or not self._free(nxt[0] * c, nxt[1] * c):
                        continue
                    nd = d + math.hypot(dx, dy)
                    if nd < dist.get(nxt, 1e9):
                        dist[nxt], prev[nxt] = nd, cur
                        heapq.heappush(heap, (nd, nxt))
        if end not in prev:
            return [goal]    # 길을 못 찾으면 곧장
        cells = [end]
        while cells[-1] != start:
            cells.append(prev[cells[-1]])
        pts = [tuple(self.xy)] + [(i * c, j * c) for i, j in reversed(cells)] + [goal]
        out, i = [], 0       # 계단 모양 길을 곧게 펴기 — 벽에 안 닿는 가장 먼 점으로 건너뜀
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._clear(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def _go(self, t):  # 목적지까지 조금씩 이동. 정지되면 False
        t0 = time.time()
        for tx, ty in self._path(self.PIX[t]):
            while True:
                if self.stop:
                    return False
                if t in self.fail and time.time() - t0 > 4.0:
                    return False
                dx, dy = tx - self.xy[0], ty - self.xy[1]
                d = (dx * dx + dy * dy) ** 0.5
                if d < 1.0:
                    break
                step = min(d, self.SPEED_PX * 0.1)
                self.xy[0] += dx / d * step
                self.xy[1] += dy / d * step
                self._emit()
                time.sleep(0.1)
        return True

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
# 2-C. [추가 1013 v4] 자동 모드 — 손님이 실시간으로 주문했다고 가정하고, 테이블을 하나씩 랜덤으로 골라 보냄
#   생각(다음 테이블 고르기) → 한 곳으로 출발 → 도착 → 다시 생각 → …  (목적지는 항상 1곳 · 방금 간 곳은 안 고름)
#   실패 · 긴급 정지가 나오면 저절로 꺼짐. 꺼도 가던 배달은 끝까지 감
# ─────────────────────────────────────────────────────────────
class Auto:
    THINK_SEC = 0.0      # 도착 후 다음 주문까지 일부러 두는 시간 — 0 = 딜레이 없이 바로 (10/13 연성 요청)
                         #   단, 매니저가 도착 후 대기(SERVE_WAIT_SEC · 3초) 동안은 새 주문을 안 받으므로, 매니저가 '주문 대기 중' 이 되는 순간 보냄
                         #   값을 늘릴 때는 매니저의 자동 주방 복귀(10초)보다 짧게
    START_SEC = 4.0      # 주문을 보내고 이 시간 안에 출발을 안 하면(매니저가 거절 등) 다시 고름
    GIVE_UP_SEC = 150.0  # 출발한 뒤 이 시간 안에 도착을 못 하면 자동 모드를 끔
    MANAGER_WAIT_SEC = 3.0   # 자동 모드를 끌 때 매니저에 되돌려 놓을 "도착 후 대기 시간" (매니저의 SERVE_WAIT_SEC 와 같은 값)

    def __init__(self, hub):
        self.hub = hub
        self.lock = threading.Lock()
        self.on = False
        self.phase = 'off'     # think 생각 중 · go 가는 중 · off 꺼짐
        self.target = ''
        self.last = ''         # 방금 간 테이블 (연달아 같은 곳은 안 고름)
        self.count = 0         # 이번 자동 모드에서 끝낸 배달 수
        self.next_at = 0.0     # 다음 주문을 보낼 시각
        self.sent_at = 0.0
        self.moving = False    # 보낸 주문으로 로봇이 실제로 출발했나
        self.note = ''         # 꺼진 이유 (화면에 한 번 보여 줌)
        self.fast = False      # 매니저의 도착 후 대기를 0초로 바꾸는 데 성공했나
        self.arrived_at = 0.0
        threading.Thread(target=self._loop, daemon=True).start()

    def _on_fast(self, ok):
        self.fast = ok
        self.hub.add_log('info', '매니저의 도착 후 대기를 0초로 바꿈 (자동 모드 동안만)' if ok
                         else '매니저가 대기 시간 바꾸기를 지원하지 않음 → 도착 후 대기(3초)는 그대로')

    def start(self):
        with self.lock:
            self.on, self.phase, self.count, self.note, self.fast = True, 'think', 0, '', False
            self.next_at = time.time() + 1.5
        self.hub.add_log('order', '🎲 자동 모드 시작 — 손님 주문을 가정해 한 곳씩 랜덤으로 보냄')
        link.set_wait(0.0, self._on_fast)   # 도착하면 바로 다음 곳으로 가게 (테스트용 모드라서)

    def stop(self, why=''):
        with self.lock:
            if not self.on:
                return
            self.on, self.phase, self.note, n = False, 'off', why, self.count
        if self.fast:
            link.set_wait(self.MANAGER_WAIT_SEC, lambda ok: self.hub.add_log('info' if ok else 'bad',
                          '매니저의 도착 후 대기를 3초로 되돌림' if ok else '매니저의 도착 후 대기를 되돌리지 못함 → 매니저를 다시 켜 주세요'))
            self.fast = False
        self.hub.add_log('info', f'자동 모드 끝 (배달 {n}건)' + (f' — {why}' if why else ''))

    def info(self):
        with self.lock:
            left = max(0, math.ceil(self.next_at - time.time())) if self.phase == 'think' else 0
            return dict(on=self.on, phase=self.phase, target=self.target, left=left, count=self.count, note=self.note)

    def _loop(self):
        while True:
            time.sleep(0.3)
            try:
                self._step()
            except Exception:
                pass   # 한 번 어긋나도 서버는 계속 돌게

    def _step(self):
        if not self.on or link is None:
            return
        status, age, _ = self.hub.snapshot()
        live, st, now = age < STATUS_FRESH_SEC, status.get('state'), time.time()
        if live and st in ('failed', 'stopped'):
            self.stop('로봇이 멈춰서 자동 모드를 껐어요')
            return
        if self.phase == 'think':
            if now < self.next_at or not live or link.listeners() == 0:
                return
            # 로봇이 주문을 받을 수 있는 상태일 때만 보냄 (synth = 매니저 로그로 만든 상태 → 'arrived' 가 마지막 상태)
            #   매니저가 '주문 대기 중' 이라고 찍었거나, 대기를 0초로 바꿨으면 도착 0.4초 뒤(매니저가 마무리할 틈)
            free = self.hub.mgr_idle or (self.fast and now - self.arrived_at > 0.4)
            if not (st in ('idle', 'hold') or (st == 'arrived' and status.get('synth') and free)):
                return
            t = random.choice([x for x in TABLES if x != self.last])
            link.ordered([t])
            link.send(t)
            with self.lock:
                self.target, self.phase, self.sent_at, self.moving = t, 'go', now, False
            self.hub.add_log('order', f'🔔 {t}번 테이블 손님 주문 (자동) → 로봇 출발')
        elif self.phase == 'go':
            mine = live and status.get('target') == self.target
            if mine and st == 'moving':
                self.moving = True
            if mine and st in ('arrived', 'hold'):
                with self.lock:
                    self.count += 1
                    self.last, self.phase, self.next_at, self.arrived_at = self.target, 'think', now + self.THINK_SEC, now
            elif not self.moving and now - self.sent_at > self.START_SEC:
                with self.lock:   # 출발을 안 함 → 잠깐 뒤 다시 고름
                    self.phase, self.next_at = 'think', now + 1.0
            elif now - self.sent_at > self.GIVE_UP_SEC:
                self.stop('오래 도착하지 못해 자동 모드를 껐어요')


# ─────────────────────────────────────────────────────────────
# 3. 웹 서버 (Flask)
# ─────────────────────────────────────────────────────────────
hub = Hub()
orders = Orders(hub)
link = None
auto = Auto(hub)
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
    live = age < STATUS_FRESH_SEC
    pose, spots = hub.where()
    if pose is None and live and status.get('x') is not None:
        pose = (status['x'], status['y'])     # TF 를 못 읽으면 매니저가 알려 준 좌표
    return jsonify(
        mode=link.name,                       # 'ros' 진짜 로봇 / 'demo' 연습
        manager=link.listeners() > 0,         # 매니저가 주문을 듣고 있나
        live=live,                            # 상태가 최근에 왔나 (매니저가 직접 알려 주거나, 매니저 로그로 만든 것)
        status=status, tables=TABLES, log=log[:12], orders=orders.active(),
        pose=None if pose is None else {'x': round(pose[0], 3), 'y': round(pose[1], 3)},   # 로봇 지도 좌표 (m)
        spots=spots,                          # 자리의 지도 좌표 — 화면이 맵 그림에 맞출 때 씀
        auto=auto.info(),                     # 자동 모드 (켜짐 · 생각 중/가는 중 · 목적지 · 남은 초 · 끝낸 배달 수)
    )


# 자동 모드 켜기 · 끄기 — {"on": true}
@app.route('/api/auto', methods=['POST'])
def api_auto():
    data = request.get_json(silent=True) or {}
    if data.get('on'):
        if link.listeners() == 0:
            return jsonify(ok=False, msg='매니저가 꺼져 있어요 (자동 모드를 켤 수 없어요)'), 503
        auto.start()
        return jsonify(ok=True, msg='자동 모드 시작 · 한 곳씩 랜덤으로 보내요')
    auto.stop('직원이 끔')
    return jsonify(ok=True, msg='자동 모드를 껐어요 (가던 배달은 끝까지 가요)')


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
    # [수정 1013] 주방(0)도 경로 중간에 넣을 수 있음 — 예: 1 → 주방 → 3 (주방에 들러 다음 음식을 싣기)
    if link.listeners() == 0:
        return jsonify(ok=False, msg='매니저가 꺼져 있어요 (ros2 run … manager 를 먼저 켜세요)'), 503
    if auto.on:
        return jsonify(ok=False, msg='자동 모드 중이에요. 자동 모드를 끈 뒤에 보내세요'), 409

    status, age, _ = hub.snapshot()
    # 매니저 로그로 만든 상태(synth)일 때는 여기서 막지 않음 — 바쁘면 매니저가 거절함
    if age < STATUS_FRESH_SEC and status.get('state') in BUSY_STATES and not status.get('synth'):
        return jsonify(ok=False, msg='로봇이 이동 중이에요. 끝나거나 정지한 뒤에 보내세요'), 409

    text = ','.join(route)
    link.ordered(route)
    link.send(text)
    orders.mark_sent(route)   # 그 테이블들의 준비 중 주문을 "배달 중" 으로
    names = ' → '.join('주방' if t == '0' else t for t in route)
    if route == ['0']:
        label = '주방 복귀'
    elif len(route) == 1:
        label = f'모드 2 · 단일 → {route[0]}번'
    elif route == TABLES:
        label = '모드 1 · 전체 → ' + names
    else:
        label = '모드 3 · 골라서 → ' + names
    hub.add_log('order', f'주문 보냄: {label}')
    return jsonify(ok=True, msg=f'보냈어요 · {label}', sent=text)


# 긴급 정지 — 언제든 받음
@app.route('/api/stop', methods=['POST'])
def api_stop():
    auto.stop('긴급 정지')   # 자동 모드도 같이 끔
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
