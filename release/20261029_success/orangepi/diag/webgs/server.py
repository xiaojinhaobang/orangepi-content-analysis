#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D题 · 单板双面盘点 · **网页版地面站**（零依赖：只要 Python 3，不用装 Qt）。

给队友用：在一台电脑上跑起来，同网段的手机/电脑用浏览器打开即可看盘点结果。

    python server.py                 # 默认 0.0.0.0:8080，机载 IP 172.25.105.210
    python server.py --port 8080 --drone-ip 172.25.105.210

页面：http://<本机IP>:8080
链路：与香橙派 gs_bridge 走同一套 UDP 帧协议（默认 8888），协议与桌面版一致。

⚠ 协议端口 8888 是**独占**的：网页版和桌面版不要同时开（只能有一个绑 8888）。

HTTP API:
  GET  /api/state                    全部状态快照（页面每 0.5s 拉一次）
  POST /api/connect  {ip, port}      连机载（UDP）
  POST /api/disconnect               断开
  POST /api/start    {mode:0}        下发 START（0=遍历盘点）
  POST /api/query    {num}           按货号查坐标
  POST /api/reset                    清空 + 复位
  POST /api/sim      {on:true, interval_ms}   内置模拟（离线演示/没飞机时用）
"""
import argparse
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import layout
import sim
from protocol import (
    FACE_MAX_POS, FACE_NAMES, FrameParser, TYPE_EVENT, TYPE_REPORT, TYPE_ROUTE,
    TYPE_START, TYPE_STATUS, pack_frame, parse_event, parse_report, parse_status,
    pos_name,
)

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, 'static')
TOTAL_CELLS = len(FACE_NAMES) * FACE_MAX_POS
STAGES = ["准备", "起飞", "B面扫描", "换面", "A面扫描", "结果核对", "降落", "完成"]


class Store:
    """全局状态（多线程访问，统一加锁）。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.cells = {}           # (face, pos) -> 货号
        self.dstate = 0           # 0就绪 1起飞中 2飞行中 3降落中 4已落地
        self.height = 0
        self.battery = 100
        self.fc = None            # (模式, 解锁)，ROS 模式下从 /fc_status 拿
        self.report = None
        self.frames = 0
        self.errors = 0
        self.connected = False
        self.mode = '--'          # udp / sim
        self.target = None
        self.last_rx = 0.0
        self.log = []

    # ---- 更新 ----
    def add_log(self, msg):
        with self.lock:
            self.log.append('%s  %s' % (time.strftime('%H:%M:%S'), msg))
            del self.log[:-200]

    def add_event(self, num, face, pos, ts):
        name = pos_name(face, pos)
        with self.lock:
            first = (face, pos) not in self.cells
            self.cells[(face, pos)] = num
        ref = layout.reference_number(FACE_NAMES[face], pos)
        mark = '' if ref is None else ('（与参考一致 ✓）' if ref == num else '（≠ 参考 %s）' % ref)
        self.add_log('%s盘点：编号 %d → %s %s'
                     % ('首次' if first else '重复', num, name, mark))

    def apply_frame(self, typ, payload):
        self.frames += 1
        self.last_rx = time.time()
        if typ == TYPE_EVENT:
            d = parse_event(payload)
            self.add_event(d['num'], d['face'], d['pos'], d['ts'])
        elif typ == TYPE_STATUS:
            d = parse_status(payload)
            with self.lock:
                self.dstate, self.height, self.battery = d['state'], d['height'], d['battery']
            self.add_log('状态：%s  高度 %d cm  电量 %d%%'
                         % (STATE_NAMES.get(d['state'], d['state']), d['height'], d['battery']))
        elif typ == TYPE_ROUTE:
            self.add_log('收到航线帧（遍历流程不画）')
        elif typ == TYPE_REPORT:
            d = parse_report(payload)
            with self.lock:
                self.report = d['num']
            self.add_log('识别报送：编号 %s' % d['num'])
        else:
            self.add_log('收到未知帧 0x%02X' % typ)

    def snapshot(self):
        with self.lock:
            cells = [{'name': pos_name(f, p), 'face': f, 'pos': p,
                      'num': self.cells.get((f, p)),
                      'ref': layout.reference_number(FACE_NAMES[f], p)}
                     for f in range(len(FACE_NAMES)) for p in range(1, FACE_MAX_POS + 1)]
            n_a = sum(1 for p in range(1, FACE_MAX_POS + 1)
                      if (FACE_NAMES.index('A'), p) in self.cells)
            n_b = sum(1 for p in range(1, FACE_MAX_POS + 1)
                      if (FACE_NAMES.index('B'), p) in self.cells)
            done = len(self.cells)
            dstate, height, battery = self.dstate, self.height, self.battery
            log = list(self.log[-60:])
            report, connected, mode = self.report, self.connected, self.mode
            frames, errors = self.frames, self.errors
            alive = (time.time() - self.last_rx) < 5.0 if self.last_rx else False

        # 阶段推算（与桌面版一致）
        if dstate == 4:
            stage = 7
        elif dstate == 3:
            stage = 6
        elif done >= TOTAL_CELLS:
            stage = 5
        elif n_a > 0:
            stage = 4
        elif n_b >= FACE_MAX_POS:
            stage = 3
        elif n_b > 0 or dstate == 2:
            stage = 2
        elif dstate == 1:
            stage = 1
        else:
            stage = 0

        return {
            'time': time.strftime('%H:%M:%S'),
            'connected': connected, 'mode': mode, 'alive': alive,
            'frames': frames, 'errors': errors,
            'done': done, 'total': TOTAL_CELLS,
            'stage': stage, 'stages': STAGES,
            'drone': {'state': dstate, 'state_name': STATE_NAMES.get(dstate, str(dstate)),
                      'height': height, 'battery': battery, 'report': report},
            'cells': cells, 'log': log,
            'reference': {f: layout.REFERENCE_NUMBERS[f] for f in layout.REFERENCE_NUMBERS},
        }


STATE_NAMES = {0: '就绪', 1: '起飞中', 2: '飞行中', 3: '降落中', 4: '已落地'}
STORE = Store()


# ---------------- 状态推算（ROS 模式下没有 STATUS 帧，自己按高度推） ----------------
class StateTracker:
    """与 gs_bridge 同一套判据：<10cm 且没离过地=就绪；落地超 2s=已落地。"""

    def __init__(self):
        self.h_prev = None
        self.t_prev = time.time()
        self.vz = 0.0
        self.land_t = 0.0
        self.ever_air = False

    def update(self, h):
        now = time.time()
        dt = now - self.t_prev
        if self.h_prev is None:
            self.t_prev, self.h_prev = now, h
        elif dt > 0.2:
            self.vz = (h - self.h_prev) / dt
            self.t_prev, self.h_prev = now, h
        if h < 10:
            if not self.land_t:
                self.land_t = now
            if not self.ever_air:
                return 0
            return 4 if now - self.land_t > 2.0 else 3
        self.ever_air = True
        self.land_t = 0.0
        if self.vz < -5:
            return 3
        if self.vz > 2 and h < 30:
            return 1
        return 2


TRACKER = StateTracker()


# ---------------- ROS 模式（直接跑在香橙派上） ----------------
ROS_MODE = False
ROS_START_PUB = None


def start_ros_backend():
    """在香橙派上跑时用这个：直接订阅机上话题，不占 UDP 8888。

    队友只要打开 http://<板子IP>:8080 就能看，板子开机自启即常驻。
    """
    global ROS_MODE, ROS_START_PUB
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int16, Int16MultiArray, Int32, UInt8MultiArray

    class Bridge(Node):
        def __init__(self):
            super().__init__('webgs_bridge')
            self.create_subscription(Int16, '/height', self.on_h, 5)
            self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 5)
            self.create_subscription(Int16MultiArray, '/gs/event', self.on_event, 5)
            self.create_subscription(Int32, '/qr/code', self.on_qr, 5)
            self.pub_start = self.create_publisher(Int16, '/gs/start', 10)

        def on_h(self, m):
            h = int(m.data)
            with STORE.lock:
                STORE.height = h
                STORE.dstate = TRACKER.update(h)
            STORE.last_rx = time.time()

        def on_fc(self, m):
            if len(m.data) >= 2:
                with STORE.lock:
                    STORE.fc = (int(m.data[0]), int(m.data[1]))
            STORE.last_rx = time.time()

        def on_event(self, m):
            if len(m.data) >= 3:
                STORE.apply_frame(TYPE_EVENT, bytes([int(m.data[0]) & 0xFF,
                                                     int(m.data[1]) & 0xFF,
                                                     int(m.data[2]) & 0xFF])
                                  + b'\x00\x00\x00\x00')

        def on_qr(self, m):
            with STORE.lock:
                STORE.report = int(m.data) & 0xFF

    # ★ 不要接管 SIGINT/SIGTERM：rclpy 默认会装信号处理器，一旦进程收到 SIGTERM
    #   （systemd 停止、手动 kill 等）它会悄悄 rcl_shutdown() → spin 线程抛
    #   "context is not valid" 死掉，而 HTTP 还活着，页面永远停在旧数据。
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO_SIGNAL_HANDLERS)
    except Exception:  # noqa: BLE001
        rclpy.init()
    node = Bridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    ROS_START_PUB = node.pub_start
    ROS_MODE = True
    with STORE.lock:
        STORE.connected, STORE.mode = True, 'ros'
        STORE.battery = None      # ROS 链路没有电量帧，界面显示 —
    STORE.add_log('ROS 直连模式：已订阅 /height /fc_status /gs/event /qr/code；'
                  '「开始任务」会发 /gs/start')


# ---------------- UDP 链路 ----------------
class UdpLink(threading.Thread):
    def __init__(self, drone_ip, port):
        super().__init__(daemon=True)
        self.drone = (drone_ip, port)
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.3)
        self.parser = FrameParser()
        self.running = True

    def run(self):
        STORE.add_log('UDP 已监听 0.0.0.0:%d，机载 %s:%d' % (self.port, *self.drone))
        while self.running:
            try:
                data, _addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            for typ, payload in self.parser.feed(data):
                STORE.apply_frame(typ, payload)
            with STORE.lock:
                STORE.frames, STORE.errors = self.parser.frame_count, self.parser.error_count

    def send(self, typ, payload=b''):
        try:
            self.sock.sendto(pack_frame(typ, payload), self.drone)
            return True
        except OSError as e:
            STORE.add_log('发送失败: %s' % e)
            return False

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------- 内置模拟 ----------------
class Sim(threading.Thread):
    def __init__(self, interval_ms=1800):
        super().__init__(daemon=True)
        self.interval = interval_ms
        self.running = True

    def run(self):
        events, _meta = sim.build_simulation(interval_ms=self.interval)
        t0 = time.time()
        for delay, typ, payload in events:
            if not self.running:
                break
            wait = delay / 1000.0 - (time.time() - t0)
            while wait > 0 and self.running:
                time.sleep(min(wait, 0.2))
                wait = delay / 1000.0 - (time.time() - t0)
            if not self.running:
                break
            STORE.apply_frame(typ, payload)
        STORE.add_log('模拟结束')

    def stop(self):
        self.running = False


LINK = None
SIM = None


# ---------------- HTTP ----------------
class Handler(BaseHTTPRequestHandler):
    server_version = 'D-GroundStation-Web/1.0'

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _file(self, path, ctype):
        try:
            with open(path, 'rb') as fh:
                body = fh.read()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get('Content-Length') or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode()) or {}
        except Exception:  # noqa: BLE001
            return {}

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._file(os.path.join(STATIC, 'index.html'), 'text/html; charset=utf-8')
        elif self.path == '/api/state':
            self._json(STORE.snapshot())
        else:
            self.send_error(404)

    def do_POST(self):
        global LINK, SIM
        path = self.path.rstrip('/')
        body = self._body()
        try:
            if path == '/api/connect':
                ip = str(body.get('ip') or '172.25.105.210').strip()
                port = int(body.get('port') or 8888)
                if LINK:
                    LINK.stop()
                if SIM:
                    SIM.stop()
                    SIM = None
                LINK = UdpLink(ip, port)
                LINK.start()
                STORE.connected, STORE.mode = True, 'udp'
                self._json({'message': '已连接 UDP…机载 %s:%d' % (ip, port)})
            elif path == '/api/disconnect':
                if LINK:
                    LINK.stop()
                    LINK = None
                STORE.connected, STORE.mode = False, '--'
                STORE.add_log('链路已断开')
                self._json({'message': '链路已断开'})
            elif path == '/api/start':
                mode = int(body.get('mode') or 0)
                if ROS_START_PUB is not None:
                    from std_msgs.msg import Int16
                    m = Int16()
                    m.data = mode
                    for _ in range(3):
                        ROS_START_PUB.publish(m)
                        time.sleep(0.1)
                    STORE.add_log('已发 /gs/start=%d' % mode)
                    self._json({'message': '已下发 /gs/start=%d（0=遍历盘点）' % mode})
                elif LINK:
                    ok = LINK.send(TYPE_START, bytes([mode]))
                    self._json({'message': '已下发 START %d（0=遍历盘点）' % mode if ok else '发送失败'})
                else:
                    self._json({'message': '未连接链路：先点“连接机载”，或用“内置模拟”离线演示'})
            elif path == '/api/query':
                num = int(body.get('num') or 0)
                snap = STORE.snapshot()
                names = [c['name'] for c in snap['cells'] if c['num'] == num]
                self._json({'message': ('编号 %d → %s' % (num, '、'.join(names))) if names
                            else '编号 %d 尚未盘点' % num, 'names': names})
            elif path == '/api/reset':
                with STORE.lock:
                    STORE.cells.clear()
                    STORE.report = None
                    STORE.dstate, STORE.height, STORE.battery = 0, 0, 100
                STORE.add_log('已清空并复位')
                self._json({'message': '已清空并复位'})
            elif path == '/api/sim':
                if body.get('on'):
                    if LINK:
                        LINK.stop()
                        LINK = None
                        STORE.connected = False
                    if SIM:
                        SIM.stop()
                    SIM = Sim(int(body.get('interval_ms') or 1800))
                    SIM.start()
                    STORE.mode = 'sim'
                    STORE.add_log('开始内置模拟（参考货号样本）')
                    self._json({'message': '开始内置模拟（用参考货号样本铺 12 格）'})
                else:
                    if SIM:
                        SIM.stop()
                        SIM = None
                    STORE.mode = '--'
                    STORE.add_log('模拟已停止')
                    self._json({'message': '模拟已停止'})
            else:
                self.send_error(404)
        except Exception as e:  # noqa: BLE001
            self._json({'message': 'ERR %s' % e}, 500)


def main():
    ap = argparse.ArgumentParser(description='D题 网页版地面站')
    ap.add_argument('--port', type=int, default=8080, help='网页端口（默认 8080）')
    ap.add_argument('--udp-port', type=int, default=8888, help='与机载通信的 UDP 端口（默认 8888）')
    ap.add_argument('--drone-ip', default='172.25.105.210', help='默认机载 IP')
    ap.add_argument('--autoconnect', action='store_true', help='启动就连机载')
    ap.add_argument('--ros', action='store_true',
                    help='ROS 直连模式：跑在香橙派上时用，直接读机上话题，不占 UDP 端口')
    args = ap.parse_args()

    ip = _guess_ip()
    print('网页版地面站：http://%s:%d   （同网段手机/电脑都能开）' % (ip, args.port))
    if args.ros:
        print('模式：ROS 直连（跑在香橙派上；队友打开 http://%s:%d 即可）' % (ip, args.port))
        start_ros_backend()
    else:
        print('机载默认 IP %s，UDP 端口 %d；页面右上角可改。' % (args.drone_ip, args.udp_port))
    if args.autoconnect and not args.ros:
        global LINK
        LINK = UdpLink(args.drone_ip, args.udp_port)
        LINK.start()
        STORE.connected, STORE.mode = True, 'udp'
    srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n已停止')
    finally:
        srv.server_close()


def _guess_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


if __name__ == '__main__':
    main()
