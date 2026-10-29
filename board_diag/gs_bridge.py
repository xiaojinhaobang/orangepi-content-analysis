#!/usr/bin/env python3
"""D 题地面站 <-> 香橙派 通信桥。

协议与地面站 protocol.py 完全一致（小端）:
  帧: AA 55 | type | len | payload | sum     sum = (type+len+Σpayload) & 0xFF
  上行(机载->地面站): 0x01 EVENT(num,face,pos,ts_u32) / 0x02 STATUS(state,height_u16,batt_u8)
                      0x04 REPORT(num)
  下行(地面站->机载): 0x10 QUERY(num) / 0x11 START(mode)   mode: 0=遍历 1=定点

UDP: 本节点 bind 0.0.0.0:<port>；收到地面站的包后记住来源地址，
     之后所有上行帧都发到该地址的同一端口（与地面站对称）。

ROS 接口:
  /height(Int16)  /fc_status(UInt8MultiArray)      -> STATUS
  /gs/event(Int16MultiArray [num,face,pos])        -> EVENT   （任务侧发布）
  /qr/code(Int32)                                  -> REPORT  （识别到货号就上报）
  /gs/start(Int16) /gs/query(Int16)                <- 地面站下行转出
"""

import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import (Int16, Int16MultiArray, Int32, UInt8MultiArray)

HEAD = b"\xAA\x55"
TYPE_EVENT, TYPE_STATUS, TYPE_REPORT = 0x01, 0x02, 0x04
TYPE_QUERY, TYPE_START = 0x10, 0x11


def pack_frame(frametype: int, payload: bytes = b"") -> bytes:
    body = bytes([frametype, len(payload)]) + payload
    csum = (frametype + len(payload) + sum(payload)) & 0xFF
    return HEAD + body + bytes([csum])


class FrameParser:
    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.err = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            idx = self.buf.find(HEAD)
            if idx < 0:
                self.buf.clear()
                break
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < 5:
                break
            plen = self.buf[3]
            total = 5 + plen
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            del self.buf[:total]
            if (frame[2] + frame[3] + sum(frame[4:4 + plen])) & 0xFF == frame[-1]:
                self.ok += 1
                out.append((frame[2], frame[4:4 + plen]))
            else:
                self.err += 1
        return out


class GsBridge(Node):
    def __init__(self):
        super().__init__('gs_bridge')
        self.port = int(self.declare_parameter('port', 8888).value)
        self.rate_hz = float(self.declare_parameter('status_hz', 2.0).value)
        # 地面站 IP：留空则等地面站先发包再回（会一直不发状态）；
        # 填上就立即开始推状态（推荐）。
        self.gs_ip = str(self.declare_parameter('gs_ip', '').value).strip()
        self.gs_port = int(self.declare_parameter('gs_port', self.port).value)
        self.height = None
        self.unlock = None
        self.gs = (self.gs_ip, self.gs_port) if self.gs_ip else None
        self.parser = FrameParser()
        self.tx = 0
        # 状态机用的历史量（STATUS 帧只有 1 字节状态，得在这边自己推）
        self._h_prev = None
        self._t_prev = time.time()
        self._vz = 0.0            # 垂直速度 cm/s（正=上升）
        self._land_t = 0.0        # 高度 <10cm 的起点
        self._ever_air = False    # 本次上电后是否真的离过地

        self.create_subscription(Int16, '/height', self._h, 5)
        self.create_subscription(UInt8MultiArray, '/fc_status', self._fc, 5)
        self.create_subscription(Int16MultiArray, '/gs/event', self._event, 5)
        self.create_subscription(Int32, '/qr/code', self._qr, 5)
        self.pub_start = self.create_publisher(Int16, '/gs/start', 10)
        self.pub_query = self.create_publisher(Int16, '/gs/query', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(0.3)
        threading.Thread(target=self._recv_loop, daemon=True).start()
        self.create_timer(1.0 / max(self.rate_hz, 0.2), self._send_status)
        self.get_logger().info('gs_bridge: UDP 监听 0.0.0.0:%d，状态 %.1fHz，地面站=%s'
                               % (self.port, self.rate_hz,
                                  ('%s:%d' % self.gs) if self.gs else '(等对方发包)'))

    # ---------- ROS 回调 ----------
    def _h(self, m):
        # ★ 2026-10-07：高度遥测偶发被串口干扰成 ~3840/3998cm（约 6% 的采样）。
        #   地面站界面会跟着跳 38 米，这里直接剔除：绝对值 >500cm 或单次跳变
        #   >100cm 的一律丢，保持上一次的值。
        h = int(m.data)
        if h > 500 or h < -50:
            return
        if self.height is not None and abs(h - self.height) > 100:
            return
        self.height = h

    def _fc(self, m):
        if len(m.data) >= 2:
            self.unlock = int(m.data[1])

    def _event(self, m):
        if len(m.data) >= 3:
            num, face, pos = (int(m.data[0]) & 0xFF, int(m.data[1]) & 0xFF,
                              int(m.data[2]) & 0xFF)
            ts = int(time.time() * 1000) & 0xFFFFFFFF
            self._send(TYPE_EVENT, struct.pack('<BBBI', num, face, pos, ts))
            self.get_logger().info('EVENT -> 地面站: 货号%d 位置%s%d'
                                   % (num, 'ABCD'[face] if face < 4 else '?', pos))

    def _qr(self, m):
        num = int(m.data) & 0xFF
        self._send(TYPE_REPORT, bytes([num]))

    # ---------- 地面站收包 ----------
    def _recv_loop(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            if self.gs != addr:
                self.gs = addr
                self.get_logger().info('锁定地面站地址: %s:%d' % addr)
            for typ, payload in self.parser.feed(data):
                if typ == TYPE_START and payload:
                    mode = payload[0]
                    self.pub_start.publish(Int16(data=int(mode)))
                    self.get_logger().info('收到 START mode=%d -> /gs/start' % mode)
                elif typ == TYPE_QUERY and payload:
                    num = payload[0]
                    self.pub_query.publish(Int16(data=int(num)))
                    self.get_logger().info('收到 QUERY num=%d -> /gs/query' % num)
                else:
                    self.get_logger().info('收到未知帧 type=0x%02X len=%d'
                                           % (typ, len(payload)))

    # ---------- 上行发送 ----------
    def _send(self, frametype, payload=b""):
        if not self.gs:
            return
        try:
            self.sock.sendto(pack_frame(frametype, payload), self.gs)
            self.tx += 1
        except OSError as e:
            self.get_logger().warn('UDP 发送失败: %s' % e)

    def _send_status(self):
        if not self.gs:
            return
        h = self.height if self.height is not None else 0
        now = time.time()
        dt = now - self._t_prev
        if self._h_prev is not None and dt > 0.2:
            self._vz = (h - self._h_prev) / dt
            self._t_prev, self._h_prev = now, h
        elif self._h_prev is None:
            self._t_prev, self._h_prev = now, h
        # 状态: 0=就绪 1=起飞中 2=飞行中 3=降落中 4=已落地
        # 原来只会"锁着就报已落地"，站着没飞也显示已落地，容易误解。
        if h < 10:
            if not self._land_t:
                self._land_t = now                   # 进入地面段，先起计时
            if not self._ever_air:
                state = 0                            # 上电没离地 = 就绪
            elif now - self._land_t > 2.0:
                state = 4                            # 落地超过 2 秒 = 已落地
            else:
                state = 3                            # 刚落 = 降落中
        else:
            self._ever_air = True
            self._land_t = 0.0
            if self._vz < -5:
                state = 3                            # 下降中
            elif self._vz > 2 and h < 30:
                state = 1                            # 刚离地 = 起飞中
            else:
                state = 2                            # 飞行中
        self._send(TYPE_STATUS, struct.pack('<BHB', state, max(0, min(65535, h)), 100))


def main():
    rclpy.init()
    node = GsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
