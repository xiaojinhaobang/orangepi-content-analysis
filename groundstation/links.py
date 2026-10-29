# -*- coding: utf-8 -*-
"""
通信层：统一 Link 抽象。
    Link (抽象) -> UdpLink    (WiFi, 地面站监听 UDP 端口)
               -> SerialLink  (蓝牙 SPP / USB 串口, Windows 上蓝牙配成 COM 口)
               -> inject_frame()  (模拟器注入, 与真实链路走同一解析路径)

收到原始字节流 -> FrameParser 增量解析 -> 逐帧 emit frame_received(type, payload)。
发送统一走 send(frametype, payload) -> pack_frame -> 链路写入。
"""
import socket
import threading

from PySide6.QtCore import QObject, Signal

from protocol import FrameParser, pack_frame


class Link(QObject):
    """链路抽象基类"""
    frame_received = Signal(int, bytes)   # (帧类型, payload)
    error = Signal(str)
    log = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parser = FrameParser()
        self._running = False

    def start(self):
        self._running = True

    def stop(self):
        self._running = False

    def send(self, frametype: int, payload: bytes = b""):
        """下行指令: 地面站 -> 机载。子类实现具体写出。"""
        raise NotImplementedError

    def inject_frame(self, frametype: int, payload: bytes = b""):
        """模拟器 / 网络线程共用的收帧入口"""
        self.frame_received.emit(frametype, payload)

    # ---- 供收帧线程调用的解析辅助 ----
    def _consume(self, data: bytes):
        try:
            for typ, payload in self.parser.feed(data):
                self.inject_frame(typ, payload)
        except Exception as e:            # 不因一包脏数据崩线程
            self.error.emit(f"解析异常: {e}")


class UdpLink(Link):
    """WiFi 链路。地面站 bind(listen_port) 收帧; send 走 target 地址。"""

    def __init__(self, listen_port: int = 8888, target=("127.0.0.1", 8889), parent=None):
        super().__init__(parent)
        self.listen_port = listen_port
        self.target = target
        self._sock = None
        self._th = None

    def start(self):
        super().start()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._sock.bind(("0.0.0.0", self.listen_port))
        except OSError as e:
            self.error.emit(f"UDP 端口 {self.listen_port} 绑定失败: {e}")
            self._sock.close()
            self._sock = None
            return
        self._th = threading.Thread(target=self._recv_loop, daemon=True)
        self._th.start()
        self.log.emit(f"UDP 已监听 :{self.listen_port}，目标 {self.target}")

    def stop(self):
        super().stop()
        if self._sock:
            try:
                self._sock.close()          # 使 recvfrom 抛错退出
            except OSError:
                pass
        self._sock = None

    def _recv_loop(self):
        while self._running and self._sock:
            try:
                data, _addr = self._sock.recvfrom(2048)
            except OSError:
                break
            self._consume(data)

    def send(self, frametype: int, payload: bytes = b""):
        if not self._sock:
            return
        try:
            self._sock.sendto(pack_frame(frametype, payload), self.target)
        except OSError as e:
            self.error.emit(f"UDP 发送失败: {e}")


class SerialLink(Link):
    """串口链路: 蓝牙 SPP(电脑配对后为 COM 口) 或 USB 转 TTL 均适用。"""

    def __init__(self, port: str, baudrate: int = 115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self._ser = None
        self._th = None

    def start(self):
        super().start()
        try:
            import serial
            self._ser = serial.Serial(self.port, self.baudrate, timeout=0.2)
        except Exception as e:
            self.error.emit(f"串口 {self.port} 打开失败: {e}")
            self._running = False
            return
        self._th = threading.Thread(target=self._recv_loop, daemon=True)
        self._th.start()
        self.log.emit(f"串口已连接: {self.port} @{self.baudrate}")

    def stop(self):
        super().stop()
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def _recv_loop(self):
        while self._running and self._ser:
            try:
                n = self._ser.in_waiting
                if n:
                    data = self._ser.read(n)
                    self._consume(data)
            except Exception as e:
                self.error.emit(f"串口读取失败: {e}")
                break

    def send(self, frametype: int, payload: bytes = b""):
        if self._ser:
            try:
                self._ser.write(pack_frame(frametype, payload))
            except Exception as e:
                self.error.emit(f"串口写入失败: {e}")
