# -*- coding: utf-8 -*-
"""
帧协议 v0（可后续调整）

帧格式（仿匿名 V7 风格，字节序均为小端）:
    [0xAA][0x55][type:u8][len:u8][payload:len 字节][sum:u8]
    sum = 低 8 位( type + len + sum(payload) )

上行（机载 -> 地面站）:
    0x01 EVENT   盘点事件:   编号u8 | 面u8(0=A..3=D) | 位u8(1..6) | 时间戳u32(ms)
    0x02 STATUS  状态:       状态u8 | 高度u16(cm) | 电量u8(%)
                             状态: 0=就绪 1=起飞中 2=飞行中 3=降落中 4=已落地
    0x03 ROUTE   航线:       点数u8 | 重复 {x:i16(cm) y:i16(cm)}
    0x04 REPORT  识别报送:   编号u8      (要求2-① 识别抽取二维码后上报)

下行（地面站 -> 机载）:
    0x10 QUERY   查询:       编号u8      (加分项/预留)
    0x11 START   启动:       模式u8      (0=遍历盘点 1=定点盘点)
"""
import struct

HEAD0 = 0xAA
HEAD1 = 0x55
MAX_PAYLOAD = 64

TYPE_EVENT = 0x01
TYPE_STATUS = 0x02
TYPE_ROUTE = 0x03
TYPE_REPORT = 0x04
TYPE_QUERY = 0x10
TYPE_START = 0x11

TYPE_NAMES = {
    TYPE_EVENT: "EVENT",
    TYPE_STATUS: "STATUS",
    TYPE_ROUTE: "ROUTE",
    TYPE_REPORT: "REPORT",
    TYPE_QUERY: "QUERY",
    TYPE_START: "START",
}

# 2026-09-28 改编版只有一块白板的两面（A/B），共 12 个货位
FACE_NAMES = "AB"            # face 0..1 -> 'A'..'B'
FACE_MAX_POS = 6             # 每面固定 6 个坐标位


def pos_name(face: int, pos: int) -> str:
    """(面, 位) -> 坐标名, 如 (0, 3) -> 'A3'"""
    return f"{FACE_NAMES[face]}{pos}"


def parse_pos_name(name: str):
    """坐标名 -> (面, 位), 如 'B5' -> (1, 5); 非法返回 None"""
    if len(name) != 2 or name[0] not in FACE_NAMES:
        return None
    try:
        pos = int(name[1])
    except ValueError:
        return None
    if not 1 <= pos <= FACE_MAX_POS:
        return None
    return FACE_NAMES.index(name[0]), pos


def pack_frame(frametype: int, payload: bytes = b"") -> bytes:
    """打包一帧。payload 超长或 type 非法会抛 ValueError。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload 超长: {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([frametype, len(payload)]) + payload
    csum = (frametype + len(payload) + sum(payload)) & 0xFF
    return bytes([HEAD0, HEAD1]) + body + bytes([csum])


# ---- 各帧 payload 的打包/解析（供机载端 / 模拟器复用） ----

def build_event(num: int, face: int, pos: int, ts: int) -> bytes:
    # num/face/pos 各 1 字节, 时间戳 4 字节(ms)
    return struct.pack("<BBBI", num, face, pos, int(ts))


def parse_event(p: bytes):
    num, face, pos, ts = struct.unpack("<BBBI", p)
    return {"num": num, "face": face, "pos": pos, "ts": ts}


def build_status(st: int, height_cm: int, battery: int) -> bytes:
    return struct.pack("<BHB", st, height_cm, battery)


def parse_status(p: bytes):
    st, height, battery = struct.unpack("<BHB", p)
    return {"state": st, "height": height, "battery": battery}


def build_route(points: list) -> bytes:
    """points: [(x_cm, y_cm), ...]"""
    n = min(len(points), 32)
    out = bytearray([n])
    for x, y in points[:n]:
        out += struct.pack("<hh", int(x), int(y))
    return bytes(out)


def parse_route(p: bytes):
    if not p:
        return {"points": []}
    n = p[0]
    pts = []
    for i in range(n):
        off = 1 + i * 4
        if off + 4 > len(p):
            break
        x, y = struct.unpack("<hh", p[off:off + 4])
        pts.append((x, y))
    return {"points": pts}


def build_report(num: int) -> bytes:
    return bytes([num & 0xFF])


def parse_report(p: bytes):
    return {"num": p[0]} if p else {"num": None}


class FrameParser:
    """增量式帧解析器：feed() 任意字节流，产出完整帧。
    记录 OK/错误计数，供 UI 显示链路质量。"""

    def __init__(self):
        self.buf = bytearray()
        self.frame_count = 0
        self.error_count = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            idx = self.buf.find(bytes([HEAD0, HEAD1]))
            if idx < 0:
                self.buf.clear()
                break
            if idx > 0:
                del self.buf[:idx]          # 丢弃头前的杂散字节
            if len(self.buf) < 5:
                break                        # 头 + type + len + 至少 1 字节校验
            plen = self.buf[3]
            total = 5 + plen                 # 4 字节帧头 + payload + 1 字节校验
            if len(self.buf) < total:
                break                        # 等完整帧
            frame = bytes(self.buf[:total])
            del self.buf[:total]
            if self._check(frame):
                self.frame_count += 1
                out.append((frame[2], frame[4:4 + plen]))
            else:
                self.error_count += 1
        return out

    @staticmethod
    def _check(frame: bytes) -> bool:
        _, _, typ, plen = frame[:4]
        body = frame[2:4 + plen]
        csum = (sum(body)) & 0xFF
        return csum == frame[-1]
