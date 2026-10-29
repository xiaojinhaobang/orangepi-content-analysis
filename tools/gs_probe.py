"""D 题地面站链路探针（不开界面，直接收/发 UDP 帧）。

协议 v0: AA 55 | type | len | payload | sum ，sum = (type+len+Σpayload) & 0xFF
  上行 0x01 EVENT(num,face,pos,ts_u32) / 0x02 STATUS(state,height_u16,batt_u8) / 0x04 REPORT(num)
  下行 0x10 QUERY(num) / 0x11 START(mode)

用法:
    python tools/gs_probe.py                      # 只听 8 秒
    python tools/gs_probe.py --secs 20
    python tools/gs_probe.py --start              # 先给机载端发 START 0 再听
    python tools/gs_probe.py --start --board 192.168.137.186
"""

import argparse
import socket
import struct
import time

HEAD = b"\xAA\x55"
TYPE_EVENT, TYPE_STATUS, TYPE_REPORT = 0x01, 0x02, 0x04
TYPE_QUERY, TYPE_START = 0x10, 0x11
TYPE_NAME = {0x01: "EVENT", 0x02: "STATUS", 0x03: "ROUTE", 0x04: "REPORT"}
STATE_NAME = {0: "就绪", 1: "起飞中", 2: "飞行中", 3: "降落中", 4: "已落地"}


def pack_frame(frametype, payload=b""):
    body = bytes([frametype, len(payload)]) + payload
    csum = (frametype + len(payload) + sum(payload)) & 0xFF
    return HEAD + body + bytes([csum])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--board", default="192.168.137.186")
    ap.add_argument("--secs", type=float, default=8.0)
    ap.add_argument("--start", action="store_true", help="先下发 START 0（遍历盘点）")
    ap.add_argument("--poke", action="store_true",
                    help="先发一个无副作用的探路帧 0x7F（验证'必须先发包才收得到'假设）")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(0.5)

    if args.start:
        sock.sendto(pack_frame(TYPE_START, bytes([0])), (args.board, args.port))
        print("已下发 START 0 -> %s:%d" % (args.board, args.port))
    if args.poke:
        sock.sendto(pack_frame(0x7F), (args.board, args.port))
        print("已发探路帧 0x7F -> %s:%d" % (args.board, args.port))

    print("监听 UDP %d 共 %.0f 秒 ..." % (args.port, args.secs))
    buf = bytearray()
    ok = err = 0
    seen = {}
    t0 = time.time()
    while time.time() - t0 < args.secs:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        buf += data
        while True:
            i = buf.find(HEAD)
            if i < 0:
                buf.clear()
                break
            if i:
                del buf[:i]
            if len(buf) < 5:
                break
            plen = buf[3]
            if len(buf) < 5 + plen:
                break
            frame = bytes(buf[:5 + plen])
            del buf[:5 + plen]
            if (frame[2] + frame[3] + sum(frame[4:4 + plen])) & 0xFF != frame[-1]:
                err += 1
                continue
            ok += 1
            typ, pl = frame[2], frame[4:4 + plen]
            name = TYPE_NAME.get(typ, "0x%02X" % typ)
            seen[name] = seen.get(name, 0) + 1
            if typ == TYPE_STATUS and len(pl) >= 4:
                st, h, b = struct.unpack("<BHB", pl[:4])
                print("  [%s] %-8s 状态=%-6s 高度=%3dcm 电量=%d%%"
                      % (addr[0], name, STATE_NAME.get(st, st), h, b))
            elif typ == TYPE_EVENT and len(pl) >= 3:
                num, face, pos = pl[0], pl[1], pl[2]
                print("  [%s] %-8s ★ 扫到货号 %d  -> 位置 %s%d"
                      % (addr[0], name, num, "ABCD"[face] if face < 4 else "?", pos))
            else:
                print("  [%s] %-8s payload=%s" % (addr[0], name, list(pl)))

    print("共 %d 帧（校验错 %d）。按类型: %s" % (ok, err, seen or "未收到任何帧"))
    if ok == 0:
        print("提示: ① 板子上的 gs_bridge 起了吗(pgrep -af gs_bridge)；"
              "② gs_ip 填的是不是这台电脑的 192.168.137.1；"
              "③ Windows 防火墙是否放行 UDP %d。" % args.port)
    sock.close()


if __name__ == "__main__":
    main()
