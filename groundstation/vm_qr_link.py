# -*- coding: utf-8 -*-
"""机载端识别链路(虚拟机代跑 "假地瓜派")。

摄像头 -> OpenCV 识别二维码 -> 解出编号/坐标 -> UDP 发给地面站显示。
地面站不需要任何改动: 它本来就只认 REPORT(识别报送) / EVENT(盘点事件) 两种帧。

数据流:
    摄像头 / 图片 / 视频 / RTSP
        -> qr_decode.decode_qr()          纯 OpenCV, 多预处理变体
        -> parse_text()                   文本 -> (编号, 坐标名)
        -> REPORT 帧 (编号) + EVENT 帧 (编号, 面, 位, 时间戳)
        -> UDP -> 地面站 (宿主机) :8888

用法(在 Ubuntu 虚拟机里):
    python3 vm_qr_link.py --pc 192.168.247.1 --source http://192.168.247.1:8080/video
    python3 vm_qr_link.py --pc 192.168.247.1                 # 本机摄像头 0
    python3 vm_qr_link.py --pc 192.168.247.1 --source a.jpg --once   # 单图自检
    python3 vm_qr_link.py --pc 192.168.247.1 --cell A3       # 二维码只含编号时补坐标

**实际推荐第一条**: VMware 的 USB 等时直通对普通 UVC 摄像头不可靠(实测
反复交付同一块全黑缓冲), 所以由宿主机 `host_cam_stream.py` 推 MJPEG 流,
虚拟机拉流识别 —— 对本脚本来说只是个 URL 源, 识别与发帧逻辑完全一致。

依赖: opencv-python, numpy, 以及同目录的 protocol.py / qr_decode.py
(把这三个 .py 一起拷到虚拟机同一目录即可)。
"""
import argparse
import json
import re
import socket
import threading
import time
from pathlib import Path

import cv2

from protocol import (FACE_MAX_POS, FACE_NAMES, TYPE_EVENT, TYPE_QUERY,
                      TYPE_REPORT, TYPE_START, TYPE_STATUS, FrameParser,
                      build_event, build_report, build_status, pack_frame)
from qr_decode import decode_qr, draw_results

# ---- 识别文本 -> (编号, 坐标名) 的容错解析 ----
# 支持: "7" | "A3" | "7,A3" | "A3,7" | "num=7;cell=A3" | {"num":7,"cell":"A3"}
_PURE_NUM = re.compile(r"^\s*(\d{1,6})\s*$")
_PURE_CELL = re.compile(r"^\s*([A-Da-d])\s*([1-6])\s*$")
_NUM_THEN_CELL = re.compile(r"(\d{1,3})\D{1,3}([A-Da-d][1-6])")
_CELL_THEN_NUM = re.compile(r"([A-Da-d][1-6])\D{1,3}(\d{1,3})")


def parse_text(text):
    """识别文本 -> (编号 int|None, 坐标名 str|None)。解析不出的部分为 None。"""
    text = (text or "").strip()
    if not text:
        return None, None

    # JSON 形式优先
    if text.startswith("{"):
        try:
            obj = json.loads(text)
            num = obj.get("num", obj.get("number", obj.get("id")))
            cell = obj.get("cell", obj.get("pos", obj.get("position")))
            return (int(num) if num is not None else None,
                    str(cell).upper() if cell else None)
        except (ValueError, TypeError):
            pass

    m = _PURE_NUM.match(text)
    if m:
        return int(m.group(1)), None
    m = _PURE_CELL.match(text)
    if m:
        return None, f"{m.group(1).upper()}{m.group(2)}"
    m = _NUM_THEN_CELL.search(text)
    if m:
        return int(m.group(1)), m.group(2).upper()
    m = _CELL_THEN_NUM.search(text)
    if m:
        return int(m.group(2)), m.group(1).upper()

    # "num=7 cell=A3" 这类键值对
    num = re.search(r"(?:num|number|id)\s*[=:]\s*(\d{1,3})", text, re.I)
    cell = re.search(r"(?:cell|pos|position)\s*[=:]\s*([A-Da-d][1-6])", text, re.I)
    return (int(num.group(1)) if num else None,
            cell.group(1).upper() if cell else None)


def cell_to_face_pos(cell):
    """'A3' -> (0, 3); 非法返回 None"""
    if not cell or len(cell) != 2 or cell[0] not in FACE_NAMES:
        return None
    pos = int(cell[1])
    return (FACE_NAMES.index(cell[0]), pos) if 1 <= pos <= FACE_MAX_POS else None


def load_num_map(spec):
    """编号->货位 对照表。spec 可以是 json 文件路径, 或内联 '7=A3,12=B5'。

    二维码只含编号时, 靠这张表补出坐标才能发 EVENT(点亮遍历页对应格)。
    真机上这份坐标来自飞控飞行计划, 这里相当于让脚本扮演飞控。
    json 两种方向都收: {"7":"A3"} 或 {"A3":7}。
    """
    if not spec:
        return {}
    out = {}
    path = Path(spec)
    if path.is_file():
        try:
            obj = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as e:
            print(f"[warn] num-map {spec} unreadable: {e}")
            return {}
        items = obj.items()
    else:
        items = []
        for part in spec.replace(";", ",").split(","):
            if "=" in part:
                k, v = part.split("=", 1)
                items.append((k.strip(), v.strip()))

    for k, v in items:
        k, v = str(k).strip(), str(v).strip().upper()
        try:
            if v and v[0] in FACE_NAMES and cell_to_face_pos(v):
                out[int(k)] = v                       # {"7": "A3"}
            elif k and k[0] in FACE_NAMES and cell_to_face_pos(k):
                out[int(v)] = k                       # {"A3": 7}
            else:
                print(f"[warn] num-map entry ignored: {k}={v}")
        except ValueError:
            print(f"[warn] num-map entry ignored: {k}={v}")
    return out


class Uplink:
    """上行发包 + (可选) 收地面站下行 START/QUERY。"""

    def __init__(self, pc, port, rx_port=0, log=print):
        self.target = (pc, port)
        self.log = log
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sent = 0
        self.armed = True                  # --require-start 时由下行 START 置位
        self.last_down = ""
        self._rx = None
        if rx_port:
            try:
                self._sock.bind(("0.0.0.0", rx_port))
                self._rx = threading.Thread(target=self._rx_loop, daemon=True)
                self._rx.start()
                self.log(f"[link] listening downstream on :{rx_port}")
            except OSError as e:
                self.log(f"[link] downstream bind :{rx_port} failed: {e}")

    def send(self, typ, payload):
        frame = pack_frame(typ, payload)
        self._sock.sendto(frame, self.target)
        self.sent += 1
        return frame

    def _rx_loop(self):
        parser = FrameParser()
        while True:
            try:
                data, addr = self._sock.recvfrom(2048)
            except OSError:
                return
            for typ, payload in parser.feed(data):
                if typ == TYPE_START:
                    mode = payload[0] if payload else 0
                    self.armed = True
                    self.last_down = f"START mode={mode}"
                    self.log(f"[link] downstream START mode={mode} -> armed")
                elif typ == TYPE_QUERY:
                    num = payload[0] if payload else 0
                    self.last_down = f"QUERY num={num}"
                    self.log(f"[link] downstream QUERY num={num}")

    def close(self):
        self._sock.close()


IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


def qr_width(result):
    """二维码在画面里的像素宽度(角点 x 跨度)。"""
    xs = [p[0] for p in result["points"]]
    return max(xs) - min(xs)


def open_capture(source):
    """source: '0' 摄像头序号 / 文件路径 / RTSP-URL。返回 cv2.VideoCapture。"""
    cap_source = int(source) if str(source).isdigit() else source
    cap = cv2.VideoCapture(cap_source)
    if not cap.isOpened():
        raise OSError(f"failed to open video source: {source}")
    return cap


def load_still(source):
    """单张图片 -> 常驻帧(不靠 VideoCapture, 循环时反复用同一张); 不是图片返回 None。"""
    p = Path(source)
    if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES:
        img = cv2.imread(str(p))
        if img is None:
            raise OSError(f"failed to read image: {source}")
        return img
    return None


def main():
    ap = argparse.ArgumentParser(
        description="VM-side QR recognition -> ground station over UDP")
    ap.add_argument("--pc", default="192.168.247.1", help="ground station IP (host)")
    ap.add_argument("--port", type=int, default=8888, help="ground station UDP port")
    ap.add_argument("--rx-port", type=int, default=8888,
                    help="bind this port to receive downstream START/QUERY (0=off)")
    ap.add_argument("--source", default="0",
                    help="camera index / image / video / rtsp url; "
                         "e.g. http://192.168.247.1:8080/video (host stream)")
    ap.add_argument("--cell", default=None,
                    help="force coordinate when the QR only carries the number, e.g. A3")
    ap.add_argument("--num-map", default=None,
                    help="number->cell table: json file or inline '7=A3,12=B5'")
    ap.add_argument("--confirm", type=int, default=2,
                    help="frames with the same text required before sending")
    ap.add_argument("--cooldown", type=float, default=3.0,
                    help="seconds before the same text can be sent again")
    ap.add_argument("--num-max", type=int, default=24,
                    help="valid number range is 1..num-max (D problem: 24 cells)")
    ap.add_argument("--min-px", type=int, default=0,
                    help="reject QR narrower than N px (0=off, default); "
                         "use e.g. 100 to enforce SOLUTION.md's >100px reliability line")
    ap.add_argument("--allow-any-num", action="store_true",
                    help="send REPORT even if the number is out of range")
    ap.add_argument("--status-hz", type=float, default=0.0,
                    help="send STATUS telemetry at this rate (0=off)")
    ap.add_argument("--require-start", action="store_true",
                    help="only send after receiving a downstream START")
    ap.add_argument("--fast", action="store_true",
                    help="decode original frame only (no preprocessing variants)")
    ap.add_argument("--no-show", action="store_true", help="no preview window")
    ap.add_argument("--once", action="store_true",
                    help="exit after the first successful send (for self-check)")
    ap.add_argument("--save", default=None, help="save annotated frame to this path")
    args = ap.parse_args()

    # 命令行 print 保持 ASCII, 免得在 GBK 控制台/日志里乱码
    link = Uplink(args.pc, args.port, args.rx_port)
    print(f"[link] uplink -> {args.pc}:{args.port}")
    if args.require_start:
        link.armed = False
        print("[link] waiting for downstream START before sending")

    forced = cell_to_face_pos(args.cell.upper()) if args.cell else None
    if args.cell and forced is None:
        print(f"[warn] bad --cell {args.cell}, ignored")
    if forced:
        print(f"[cfg] forced cell {args.cell.upper()} -> face {forced[0]} pos {forced[1]}")

    num_map = load_num_map(args.num_map)
    if args.num_map:
        print(f"[cfg] num-map: {len(num_map)} entries"
              + (f" -> {sorted(num_map.items())[:8]}" if num_map else " (empty)"))

    # 摄像头和网络流都是"活源": 读失败要重试, 不能当成文件读完了退出
    # (MJPEG 拉流偶尔丢一帧是常事, 直接 break 会让人以为脚本崩了)
    is_live = str(args.source).isdigit() or "://" in str(args.source)
    still = load_still(args.source)          # 单图自检: 反复用同一张, 便于多帧确认
    cap = None if still is not None else open_capture(args.source)
    print(f"[cam] source={args.source} "
          + ("still image" if still is not None else f"opened={cap.isOpened()}"))

    last_text = None
    streak = 0
    last_sent = {}                       # text -> 上次发送时刻
    last_result = None                   # (text, num, cell) 供 'r' 重发
    sent_ok = 0
    total = 0                            # 总帧数(FPS 统计用)
    t0 = time.time()                     # 程序起点, EVENT 时间戳用相对毫秒(u32 安全)
    t_status = 0.0
    t_fps = time.time()
    fps_base = 0                         # 上次 FPS 统计时的 total
    fps = 0.0
    status = "scanning..."
    quit_now = False

    try:
        while not quit_now:
            if still is not None:
                frame = still
            else:
                ok, frame = cap.read()
                if not ok:
                    if is_live:
                        print("[cam] read failed, retrying")
                        time.sleep(0.2)
                        continue
                    print("[cam] end of stream")
                    break
            total += 1

            results = decode_qr(frame, fast=args.fast)

            # 可选近距门限(默认关): 码太小就读数不可靠(SOLUTION.md: >100px 才 >90%)。
            # 默认关是因为误杀的代价更高——码明明解出来了却不发, 看起来就像没反应。
            near = results
            if args.min_px > 0:
                near = [r for r in results if qr_width(r) >= args.min_px]
                if results and not near:
                    status = (f"too far: {qr_width(results[0])}px "
                              f"< {args.min_px}px, move closer")
                    last_text, streak = None, 0

            # 多帧确认: 同一内容连续 confirm 帧才算识别成功(滤偶发误识别)
            text = near[0]["text"] if len(near) == 1 else None
            if text and text == last_text:
                streak += 1
            else:
                streak = 1 if text else 0
            last_text = text

            if text and streak >= args.confirm and link.armed:
                now = time.time()
                if now - last_sent.get(text, -1e9) >= args.cooldown:
                    num, cell = parse_text(text)
                    if cell is None and args.cell:
                        cell = args.cell.upper()
                    if cell is None and num is not None:
                        cell = num_map.get(num)          # 二维码只有编号时查对照表
                    face_pos = cell_to_face_pos(cell)
                    num_ok = num is not None and (1 <= num <= args.num_max
                                                  or args.allow_any_num)
                    if num is None or not num_ok:
                        status = f"BAD NUM text={text} num={num}"
                        print(f"[skip] {status} (expect 1..{args.num_max})")
                    elif face_pos is None:
                        # 只有编号: 发 REPORT, 坐标交给地面站按遍历结果查
                        link.send(TYPE_REPORT, build_report(num))
                        why = "not in num-map" if num_map else "no cell"
                        status = f"REPORT num={num} ({why})"
                        print(f"[send] REPORT num={num} text={text} ({why})")
                        sent_ok += 1
                    else:
                        # 编号 + 扫描结果: REPORT 上报编号, EVENT 上报盘点坐标
                        face, pos = face_pos
                        link.send(TYPE_REPORT, build_report(num))
                        link.send(TYPE_EVENT,
                                  build_event(num, face, pos,
                                              int((now - t0) * 1000) & 0xFFFFFFFF))
                        status = f"REPORT num={num} + EVENT {cell}"
                        print(f"[send] REPORT num={num} + EVENT "
                              f"num={num} face={face} pos={pos} text={text}")
                        sent_ok += 1
                    last_sent[text] = now
                    last_result = (text, num, cell)
                    if args.once:
                        if args.save:
                            cv2.imwrite(args.save, draw_results(frame, results))
                        break

            # 可选遥测
            if args.status_hz > 0 and time.time() - t_status >= 1.0 / args.status_hz:
                link.send(TYPE_STATUS, build_status(2, 0, 100))
                t_status = time.time()

            if not args.no_show:
                annotated = draw_results(frame, results)
                hud = (f"-> {args.pc}:{args.port}  sent={sent_ok}  streak={streak}"
                       f"  rx={link.last_down or '-'}")
                cv2.putText(annotated, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1)
                cv2.putText(annotated, status, (8, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1)
                cv2.imshow("VM QR -> Ground Station", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    quit_now = True
                elif key == ord("r") and last_result:
                    _t, _n, _c = last_result
                    last_sent.pop(_t, None)          # 清冷却, 下一帧立即重发
                    streak = 0
                    print("[key] resend armed")
            now = time.time()
            if now - t_fps >= 1.0:
                fps = (total - fps_base) / (now - t_fps)
                fps_base = total
                t_fps = now
            if args.no_show and total % 30 == 0:
                print(f"[scan] frames={total} fps~{fps:.1f} sent={sent_ok} "
                      f"last={status}")
    except KeyboardInterrupt:
        print("\n[exit] interrupted")
    finally:
        if cap is not None:
            cap.release()
        if not args.no_show:
            cv2.destroyAllWindows()
        link.close()
        print(f"[done] frames={total} sent={sent_ok} packets={link.sent}")


if __name__ == "__main__":
    main()
