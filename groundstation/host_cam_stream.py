# -*- coding: utf-8 -*-
"""宿主机摄像头 -> MJPEG HTTP 流(给虚拟机拉)。

为什么需要它: VMware Workstation 的 USB 等时(isochronous)直通对普通 UVC
摄像头不可靠 —— 实测表现为 dmesg "still N active urbs on EP #85", 之后
反复交付同一块从未更新的缓冲(全黑且帧字节完全相同)。摄像头本身没问题,
同一时刻在 Windows 下画面正常。

所以改成: 宿主机负责"取帧", 虚拟机负责"识别"(OpenCV 解码 + 发协议帧)。
对虚拟机来说这就是个网络摄像头, 相当于"地瓜派"外接了一个 IP Camera,
代跑识别的意义不变。

用法(宿主机):
    python host_cam_stream.py                 # index 1, 1280x720@15, 端口 8080
    python host_cam_stream.py --index 0       # 换摄像头
    python host_cam_stream.py --list          # 先枚举可用 index

虚拟机侧:
    python3 vm_qr_link.py --pc 192.168.247.1 --num-map num_cell.json \\
        --source http://192.168.247.1:8080/video
"""
import argparse
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

BOUNDARY = "frame"


def local_ips():
    """本机在虚拟网段上的地址(给虚拟机填 -pc 用)。"""
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    try:                                   # 不实际发包, 只问路由表选哪张网卡
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.168.247.131", 9))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(i for i in ips if not i.startswith("127."))


class Camera:
    """后台抓帧线程, 只保留最新一帧 —— 慢的客户端不会拖慢采集。"""

    def __init__(self, index, width, height, quality, fps):
        self.index = index
        self.width = width
        self.height = height
        self.quality = quality
        self.fps = fps
        self.latest = None
        self.seq = 0
        self.error = None
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        if self.width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps:
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not cap.isOpened():
            self.error = f"cannot open camera index {self.index}"
            return

        period = 1.0 / self.fps if self.fps else 0.0
        fails = 0
        while not self._stop.is_set():
            t = time.time()
            ok, frame = cap.read()
            if not ok:
                fails += 1
                if fails > 60:
                    self.error = "camera stopped delivering frames"
                    break
                time.sleep(0.05)
                continue
            fails = 0
            ok, buf = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                with self._lock:
                    self.latest = buf.tobytes()
                    self.seq += 1
            if period:
                time.sleep(max(0.0, period - (time.time() - t)))
        cap.release()

    def frame(self):
        with self._lock:
            return self.latest

    def stop(self):
        self._stop.set()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"          # 关掉 keep-alive, 流式响应简单可靠

    def log_message(self, *a):             # 静音默认的每请求日志
        pass

    def do_GET(self):
        if self.path.startswith("/video"):
            self._stream()
        elif self.path in ("/", "/index.html"):
            self._page()
        elif self.path.startswith("/snapshot"):
            buf = self.cam.frame()
            if not buf:
                self.send_error(503, "no frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(buf)))
            self.end_headers()
            self.wfile.write(buf)
        else:
            self.send_error(404)

    def _page(self):
        html = (b"<html><head><title>host cam</title></head><body>"
                b"<h3>MJPEG stream</h3>"
                b"<img src='/video' width='640'>"
                b"<p>VM: --source http://HOST_IP:PORT/video</p>"
                b"</body></html>")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        period = 1.0 / self.cam.fps if self.cam.fps else 0.0
        last = -1
        try:
            while True:
                buf = self.cam.frame()
                if buf is None or self.cam.seq == last:
                    time.sleep(0.005)
                    continue
                last = self.cam.seq
                self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(buf)}\r\n\r\n".encode())
                self.wfile.write(buf)
                self.wfile.write(b"\r\n")
                if period:
                    time.sleep(period)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    ap = argparse.ArgumentParser(description="host camera -> MJPEG http stream")
    ap.add_argument("--index", type=int, default=1,
                    help="host camera index (default 1 = the USB camera; "
                         "0 is usually the built-in laptop cam)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality 1-100")
    ap.add_argument("--list", action="store_true", help="enumerate cameras and exit")
    args = ap.parse_args()

    if args.list:
        for i in range(6):
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if cap.isOpened():
                ok, f = cap.read()
                print(f"index {i}: OK shape={None if not ok else f.shape}")
            else:
                print(f"index {i}: -")
            cap.release()
        return

    cam = Camera(args.index, args.width, args.height, args.quality, args.fps)
    cam.start()
    time.sleep(1.5)                        # 等第一帧 / 等打开失败暴露出来
    if cam.error:
        print(f"[cam] {cam.error}")
        return 1
    buf = cam.frame()
    print(f"[cam] index {args.index} streaming {args.width}x{args.height}"
          f"@{args.fps:g} q{args.quality}"
          f" first_frame={'yes' if buf else 'NO'}"
          f" ({len(buf) if buf else 0} bytes)")

    handler = type("H", (Handler,), {"cam": cam})
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    for ip in local_ips():
        print(f"[url] http://{ip}:{args.port}/video")
    print("[url] (VM 侧: --source http://<上面那个IP>:"
          f"{args.port}/video )")
    print("[run] Ctrl+C 停止")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[exit] stopped")
    finally:
        cam.stop()
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
