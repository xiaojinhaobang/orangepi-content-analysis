"""不开界面，直接让 D 题地面站去连真机，验证 UDP 链路。

用法（电脑上，用 anaconda 的 python）:
    python tools/test_desktop_link.py                # 只连，收 6 秒
    python tools/test_desktop_link.py --start        # 连上后点「开始任务」下发 START 0
"""

import argparse
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
GS_DIR = r"C:\Users\金洲毅\Desktop\学习资料\D题地面站"
sys.path.insert(0, GS_DIR)

from PySide6.QtCore import QEventLoop, QTimer           # noqa: E402
from PySide6.QtWidgets import QApplication              # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.137.186")
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--secs", type=float, default=6.0)
    ap.add_argument("--start", action="store_true")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    from ui.main_window import MainWindow               # noqa: E402

    win = MainWindow()
    logs = []
    _orig_log = win.log

    def _cap(m):
        logs.append(str(m))
        _orig_log(m)

    win.log = _cap
    win.cmb_mode.setCurrentText("UDP (WiFi)")
    win.edit_ip.setText(args.ip)
    win.spin_port.setValue(args.port)
    win._toggle_connect()                               # 点「连接」
    print("已连接 %s:%d" % (args.ip, args.port))
    print("link=%s  模式=%r  listen=%s  target=%s  sock=%s"
          % (type(getattr(win, 'link', None)).__name__, win.cmb_mode.currentText(),
             getattr(getattr(win, 'link', None), 'listen_port', None),
             getattr(getattr(win, 'link', None), 'target', None),
             getattr(getattr(win, 'link', None), '_sock', None)))

    if args.start:
        QTimer.singleShot(1500, win.btn_start_mission.click)
        print("1.5s 后点「开始任务」")

    loop = QEventLoop()
    QTimer.singleShot(int(args.secs * 1000), loop.quit)
    loop.exec()

    p = win.link.parser if getattr(win, "link", None) else None
    ok = getattr(p, "ok", 0)
    err = getattr(p, "err", 0)
    print("收到帧 %d（校验错 %d）" % (ok, err))
    if logs:
        print("程序日志: %s" % logs[-5:])
    print("盘点表里已有 %d 格: %s"
          % (win.inv.count(), [n for n, v in win.inv.all_cells() if v is not None]))
    if ok == 0:
        print("!! 一帧都没收到 —— 检查板上 gs_bridge 是否在跑、gs_ip 是否为 192.168.137.1、"
              "防火墙是否放行 UDP %d" % args.port)
        return 1
    win.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
