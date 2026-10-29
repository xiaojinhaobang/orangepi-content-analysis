# -*- coding: utf-8 -*-
"""临时验证: 定点目标语义(输入框说了算) + 遍历漏格时靠事件校准。"""
import os
import sys

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, r"C:\Users\金洲毅\Desktop\D题地面站")

from PySide6.QtWidgets import QApplication            # noqa: E402

from protocol import TYPE_EVENT, TYPE_REPORT, build_event, build_report  # noqa: E402

A_FACE = {8: 1, 11: 2, 9: 3, 7: 4, 5: 5, 6: 6}
fails = []


def check(label, got, want):
    ok = got == want
    if not ok:
        fails.append(label)
    print(f"  [{'OK ' if ok else 'FAIL'}] {label}: {got!r}"
          + ("" if ok else f"  want {want!r}"))


app = QApplication([])
from ui.main_window import MainWindow                 # noqa: E402

w = MainWindow()
feed = w._on_frame

print("1) traverse: fill A face")
for num, pos in A_FACE.items():
    feed(TYPE_EVENT, build_event(num, 0, pos, 1000))
check("cells", w.inv.count(), 6)

print("2) input target 7 -> plan immediately")
w.page_targeted.edit_num.setText("7")
w.page_targeted._on_start_clicked()
check("target", w.page_targeted.target_num(), 7)
check("route has points", len(w._route_points) > 0, True)
check("target line has A4", "A4" in w.page_targeted.lbl_target.text(), True)

print("3) scan 11 -> target unchanged")
feed(TYPE_REPORT, build_report(11))
check("target still 7", w.page_targeted.target_num(), 7)
check("line still A4", "A4" in w.page_targeted.lbl_target.text(), True)

print("4) arrive & confirm at target")
feed(TYPE_REPORT, build_report(7))
feed(TYPE_EVENT, build_event(7, 0, 4, 2500))
info = w.page_targeted.lbl_info.text()
check("info has num 7", "7" in info, True)
check("info has A4", "A4" in info, True)
check("route still there", len(w._route_points) > 0, True)

print("5) traverse missed the target -> EVENT calibrates route")
w.page_ctrl._reset()
for num, pos in A_FACE.items():                 # 故意不灌 7 号
    if num != 7:
        feed(TYPE_EVENT, build_event(num, 0, pos, 3000))
w.page_targeted.edit_num.setText("7")
w.page_targeted._on_start_clicked()
check("no route yet (7 not in inventory)", len(w._route_points), 0)
feed(TYPE_EVENT, build_event(7, 0, 4, 4000))    # 无人机报回 A4
check("route calibrated from event", len(w._route_points) > 0, True)

print("6) reset clears target & route")
w.page_ctrl._reset()
check("target cleared", w.page_targeted.target_num(), None)
check("route cleared", len(w._route_points), 0)

print("7) blank input -> random mode follows REPORT")
for num, pos in A_FACE.items():
    feed(TYPE_EVENT, build_event(num, 0, pos, 5000))
w.page_targeted.edit_num.setText("")
w.page_targeted._on_start_clicked()
check("no manual target", w.page_targeted.target_num(), None)
feed(TYPE_REPORT, build_report(9))
check("follows report", w.page_targeted.target_num(), 9)

print()
if fails:
    print(f"!!! {len(fails)} FAILED: {fails}")
    sys.exit(1)
print("ALL PASS")
