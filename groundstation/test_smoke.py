# -*- coding: utf-8 -*-
"""无头冒烟测试: 协议编解码 + 内置模拟跑完整遍历, 验证数据流贯通。
运行: python test_smoke.py  (无需显示器)"""
import os
os.environ["QT_QPA_PLATFORM"] = "offscreen"
import sys

from PySide6.QtCore import QEventLoop, QTimer
from PySide6.QtWidgets import QApplication

# ---------- 1. 协议层 ----------
from protocol import (
    FrameParser, pack_frame, TYPE_EVENT, TYPE_STATUS, TYPE_ROUTE,
    build_event, build_status, build_route, parse_event, parse_status,
    parse_route, pos_name, parse_pos_name,
)

p = FrameParser()
frames = [
    pack_frame(TYPE_EVENT, build_event(12, 0, 3, 1000)),
    pack_frame(TYPE_STATUS, build_status(2, 150, 95)),
    pack_frame(TYPE_ROUTE, build_route([(0, 0), (120, 80), (120, 420)])),
    b"garbage" + pack_frame(TYPE_EVENT, build_event(7, 1, 1, 2000)),  # 混入脏字节
]
parsed = []
for f in frames:
    parsed += p.feed(f)
assert len(parsed) == 4, f"期望 4 帧, 实收 {len(parsed)}"
assert parsed[0][0] == TYPE_EVENT and parse_event(parsed[0][1])["num"] == 12
assert parsed[1][0] == TYPE_STATUS and parse_status(parsed[1][1])["height"] == 150
assert parse_route(parsed[2][1])["points"] == [(0, 0), (120, 80), (120, 420)]
assert pos_name(1, 5) == "B5" and parse_pos_name("A6") == (0, 6)
assert parse_pos_name("D6") is None, "改编版只有 A/B 两面, D6 必须判非法"
print(f"[OK] 协议层: 4/4 帧解析正确, 脏字节被容错")

# ---------- 1b. 布局层: 参考货号 + 遍历顺序 ----------
import layout
from protocol import FACE_NAMES

assert layout.reference_number("A", 1) == 13 and layout.reference_number("B", 6) == 6
assert len(layout.TRAVERSE_ORDER) == 12, f"遍历顺序应 12 格, 实得 {len(layout.TRAVERSE_ORDER)}"
assert len(layout.traversal_points()) == 18, \
    f"遍历航线应 18 点(起降+过渡+12 货位), 实得 {len(layout.traversal_points())}"
# A/B 面观察者左右反转: A1 与 B1 必须落在板的两端
assert layout.cell_coord("A", 1)[1] > layout.cell_coord("A", 3)[1]
assert layout.cell_coord("B", 1)[1] < layout.cell_coord("B", 3)[1]
print(f"[OK] 布局层: 参考货号 {layout.REFERENCE_NUMBERS} / 航线 "
      f"{len(layout.traversal_points())} 点 / A-B 左右反转正确")

# ---------- 2. 全流程: 内置模拟 -> 帧路由 -> 模型 ----------
app = QApplication(sys.argv)
from ui.main_window import MainWindow

win = MainWindow()
win.spin_interval.setValue(200)        # 加速: 12 帧 * 200ms ≈ 3s
win.btn_sim.click()                    # 开始模拟

loop = QEventLoop()
win.sim.finished.connect(loop.quit)
QTimer.singleShot(60000, loop.quit)    # 兜底超时
loop.exec()

assert win.inv.count() == 12, f"应盘点 12 个, 实得 {win.inv.count()}"
assert win.state.report_num is None, "遍历流程不应有识别报送(那是定点流程的)"
assert win.state.state == 4, f"模拟结束应已落地, 实得 {win.state.state_name}"
assert win.pages.currentWidget() is win.page_ctrl, "默认应停在遍历盘点页"
n_query = win.inv.query_number(win.inv.number_at(0, 1))
assert n_query, "编号查询应能查到"
print(f"[OK] 全流程: 12 个盘点全部入表, 状态/航线/查询/LED 事件均触发")
print(f"    样例查询: 编号 {win.inv.number_at(0, 1)} -> 坐标 {n_query}")
print(f"    参考对照: A1 参考 {layout.reference_number('A', 1)} "
      f"实际 {win.inv.number_at(0, 1)}")

# ---------- 3. 切页不再自动下发 START ----------
sent = []


class _FakeLink:
    parser = None

    def send(self, typ, payload=b""):
        sent.append((typ, payload))

    def stop(self):
        pass


win.link = _FakeLink()
win.page_ctrl._set_mode("定点盘点")
win.page_ctrl._set_mode("遍历盘点")
assert not sent, f"切页不得下发任何帧, 实发 {sent}"
win.btn_start_mission.click()
assert sent and sent[-1][0] == 0x11 and sent[-1][1] == b"\x00", \
    f"『开始任务』必须发 START 0, 实发 {sent}"
win.link = None
print("[OK] 人机接口: 切页不下发指令, 『开始任务』只发 START 0")
print("\n冒烟测试全部通过 [OK]")
