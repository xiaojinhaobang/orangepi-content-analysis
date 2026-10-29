# -*- coding: utf-8 -*-
"""数据模型: 盘点结果表 + 无人机状态。UI 通过 Qt 信号订阅更新。"""
from PySide6.QtCore import QObject, Signal

from protocol import FACE_NAMES, FACE_MAX_POS, pos_name

# 2026-09-28 改编版：单板双面 × 每面 6 位 = 12
TOTAL_CELLS = 12


class Inventory(QObject):
    """24 个坐标位的盘点结果。
    _cells[(face,pos)] -> 编号; _order 记录盘点先后顺序。
    """
    item_added = Signal(int, str)      # (编号, 坐标名) 每次成功盘点一个货物
    changed = Signal()                 # 任意更新(总览页刷新用)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._cells = {}
        self._order = []

    # ---- 更新 ----
    def add_event(self, num: int, face: int, pos: int, ts: int):
        name = pos_name(face, pos)
        self._cells[(face, pos)] = num
        self._order.append((num, name, ts))
        self.item_added.emit(num, name)
        self.changed.emit()

    def reset(self):
        """清空全部盘点结果"""
        self._cells.clear()
        self._order.clear()
        self.changed.emit()

    # ---- 查询 ----
    def query_number(self, num: int):
        """按编号查坐标, 返回坐标名列表(可能多个位置同编号/未找到为空)"""
        return [pos_name(f, p) for (f, p), n in self._cells.items() if n == num]

    def number_at(self, face: int, pos: int):
        return self._cells.get((face, pos))

    def count(self):
        return len(self._cells)

    def done(self):
        return len(self._cells) >= TOTAL_CELLS

    def recent(self, n=8):
        """最近盘点的 n 条, 返回 [(编号, 坐标名, 时间戳)]"""
        return list(reversed(self._order[-n:]))

    def all_cells(self):
        """所有坐标位 -> 编号(未盘点为 None), 按 A1..D6 顺序"""
        out = []
        for face in range(len(FACE_NAMES)):
            for pos in range(1, FACE_MAX_POS + 1):
                out.append((pos_name(face, pos), self._cells.get((face, pos))))
        return out


class DroneState(QObject):
    """无人机遥测状态"""
    changed = Signal()

    STATE_NAMES = {0: "就绪", 1: "起飞中", 2: "飞行中", 3: "降落中", 4: "已落地"}

    def __init__(self, parent=None):
        super().__init__(parent)
        self.state = 0
        self.height = 0
        self.battery = 100
        self.report_num = None          # 要求2-① 识别报送的编号

    def update(self, state=None, height=None, battery=None):
        if state is not None:
            self.state = state
        if height is not None:
            self.height = height
        if battery is not None:
            self.battery = battery
        self.changed.emit()

    def set_report(self, num):
        self.report_num = num
        self.changed.emit()

    def reset(self):
        """复位: 状态/高度/电量/识别报送全部归零"""
        self.state = 0
        self.height = 0
        self.battery = 100
        self.report_num = None
        self.changed.emit()

    @property
    def state_name(self):
        return self.STATE_NAMES.get(self.state, f"未知({self.state})")
