# -*- coding: utf-8 -*-
"""网页版地面站的内置模拟机载端（纯标准库，无 Qt）。

从桌面版 simulator.py 摘出来的时间线生成部分，语义保持一致：
  遍历：起飞 → B1 B2 B3 B6 B5 B4 → 绕板端 → A3 A2 A1 A4 A5 A6 → 降落
  默认铺《方案基线》§1.1 的参考货号样本。
"""
import random

import layout
from protocol import (
    TYPE_EVENT, TYPE_STATUS, TYPE_ROUTE, build_event, build_status, build_route,
    FACE_NAMES,
)


def random_layout(seed=None):
    """把 1~24 里的 12 个编号随机分配到 12 个坐标位: {(face_idx, pos): number}"""
    rng = random.Random(seed)
    nums = list(range(1, 25))
    rng.shuffle(nums)
    cell_map = {}
    i = 0
    for face in range(len(FACE_NAMES)):
        for pos in range(1, layout.COLS * layout.ROWS + 1):
            cell_map[(face, pos)] = nums[i]
            i += 1
    return cell_map


def build_simulation(interval_ms=1800, seed=None):
    """返回 [(delay_ms, type, payload_bytes), ...] 与 meta。"""
    cell_map = layout.reference_layout() if seed is None else random_layout(seed)
    events = []
    t = 0

    def push(delta, typ, payload):
        nonlocal t
        t += delta
        events.append((t, typ, payload))

    push(300, TYPE_STATUS, build_status(0, 0, 100))
    push(600, TYPE_STATUS, build_status(1, 0, 100))
    push(800, TYPE_STATUS, build_status(2, layout.ROW_HEIGHTS[0], 100))
    push(400, TYPE_ROUTE, build_route(layout.traversal_points()))

    total = len(layout.TRAVERSE_ORDER)
    prev_row = None
    for i, (face_char, pos) in enumerate(layout.TRAVERSE_ORDER):
        row = (pos - 1) // layout.COLS
        battery = round(100 - (i + 1) / total * 35)
        height = layout.ROW_HEIGHTS[row]
        if row != prev_row:
            push(200, TYPE_STATUS, build_status(2, height, battery))
            prev_row = row
        face = FACE_NAMES.index(face_char)
        push(interval_ms, TYPE_EVENT, build_event(cell_map[(face, pos)], face, pos, int(t)))

    push(300, TYPE_STATUS, build_status(3, 40, 60))
    push(900, TYPE_STATUS, build_status(4, 0, 60))
    return events, {"cell_map": cell_map, "route": layout.traversal_points()}
