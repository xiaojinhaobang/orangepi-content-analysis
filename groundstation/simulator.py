# -*- coding: utf-8 -*-
"""
模拟机载端（代替真机联调）。

两种用法:
1) GUI 内嵌: SimulatorCore(frame_out 信号) -> 接到 Link.inject_frame
   python main.py --sim
2) 独立进程真实 UDP: python simulator.py --udp [--port 8888] [--speed 3]
   与 UdpLink 走同一网络路径, 验证真实链路。

模拟一次"遍历盘点"（改编版·单板双面）: 起飞 -> B1 B2 B3 B6 B5 B4 ->
   绕板端 -> A1 A2 A3 A6 A5 A4 -> 降落（顺序取自 layout.TRAVERSE_ORDER）。
1~24 编号随机分布到 12 个坐标位。
"""
import argparse
import random
import socket
import time

from PySide6.QtCore import QObject, Signal, QTimer

from protocol import (
    TYPE_EVENT, TYPE_STATUS, TYPE_ROUTE, TYPE_REPORT,
    build_event, build_status, build_route, build_report, pack_frame, FACE_NAMES,
)
import layout


def random_layout(seed=None):
    """把 1~24 里的 12 个编号随机分配到 12 个坐标位: {(face,pos): number}"""
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


# 《D题货架盘点-香橙派方案基线》§1.1 离线回归样本（现场货号随机，这里仅是测试样本）。
# 单一来源在 layout.REFERENCE_NUMBERS；走真实链路时这份表会被现场识别结果覆盖。
SAMPLE_LAYOUT = layout.reference_layout()


def build_simulation(interval_ms=1800, seed=None):
    """生成模拟时间线: [(delay_ms, type, payload_bytes), ...]
    返回 (events, meta)，meta 含航线与目标位置信息供 UI 使用。
    编号张贴布局: seed=None 用方案基线的离线回归样本(SAMPLE_LAYOUT);
    传 seed 则按种子随机生成, 便于演示"现场随机货号"。"""
    cell_map = dict(SAMPLE_LAYOUT) if seed is None else random_layout(seed)
    events = []
    t = 0

    def push(delta, typ, payload):
        nonlocal t
        t += delta
        events.append((t, typ, payload))

    # 状态: 就绪 -> 起飞 -> 巡航(顶排)高度
    push(300, TYPE_STATUS, build_status(0, 0, 100))
    push(600, TYPE_STATUS, build_status(1, 0, 100))
    push(800, TYPE_STATUS, build_status(2, layout.ROW_HEIGHTS[0], 100))

    # 航线: 按实际盘点顺序生成飞行路径
    route = layout.traversal_points()
    push(400, TYPE_ROUTE, build_route(route))

    # 依次盘点 12 个坐标位（单板双面，实际蛇形顺序）
    #  高度: 扫顶排(排0)飞顶排高度, 扫底排(排1)飞底排高度, 排切换时上报
    #  电量: 随盘点进度线性下降 100% -> 65% (模拟真实放电, 依据可改)
    total = len(layout.TRAVERSE_ORDER)
    prev_row = None
    for i, (face_char, pos) in enumerate(layout.TRAVERSE_ORDER):
        row = (pos - 1) // layout.COLS
        battery = round(100 - (i + 1) / total * 35)      # 100% -> 65%
        height = layout.ROW_HEIGHTS[row]
        if row != prev_row:                              # 切换顶/底排时上报新高度
            push(200, TYPE_STATUS, build_status(2, height, battery))
            prev_row = row
        face = FACE_NAMES.index(face_char)
        num = cell_map[(face, pos)]
        push(interval_ms, TYPE_EVENT, build_event(num, face, pos, t))

    # 降落
    push(300, TYPE_STATUS, build_status(3, 40, 60))
    push(900, TYPE_STATUS, build_status(4, 0, 60))

    # 注意: 遍历流程不发 REPORT(识别报送) —— 那是定点流程识别随机二维码才发的
    meta = {
        "cell_map": cell_map,
        "route": route,
        "report_num": None,
        "takeoff": layout.TAKEOFF,
        "landing": layout.LANDING,
        "warehouse": (layout.WAREHOUSE_W, layout.WAREHOUSE_H),
    }
    return events, meta


def build_targeted_simulation(interval_ms=1800, cell_map=None, target_num=None):
    """定点盘点模拟时间线(要求2):
    无人机识别随机二维码 -> 上报编号 -> 地面站按遍历结果(共享货架布局)定位并
    规划航线(起飞->目标坐标->降落) -> 无人机飞抵识别 -> 传回编号+信息 -> 降落。
    cell_map: 货架布局 {(face,pos): num}。须与遍历盘点共用同一布局(编号才能对上);
              None 则随机生成一份(独立演示用)。
    target_num: 识别到的编号(1~24); None 则随机挑一个(模拟"随机二维码")。
    返回 (events, meta)。"""
    if cell_map is None:
        cell_map = random_layout(None)
    if target_num is None:
        target_num = random.Random().randrange(1, 25)
    # 编号 -> 坐标 (要求2-②: 按遍历盘点的结果定位目标)
    cell_pos = next(((f, p) for (f, p), n in cell_map.items() if n == target_num), None)
    if cell_pos is None:
        raise ValueError(f"编号 {target_num} 不在货架布局中, 请先完成遍历盘点")
    face, pos = cell_pos
    target_name = f"{FACE_NAMES[face]}{pos}"
    height = layout.ROW_HEIGHTS[(pos - 1) // layout.COLS]

    events = []
    t = 0

    def push(delta, typ, payload):
        nonlocal t
        t += delta
        events.append((t, typ, payload))

    # 上电 -> 识别随机二维码, 上报目标编号(要求2-①)
    push(300, TYPE_STATUS, build_status(0, 0, 100))
    push(500, TYPE_REPORT, build_report(target_num))
    # 起飞
    push(600, TYPE_STATUS, build_status(1, 0, 100))
    push(800, TYPE_STATUS, build_status(2, height, 99))
    # 飞抵目标坐标 -> 识别成功, 传回编号+信息(要求2-③, 触发 LED)
    push(1200, TYPE_EVENT, build_event(target_num, face, pos, t))
    # 返航降落
    push(600, TYPE_STATUS, build_status(3, 40, 98))
    push(900, TYPE_STATUS, build_status(4, 0, 98))

    meta = {
        "cell_map": cell_map,
        # face 在 cell_map 里是 int(0..3), layout.* 一律按面字符 'A'..'D' 索引
        "route": layout.targeted_route(FACE_NAMES[face], pos),
        "target": target_name,
        "target_num": target_num,
        "target_name": target_name,
        "report_num": target_num,
        "takeoff": layout.TAKEOFF,
        "landing": layout.LANDING,
        "warehouse": (layout.WAREHOUSE_W, layout.WAREHOUSE_H),
    }
    return events, meta


class SimulatorCore(QObject):
    """GUI 内嵌模拟器: 播放事件时间线, 逐帧 emit frame_out。
    单个 QTimer 按 50ms 轮询推进; stop() 取消定时器, 立即不再发帧。"""
    frame_out = Signal(int, bytes)          # (type, payload)
    finished = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.events = []
        self._meta = {}
        self.mode = "traverse"
        self.cell_map = None                # 当前物理货架布局(遍历/定点共用)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._idx = 0
        self._elapsed = 0
        self._total = 0

    def start(self, interval_ms=1800, seed=None, mode="traverse", cell_map=None,
              target_num=None):
        """mode="traverse" 生成新布局并保存; mode="targeted" 复用 cell_map
        (与遍历盘点同一布局, 编号才能对上), 未给则随机生成一份独立布局。
        target_num: 定点目标编号(手动输入), None=随机二维码。"""
        self.mode = mode
        if mode == "targeted":
            if cell_map is None:
                cell_map = random_layout(None)
            self.cell_map = cell_map
            self.events, self._meta = build_targeted_simulation(
                interval_ms, cell_map, target_num=target_num)
        else:
            self.events, self._meta = build_simulation(interval_ms, seed)
            self.cell_map = self._meta["cell_map"]
        self._idx = 0
        self._elapsed = 0
        self._total = (self.events[-1][0] + 500) if self.events else 0
        self._timer.start(50)

    def _tick(self):
        self._elapsed += 50
        while self._idx < len(self.events) and self.events[self._idx][0] <= self._elapsed:
            _delay, typ, payload = self.events[self._idx]
            self.frame_out.emit(typ, payload)
            self._idx += 1
        if self._elapsed >= self._total:
            self._timer.stop()
            self.finished.emit()

    def stop(self):
        """立即停止: 取消定时器, 不再发任何帧"""
        self._timer.stop()
        self._idx = 0
        self._elapsed = 0

    @property
    def meta(self):
        return self._meta


def run_udp(port=8888, loops=2, speed=3.0, interval_ms=1800, seed=None):
    """独立进程: 通过真实 UDP 向地面站发帧。"""
    events, _ = build_simulation(interval_ms, seed)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = ("127.0.0.1", port)
    prev = 0
    print(f"模拟机载端 -> UDP {target}, {len(events)} 帧, 倍速 x{speed}, 循环 {loops} 遍")
    for loop in range(loops):
        t0 = time.time()
        prev = 0
        for delay, typ, payload in events:
            time.sleep(max(0, (delay - prev) / speed / 1000.0))
            sock.sendto(pack_frame(typ, payload), target)
            prev = delay
        time.sleep(1.0)
        print(f"第 {loop + 1} 遍完成, 实际耗时 {time.time() - t0:.1f}s")
    sock.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="模拟机载端 (UDP)")
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--loops", type=int, default=2)
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--interval", type=int, default=1800)
    ap.add_argument("--seed", type=int, default=None, help="编号布局种子(默认随机, 固定可复现)")
    args = ap.parse_args()
    run_udp(args.port, args.loops, args.speed, args.interval, args.seed)
