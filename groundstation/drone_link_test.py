# -*- coding: utf-8 -*-
"""
drone_link_test.py -- 机载端(UDP)雏形 / 自测。

它扮演"真机(无人机)那一侧": 开机后周期发 STATUS(像小车一样持续有实时遥测),
收到地面站下行 START 就跑一遍遍历/定点盘点流程, 全部经真实 UDP 链路发送。
只依赖 stdlib + protocol.py + layout.py(均无 Qt), 可原样搬到 RDX X5 上跑,
把占位处换成飞控/相机读数就是真机机载端。

飞行计时是"物理 + 事件驱动"的, 而且**每次运行不一样**(和真机一样):
  - 飞行段: 每段用"加速-匀速-减速"梯形曲线计时, 实际巡航速度在标称 ±8% 内逐段
    随机(短段往往到不了巡航就减速, 用时比距离/巡航长); 垂直按爬降速度。几何全来自
    layout.py 真实坐标, 所以段间距离、换面/换排的长短差异是真实比例。
  - 盘点结束 = 识别子过程"解码成功"那一刻(不是飞到+固定停): 每帧识别耗时在标称
    附近逐帧抖动(~±30%, 模拟 YOLO 推理不是恒定延迟), 按成功率判定是否成功, 失败
    则重试(偶发需重新对准), 成功才发 EVENT。真机接入时把这个识别子过程换成相机
    回调, 时间/抖动自然变真。
因此只需按真机标定几个**标称值**(--cruise/--accel/--rec-attempt 与识别成功率),
每次运行的时间就自动在标称附近真实地波动, 而不是节拍固定不变。

用法
----
本地单机(不开 GUI 的"开始模拟", 真发 UDP 给地面站):
  python drone_link_test.py --run traverse                 # 自动跑一次遍历盘点
  python drone_link_test.py --run traverse --run targeted  # 先遍历建立库存, 再定点
  python drone_link_test.py                                # 只待机发 STATUS, 等地面站下 START
  python drone_link_test.py --selfcheck                    # 无网络自检(帧数/校验和)
  python drone_link_test.py --speed-mult 8 --run traverse  # 整体快 8 倍看流程(仍按物理比例)

两机(PC 地面站 <-> RDX X5):
  机载侧(RDK X5) : python drone_link_test.py --pc <电脑IP> --port 8888
  地面站(电脑)   : 链路 UDP(WiFi), 机载 IP=<RDK X5 IP>, 端口 8888 -> 连接
    - 机载 bind 8888 收下行 START; 上行(STATUS/EVENT/REPORT)发到 <电脑IP>:8888
注意: 本地单机时, 地面站已占用 8888, 本程序收不了下行 -> 用 --run 直接触发流程。

真机标定(飞控/相机到手后, 改这几个默认值或传参):
  --cruise cm/s    平飞巡航速度(货位间水平移动)  默认 50 = 0.5m/s, 待实测
  --rec-attempt s  单次识别尝试耗时(一次检测+解码) 默认 0.6s, 待实测
  识别成功率/重新对准见类内 D_REC_* (默认 0.85/隔 3 次失败需重对位)
标定法: 拿真机在两列货位间飞一段已知距离测耗时反推巡航速度;
        单次识别的耗时与成功率等相机管线实测量后填。

移植到真机时替换点(见类内注释):
  _read_sensors()  : 高度/电量改读飞控(气压/电压)
  targeted 的目标编号: 改由相机 YOLO 识别二维码得到(参考 RDK X5 检测管线)
"""
import argparse
import math
import random
import socket
import threading
import time

from protocol import (
    TYPE_STATUS, TYPE_EVENT, TYPE_REPORT, TYPE_START, TYPE_QUERY,
    FACE_NAMES,
    build_status, build_event, build_report,
    parse_status, parse_event, pack_frame, FrameParser,
)
import layout

# ---- 飞行/识别模型默认值(标称值, 待真机实测后填; 每次运行按分布随机取) ----
D_CRUISE = 50.0        # 平飞巡航速度标称 cm/s (约 0.5 m/s, 保守)
D_ACC = 45.0           # 平飞加减速 cm/s^2(梯形速度曲线用, 影响短段耗时)
D_CLIMB = 30.0         # 垂直爬升/下降速度 cm/s
D_TAKEOFF_EXTRA = 1.2  # 起飞稳定附加 s
D_LAND_EXTRA = 0.8     # 落地停稳附加 s
D_REC_ATTEMPT = 0.6    # 单次识别尝试耗时标称(一帧检测+解码), 每次按 ~±30% 抖动
D_REC_P = 0.85         # 单次尝试成功率(模拟偶发没对准/反光等)
D_REC_REPOSITION = 1.2 # 连续几次失败后"重新对准机位"额外耗时(也抖动) s
D_SPEED_JIT = 0.08     # 每段实际巡航速度相对标称的随机波动 ±8%


def random_layout(seed=None):
    """把 1~24 编号随机贴到 24 个坐标位: {(面idx, 位1..6): 编号}"""
    rng = random.Random(seed)
    nums = list(range(1, 25))
    rng.shuffle(nums)
    m = {}
    for f in range(4):
        for p in range(1, 7):
            m[(f, p)] = nums.pop()
    return m


class MockDrone:
    """机载端模拟。开机: 周期 STATUS 遥测线程; 收到 START -> 跑遍历/定点流程线程。
    计时模型: 节点耗时 = 距离/速度(水平) + 高度差/爬升(垂直) + 悬停, 全乘 1/speed_mult。
    """

    def __init__(self, pc="127.0.0.1", port=8888, hz=2.0, seed=None, rx_port=None,
                 cruise=D_CRUISE, climb=D_CLIMB, acc=D_ACC,
                 rec_attempt=D_REC_ATTEMPT, rec_p=D_REC_P, speed_mult=1.0):
        self.pc = pc
        self.port = port
        self.rx_port = rx_port or port       # 默认收发同端口(两机联调); 单机测下行可分开
        self.period = 1.0 / max(hz, 0.5)     # STATUS 遥测周期(秒)
        self.cruise = max(cruise, 1.0)       # cm/s 标称
        self.climb = max(climb, 1.0)         # cm/s
        self.acc = max(acc, 5.0)             # cm/s^2(梯形加减速)
        self.rec_attempt = max(rec_attempt, 0.05)  # s/次标称(每帧抖动)
        self.rec_p = min(max(rec_p, 0.1), 1.0)     # 单次成功率
        self.mult = max(speed_mult, 0.05)    # 所有时长 ÷ mult (加速演示/测试用)
        self.seed = seed
        self._rng = random.Random(seed)      # 时间/识别/目标的随机源(给 --seed 可复现)
        self.cell_map = random_layout(seed)  # 本机"贴好的二维码布局"
        self.batt = 100.0                    # 电量(%, 由 STATUS 帧逐步下发)

        # 当前遥测快照(锁保护): 状态机 0就绪 1起飞 2飞行 3降落 4落地
        self._tele = {"st": 0, "ht": 0, "batt": int(self.batt)}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._busy = False                   # 正在跑一次流程时忽略新的 START

        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = None
        self._sent = 0

    # ---------------- 链路 ----------------
    def start(self):
        self._bind_rx()
        threading.Thread(target=self._rx_loop, daemon=True).start()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()
        print(f"[drone] boot: tx -> {self.pc}:{self.port}, telemetry {1/self.period:.1f} Hz")
        print(f"[drone] model: cruise={self.cruise:.0f}cm/s(+/-{D_SPEED_JIT*100:.0f}% jit)"
              f" acc={self.acc:.0f}cm/s^2 climb={self.climb:.0f}cm/s")
        print(f"[drone] model: rec_attempt~{self.rec_attempt:.2f}s+/-30% p_ok={self.rec_p:.2f}"
              f" speed_mult=x{self.mult} seed={self.seed}")
        if self.rx is None:
            print("[drone] WARN rx port busy (ground station on same PC?), "
                  "use --run to trigger a flow; downlink START ignored")
        else:
            print(f"[drone] rx on :{self.rx_port} (waiting START ...)")
        self._push_status()

    def stop(self):
        self._stop.set()
        for s in (self.tx, self.rx):
            if s:
                try:
                    s.close()
                except OSError:
                    pass

    def _bind_rx(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("0.0.0.0", self.rx_port))
            s.settimeout(0.3)
            self.rx = s
        except OSError:
            self.rx = None

    def _send(self, typ, payload):
        try:
            self.tx.sendto(pack_frame(typ, payload), (self.pc, self.port))
            self._sent += 1
        except OSError as e:
            print(f"[drone] send err: {e}")

    # ---------------- 两个常驻线程 ----------------
    def _telemetry_loop(self):
        """周期推 STATUS: 这就是"像小车一样持续实时遥测"的部分。"""
        while not self._stop.is_set():
            self._push_status()
            self._stop.wait(self.period)

    def _rx_loop(self):
        while self.rx and not self._stop.is_set():
            try:
                data, _ = self.rx.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            for typ, payload in FrameParser().feed(data):
                self._handle_down(typ, payload)

    def _handle_down(self, typ, payload):
        if typ == TYPE_START and payload:
            mode = payload[0]
            if self._busy:
                print(f"[drone] busy, ignore START mode={mode}")
                return
            self._busy = True
            threading.Thread(
                target=self._run_flow, args=("traverse" if mode == 0 else "targeted",),
                daemon=True).start()
        elif typ == TYPE_QUERY:
            print(f"[drone] QUERY {payload[0] if payload else '?'} "
                  "(real: aim camera, identify, report)")

    # ---------------- 流程(遍历 / 定点) ----------------
    def _run_flow(self, mode):
        try:
            print(f"[drone] START mode={mode}")
            if mode == "traverse":
                self._play(self._traverse_plan())
            else:
                self._play(self._targeted_plan())
            print(f"[drone] flow done ({self._sent} frames sent)")
        finally:
            self._busy = False

    def run_once(self, mode):
        """--run 本地直接触发(不经下行 START)。"""
        self._busy = True
        try:
            print(f"[drone] run_once mode={mode}")
            self._play(self._traverse_plan() if mode == "traverse"
                       else self._targeted_plan())
            print(f"[drone] flow done ({self._sent} frames sent)")
        finally:
            self._busy = False

    def _play(self, steps):
        """按 (绝对时刻ms, 帧类型, payload) 逐步发真实 UDP 帧。
        首元素是各规划的累计时刻(与 build_simulation 同约定), 不是增量。"""
        base = time.time()
        for t_ms, typ, payload in steps:
            target = base + t_ms / 1000.0
            while True:                       # 对齐到绝对时刻, 防 sleep 漂移
                remain = target - time.time()
                if remain <= 0:
                    break
                if self._stop.wait(min(remain, 0.05)):
                    return
            # 同步遥测快照: STATUS 直接取解析值; EVENT 令状态=飞行中、高度=该排高
            if typ == TYPE_STATUS:
                d = parse_status(payload)
                with self._lock:
                    self._tele.update(st=d["state"], ht=d["height"], batt=d["battery"])
            elif typ == TYPE_EVENT:
                d = parse_event(payload)
                row = (d["pos"] - 1) // layout.COLS
                with self._lock:
                    self._tele.update(st=2, ht=layout.ROW_HEIGHTS[row])
            self._send(typ, payload)

    # ---- 飞行计时 ----
    def _leg_s(self, a, b):
        """a/b=(x,y,z)。水平: 梯形速度曲线计时——加/减速到巡航, 短段到不了巡航
        就减速(2*sqrt(d/a)); 实际巡航速度每段在标称 ±D_SPEED_JIT 内随机。
        垂直: 高度差/爬降速度。"""
        hd = math.hypot(a[0] - b[0], a[1] - b[1])
        vd = abs(a[2] - b[2])
        th = 0.0
        if hd > 0.5:
            v = self.cruise * self._rng.uniform(1 - D_SPEED_JIT, 1 + D_SPEED_JIT)
            if hd <= v * v / self.acc:            # 短段: 刚到/没到巡航就要减速
                th = 2.0 * math.sqrt(hd / self.acc)
            else:                                 # 长段: 加速 + 匀速 + 减速
                th = hd / v + v / self.acc
        tv = vd / self.climb if vd > 0.5 else 0.0
        return th + tv

    def _climb_s(self, z):
        """从地面爬到高度 z: 稳定附加 + z/爬升速度。"""
        return D_TAKEOFF_EXTRA + z / self.climb

    def _descend_s(self, z):
        """从高度 z 落到地面 + 停稳。"""
        return D_LAND_EXTRA + z / self.climb

    # ---- 占位: 换真机时改成读飞控/相机 ----
    def _read_sensors(self):
        """return (状态机值, 高度cm, 电量%). 真机: 高度读气压计/飞控, 电量读电压折算。"""
        with self._lock:
            return self._tele["st"], self._tele["ht"], self._tele["batt"]

    def _pick_random_cell(self):
        """定点目标: 从本机布局抽一个二维码位。真机: 相机识别随机二维码得编号。"""
        return self._rng.choice(list(self.cell_map.keys()))

    def _recognize_cell(self):
        """模拟一次识别直到成功, 返回 (耗时秒, 尝试次数)。
        一次尝试 = 一帧检测+解码, 每帧耗时在 rec_attempt 标称附近 ~±30% 随机
        (YOLO 推理本来就不是恒定延迟); 按 rec_p 判定成功, 失败重试;
        连续失败到第 3/6/... 次视为没对准, 重新调整机位(耗时也抖动)。
        **真机替换点**: 改成等待相机 YOLO 检测 + 二维码解码的回调, 返回编号即成功;
        本函数模拟的逐帧耗时/重试自然变成真机实际耗时。"""
        t, tries = 0.0, 0
        while True:
            tries += 1
            t += self.rec_attempt * self._rng.uniform(0.7, 1.3)
            if self._rng.random() < self.rec_p:
                return t, tries
            if tries % 3 == 0:            # 偶发没对准/反光 -> 重新对位再试
                t += D_REC_REPOSITION * self._rng.uniform(0.8, 1.4)

    # ---------------- 规划两条流程(物理计时) ----------------
    def _traverse_plan(self):
        """遍历盘点: 起飞 -> 蛇形 24 位(每格: 飞到=距离/速度, 识别成功才 EVENT) -> 降落。
        首段/换面/换排按真实坐标距离算时, 电量每格降 0.6%。"""
        steps, t = [], 0.0

        def add(dur_s, typ, payload):
            nonlocal t
            t += max(dur_s, 0.02) * 1000.0 / self.mult
            steps.append((round(t), typ, payload))

        def xyz(face_char, pos):
            return layout.cell_xyz(face_char, pos)

        first_fc, first_pos = layout.TRAVERSE_ORDER[0]
        z0 = xyz(first_fc, first_pos)[2]                 # 顶排 140
        b = int(self.batt)
        add(0.4, TYPE_STATUS, build_status(0, 0, b))     # 就绪
        add(0.3, TYPE_STATUS, build_status(1, 0, b))     # 起飞中
        add(self._climb_s(z0), TYPE_STATUS, build_status(2, z0, b))  # 爬到顶排高度
        prev = (layout.TAKEOFF[0], layout.TAKEOFF[1], z0)

        for i, (fc, pos) in enumerate(layout.TRAVERSE_ORDER):
            x, y, z = xyz(fc, pos)
            num = self.cell_map[(FACE_NAMES.index(fc), pos)]
            # 飞到本格(距离/巡航 + 换排的爬降), 飞抵即报该排高度
            add(self._leg_s(prev, (x, y, z)), TYPE_STATUS, build_status(2, z, b))
            # 识别直到成功才 EVENT(尝试含失败重试); 电耗按格计
            b = max(5, int(self.batt - (i + 1) * 0.6))
            rec_s, tries = self._recognize_cell()
            add(rec_s, TYPE_EVENT,
                build_event(num, FACE_NAMES.index(fc), pos, round(t)))
            if tries > 3:
                print(f"[drone] cell {fc}{pos}: QR rec retried {tries}x, "
                      f"took {rec_s:.1f}s (repositioned)")
            add(0.02, TYPE_STATUS, build_status(2, z, b))
            prev = (x, y, z)

        # 降落
        add(self._descend_s(prev[2]), TYPE_STATUS, build_status(3, 40, b))
        add(0.6, TYPE_STATUS, build_status(4, 0, b))
        self.batt = float(b)
        return steps

    def _targeted_plan(self):
        """定点盘点: 识别随机二维码->上报(REPORT) -> 起飞爬升 -> 飞到该排高度识别成功
        (EVENT) -> 降落。坐标由地面站按遍历库存定位/规划, 机载只报到与飞抵。"""
        face, pos = self._pick_random_cell()
        num = self.cell_map[(face, pos)]
        x, y, z = layout.cell_xyz(FACE_NAMES[face], pos)
        print(f"[drone] targeted: sees QR#{num} @ {FACE_NAMES[face]}{pos} (z={z}cm)")

        steps, t = [], 0.0

        def add(dur_s, typ, payload):
            nonlocal t
            t += max(dur_s, 0.02) * 1000.0 / self.mult
            steps.append((round(t), typ, payload))

        b = int(self.batt)
        add(0.3, TYPE_REPORT, build_report(num))          # ① 识别报送编号
        add(0.4, TYPE_STATUS, build_status(0, 0, b))
        add(0.3, TYPE_STATUS, build_status(1, 0, b))      # 起飞
        add(self._climb_s(z), TYPE_STATUS, build_status(2, z, b))   # 爬到该排高度
        add(self._leg_s((layout.TAKEOFF[0], layout.TAKEOFF[1], z), (x, y, z)),
            TYPE_STATUS, build_status(2, z, b))            # 飞抵目标位
        rec_s, tries = self._recognize_cell()              # ② 识别成功才算盘点
        add(rec_s, TYPE_EVENT, build_event(num, face, pos, round(t)))
        if tries > 3:
            print(f"[drone] targeted rec retried {tries}x, took {rec_s:.1f}s")
        add(0.02, TYPE_STATUS, build_status(2, z, b))
        add(self._descend_s(z), TYPE_STATUS, build_status(3, 40, b))
        add(0.6, TYPE_STATUS, build_status(4, 0, b))
        return steps

    # ---------------- 工具 ----------------
    def _push_status(self):
        with self._lock:
            self._send(TYPE_STATUS,
                       build_status(self._tele["st"], self._tele["ht"],
                                    self._tele["batt"]))

    def snapshot(self):
        with self._lock:
            return tuple(self._tele[k] for k in ("st", "ht", "batt"))


# ---------------- 无网络自检 ----------------
def selfcheck():
    """不碰网络: 把两种流程的帧灌进 FrameParser, 验帧数/校验和/关键帧。"""
    drone = MockDrone()
    parser = FrameParser()
    kinds = {}

    def feed(typ, payload):
        frame = pack_frame(typ, payload)
        kinds[typ] = kinds.get(typ, 0) + 1
        for t2, p2 in parser.feed(frame):
            assert t2 == typ and p2 == payload, "parse round-trip mismatch"
            _ = parse_status(p2) if t2 == TYPE_STATUS else \
                parse_event(p2) if t2 == TYPE_EVENT else p2

    for _dt, typ, payload in drone._traverse_plan():
        feed(typ, payload)
    ev = kinds.get(TYPE_EVENT, 0)
    assert ev == 24, f"traverse EVENT should be 24, got {ev}"
    tr_status = kinds.get(TYPE_STATUS, 0)
    assert tr_status >= 8                   # 起飞/换排/降落/落地等状态帧

    kinds = {}
    for _dt, typ, payload in drone._targeted_plan():
        feed(typ, payload)
    tg_status = kinds.get(TYPE_STATUS, 0)
    assert kinds.get(TYPE_REPORT, 0) == 1, "targeted should REPORT once"
    assert kinds.get(TYPE_EVENT, 0) == 1
    assert parser.error_count == 0, f"checksum errors: {parser.error_count}"
    print(f"[selfcheck] OK  traverse: 24 EVENT + {tr_status} STATUS; "
          f"targeted: 1 REPORT + 1 EVENT + {tg_status} STATUS; "
          f"frames parsed {parser.frame_count}, no checksum error")


def main():
    ap = argparse.ArgumentParser(description="机载端模拟(UDP 真发, 物理计时)")
    ap.add_argument("--pc", default="127.0.0.1", help="地面站 IP(真机填电脑 IP)")
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--rx-port", type=int, default=None,
                    help="收下行端口(默认同 --port; 单机测下行时才需分开)")
    ap.add_argument("--hz", type=float, default=2.0, help="STATUS 遥测频率")
    ap.add_argument("--cruise", type=float, default=D_CRUISE,
                    help="平飞巡航速度标称 cm/s(默认 50, 真机标定后填实测)")
    ap.add_argument("--accel", type=float, default=D_ACC,
                    help="平飞加减速 cm/s^2(默认 45, 影响短段耗时)")
    ap.add_argument("--rec-attempt", type=float, default=D_REC_ATTEMPT,
                    help="单次识别尝试标称秒/帧(默认 0.6, 相机管线实测后填)")
    ap.add_argument("--speed-mult", type=float, default=1.0,
                    help="整体时长 ÷mult(演示/测试用快进, 真机标定始终=1)")
    ap.add_argument("--seed", type=int, default=None, help="编号布局种子")
    ap.add_argument("--run", action="append", choices=["traverse", "targeted"],
                    help="本地直接触发一次流程(可重复给: 先遍历再定点)")
    ap.add_argument("--selfcheck", action="store_true")
    args = ap.parse_args()

    if args.selfcheck:
        selfcheck()
        return

    drone = MockDrone(args.pc, args.port, args.hz, args.seed, rx_port=args.rx_port,
                      cruise=args.cruise, acc=args.accel,
                      rec_attempt=args.rec_attempt, speed_mult=args.speed_mult)
    drone.start()
    try:
        if args.run:
            for mode in args.run:
                drone.run_once(mode)
            print("[drone] demo flows done; still streaming STATUS (Ctrl+C to stop)")
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        print("\n[drone] stopped")
    finally:
        drone.stop()


if __name__ == "__main__":
    main()
