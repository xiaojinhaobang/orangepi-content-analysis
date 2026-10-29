#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键起飞 → 悬停 → 自动降落（走飞控自己的一键起飞，我们再插手）。

和「纯飞控自测」同一套飞控动作，区别只有两点：
  1. 起飞命令只发 1 帧 0x66[1,h] —— 407 侧收到后会每毫秒重试
     OneKey_Takeoff() 直到被接受，连发多帧会把飞控的任务步骤推快
     （2026-10-11 实测）。
  2. 全程打印 高度 / 油门 / 模式 / 解锁，爬不上去、掉高会当场报出来，
     不用飞完再猜。

用法:
  python3 ~/diag/hover_auto.py                 # 起飞120 悬停20s 自动降落
  python3 ~/diag/hover_auto.py --h 120 --hov 20
  python3 ~/diag/hover_auto.py --no-land       # 只起飞+悬停，不自动降落
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray, UInt8MultiArray


class HoverAuto(Node):
    def __init__(self):
        super().__init__('hover_auto')
        self.pub = self.create_publisher(Int16MultiArray, '/mission_cmd', 10)
        self.h = None
        self.fc = None      # [mode_sta, unlock_sta, mode_cmd, unlock_cmd, cid, cmd0, cmd1]
        self.thr = None
        self.create_subscription(Int16, '/height', self.on_h, 10)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 10)
        self.create_subscription(Float32MultiArray, '/fc_target', self.on_tgt, 10)

    def on_h(self, m):
        self.h = int(m.data)

    def on_fc(self, m):
        self.fc = [int(x) for x in m.data]

    def on_tgt(self, m):
        if len(m.data) >= 3:
            self.thr = int(m.data[2])

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, sub, h=0):
        m = Int16MultiArray()
        m.data = [int(sub), int(h)]
        self.pub.publish(m)          # 只发 1 帧
        self.spin(0.15)

    def line(self, tag, t0):
        fc = self.fc or []
        print('[%5.1fs] %-8s h=%-5s thr=%-5s mode=%-3s unlock=%-3s cmd1=%-3s' % (
            time.time() - t0, tag, self.h, self.thr,
            fc[0] if len(fc) > 0 else '-', fc[1] if len(fc) > 1 else '-',
            fc[6] if len(fc) > 6 else '-'), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--h', type=float, default=120.0, help='起飞目标高度 cm')
    ap.add_argument('--hov', type=float, default=20.0, help='悬停秒数')
    ap.add_argument('--reach', type=float, default=0.8, help='到达判定 = h * 该系数')
    ap.add_argument('--no-land', action='store_true', help='不自动降落')
    ap.add_argument('--tries', type=int, default=3, help='起飞最多重发几次（只在完全没离地时重发）')
    ap.add_argument('--try-wait', type=float, default=4.0, help='每次起飞后等几秒看有没有离地')
    a = ap.parse_args()

    rclpy.init()
    n = HoverAuto()
    for _ in range(60):
        n.spin(0.1)
        if n.h is not None and n.fc:
            break
    if not n.fc or n.fc[1] != 1:
        print('飞控未解锁（fc_status=%s）→ 先遥控解锁（CH6 高位）' % (n.fc,), flush=True)
        return 1

    t0 = time.time()
    print('就绪：h=%s thr=%s mode=%s 已解锁' % (n.h, n.thr, n.fc[0]), flush=True)
    if n.fc[0] != 2:
        print('注意：飞控模式 =%s（不是 2/定点）' % n.fc[0], flush=True)

    # ---- 起飞：失败自动重发 ----
    # 2026-10-11 实测：0x66[1,h] 这条路大约一半的尝试会"收下命令但不给油门"
    # （cmd1 正常回到 4、h 却不动、thr 停在 32），重发一帧就能起来。
    # 只在"完全没离地"时重发 —— 已经离地又掉下来（cmd1 跳到 6）是另一种失败，
    # 那种情况重发等于在空中重新触发一键起飞，更危险，直接判失败。
    got_air = False
    for attempt in range(1, a.tries + 1):
        print('→ 起飞 #%d：0x66[1,%d]（单帧）' % (attempt, a.h), flush=True)
        n.send(1, a.h)
        t_try = time.time()
        while time.time() - t_try < a.try_wait:
            n.spin(0.3)
            if (n.h or 0) >= 25:
                got_air = True
                break
        if got_air:
            print('✓ 离地（%.1fs，h=%d thr=%s，第 %d 次命令生效）'
                  % (time.time() - t0, n.h, n.thr, attempt), flush=True)
            break
        print('  #%d 没离地（h=%s thr=%s）→ 重发' % (attempt, n.h, n.thr), flush=True)
    if not got_air:
        print('✗ %d 次都没离地 → 放弃（飞控收下命令但没给油门）' % a.tries, flush=True)
        n.line('失败', t0)
        return 2

    # ---- 爬到目标高度 ----
    peak = n.h or 0
    t_air = time.time()
    ok = False
    drop = False
    last_print = 0.0
    while time.time() - t0 < 45:
        n.spin(0.3)
        h = n.h or 0
        peak = max(peak, h)
        if peak >= a.h * a.reach:
            ok = True
            break
        if time.time() - t_air > 6 and h < peak * 0.5:
            drop = True
            break
        if time.time() - last_print >= 1.0:
            last_print = time.time()
            n.line('爬升中', t0)

    if drop:
        print('✗ 爬起来又掉：峰值 %dcm → 现在 %dcm，油门 thr=%s 全程没再往上加' %
              (peak, n.h or 0, n.thr), flush=True)
        print('  这是 cmd1 跳到 6（飞控切进降落步骤）的典型表现，不是动力问题；重跑一次', flush=True)
        return 2
    if not ok:
        print('✗ 45 秒内没爬到 %.0fcm（峰值 %d，thr=%s）' %
              (a.h * a.reach, peak, n.thr), flush=True)
        n.line('超时', t0)
        return 2

    print('✓ 到达 %.0fcm（峰值 %d，耗时 %.1fs）→ 悬停 %.0f 秒' %
          (a.h * a.reach, peak, time.time() - t0, a.hov), flush=True)

    # ---- 悬停 ----
    hs = []
    t1 = time.time()
    last_print = 0.0
    locked = False
    while time.time() - t1 < a.hov:
        n.spin(0.5)
        if n.h is not None:
            hs.append(n.h)
        if n.fc and n.fc[1] != 1:
            locked = True
            break
        if time.time() - last_print >= 1.0:
            last_print = time.time()
            n.line('悬停', t0)
    if hs:
        print('悬停高度 min=%d max=%d avg=%.0f（%d 个采样）' %
              (min(hs), max(hs), sum(hs) / len(hs), len(hs)), flush=True)
    if locked:
        print('✗ 悬停中飞控自己上锁了（unlock=0）→ 中止', flush=True)
        return 3

    if a.no_land:
        print('--no-land：不降落，保持当前位置', flush=True)
        return 0

    # ---- 降落 ----
    print('→ 降落：0x66[2]', flush=True)
    n.send(2, 0)
    t2 = time.time()
    while time.time() - t2 < 30 and (n.h or 99) > 8:
        n.spin(0.3)
    print('落地 h=%s → 锁定 0x66[4]' % n.h, flush=True)
    n.send(4, 0)
    n.spin(1.0)
    n.line('结束', t0)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
