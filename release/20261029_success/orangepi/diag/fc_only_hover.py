#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯飞控悬停测试（我们完全不参与）：0x66 起飞 → 悬停 N 秒 → 降落 → 锁定。

不依赖控制台。用法： python3 ~/diag/fc_only_hover.py --h 150 --hov 20
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Int16MultiArray, UInt8MultiArray


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--h', type=float, default=150.0)
    ap.add_argument('--hov', type=float, default=20.0)
    a = ap.parse_args()

    rclpy.init()
    n = Node('fc_only_hover')
    pub_cmd = n.create_publisher(Int16MultiArray, '/mission_cmd', 10)
    st = {'h': None, 'fc': None}
    n.create_subscription(Int16, '/height', lambda m: st.__setitem__('h', int(m.data)), 5)

    def on_fc(m):
        if len(m.data) >= 2:
            st['fc'] = (int(m.data[0]), int(m.data[1]))
    n.create_subscription(UInt8MultiArray, '/fc_status', on_fc, 5)

    def spin(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(n, timeout_sec=0.05)

    for _ in range(60):
        spin(0.1)
        if st['h'] is not None and st['fc']:
            break
    if not st['fc'] or st['fc'][1] != 1:
        print('飞控未解锁（/fc_status=%s）→ 中止' % (st['fc'],))
        return 1
    print('飞控 mode=%d 已解锁，初始高度 %s cm' % (st['fc'][0], st['h']), flush=True)

    take = Int16MultiArray()
    take.data = [1, int(a.h)]
    for _ in range(10):
        pub_cmd.publish(take)
        spin(0.1)
    t0 = time.time()
    while time.time() - t0 < 25 and (st['h'] or 0) < a.h * 0.85:
        spin(0.2)
    print('高度 %s cm → 悬停 %.0f 秒（纯飞控，我们不管）' % (st['h'], a.hov), flush=True)
    spin(a.hov)

    print('发降落（0x66[2]）', flush=True)
    land = Int16MultiArray()
    land.data = [2, 0]
    for _ in range(10):
        pub_cmd.publish(land)
        spin(0.1)
    t0 = time.time()
    while time.time() - t0 < 30 and (st['h'] or 99) > 8:
        spin(0.2)
    print('落地高度 %s cm → 锁定（0x66[4]）' % st['h'], flush=True)
    lock = Int16MultiArray()
    lock.data = [4, 0]
    for _ in range(8):
        pub_cmd.publish(lock)
        spin(0.1)
    print('完成', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
