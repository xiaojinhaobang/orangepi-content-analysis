#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用自己的位置环悬停 N 秒（**不依赖控制台**，控制台最近老挂）。

流程：读当前位姿 → 设目标=当前位置@H cm（我们开始接管）→ 激活位置环
      → 0x66[1,H] 起飞 → 悬停 N 秒 → 0x66[2] 降落 → 0x66[4] 锁定

用法： python3 ~/diag/our_loop_hover.py --h 150 --hov 20
"""
import argparse
import math
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import (Float32MultiArray, Int16, Int16MultiArray, UInt8,
                          UInt8MultiArray)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--h', type=float, default=150.0, help='悬停高度 cm')
    ap.add_argument('--hov', type=float, default=20.0, help='悬停秒数')
    a = ap.parse_args()

    rclpy.init()
    n = Node('our_loop_hover')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    pub_t = n.create_publisher(Float32MultiArray, '/target_position', 10)
    pub_a = n.create_publisher(UInt8, '/active_controller', 10)
    pub_c = n.create_publisher(Int16MultiArray, '/mission_cmd', 10)
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
    try:
        tr = buf.lookup_transform('map', 'dr_base', rclpy.time.Time())
        x = tr.transform.translation.x * 100.0
        y = tr.transform.translation.y * 100.0
        q = tr.transform.rotation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    except Exception as e:  # noqa: BLE001
        print('拿不到 tf map→dr_base: %s → 中止' % e)
        return 1

    print('当前位置 (%.1f, %.1f) yaw=%.1f → 目标=这个位置 @ %.0fcm（我们接管）'
          % (x, y, yaw, a.h), flush=True)
    tgt = Float32MultiArray()
    tgt.data = [x, y, a.h, yaw]
    act = UInt8()
    act.data = 2
    for _ in range(25):
        pub_t.publish(tgt)
        pub_a.publish(act)
        spin(0.05)

    print('发起飞（0x66[1,%d]）' % int(a.h), flush=True)
    take = Int16MultiArray()
    take.data = [1, int(a.h)]
    for _ in range(10):
        pub_c.publish(take)
        spin(0.1)
    t0 = time.time()
    while time.time() - t0 < 25 and (st['h'] or 0) < a.h * 0.85:
        spin(0.2)
    print('高度 %s cm → 悬停 %.0f 秒（我们的位置环在保持）' % (st['h'], a.hov), flush=True)
    spin(a.hov)

    print('发降落（0x66[2]）', flush=True)
    land = Int16MultiArray()
    land.data = [2, 0]
    for _ in range(10):
        pub_c.publish(land)
        spin(0.1)
    t0 = time.time()
    while time.time() - t0 < 30 and (st['h'] or 99) > 8:
        spin(0.2)
    print('落地高度 %s cm → 发锁定（0x66[4]）' % st['h'], flush=True)
    lock = Int16MultiArray()
    lock.data = [4, 0]
    for _ in range(8):
        pub_c.publish(lock)
        spin(0.1)
    print('完成', flush=True)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
