#!/usr/bin/env python3
"""试飞记录：tf 位姿 + 飞控速度回显 + SLAM 速度 → CSV + 每秒屏幕摘要。

用法: ~/diag/flight_trace.py [秒数,默认60]
输出: ~/traces/trace_<时间>.csv
判读: "漂移"列 = 相对记录起点的水平位移，用来回答"起飞后稳住没有"。
"""
import csv
import math
import os
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int16, Int16MultiArray, UInt8MultiArray


class Trace(Node):
    def __init__(self):
        super().__init__('flight_trace')
        self.pose = None
        self.tacc = (0, 0)
        self.status = None
        self.slam = (0.0, 0.0)
        self.link_len = None
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(Int16MultiArray, '/fc_vel_echo', self.on_tacc, 10)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_status, 10)
        self.create_subscription(Int16, '/fc_link_len', self.on_len, 10)
        self.create_subscription(Twist, '/velocity_map', self.on_slam, 10)

    def on_tacc(self, m):
        if len(m.data) >= 2:
            self.tacc = (m.data[0], m.data[1])

    def on_status(self, m):
        if len(m.data) >= 7:
            self.status = list(m.data)

    def on_len(self, m):
        self.link_len = m.data

    def on_slam(self, m):
        self.slam = (m.linear.x, m.linear.y)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    node = Trace()
    outdir = os.path.expanduser('~/traces')
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, time.strftime('trace_%Y%m%d_%H%M%S.csv'))
    fh = open(path, 'w', newline='')
    wr = csv.writer(fh)
    wr.writerow(['t', 'x_m', 'y_m', 'yaw_deg', 'slam_vx_cms', 'slam_vy_cms',
                 'tacc_x', 'tacc_y', 'mode', 'unlock', 'link_len'])
    print('记录 %.0f 秒 -> %s' % (secs, path))

    t0 = time.time()
    x0 = y0 = None
    last = 0.0
    while time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tr = node.buf.lookup_transform('map', 'laser_link', rclpy.time.Time())
            q = tr.transform.rotation
            yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            x = tr.transform.translation.x
            y = tr.transform.translation.y
            node.pose = (x, y, yaw)
            if x0 is None:
                x0, y0 = x, y
        except Exception:
            pass

        now = time.time() - t0
        if node.pose and now - last > 1.0:
            last = now
            st = node.status or [None] * 7
            wr.writerow(['%.1f' % now, '%.3f' % node.pose[0], '%.3f' % node.pose[1],
                         '%.1f' % node.pose[2],
                         '%.1f' % (node.slam[0] * 100), '%.1f' % (node.slam[1] * 100),
                         node.tacc[0], node.tacc[1], st[0], st[1], node.link_len])
            fh.flush()
            drift = 0.0
            if x0 is not None:
                drift = math.hypot(node.pose[0] - x0, node.pose[1] - y0) * 100
            print('[%5.1fs] x=%+.3f y=%+.3f yaw=%+6.1f  drift=%5.1fcm  slam_v=(%+.1f,%+.1f)cm/s  tacc=(%d,%d)  mode=%s lock=%s LEN=%s'
                  % (now, node.pose[0], node.pose[1], node.pose[2], drift,
                     node.slam[0] * 100, node.slam[1] * 100, node.tacc[0], node.tacc[1],
                     st[0], st[1], node.link_len))
    fh.close()
    print('完成:', path)


if __name__ == '__main__':
    main()
