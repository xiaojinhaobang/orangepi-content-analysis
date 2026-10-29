#!/usr/bin/env python3
"""试飞记录（定时采样版）—— 修正 hover_test 的日志积压问题。

hover_test.py 用单线程 spin_once 每 0.2s 跑一次，却订阅了 6 个高频话题
（约 600 msg/s），结果记录出来的是积压的旧数据：raw 列会连续几秒完全一样。
本脚本改成：所有订阅只做「缓存最新值」，由 20Hz 定时器统一采样写盘。

用法: flight_log.py [秒数,默认60] [输出文件,默认 ~/traces/flight_<时间>.log]
记录列: t x y yaw | filt(注入飞控) raw(雷达原始) tvd(位置环指令) | h mode unlock
判读: 把 raw 和 tvd 对齐看 —— raw 是"看到的速度"，tvd 是"我们的反应"。
"""
import math
import os
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray, UInt8MultiArray


class FlightLog(Node):
    def __init__(self):
        super().__init__('flight_log')
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.filt = (0.0, 0.0)
        self.raw = (0.0, 0.0)
        self.tvd = None
        self.h = None
        self.fc = None
        self.echo = (0, 0)
        self.create_subscription(Twist, '/velocity_map_filtered', self.on_filt, 1)
        self.create_subscription(Twist, '/velocity_map', self.on_raw, 1)
        self.create_subscription(Float32MultiArray, '/target_velocity', self.on_tvd, 1)
        self.create_subscription(Int16, '/height', self.on_h, 1)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 1)
        self.create_subscription(Int16MultiArray, '/fc_vel_echo', self.on_echo, 1)

    def on_filt(self, m):
        self.filt = (m.linear.x * 100.0, m.linear.y * 100.0)

    def on_raw(self, m):
        self.raw = (m.linear.x * 100.0, m.linear.y * 100.0)

    def on_tvd(self, m):
        if len(m.data) >= 2:
            self.tvd = (float(m.data[0]), float(m.data[1]),
                        float(m.data[2]) if len(m.data) > 2 else 0.0,
                        float(m.data[3]) if len(m.data) > 3 else 0.0)

    def on_h(self, m):
        self.h = int(m.data)

    def on_fc(self, m):
        if len(m.data) >= 2:
            self.fc = (int(m.data[0]), int(m.data[1]))

    def on_echo(self, m):
        if len(m.data) >= 2:
            self.echo = (int(m.data[0]), int(m.data[1]))

    def pose(self):
        try:
            tr = self.buf.lookup_transform('map', 'dr_base', rclpy.time.Time())
            q = tr.transform.rotation
            yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            return (tr.transform.translation.x * 100.0,
                    tr.transform.translation.y * 100.0, yaw)
        except Exception:
            return (float('nan'), float('nan'), float('nan'))


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    outdir = os.path.expanduser('~/traces')
    os.makedirs(outdir, exist_ok=True)
    path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        outdir, time.strftime('flight_%Y%m%d_%H%M%S.log'))

    rclpy.init()
    n = FlightLog()
    fh = open(path, 'w')
    print('>>> 定时采样记录到', path, flush=True)
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.005)
        now = time.time() - t0
        if now - last >= 0.05:                    # 20Hz 定时采样，不等回调
            last = now
            tv = n.tvd if n.tvd else (float('nan'),) * 4
            fc = n.fc if n.fc else (None, None)
            px, py, psi = n.pose()
            line = ('t=%6.2f x=%7.2f y=%7.2f psi=%7.1f | filt=(%6.1f,%6.1f) raw=(%6.1f,%6.1f) '
                    'tvd=(%6.1f,%6.1f) z=%6.1f yaw=%6.1f | fc回显=(%4d,%4d) h=%s mode=%s unlock=%s'
                    % (now, px, py, psi, n.filt[0], n.filt[1], n.raw[0], n.raw[1],
                       tv[0], tv[1], tv[2], tv[3], n.echo[0], n.echo[1], n.h, fc[0], fc[1]))
            fh.write(line + '\n')
            fh.flush()
    fh.close()
    print('>>> done ->', path, flush=True)


if __name__ == '__main__':
    main()
