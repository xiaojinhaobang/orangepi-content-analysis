#!/usr/bin/env python3
"""高速记录（按话题原始频率，不做 20Hz 降采样）。

现有记录器 flight_log.py 是 20 Hz 采样，速度信号 >10 Hz 的成分全丢；
要看清速度噪声/延迟就得用这个。

记录: /velocity_map(47Hz) /velocity_map_filtered(20Hz) /fc_gyro(200Hz) /fc_vel_echo(20Hz)

用法:
  python3 ~/diag/fast_log.py               # 记 30 秒
  python3 ~/diag/fast_log.py 60 /tmp/x.csv

输出 CSV: t_s,topic,v1,v2,v3
  vmap/vfilt : linear.x, linear.y, angular.z
  gyro       : x, y, z（原始 LSB）
  echo       : vx, vy（飞控手里握着的速度）
"""
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray


class FastLog(Node):
    def __init__(self, fh):
        super().__init__('fast_log')
        self.fh = fh
        self.t0 = time.time()
        self.n = 0
        self.create_subscription(Twist, '/velocity_map', lambda m: self.on_twist('vmap', m), 50)
        self.create_subscription(Twist, '/velocity_map_filtered',
                                 lambda m: self.on_twist('vfilt', m), 50)
        self.create_subscription(Twist, '/velocity_fused',
                                 lambda m: self.on_twist('vfus', m), 50)
        self.create_subscription(Vector3, '/fc_gyro', self.on_gyro, 200)
        self.create_subscription(Int16MultiArray, '/fc_vel_echo', self.on_echo, 50)

    def w(self, topic, a, b, c=None):
        self.fh.write('%.4f,%s,%.4f,%.4f,%s\n'
                      % (time.time() - self.t0, topic, a, b,
                         '' if c is None else '%.4f' % c))
        self.n += 1

    def on_twist(self, tag, m):
        # 统一按 cm/s 存（和 flight_log 的 raw=/filt= 一致），免得分析时看错单位
        self.w(tag, m.linear.x * 100.0, m.linear.y * 100.0, m.angular.z)

    def on_gyro(self, m):
        self.w('gyro', m.x, m.y, m.z)

    def on_echo(self, m):
        d = list(m.data)
        self.w('echo', d[0] if len(d) > 0 else 0, d[1] if len(d) > 1 else 0)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    out = (sys.argv[2] if len(sys.argv) > 2
           else '/tmp/fast_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
    rclpy.init()
    fh = open(out, 'w')
    fh.write('# t_s,topic,v1,v2,v3\n')
    n = FastLog(fh)
    print('>>> 记录到 %s（%.0f 秒）' % (out, secs), flush=True)
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.002)
    fh.close()
    print('>>> done %s  共 %d 条' % (out, n.n), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
