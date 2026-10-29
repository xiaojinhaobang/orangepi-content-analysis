#!/usr/bin/env python3
"""磁干扰监测：订阅 /fc_mag（407 转发的 IMU 0x02 帧），实时打印磁场强度与罗盘状态。

为什么需要它：铁柱/货架等金属会把飞控罗盘拉偏 → 航向基准跑偏 → 偏航环持续输出
差速 → 飞机自转/漂移（2026-10-16 实测：铁柱旁悬停 err 25~56° 撞墙；同一架飞机
搬到空地 err 1~8°）。有了数值就能在**起飞前**判断这个点位能不能飞。

/fc_mag 数据 = [mag_x, mag_y, mag_z, mag_sta, bar_sta, tmp, alt_bar]
  · |B| = sqrt(mx²+my²+mz²)   —— 地磁强度，当地干扰会让它明显偏离
  · mag_sta = 飞控自己的罗盘状态字节（飞控手册：罗盘指示 绿=无干扰 / 红=有干扰）

用法：
  python3 ~/diag/mag_monitor.py            # 自动取启动后前 3 秒均值为基准
  python3 ~/diag/mag_monitor.py 1200       # 手动指定基准 |B|（干净位置量到的值）

判据（经验值）：|B| 偏离基准 >20% 就是强干扰 → 别在这里起飞，或去无钢铁处校准。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

BASE_ARG = float(sys.argv[1]) if len(sys.argv) > 1 else None


class MagMon(Node):
    def __init__(self):
        super().__init__('mag_monitor')
        self.rows = []
        self.baseline = BASE_ARG
        self.t0 = time.time()
        self.last = 0.0
        self.create_subscription(Float32MultiArray, '/fc_mag', self.on_mag, 10)

    def on_mag(self, m):
        if len(m.data) < 4:
            return
        mx, my, mz = (float(m.data[0]), float(m.data[1]), float(m.data[2]))
        mag_sta = int(m.data[3])
        bar_sta = int(m.data[4]) if len(m.data) > 4 else -1
        tmp = float(m.data[5]) if len(m.data) > 5 else float('nan')
        alt_bar = float(m.data[6]) if len(m.data) > 6 else float('nan')
        b = math.sqrt(mx * mx + my * my + mz * mz)
        t = time.time() - self.t0
        # 前 3 秒自动取基准
        if self.baseline is None:
            self.rows.append(b)
            if t >= 3.0 and self.rows:
                self.baseline = sum(self.rows) / len(self.rows)
                print('基准 |B| = %.1f（启动前 3 秒均值，%d 个样本）'
                      % (self.baseline, len(self.rows)), flush=True)
            return
        if t - self.last < 1.0:
            return
        self.last = t
        dev = (b - self.baseline) / self.baseline * 100.0 if self.baseline else 0.0
        flag = ''
        if abs(dev) > 20.0:
            flag = '  ⚠⚠ 磁干扰（偏离基准 %.0f%%）→ 别在这里起飞 / 去无钢铁处校准' % dev
        elif abs(dev) > 8.0:
            flag = '  ⚠ 磁场略偏（%.0f%%）' % dev
        print('t=%6.1f  x=%7.1f y=%7.1f z=%7.1f  |B|=%7.1f (%+5.1f%%)  '
              'mag_sta=%d bar_sta=%d  alt_bar=%6.0f  tmp=%5.1f%s'
              % (t, mx, my, mz, b, dev, mag_sta, bar_sta, alt_bar, tmp, flag), flush=True)


def main():
    rclpy.init()
    n = MagMon()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
