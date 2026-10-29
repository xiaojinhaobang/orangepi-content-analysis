#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gyro_log — 全速记录飞控原始 IMU（/fc_gyro ~200Hz + /fc_acc），用于振动频谱分析。

为什么：机上 rec 日志只采 20Hz，看不到桨带来的 80~120Hz 振动；要分辨
  · 桨不平衡 → 窄带峰，出现在"转速基频 f0"上
  · 桨叶/桨夹问题 → 峰出现在 "桨叶数 × f0"（2 叶=2f0）
  · 气流湍流 → 宽频抬升（没有明显窄峰）
必须按原始采样率记录。

用法：
  source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=6
  python3 ~/diag/gyro_log.py /tmp/vib_idle.csv 20      # 记 20 秒
输出：CSV 头 t,gx,gy,gz,ax,ay,az（t 为相对秒；gyro 单位 LSB，18.4 LSB=1°/s）
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class GyroLog(Node):
    def __init__(self, path, secs):
        super().__init__('gyro_log')
        self.secs = secs
        self.f = open(path, 'w', buffering=1)
        self.f.write('t,gx,gy,gz,ax,ay,az\n')
        self.n = 0
        self.acc = (0.0, 0.0, 0.0)
        self.t0 = time.time()
        self.create_subscription(Vector3, '/fc_gyro', self.on_g, 100)
        self.create_subscription(Vector3, '/fc_acc', self.on_a, 100)
        print('gyro_log: 写 %s，采 %.0f 秒…（桨转起来即可）' % (path, secs), flush=True)

    def on_a(self, m):
        self.acc = (float(m.x), float(m.y), float(m.z))

    def on_g(self, m):
        t = time.time() - self.t0
        if t > self.secs:
            if self.n:
                print('完成：%d 样本，平均 %.1f Hz → %s'
                      % (self.n, self.n / t, '/tmp/vib_idle.csv'), flush=True)
                self.n = 0
            raise SystemExit(0)
        self.n += 1
        self.f.write('%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n'
                     % (t, m.x, m.y, m.z, self.acc[0], self.acc[1], self.acc[2]))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/vib_idle.csv'
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    rclpy.init()
    n = GyroLog(path, secs)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
