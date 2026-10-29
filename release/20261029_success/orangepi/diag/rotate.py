#!/usr/bin/env python3
"""原地自动旋转测试：发固定偏航角速度，持续指定时间。

用途：方案文档 §2.5 的「原地偏航整圈实验」——判断飞控期望的速度坐标系
      （地面系 or 机体系）。转一圈观察位置是否漂移/画圈。

重要：运行前必须停掉 position_pid_controller，否则两个节点同时往
      /target_velocity_nav 发（PID 发 yaw=0），偏航指令会被打断。

参数:
  yaw_dps     默认 20    偏航角速度(deg/s)，飞控限幅 ±30
  duration_s  默认 18    持续时间(18s × 20dps = 360°)
  delay_s     默认 0     起飞后再等多久开始转

用法: python3 rotate.py --ros-args -p yaw_dps:=20.0 -p duration_s:=18.0
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class Rotate(Node):
    def __init__(self):
        super().__init__('rotate_test')
        self.yaw_dps = float(self.declare_parameter('yaw_dps', 20.0).value)
        self.duration = float(self.declare_parameter('duration_s', 18.0).value)
        self.delay = float(self.declare_parameter('delay_s', 0.0).value)
        self.pub = self.create_publisher(Float32MultiArray, '/target_velocity_nav', 10)


def main():
    rclpy.init()
    n = Rotate()
    if n.delay > 0:
        print('>>> 等 %.1fs 后开始旋转' % n.delay, flush=True)
        time.sleep(n.delay)
    print('>>> 开始旋转: %.1f deg/s x %.1fs (=%.0f deg)'
          % (n.yaw_dps, n.duration, n.yaw_dps * n.duration), flush=True)
    t0 = time.time()
    while time.time() - t0 < n.duration:
        m = Float32MultiArray()
        m.data = [0.0, 0.0, 0.0, n.yaw_dps]
        n.pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.005)
        time.sleep(0.02)
    print('>>> 旋转结束', flush=True)
    # 停转：多发几帧 yaw=0
    for _ in range(20):
        m = Float32MultiArray()
        m.data = [0.0, 0.0, 0.0, 0.0]
        n.pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.005)
        time.sleep(0.02)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
