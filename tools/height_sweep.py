#!/usr/bin/env python3
"""自动高度扫描：起降后自动改高度 → 让油门（转速）变化，用于振动频谱分析。

原理：/mission_cmd 发 [0x01, h] = 匿名 0x66 任务起飞/改高度指令
      → 407 侧 OneKey_Takeoff(h) → 飞控爬到新高度 → 油门跟着变
      → 不同转速的窗口能在频谱里分开"转动频率"和"结构谐振频率"

安全：
  · 只发子命令 0x01（改高度），绝不发 0x02/0x03/0x04（降落/返航/锁定）
  · 高度限制在 [H_MIN, H_MAX]，绝不越界
  · 只有"已解锁且高度 > 40cm"时才动作；落地/锁定就复位，可反复飞

用法: python3 ~/diag/height_sweep.py [基准高度] [档1] [档2] [档3] [每档秒数]
      默认: 基准 150 → 175 → 140 → 168 → 回 150，每档 10 秒
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Int16MultiArray, UInt8MultiArray

H_AIR = 40.0
H_MIN, H_MAX = 90, 200          # 安全边界（飞控单位）


class HeightSweep(Node):
    def __init__(self, base, steps, dwell):
        super().__init__('height_sweep')
        self.pub = self.create_publisher(Int16MultiArray, '/mission_cmd', 10)
        self.create_subscription(Int16, '/height', self.on_h, 1)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 1)
        self.base, self.dwell = base, dwell
        # 基准 → 各档 → 回基准
        self.plan = [base] + [s for s in steps] + [base]
        self.h = None
        self.unlock = None
        self.stage = None
        self.t_stage = None
        self.create_timer(0.5, self.tick)
        self.get_logger().info('height_sweep: 计划 %s（每档 %.0fs），高度限制 [%d,%d]'
                               % (self.plan, dwell, H_MIN, H_MAX))

    def on_h(self, m):
        self.h = float(m.data)

    def on_fc(self, m):
        if len(m.data) >= 2:
            self.unlock = int(m.data[1])

    def send(self, h):
        h = max(H_MIN, min(H_MAX, int(h)))
        m = Int16MultiArray()
        m.data = [1, h]
        self.pub.publish(m)
        self.get_logger().info('→ 0x66[1, %d]' % h)

    def tick(self):
        airborne = (self.unlock == 1) and (self.h is not None) and self.h > H_AIR
        if not airborne:
            if self.stage is not None:
                self.get_logger().info('落地/锁定 → 复位，可再次飞')
            self.stage = None
            self.t_stage = None
            return
        if self.stage is None:
            self.stage = 0
            self.t_stage = time.time()
            self.send(self.plan[0])
            return
        if time.time() - self.t_stage >= self.dwell:
            nxt = self.stage + 1
            if nxt >= len(self.plan):
                self.get_logger().info('扫描完成，保持基准 %d cm' % self.plan[-1])
                self.stage = 999          # 完成，不再动作
                return
            self.stage = nxt
            self.t_stage = time.time()
            self.send(self.plan[nxt])


def main():
    a = sys.argv[1:]
    base = float(a[0]) if len(a) > 0 else 150.0
    steps = [float(x) for x in a[1:4]] if len(a) > 3 else [175.0, 140.0, 168.0]
    dwell = float(a[4]) if len(a) > 4 else 10.0
    rclpy.init()
    n = HeightSweep(base, steps, dwell)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
