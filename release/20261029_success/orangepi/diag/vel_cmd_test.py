#!/usr/bin/env python3
"""恒定速度指令测试：验证 /target_velocity → 0x31 → 0x41 CTRL_SPD 这条通道
到底能不能驱动飞机。

原理：飞控固件里 0x41 的 CTRL_SPD_X/Y 只在"定点飞行"模式有效（匿名协议 p14
有效位表），而它的值来自我们的 /target_velocity。之前我们从没给它非零值，
所以无法判断通道是"通的但指令为 0"还是"根本没接上"。

做法：一直发 0，直到**检测到真的离地**（unlock=1 且高度 > 60cm），再发
dur 秒的 (vx, vy)，然后永远回 0（保持 0x31 不断流，飞控看门狗才不会把指令清掉）。
—— 不需要和起飞动作对时间，挂后台就能用。

判读：
  · 日志里 fcTgtVx/fcTgtVy 跟着变 → 通道通；
  · 飞机实际速度朝指令方向偏移约 vx/vy → 飞控真的跟随 CTRL_SPD；
  · 实际速度不变 → 定点模式没吃 CTRL_SPD（或模式不对）。

用法: python3 ~/diag/vel_cmd_test.py [vx] [vy] [持续s]
      默认: vx=0 vy=-3 持续15s   （vy 正=左，负=右；cm/s）
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, UInt8MultiArray


class VelCmdTest(Node):
    def __init__(self, vx, vy, dur):
        super().__init__('vel_cmd_test')
        self.pub = self.create_publisher(Float32MultiArray, '/target_velocity', 10)
        self.create_subscription(Int16, '/height', self.on_h, 1)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 1)
        self.vx, self.vy, self.dur = vx, vy, dur
        self.h = None
        self.unlock = None
        self.t_air = None
        self.last_msg = ''
        self.create_timer(0.05, self.tick)          # 20Hz，和 0x31 的节奏一致
        self.get_logger().info(
            'vel_cmd_test: 等离地(unlock=1 且 h>60cm) 后发 %.0fs 的 v=(%.1f, %.1f) cm/s'
            % (dur, vx, vy))

    def on_h(self, m):
        self.h = int(m.data)

    def on_fc(self, m):
        if len(m.data) >= 2:
            self.unlock = int(m.data[1])

    def tick(self):
        airborne = (self.unlock == 1) and (self.h is not None) and self.h > 60
        if airborne and self.t_air is None:
            self.t_air = time.time()
            self.get_logger().info('离地了 (h=%s unlock=%s) → 开始计时' % (self.h, self.unlock))
        if not airborne:
            self.t_air = None                       # 落地/锁定就复位，可反复飞
        testing = (self.t_air is not None) and (time.time() - self.t_air < self.dur)
        if testing:
            vx, vy = self.vx, self.vy
        else:
            vx, vy = 0.0, 0.0
        m = Float32MultiArray()
        m.data = [float(vx), float(vy), 0.0, 0.0]
        self.pub.publish(m)
        tag = 'TEST' if (vx or vy) else 'zero'
        if tag != self.last_msg:
            self.last_msg = tag
            self.get_logger().info('→ %s v=(%.1f, %.1f) cm/s (h=%s)' % (tag, vx, vy, self.h))


def main():
    a = sys.argv[1:]
    vx = float(a[0]) if len(a) > 0 else 0.0
    vy = float(a[1]) if len(a) > 1 else -3.0
    dur = float(a[2]) if len(a) > 2 else 15.0
    rclpy.init()
    n = VelCmdTest(vx, vy, dur)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
