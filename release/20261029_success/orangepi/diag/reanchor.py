#!/usr/bin/env python3
"""周期重锚：让飞控的定点保持点跟着飞机走。

问题：模式2 的飞控定点把保持点锁在"起飞那一刻的位置"，协议里没有移动它的指令。
     实测弹簧刚度 K≈0.38/s —— 飞到 1.4m 处拉力约 54cm/s，远超我们 22cm/s 的
     速度上限，结果是"能飞出去，守不住，被拽回起飞点"。

做法：每隔 period_s 重发一次 0x66[1, 当前高度]，把保持点重新锚定到飞机当前位置。
     弹簧只累积最近 period_s 的位移（1s @20cm/s = 20cm → 拉力 ~8cm/s），
     在我们能力范围内，于是能持续前进；到点停下后最后一次重锚就把它锁在目标处。

安全：只有 模式=2 且 已解锁 时才重锚；高度取实时 /height，不会自己爬升。

参数: period_s (默认1.0) / min_height_cm (默认20，离地才重锚)
用法: reanchor.py --ros-args -p period_s:=1.0
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Int16MultiArray, UInt8, UInt8MultiArray


class ReAnchor(Node):
    def __init__(self):
        super().__init__('reanchor')
        self.period = float(self.declare_parameter('period_s', 1.0).value)
        self.min_h = float(self.declare_parameter('min_height_cm', 20.0).value)
        self.height = None
        self.fc = None
        self.active = False

        self.pub = self.create_publisher(Int16MultiArray, '/mission_cmd', 10)
        self.create_subscription(Int16, '/height', self.on_h, 10)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 10)
        self.create_subscription(UInt8, '/active_controller', self.on_active, 10)
        self.create_timer(self.period, self.step)
        self.n = 0
        self.get_logger().info('reanchor: period=%.1fs min_h=%.0fcm (只在 模式2+已解锁 时工作)'
                               % (self.period, self.min_h))

    def on_h(self, m):
        self.height = float(m.data)

    def on_fc(self, m):
        if len(m.data) >= 2:
            self.fc = (int(m.data[0]), int(m.data[1]))   # (mode, unlock)

    def on_active(self, m):
        self.active = (m.data == 2)

    def step(self):
        if not self.active or self.height is None or self.fc is None:
            return
        mode, unlock = self.fc
        if mode != 2 or unlock != 1:
            return
        if self.height < self.min_h:
            return
        m = Int16MultiArray()
        m.data = [1, int(round(self.height))]
        self.pub.publish(m)
        self.n += 1
        if self.n % 5 == 1:
            self.get_logger().info('重锚 #%d: 保持点 -> 当前位置 (高度 %dcm)'
                                   % (self.n, int(round(self.height))))


def main():
    rclpy.init()
    node = ReAnchor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
