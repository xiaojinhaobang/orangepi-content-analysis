#!/usr/bin/env python3
"""激光笔驱动：订阅 /laser/trigger (Bool) → 控制香橙派 GPIO，识别到码就点亮。

引脚与电平（2026-10-07 现场实测确认）：
  · 排针 **pin22 = GPIO1_B0 = 全局 GPIO 40**
    （RK3588 每组 32 线：GPIO1 组基址 32；A=0..7、B=8..15 ⇒ B0 = 32+8+0 = 40）
  · **低电平点亮**（写 0 亮、写 1 灭）

调用方：qr_vision 在解到码时发 /laser/trigger True，0.5s 后发 False。
本节点默认输出高电平（灭），收到 True 才拉低。

注意：需要 root（sysfs GPIO 的 export/direction/value 都是 root 权限），
      所以做成 systemd 服务 laser-driver.service 跑。
"""

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

SYS = '/sys/class/gpio'
DEFAULT_PIN = 40          # GPIO1_B0 = 排针 pin22


def _write(path, value):
    with open(path, 'w') as fh:
        fh.write(str(value))


class LaserDriver(Node):
    def __init__(self):
        super().__init__('laser_driver')
        self.pin = int(self.declare_parameter('gpio', DEFAULT_PIN).value)
        self._setup()
        self._level = None
        self.set_level(1)                     # 默认关（高电平）
        self.create_subscription(Bool, '/laser/trigger', self.on_trig, 10)
        self.get_logger().info(
            '激光驱动就绪：GPIO%d（排针 pin22 / GPIO1_B0，低电平点亮）' % self.pin)

    def _setup(self):
        d = '%s/gpio%d' % (SYS, self.pin)
        # export / direction 只有 root 能写 —— 由 laser-gpio-init.service（root，
        # 开机执行）负责导出引脚、设成 out、并把 value 放开到 666。
        # 这里尽力而为，失败不致命（本节点只需要能写 value）。
        try:
            if not os.path.isdir(d):
                _write('%s/export' % SYS, self.pin)
            _write('%s/direction' % d, 'out')
        except OSError as exc:                # noqa: BLE001
            self.get_logger().warn(
                'GPIO 导出/方向设置跳过（%s）—— 由 laser-gpio-init.service 负责' % exc)
        if not os.path.exists('%s/value' % d):
            raise RuntimeError(
                'GPIO%d 未导出：先跑 sudo systemctl start laser-gpio-init' % self.pin)

    def set_level(self, level):
        if level == self._level:
            return
        _write('%s/gpio%d/value' % (SYS, self.pin), level)
        self._level = level
        self.get_logger().info('激光%s' % ('亮（低电平）' if level == 0 else '灭（高电平）'))

    def on_trig(self, m):
        # 低电平有效：True → 0（亮）
        self.set_level(0 if m.data else 1)


def main():
    rclpy.init()
    node = LaserDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.set_level(1)                 # 退出前确保熄灭
        except Exception:                     # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
