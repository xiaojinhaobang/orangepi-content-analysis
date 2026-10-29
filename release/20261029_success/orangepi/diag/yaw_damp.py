#!/usr/bin/env python3
"""航向阻尼：用飞控陀螺 Z 的角速度反馈，压 6~8 秒周期的航向慢摆。

背景（2026-10-09 实测）：
  · 悬停时航向在 6~8 秒周期上来回摆，摆幅最大 ±15°，而 fcYawCmd 全程 = 0
    → 飞控没有在命令这个摆动，是外部扰动 + 飞控航向环欠阻尼。
  · 位置侧已经确认：飞控会跟随 /target_velocity（0x31 → 0x41 CTRL_SPD/CTRL_YAWDPS）。
  · 之前用 SLAM 航向做偏航控制失败过，相位分析显示"我们的偏航指令滞后实际角速度
    1.0~1.25s"，追不住。所以这次**不用位置角，用陀螺角速度**做反馈（相位超前）。

控制律（机体系，角度：正 = 左转）：
    vyaw = -(kd * gyro_z_dps + kp * wrap(psi - psi_ref))
限制 |vyaw| <= vmax。vyaw 走 /target_velocity 第 4 个字段 → 0x41 CTRL_YAWDPS。

安全：符号搞反会放大摆动。第一次飞用小增益，如果摆幅**变大**立刻降落，
      把 kd 改成负号再飞一次即可（参数都在命令行）。

用法: python3 ~/diag/yaw_damp.py [kd] [kp] [vmax_dps]
      默认 kd=0.40 kp=0.20 vmax=12
"""
import math
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, UInt8MultiArray

GYRO_SCALE = 18.4          # LSB / (deg/s)，和 hover_drift_check 用的一致
H_AIR = 60.0


def wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0


class YawDamp(Node):
    def __init__(self, kd, kp, vmax):
        super().__init__('yaw_damp')
        self.kd, self.kp, self.vmax = kd, kp, vmax
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(Float32MultiArray, '/target_velocity', 10)
        self.create_subscription(Vector3, '/fc_gyro', self.on_gyro, 1)
        self.create_subscription(Int16, '/height', self.on_h, 1)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 1)
        self.gz = None
        self.h = None
        self.unlock = None
        self.psi_ref = None
        self.last_log = 0.0
        self.create_timer(0.05, self.tick)      # 20Hz
        self.get_logger().info('yaw_damp: kd=%.2f kp=%.2f vmax=%.0f°/s (vyaw = -kd*陀螺 - kp*航向误差)'
                               % (kd, kp, vmax))

    def on_gyro(self, m):
        self.gz = float(m.z)

    def on_h(self, m):
        self.h = int(m.data)

    def on_fc(self, m):
        if len(m.data) >= 2:
            self.unlock = int(m.data[1])

    def yaw(self):
        try:
            tr = self.buf.lookup_transform('map', 'dr_base', rclpy.time.Time())
            q = tr.transform.rotation
            return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        except Exception:
            return float('nan')

    def send(self, vyaw):
        m = Float32MultiArray()
        m.data = [0.0, 0.0, 0.0, float(vyaw)]
        self.pub.publish(m)

    def tick(self):
        airborne = (self.unlock == 1) and (self.h is not None) and self.h > H_AIR
        psi = self.yaw()
        if not (airborne and not math.isnan(psi) and self.gz is not None):
            self.psi_ref = None
            self.send(0.0)
            return
        if self.psi_ref is None:
            self.psi_ref = psi
            self.get_logger().info('离地 → 锁航向 %.1f°' % psi)
        gz_dps = self.gz / GYRO_SCALE
        err = wrap_deg(psi - self.psi_ref)
        vyaw = -(self.kd * gz_dps + self.kp * err)
        vyaw = max(-self.vmax, min(self.vmax, vyaw))
        self.send(vyaw)
        now = time.time()
        if now - self.last_log >= 1.0:
            self.last_log = now
            self.get_logger().info('psi=%+7.1f (err%+6.1f) 陀螺Z%+6.2f°/s → vyaw%+6.2f°/s'
                                   % (psi, err, gz_dps, vyaw))


def main():
    a = sys.argv[1:]
    kd = float(a[0]) if len(a) > 0 else 0.40
    kp = float(a[1]) if len(a) > 1 else 0.20
    vmax = float(a[2]) if len(a) > 2 else 12.0
    rclpy.init()
    n = YawDamp(kd, kp, vmax)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
