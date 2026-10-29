#!/usr/bin/env python3
"""悬停位置保持 + 偏航阻尼（合并版，2026-10-11）。

为什么合并：pos_hold 和 yaw_damp 都要发 /target_velocity，
它是 Float32MultiArray[4] = [vx, vy, vz, vyaw]（机体系）。两个节点同时发会
互相覆盖 —— yaw_damp 发的是 [0, 0, 0, vyaw]，会把位置环的 vx/vy 打回 0，
位置环整个失效。所以并进同一个 tick，四个分量一次发出。

位置环（原有）：SLAM 位置 → PI → 机体系 vx/vy（cm/s）→ 飞控 0x41 CTRL_SPD
  为什么放我们这边闭环：飞控自己的定点对 ~1 cm/s 慢漂不纠（20 段历史悬停
  里 19 段向左），但飞控**会跟随** /target_velocity（实测增益 ≈0.55、方向正确）。

偏航阻尼（新增）：飞控陀螺 Z 角速度 + 航向角误差 → vyaw（deg/s，正 = 左转）
      vyaw = -(kd * gz_dps + kyaw * err + kiyaw * ∫err dt)
  用陀螺角速度而非纯角度，是为了相位超前：2026-10-09 用 SLAM 航向直接做偏航
  控制失败过（我们的指令滞后实际角速度 1.0~1.25s，追不住）。

  2026-10-11 追加积分项 kiyaw：整条链路上偏航**没有任何 I** ——
  飞控 PID6（YAW 角度环）的 I 是 NA 改不了，而原外挂环也只有 P+D，
  所以那个恒定偏航力矩无人消除，表现为单向慢漂（实测 +6~12°）。
  加了 kiyaw 才有消静差的能力。积分带限幅 ilim = vmax_dps / kiyaw。

安全：符号搞反会把摆动放大。第一次飞用小增益，如果航向峰峰变大，
      把 kd/kyaw 取负或调小再飞；三个偏航参数全 0 时等价于旧版（纯位置环）。

用法: python3 ~/diag/pos_hold.py [kp] [ki] [tvx] [tvy] [vmax] [kd] [kyaw] [vmax_dps] [kiyaw]
      默认 kp=0.40 ki=0.04 trim=(0.0,-1.8) vmax=12
           kd=0.0 kyaw=0.0 vmax_dps=12 kiyaw=0.0（默认关掉偏航控制）
      本次实验： 0.35 0.05 0.0 -1.8 12 0.25 0.10 12 0.02
"""
import math
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, UInt8MultiArray

H_AIR = 60.0          # cm，高于它才算真的离地
E_DEAD = 1.0          # cm，位置误差死区
GYRO_SCALE = 18.4     # LSB / (deg/s)，和 hover_drift_check 用的一致


def wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0


class PosHold(Node):
    def __init__(self, kp, ki, tvx, tvy, vmax, kd=0.0, kyaw=0.0, vmax_dps=12.0, kiyaw=0.0):
        super().__init__('pos_hold')
        self.kp, self.ki, self.tvx, self.tvy, self.vmax = kp, ki, tvx, tvy, vmax
        self.kd, self.kyaw, self.vmax_dps, self.kiyaw = kd, kyaw, vmax_dps, kiyaw
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(Float32MultiArray, '/target_velocity', 10)
        self.create_subscription(Int16, '/height', self.on_h, 1)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 1)
        self.create_subscription(Vector3, '/fc_gyro', self.on_gyro, 1)
        self.h = None
        self.unlock = None
        self.gz = None
        self.ref = None
        self.psi_ref = None
        self.integ = [0.0, 0.0]
        self.iyaw = 0.0
        self.last_t = None
        self.last_log = 0.0
        self.state = 'ground'
        self.create_timer(0.05, self.tick)      # 20Hz
        self.get_logger().info(
            'pos_hold: kp=%.2f ki=%.2f trim=(%.1f,%.1f) vmax=%.0f cm/s | '
            'yaw: kd=%.2f kyaw=%.2f kiyaw=%.3f vmax=%.0f deg/s'
            % (kp, ki, tvx, tvy, vmax, kd, kyaw, kiyaw, vmax_dps))

    def on_h(self, m):
        self.h = int(m.data)

    def on_fc(self, m):
        if len(m.data) >= 2:
            self.unlock = int(m.data[1])

    def on_gyro(self, m):
        self.gz = float(m.z)

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

    def send(self, vx, vy, vyaw=0.0):
        m = Float32MultiArray()
        m.data = [float(vx), float(vy), 0.0, float(vyaw)]
        self.pub.publish(m)

    def tick(self):
        now = time.time()
        dt = (now - self.last_t) if self.last_t else 0.05
        self.last_t = now
        px, py, psi = self.pose()
        airborne = (self.unlock == 1) and (self.h is not None) and self.h > H_AIR
        ok = airborne and not math.isnan(px) and not math.isnan(psi)

        if not ok:
            if self.state != 'ground':
                self.get_logger().info('落地/锁定 → 位置环停，输出 0')
            self.state = 'ground'
            self.ref = None
            self.psi_ref = None
            self.integ = [0.0, 0.0]
            self.iyaw = 0.0
            self.send(0.0, 0.0, 0.0)
            return

        if self.state == 'ground':
            self.ref = (px, py)
            self.psi_ref = psi
            self.integ = [0.0, 0.0]
            self.iyaw = 0.0
            self.state = 'hold'
            self.get_logger().info(
                '离地 → 记住悬停点 (%.1f, %.1f) cm，锁航向 %.1f°' % (px, py, psi))

        # ---- 水平位置 PI ----
        ex, ey = px - self.ref[0], py - self.ref[1]
        if math.hypot(ex, ey) < E_DEAD:
            ex = ey = 0.0
        self.integ[0] += ex * dt
        self.integ[1] += ey * dt
        ilim = self.vmax / max(self.ki, 1e-6)
        self.integ[0] = max(-ilim, min(ilim, self.integ[0]))
        self.integ[1] = max(-ilim, min(ilim, self.integ[1]))

        # 地图系速度指令（负号 = 往误差反方向推）
        vmx = -(self.kp * ex + self.ki * self.integ[0])
        vmy = -(self.kp * ey + self.ki * self.integ[1])
        # 地图系 → 机体系（和 uart_to_stm32 里那个 R(-yaw) 一致）
        cr, sr = math.cos(math.radians(psi)), math.sin(math.radians(psi))
        vbx = vmx * cr + vmy * sr + self.tvx
        vby = -vmx * sr + vmy * cr + self.tvy
        mag = math.hypot(vbx, vby)
        if mag > self.vmax:
            k = self.vmax / mag
            vbx, vby = vbx * k, vby * k

        # ---- 偏航阻尼 + 积分 ----
        vyaw = 0.0
        if (self.kd or self.kyaw or self.kiyaw) and self.gz is not None:
            gz_dps = self.gz / GYRO_SCALE
            err = wrap_deg(psi - self.psi_ref)
            if self.kiyaw:
                self.iyaw += err * dt
                ilim_y = self.vmax_dps / max(self.kiyaw, 1e-6)
                self.iyaw = max(-ilim_y, min(ilim_y, self.iyaw))
            vyaw = -(self.kd * gz_dps + self.kyaw * err + self.kiyaw * self.iyaw)
            vyaw = max(-self.vmax_dps, min(self.vmax_dps, vyaw))

        self.send(vbx, vby, vyaw)

        if now - self.last_log >= 1.0:
            self.last_log = now
            self.get_logger().info(
                'psi=%+7.1f (err%+6.1f) gz%+6.2f°/s → vyaw%+6.2f°/s | vxy=(%.1f,%.1f)'
                % (psi, wrap_deg(psi - self.psi_ref),
                   (self.gz or 0.0) / GYRO_SCALE, vyaw, vbx, vby))


def main():
    a = sys.argv[1:]
    kp = float(a[0]) if len(a) > 0 else 0.40
    ki = float(a[1]) if len(a) > 1 else 0.04
    tvx = float(a[2]) if len(a) > 2 else 0.0
    tvy = float(a[3]) if len(a) > 3 else -1.8
    vmax = float(a[4]) if len(a) > 4 else 12.0
    kd = float(a[5]) if len(a) > 5 else 0.0
    kyaw = float(a[6]) if len(a) > 6 else 0.0
    vmax_dps = float(a[7]) if len(a) > 7 else 12.0
    kiyaw = float(a[8]) if len(a) > 8 else 0.0
    rclpy.init()
    n = PosHold(kp, ki, tvx, tvy, vmax, kd, kyaw, vmax_dps, kiyaw)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
