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

偏航振荡阻尼器（2026-10-11 v2 追加，默认只记录、不下发）：
  实测飞控偏航回路存在 0.1~0.3Hz 自激振荡（到位后 4~5s 起摆，
  幅值 ±4°、角速度 ±2.7~3.7°/s，摆到 20s 末在降落急降段转走 ~6°）。
  成因：回路里有积分环节（飞控 PID6 角度环）+ 我们的链条往返延迟
  1.8~2.0s → 180° 相位落在 0.1~0.3Hz，环路增益被飞控内环放大到 ≥1。
  香橙派侧唯一可行的压制手段 = "早发现 + 打对相位的整数脉冲"：
    - 带通 0.05~0.5Hz：只在振荡频段出手，避开高频振动与慢漂
    - 相位超前 T_lead≈1.8s：不补相位就是负阻尼（帮它振，而不是压它）
    - 量化：飞控 vyaw 分辨率 1°/s（实测我们发 -1.4 → 飞控回显 -1.0），
      小于 1 的指令被截断成 0，所以输出取整并留死区
  开关走环境变量，不动命令行签名（restart_poshold.sh 仍传 9 个参数）：
    POSHOLD_DAMP = off | log | on    默认 log（只写 CSV，观察相位）
    POSHOLD_DAMP_K     阻尼增益，默认 1.2
    POSHOLD_DAMP_TLEAD 相位超前秒数，默认 1.8
    POSHOLD_DAMP_FHP   带通下沿 Hz，默认 0.05
    POSHOLD_DAMP_FLP   带通上沿 Hz，默认 0.5
    POSHOLD_DAMP_VMAX  阻尼指令限幅 °/s，默认 5.0
    POSHOLD_DAMP_DEAD  死区 °/s，默认 0.8
    POSHOLD_DAMP_CSV   记录文件，默认 /tmp/yaw_damp.csv
  极性/延迟探针（默认关）：POSHOLD_PROBE=on 时，离地后
    2.5~4.5s 发 +3°/s、6.5~8.5s 发 -3°/s（覆盖阻尼器输出），
    用来验证"我们发正 vyaw，飞控是不是真的往正转"以及真实延迟。
  水平死区：POSHOLD_E_DEAD（默认 1.0 cm）—— 误差小于它就不修正。
    2026-10-12 试过 0.2：水平摆（sd ±2~3cm）没有改善，见交接文档。

用法: python3 ~/diag/pos_hold.py [kp] [ki] [tvx] [tvy] [vmax] [kd] [kyaw] [vmax_dps] [kiyaw]
      默认 kp=0.40 ki=0.04 trim=(0.0,-1.8) vmax=12
           kd=0.0 kyaw=0.0 vmax_dps=12 kiyaw=0.0（默认关掉偏航控制）
      本次实验： 0.35 0.05 0.0 -1.8 12 0.25 0.10 12 0.02
"""
import math
import os
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


class YawDamper:
    """0.05~0.5Hz 带通 + 相位超前 → 反向阻尼指令（量化成飞控能执行的整数 °/s）。"""

    def __init__(self, f_hp=0.05, f_lp=0.5, t_lead=1.8, k=1.2,
                 vmax=5.0, dead=0.8):
        self.f_hp, self.f_lp, self.t_lead = f_hp, f_lp, t_lead
        self.k, self.vmax, self.dead = k, vmax, dead
        self.reset()

    def reset(self):
        self.x_prev = None
        self.hp = 0.0          # 高通状态
        self.lp = 0.0          # 低通状态（= 振荡分量估计）
        self.lp_prev = 0.0
        self.dlp = 0.0         # 平滑后的微分
        self.raw = 0.0
        self.q = 0.0

    def update(self, x, dt):
        """x = 偏航角速度 °/s（正 = 左转）。返回 (raw, q)。"""
        dt = min(max(dt, 1e-3), 0.2)
        if self.x_prev is None:
            self.x_prev = x
            return (0.0, 0.0)

        rc_hp = 1.0 / (2.0 * math.pi * self.f_hp)
        a_hp = rc_hp / (rc_hp + dt)
        self.hp = a_hp * (self.hp + x - self.x_prev)      # 一阶高通
        self.x_prev = x

        rc_lp = 1.0 / (2.0 * math.pi * self.f_lp)
        b_lp = dt / (rc_lp + dt)
        self.lp += b_lp * (self.hp - self.lp)             # 一阶低通

        d = (self.lp - self.lp_prev) / dt
        self.lp_prev = self.lp
        self.dlp += 0.25 * (d - self.dlp)                 # 平滑微分

        pred = self.lp + self.t_lead * self.dlp           # 相位超前预测
        cmd = -self.k * pred                              # 反向 = 阻尼
        cmd = max(-self.vmax, min(self.vmax, cmd))
        self.raw = cmd

        if abs(cmd) < self.dead:
            self.q = 0.0                                  # 死区：宁可不发
        else:
            q = int(round(cmd))
            if q == 0:
                q = 1 if cmd > 0 else -1                  # 至少 ±1°/s 才不会被截断
            self.q = float(q)
        return (self.raw, self.q)


class PosHold(Node):
    def __init__(self, kp, ki, tvx, tvy, vmax, kd=0.0, kyaw=0.0, vmax_dps=12.0, kiyaw=0.0):
        super().__init__('pos_hold')
        self.kp, self.ki, self.tvx, self.tvy, self.vmax = kp, ki, tvx, tvy, vmax
        self.kd, self.kyaw, self.vmax_dps, self.kiyaw = kd, kyaw, vmax_dps, kiyaw
        self.e_dead = float(os.environ.get('POSHOLD_E_DEAD', E_DEAD))
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

        # ---- 偏航振荡阻尼器（默认只记录）----
        self.damp_mode = os.environ.get('POSHOLD_DAMP', 'log').strip().lower()
        self.damp = YawDamper(
            f_hp=float(os.environ.get('POSHOLD_DAMP_FHP', 0.05)),
            f_lp=float(os.environ.get('POSHOLD_DAMP_FLP', 0.5)),
            t_lead=float(os.environ.get('POSHOLD_DAMP_TLEAD', 1.8)),
            k=float(os.environ.get('POSHOLD_DAMP_K', 1.2)),
            vmax=float(os.environ.get('POSHOLD_DAMP_VMAX', 5.0)),
            dead=float(os.environ.get('POSHOLD_DAMP_DEAD', 0.8)))
        self.damp_csv = None
        self.damp_rows = 0
        if self.damp_mode != 'off':
            path = os.environ.get('POSHOLD_DAMP_CSV', '/tmp/yaw_damp.csv')
            try:
                self.damp_csv = open(path, 'w')
                self.damp_csv.write(
                    't,airborne,psi,err,gz_dps,bp,pred,damp_raw,damp_q,'
                    'vyaw_pd,vyaw_sent,vbx,vby,dt\n')
                self.get_logger().info('阻尼器 CSV → %s（模式 %s）'
                                       % (path, self.damp_mode))
            except Exception as e:
                self.damp_csv = None
                self.get_logger().warn('阻尼器 CSV 打不开：%s' % e)
        self.get_logger().info(
            '阻尼器: mode=%s k=%.2f t_lead=%.2fs band=%.2f~%.2fHz vmax=%.1f dead=%.1f'
            % (self.damp_mode, self.damp.k, self.damp.t_lead, self.damp.f_hp,
               self.damp.f_lp, self.damp.vmax, self.damp.dead))
        self.probe_on = os.environ.get('POSHOLD_PROBE', 'off').strip().lower() in (
            'on', '1', 'true', 'yes')
        self.t_air = 0.0
        self.probe_v = 0.0
        self.get_logger().info('探针: %s' % ('开（2.5~4.5s +3°/s，6.5~8.5s -3°/s）'
                                             if self.probe_on else '关'))
        self.get_logger().info(
            'pos_hold: kp=%.2f ki=%.2f trim=(%.1f,%.1f) vmax=%.0f cm/s | '
            'yaw: kd=%.2f kyaw=%.2f kiyaw=%.3f vmax=%.0f deg/s | E_DEAD=%.2f cm'
            % (kp, ki, tvx, tvy, vmax, kd, kyaw, kiyaw, vmax_dps, self.e_dead))

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

    def damp_write(self, now, airborne, psi, err, gz_dps, vyaw_pd, vyaw,
                   vbx, vby, dt):
        """写一行阻尼器记录（地面也写，便于起飞前先把格式/量纲验证掉）。"""
        if self.damp_csv is None:
            return
        try:
            self.damp_csv.write(
                '%.2f,%d,%+.2f,%+.2f,%+.3f,%+.3f,%+.3f,%+.3f,%+.1f,'
                '%+.3f,%+.3f,%+.2f,%+.2f,%.4f\n'
                % (now, 1 if airborne else 0, psi, err, gz_dps,
                   self.damp.lp,
                   self.damp.lp + self.damp.t_lead * self.damp.dlp,
                   self.damp.raw, self.damp.q, vyaw_pd, vyaw, vbx, vby, dt))
            self.damp_rows += 1
            if self.damp_rows % 20 == 0:
                self.damp_csv.flush()
        except Exception:
            pass

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
            self.damp.reset()
            self.damp_write(now, False, self.psi_ref if self.psi_ref else 0.0,
                            0.0, (self.gz or 0.0) / GYRO_SCALE,
                            0.0, 0.0, 0.0, 0.0, dt)
            self.send(0.0, 0.0, 0.0)
            return

        if self.state == 'ground':
            self.ref = (px, py)
            self.psi_ref = psi
            self.integ = [0.0, 0.0]
            self.iyaw = 0.0
            self.t_air = 0.0
            self.state = 'hold'
            self.get_logger().info(
                '离地 → 记住悬停点 (%.1f, %.1f) cm，锁航向 %.1f°' % (px, py, psi))

        # ---- 水平位置 PI ----
        ex, ey = px - self.ref[0], py - self.ref[1]
        if math.hypot(ex, ey) < self.e_dead:
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
        vyaw_pd = 0.0
        damp_raw = damp_q = 0.0
        gz_dps = (self.gz or 0.0) / GYRO_SCALE
        if (self.kd or self.kyaw or self.kiyaw) and self.gz is not None:
            err = wrap_deg(psi - self.psi_ref)
            if self.kiyaw:
                self.iyaw += err * dt
                ilim_y = self.vmax_dps / max(self.kiyaw, 1e-6)
                self.iyaw = max(-ilim_y, min(ilim_y, self.iyaw))
            vyaw_pd = -(self.kd * gz_dps + self.kyaw * err + self.kiyaw * self.iyaw)
            vyaw_pd = max(-self.vmax_dps, min(self.vmax_dps, vyaw_pd))
            vyaw = vyaw_pd

        # 阻尼器：log 模式只记录，on 模式叠加到 vyaw（默认 log）
        if self.damp_mode != 'off' and self.gz is not None:
            damp_raw, damp_q = self.damp.update(gz_dps, dt)
            if self.damp_mode == 'on':
                vyaw = max(-self.vmax_dps,
                           min(self.vmax_dps, vyaw + damp_q))

        # 极性/延迟探针：覆盖阻尼器输出，发已知台阶
        self.t_air += dt
        self.probe_v = 0.0
        if self.probe_on:
            if 2.5 <= self.t_air < 4.5:
                self.probe_v = 3.0
            elif 6.5 <= self.t_air < 8.5:
                self.probe_v = -3.0
            if self.probe_v:
                vyaw = self.probe_v

        self.damp_write(now, True, psi, wrap_deg(psi - self.psi_ref), gz_dps,
                        vyaw_pd, vyaw, vbx, vby, dt)

        self.send(vbx, vby, vyaw)

        if now - self.last_log >= 1.0:
            self.last_log = now
            self.get_logger().info(
                'psi=%+7.1f (err%+6.1f) gz%+6.2f°/s → vyaw%+6.2f°/s '
                '| damp raw%+5.2f q%+5.1f | vxy=(%.1f,%.1f)'
                % (psi, wrap_deg(psi - self.psi_ref),
                   (self.gz or 0.0) / GYRO_SCALE, vyaw,
                   damp_raw, damp_q, vbx, vby))


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
