#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直线航段测试（空地，不跑地图 / 不跑航线）。

一键起飞 → 按「四方向 × 速度档」发机体系速度阶跃 → 自动降落。
只往 /target_velocity 发速度，位置由飞控自己的 mode 2（定点）兜着 ——
所以测的是"单段直线飞得稳不稳、快不快"：
  航向稳不稳 / 轨迹直不直 / 高度掉不掉 / 速度跟不跟得上。

用法:
  python3 ~/diag/leg_test.py                                  # f,b,r,l × 10,20 cm/s，每段 3s
  python3 ~/diag/leg_test.py --h 150 --speeds 10,20,30 --t 3 --gap 2.5
  python3 ~/diag/leg_test.py --dirs f,b --speeds 20 --t 4
  LEG_YAW_DAMP=1 python3 ~/diag/leg_test.py ...               # 顺带叠加偏航阻尼（默认关）

环境变量:
  LEG_YAW_DAMP=0|1    叠加偏航阻尼器（默认 0 = 纯飞控守航向）
  LEG_DAMP_K / LEG_DAMP_DEAD / LEG_DAMP_TLEAD / LEG_DAMP_FHP / LEG_DAMP_FLP / LEG_DAMP_VMAX
  LEG_CSV=/tmp/leg_test.csv
"""
import argparse
import math
import os
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray, UInt8MultiArray

GYRO_SCALE = 18.4          # LSB/(deg/s)，和 pos_hold 一致


def wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0


class YawDamper:
    """0.05~0.5Hz 带通 + 相位超前（速率阻尼）+ 航向误差项（角度保持），量化成整数 °/s。

    为什么要有角度项 kyaw：带通看不见"朝一个方向慢慢转"的恒定分量（DC 被高通滤掉），
    而飞控自己的偏航角度环没有积分（I=NA）→ 停着不动时那股恒定偏航力矩没人管。
    kyaw 项顶着它：速率归零，只留一个小静差。
    """

    def __init__(self, f_hp, f_lp, t_lead, k, vmax, dead, kyaw=0.0):
        self.f_hp, self.f_lp, self.t_lead = f_hp, f_lp, t_lead
        self.k, self.vmax, self.dead = k, vmax, dead
        self.kyaw = kyaw
        self.x_prev = None
        self.hp = 0.0
        self.lp = 0.0
        self.lp_prev = 0.0
        self.dlp = 0.0
        self.raw = self.q = 0.0

    def update(self, x, dt, err_deg=0.0):
        dt = min(max(dt, 1e-3), 0.2)
        if self.x_prev is None:
            self.x_prev = x
            return 0.0
        rc_hp = 1.0 / (2.0 * math.pi * self.f_hp)
        self.hp = (rc_hp / (rc_hp + dt)) * (self.hp + x - self.x_prev)
        self.x_prev = x
        rc_lp = 1.0 / (2.0 * math.pi * self.f_lp)
        self.lp += (dt / (rc_lp + dt)) * (self.hp - self.lp)
        d = (self.lp - self.lp_prev) / dt
        self.lp_prev = self.lp
        self.dlp += 0.25 * (d - self.dlp)
        cmd = -(self.k * (self.lp + self.t_lead * self.dlp) + self.kyaw * err_deg)
        cmd = max(-self.vmax, min(self.vmax, cmd))
        self.raw = cmd
        if abs(cmd) < self.dead:
            self.q = 0.0
        else:
            q = int(round(cmd))
            self.q = float(q if q else (1 if cmd > 0 else -1))
        return self.q


class LegTest(Node):
    def __init__(self, args):
        super().__init__('leg_test')
        self.a = args
        self.cmd_pub = self.create_publisher(Int16MultiArray, '/mission_cmd', 10)
        self.vel_pub = self.create_publisher(Float32MultiArray, '/target_velocity', 10)
        self.h = None
        self.fc = None
        self.thr = None
        self.gz = None
        self.create_subscription(Int16, '/height', self.on_h, 10)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 10)
        self.create_subscription(Float32MultiArray, '/fc_target', self.on_tgt, 10)
        self.create_subscription(Vector3, '/fc_gyro', self.on_gyro, 10)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.damp = None
        if args.yaw_damp:
            self.damp = YawDamper(args.damp_fhp, args.damp_flp, args.damp_tlead,
                                  args.damp_k, args.damp_vmax, args.damp_dead,
                                  kyaw=args.kyaw)
        self.csv = None
        try:
            self.csv = open(args.csv, 'w')
            self.csv.write('t,phase,dir,v_cmd,vyaw_cmd,damp_q,psi,h,gz_dps,vx_act,vy_act,x,y,psi_err\n')
        except Exception as e:
            print('CSV 打不开：%s' % e, flush=True)
        self.vx = self.vy = self.vyaw = 0.0
        self.t0 = time.time()
        self.last_t = None
        self.last_log = 0.0
        self.phase = 'init'
        self.dir_name = '-'
        self.v_cmd = 0.0
        self.psi_ref = None

    # ---- 订阅 ----
    def on_h(self, m):
        self.h = int(m.data)

    def on_fc(self, m):
        self.fc = [int(x) for x in m.data]

    def on_tgt(self, m):
        if len(m.data) >= 3:
            self.thr = int(m.data[2])

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

    # ---- 基础动作 ----
    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send_cmd(self, sub, h=0):
        m = Int16MultiArray()
        m.data = [int(sub), int(h)]
        self.cmd_pub.publish(m)          # 只发 1 帧
        self.spin(0.15)

    def publish_vel(self, vx, vy, vyaw):
        m = Float32MultiArray()
        m.data = [float(vx), float(vy), 0.0, float(vyaw)]
        self.vel_pub.publish(m)
        self.vx, self.vy, self.vyaw = vx, vy, vyaw

    def log_row(self, now, px, py, psi, err=0.0):
        if self.csv is None:
            return
        gz = (self.gz or 0.0) / GYRO_SCALE
        try:
            self.csv.write('%.2f,%s,%s,%.2f,%.2f,%.1f,%.2f,%s,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f\n'
                           % (now, self.phase, self.dir_name, self.v_cmd, self.vyaw,
                              self.damp.q if self.damp else 0.0, psi, self.h, gz,
                              self.vx, self.vy, px, py, err))
        except Exception:
            pass

    def status(self, tag):
        fc = self.fc or []
        print('[%5.1fs] %-10s %-4s h=%-5s thr=%-5s mode=%-3s unlock=%-3s cmd1=%-3s v=%.0f'
              % (time.time() - self.t0, tag, self.dir_name, self.h, self.thr,
                 fc[0] if len(fc) > 0 else '-', fc[1] if len(fc) > 1 else '-',
                 fc[6] if len(fc) > 6 else '-', self.v_cmd), flush=True)

    def hold(self, vx, vy, sec, phase, dir_name, v_cmd):
        """按 20Hz 发 (vx,vy,vyaw) 持续 sec 秒（vyaw = 速率阻尼 + 航向保持，关时恒 0）。"""
        self.phase, self.dir_name, self.v_cmd = phase, dir_name, v_cmd
        t0 = time.time()
        last = t0
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - last
            last = now
            px, py, psi = self.pose()
            err = 0.0
            if self.psi_ref is not None and not math.isnan(psi):
                err = wrap_deg(psi - self.psi_ref)
            vyaw = 0.0
            if self.damp is not None and self.gz is not None:
                vyaw = self.damp.update((self.gz or 0.0) / GYRO_SCALE, dt, err)
            self.publish_vel(vx, vy, vyaw)
            self.log_row(now, px, py, psi, err)
            if now - self.last_log >= 1.0:
                self.last_log = now
                self.status(phase)


# ROS 机体系：+x=前、+y=**左**（REP-103）→ 所以"右"是 −y
DIRS = {'f': (1, 0), 'b': (-1, 0), 'r': (0, -1), 'l': (0, 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--h', type=float, default=150.0, help='起飞目标高度 cm')
    ap.add_argument('--reach', type=float, default=0.75, help='到达判定 = h*该系数')
    ap.add_argument('--settle', type=float, default=3.0, help='起飞后先稳几秒')
    ap.add_argument('--t', type=float, default=3.0, help='每段速度持续秒数')
    ap.add_argument('--gap', type=float, default=2.5, help='两段之间停几秒')
    ap.add_argument('--speeds', default='10,20', help='速度档 cm/s，逗号分隔')
    ap.add_argument('--dirs', default='f,r,b,l',
                    help='方向 f=前 b=后 r=右 l=左（ROS 机体系 +y=左）')
    ap.add_argument('--tries', type=int, default=3)
    ap.add_argument('--try-wait', type=float, default=4.0)
    ap.add_argument('--no-land', action='store_true')
    a = ap.parse_args()
    a.csv = os.environ.get('LEG_CSV', '/tmp/leg_test.csv')
    a.yaw_damp = os.environ.get('LEG_YAW_DAMP', '0') in ('1', 'on', 'true', 'yes')
    a.damp_k = float(os.environ.get('LEG_DAMP_K', 2.2))
    a.damp_dead = float(os.environ.get('LEG_DAMP_DEAD', 0.5))
    a.damp_tlead = float(os.environ.get('LEG_DAMP_TLEAD', 0.2))
    a.damp_fhp = float(os.environ.get('LEG_DAMP_FHP', 0.05))
    a.damp_flp = float(os.environ.get('LEG_DAMP_FLP', 0.5))
    a.damp_vmax = float(os.environ.get('LEG_DAMP_VMAX', 5.0))
    a.kyaw = float(os.environ.get('LEG_KYAW', 0.25))
    speeds = [float(x) for x in a.speeds.split(',') if x.strip()]
    dirs = [x.strip() for x in a.dirs.split(',') if x.strip() in DIRS]

    rclpy.init()
    n = LegTest(a)
    for _ in range(60):
        n.spin(0.1)
        if n.h is not None and n.fc:
            break
    if not n.fc or n.fc[1] != 1:
        print('飞控未解锁（fc_status=%s）→ 先遥控解锁（CH6 高位）' % n.fc, flush=True)
        return 1
    print('就绪：h=%s thr=%s mode=%s 已解锁 | 方向 %s | 速度 %s | 每段 %.1fs'
          % (n.h, n.thr, n.fc[0], dirs, speeds, a.t), flush=True)
    if n.fc[0] != 2:
        print('注意：飞控模式 =%s（不是 2/定点）' % n.fc[0], flush=True)
    print('偏航阻尼：%s（速率阻尼 k=%.2f + 角度保持 kyaw=%.2f，t_lead=%.2fs）'
          % ('开' if a.yaw_damp else '关（纯飞控守航向）',
             a.damp_k, a.kyaw, a.damp_tlead), flush=True)

    # ---- 起飞（复用 hover_auto 的重发策略）----
    got_air = False
    for attempt in range(1, a.tries + 1):
        print('→ 起飞 #%d：0x66[1,%d]（单帧）' % (attempt, a.h), flush=True)
        n.phase = 'takeoff'
        n.send_cmd(1, a.h)
        t_try = time.time()
        while time.time() - t_try < a.try_wait:
            n.spin(0.3)
            if (n.h or 0) >= 25:
                got_air = True
                break
        if got_air:
            print('✓ 离地（%.1fs）' % (time.time() - n.t0), flush=True)
            break
        print('  #%d 没离地（h=%s）→ 重发' % (attempt, n.h), flush=True)
    if not got_air:
        print('✗ 没离地 → 放弃', flush=True)
        return 2

    t_air = time.time()
    peak = n.h or 0
    while time.time() - t_air < 30:
        n.spin(0.3)
        peak = max(peak, n.h or 0)
        if (n.h or 0) >= a.h * a.reach:
            break
        if time.time() - t_air > 6 and (n.h or 0) < peak * 0.5:
            print('✗ 爬起来又掉（峰值 %d → %d）→ 中止' % (peak, n.h or 0), flush=True)
            return 2
    print('✓ 到达 %.0fcm（峰值 %d，耗时 %.1fs）' % (a.h * a.reach, peak, time.time() - n.t0),
          flush=True)

    # ---- 记住起飞后的航向作为基准（后面全程保持不转机头）----
    n.spin(0.5)
    _px, _py, _psi = n.pose()
    if not math.isnan(_psi):
        n.psi_ref = _psi
        print('✓ 航向基准 psi_ref=%.1f°（本轮全程保持这个机头方向）' % _psi, flush=True)
    else:
        print('⚠ 拿不到 TF 航向 → 角度保持项本次不生效', flush=True)

    # ---- 先稳一下（记一段"零速"基线）----
    n.hold(0.0, 0.0, a.settle, 'settle', '-', 0.0)

    # ---- 四方向 × 速度档 ----
    for v in speeds:
        for dname in dirs:
            ux, uy = DIRS[dname]
            n.hold(ux * v, uy * v, a.t, 'leg', dname, v)
            n.hold(0.0, 0.0, a.gap, 'gap', dname, 0.0)
            if n.fc and n.fc[1] != 1:
                print('✗ 飞控上锁了（unlock=0）→ 中止', flush=True)
                return 3

    # ---- 收尾 ----
    n.hold(0.0, 0.0, 2.0, 'end', '-', 0.0)
    if n.csv:
        n.csv.flush()
    if a.no_land:
        print('--no-land：保持悬停', flush=True)
        return 0
    print('→ 降落：0x66[2]', flush=True)
    n.phase = 'land'
    n.send_cmd(2, 0)
    t2 = time.time()
    while time.time() - t2 < 30 and (n.h or 99) > 8:
        n.spin(0.3)
    print('落地 h=%s → 锁定' % n.h, flush=True)
    n.send_cmd(4, 0)
    n.spin(1.0)
    if n.csv:
        n.csv.close()
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
