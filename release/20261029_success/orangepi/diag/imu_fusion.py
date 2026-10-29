#!/usr/bin/env python3
"""IMU 融合速度：用飞控 200Hz 的 acc/gyro + 姿态，融合雷达速度，输出 /velocity_fused。

目的：/velocity_map 干净但**滞后**（14.4Hz 扫描 + cartographer 处理，约 100~200ms）。
      用 IMU 的高频加速度补上快速变化，用雷达速度做低频绝对修正。

算法（互补滤波 / 一阶观测器）:
    a_nav = R(姿态) · acc_body − g          # 比力去掉重力
    v    += a_nav · dt                       # 200Hz 积分（低延迟）
    v    += K · (v_lidar − v)                # 每次收到雷达速度就拉一把（去漂）

标定（脚本开头会自动做，**期间飞机必须静止**）:
    acc 量纲: 静止 |a| = 1g            → LSB per (m/s^2)
    gyro 零偏: 静止均值
    acc 零偏: mean(acc) − R^T·[0,0,g]
    导航系偏航: psi(SLAM) − yaw(IMU)   → 把 IMU 姿态转到与 /velocity_map 同系

用法:
  python3 ~/diag/imu_fusion.py              # 标定 8 秒，然后一直跑
  python3 ~/diag/imu_fusion.py 12           # 标定 12 秒
  python3 ~/diag/imu_fusion.py 8 --dry      # 只标定+打印，不发控制
输出: /velocity_fused (geometry_msgs/Twist, m/s, 与 /velocity_map 同坐标系)
"""
import math
import sys
import time
from collections import deque

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

G = 9.80665


def rot_nav_from_body(roll, pitch, yaw):
    """R = Rz(yaw)·Ry(pitch)·Rx(roll)，body -> nav"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


class Fusion(Node):
    def __init__(self, calib_s, logpath=None):
        super().__init__('imu_fusion')
        self.calib_s = calib_s
        self.t_start = time.time()
        self.calibrated = False
        # 自写日志（和 fast_log 同格式，可直接用 fast_analyze.py 分析）
        self.logf = open(logpath, 'w') if logpath else None
        if self.logf:
            self.logf.write('# t_s,topic,v1,v2,v3\n')
        # 原始采样缓存（标定用）
        self.accs, self.gyros, self.rpys = [], [], []
        # 标定结果
        self.acc_bias = [0.0, 0.0, 0.0]
        self.acc_lsb_per_mps2 = 1.0
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.yaw_off = 0.0
        self.psi0 = None
        # 状态
        self.v = [0.0, 0.0, 0.0]
        self.v_lidar = [0.0, 0.0, 0.0]
        self.last_t = None
        self.att_now = None
        self.last_acc_t = None
        self.a_nav = [0.0, 0.0, 0.0]
        self.acc_last = (0.0, 0.0, 0.0)
        self.ab_debug = [0.0, 0.0, 0.0]
        self.k_lidar = 0.12          # 每次雷达速度的修正增益（47.5Hz -> tau≈0.18s）
        # ★ 延迟补偿：v_out = v_雷达(最新) + [v_IMU(now) - v_IMU(now-L)]
        #   差值形式没有零漂累积；L = 雷达速度相对"真值"的滞后
        self.latency_s = 0.20          # ★ 2026-10-09 实测：雷达速度滞后 IMU 约 200ms
        self.v_imu = [0.0, 0.0, 0.0]
        self.hist = deque()
        self.n_acc = 0
        self.n_lidar = 0

        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(Vector3, '/fc_acc', self.on_acc, 200)
        self.create_subscription(Vector3, '/fc_gyro', self.on_gyro, 200)
        self.create_subscription(Float32MultiArray, '/fc_attitude', self.on_att, 200)
        self.create_subscription(Twist, '/velocity_map', self.on_lidar, 50)
        self.pub = self.create_publisher(Twist, '/velocity_fused', 50)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(
            'imu_fusion: 标定 %.0f 秒，期间不要动飞机 …' % calib_s)

    # ---------- 标定 ----------
    def on_acc(self, m):
        if not self.calibrated:
            self.accs.append((m.x, m.y, m.z))
            return
        # ★ 标定后必须继续积分加速度（否则高频通路是死的，只输出"延迟的雷达速度"）
        if self.att_now is None:
            return
        now = time.time()
        self.acc_last = (m.x, m.y, m.z)
        dt = 0.005 if self.last_acc_t is None else min(0.05, now - self.last_acc_t)
        self.last_acc_t = now
        r, p, y = self.att_now
        R = rot_nav_from_body(math.radians(r), math.radians(p),
                              math.radians(y + self.yaw_off))
        abc = (m.x, m.y, m.z)          # ★ 之前这里全写成 m.x，导致 z 用了 x 的值 →
        ab = [(abc[i] - self.acc_bias[i]) / self.acc_lsb_per_mps2 for i in range(3)]
        self.ab_debug = ab
        an = [R[i][0] * ab[0] + R[i][1] * ab[1] + R[i][2] * ab[2] for i in range(3)]
        an[2] -= G
        self.a_nav = an
        for i in range(3):
            self.v_imu[i] += an[i] * dt
        self.hist.append((now, tuple(self.v_imu)))
        while len(self.hist) > 2 and self.hist[-1][0] - self.hist[0][0] > 0.6:
            self.hist.popleft()

    def on_gyro(self, m):
        if self.calibrated:
            return
        self.gyros.append((m.x, m.y, m.z))

    def on_att(self, m):
        if len(m.data) < 3:
            return
        v = (float(m.data[0]), float(m.data[1]), float(m.data[2]))
        if not self.calibrated:
            self.rpys.append(v)
        self.att_now = v

    def psi_now(self):
        try:
            tr = self.buf.lookup_transform('map', 'dr_base', rclpy.time.Time())
            q = tr.transform.rotation
            return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        except Exception:                      # noqa: BLE001
            return None

    def do_calib(self):
        n = min(len(self.accs), len(self.gyros), len(self.rpys))
        if n < 30:
            self.get_logger().warn('标定样本太少（%d），延长标定时间' % n)
            return False
        acc = [sum(a[i] for a in self.accs[:n]) / n for i in range(3)]
        gyr = [sum(g[i] for g in self.gyros[:n]) / n for i in range(3)]
        rpy = [sum(r[i] for r in self.rpys[:n]) / n for i in range(3)]
        mag = math.sqrt(sum(x * x for x in acc))
        if mag < 1.0:
            self.get_logger().warn('加速度读数为 0，检查 /fc_acc')
            return False
        self.acc_lsb_per_mps2 = mag / G
        R = rot_nav_from_body(math.radians(rpy[0]), math.radians(rpy[1]),
                              math.radians(rpy[2]))
        g_body = [G * R[0][2], G * R[1][2], G * R[2][2]]      # R^T·[0,0,G]
        self.acc_bias = [acc[i] - self.acc_lsb_per_mps2 * g_body[i] for i in range(3)]
        self.gyro_bias = gyr
        psi = self.psi_now()
        if psi is None:
            self.get_logger().warn('读不到 SLAM 的 psi，暂时用 IMU 偏航（导航系可能差一个常量）')
            self.psi0 = rpy[2]
        else:
            self.psi0 = psi
        self.yaw_off = self.psi0 - rpy[2]
        self.calibrated = True
        self.get_logger().info(
            '标定完成: |acc|=%.0f LSB (=1g) → %.1f LSB/(m/s^2) | acc偏置=(%.0f,%.0f,%.0f) '
            '| gyro偏置=(%.1f,%.1f,%.1f) | yaw偏移=%.1f deg'
            % (mag, self.acc_lsb_per_mps2, self.acc_bias[0], self.acc_bias[1],
               self.acc_bias[2], self.gyro_bias[0], self.gyro_bias[1],
               self.gyro_bias[2], self.yaw_off))
        return True

    # ---------- 运行 ----------
    def on_lidar(self, m):
        if not self.calibrated:
            return
        self.v_lidar = [m.linear.x, m.linear.y, m.linear.z]
        if self.logf:
            self.logf.write('%.4f,vmap,%.5f,%.5f,%.5f\n'
                            % (time.time() - self.t_start, m.linear.x, m.linear.y,
                               m.linear.z))
            self.logf.flush()
        self.n_lidar += 1

    def tick(self):
        now = time.time()
        if not self.calibrated:
            if now - self.t_start >= self.calib_s:
                self.do_calib()
            return
        dt = 0.02 if self.last_t is None else min(0.1, now - self.last_t)
        self.last_t = now
        if self.att_now is None:
            return
        # 延迟补偿输出：把最新雷达速度用 IMU 的 Δv 推到当前时刻
        t_target = now - self.latency_s
        past = None
        for tt, vv in self.hist:
            if tt <= t_target:
                past = vv
            else:
                break
        if past is None and self.hist:
            past = self.hist[0][1]
        if past is not None:
            self.v = [self.v_lidar[i] + (self.v_imu[i] - past[i]) for i in range(3)]
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = self.v[0], self.v[1], self.v[2]
        self.pub.publish(msg)
        self.n_acc += 1
        if self.logf and self.n_acc % 2 == 0:
            self.logf.write('%.4f,vfus,%.5f,%.5f,%.5f\n'
                            % (time.time() - self.t_start, self.v[0], self.v[1], self.v[2]))
            self.logf.write('%.4f,vimu,%.5f,%.5f,%.5f\n'
                            % (time.time() - self.t_start, self.v_imu[0], self.v_imu[1],
                               self.v_imu[2]))
            self.logf.flush()
        if self.n_acc % 500 == 0:
            self.get_logger().info(
                'fused v=(%+.3f,%+.3f,%+.3f)  lidar v=(%+.3f,%+.3f,%+.3f)  a_nav=(%+.2f,%+.2f,%+.2f)'
                '  acc_raw=(%.0f,%.0f,%.0f) att=(%.1f,%.1f,%.1f)'
                '  bias=(%.1f,%.1f,%.1f) scale=%.1f ab=(%.2f,%.2f,%.2f)'
                % (self.v[0], self.v[1], self.v[2], self.v_lidar[0], self.v_lidar[1],
                   self.v_lidar[2], self.a_nav[0], self.a_nav[1], self.a_nav[2],
                   self.acc_last[0], self.acc_last[1], self.acc_last[2],
                   self.att_now[0], self.att_now[1], self.att_now[2],
                   self.acc_bias[0], self.acc_bias[1], self.acc_bias[2],
                   self.acc_lsb_per_mps2, self.ab_debug[0], self.ab_debug[1],
                   self.ab_debug[2]))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    calib_s = float(args[0]) if args else 8.0
    lat = None
    if '--latency' in sys.argv:
        i = sys.argv.index('--latency')
        if len(sys.argv) > i + 1:
            lat = float(sys.argv[i + 1])
    logpath = None
    if '--log' in sys.argv:
        i = sys.argv.index('--log')
        if len(sys.argv) > i + 1:
            logpath = sys.argv[i + 1]
    else:
        logpath = '/tmp/fusion_log.csv'
    rclpy.init()
    n = Fusion(calib_s, logpath)
    if lat is not None:
        n.latency_s = lat
        n.get_logger().info('延迟补偿 L = %.0f ms' % (lat * 1000))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
