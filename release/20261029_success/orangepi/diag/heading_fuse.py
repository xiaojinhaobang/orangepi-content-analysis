#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""heading_fuse — 把「飞控陀螺航向」和「雷达 SLAM 航向」融合成一个可信航向，发布 /heading_fused。

为什么做（2026-10-16 实测）：
  · 起飞点悬停：雷达航向 1 秒内跳 42°、全程最大偏离真值 66.3°；同一时刻飞控陀螺显示飞机只平滑转 −3.7°/s
  · 手转 180°：雷达航向卡住 5~6 秒不动；飞控陀螺两次给出 196.9°/198.4°（一致）
  → 航向的真值来源应该是陀螺；雷达只配当"缓慢零点参考"。

算法（已用今天的日志离线验证，见 tools/offline_heading_fuse.py）：
  d          = 雷达航向 − 飞控航向        （零点偏差，物理上缓慢变化）
  psi_fused  = fcYaw + d
  · |innovation| = |wrap(psi_radar − psi_fused)| < GATE  → 该帧可信：d 慢速更新（K, MAX_RATE）
  · 超 GATE                                              → 该帧拒绝，d 冻结（跳变/卡顿吃不到我们头上）
  · **没有"中途重新捕获"**：d 对物理转动免疫，只该在开机标定时确定（连续 2 秒偏移散度 <2° 才锁定），
    之后只允许上述慢速修正。2026-10-16 实测：落地瞬间雷达位姿跳了 −40°，中途重新捕获会把它当令真值。

离线验证结果（今天的真实日志）：
  空地悬停 32.3s：雷达最大偏离 2.9°、被拒 0/637 帧  → 雷达好时融合不添乱
  起点悬停 11.6s：雷达最大偏离 66.3°、被拒 139/229 帧 → 跳变帧全挡住

输出：
  /heading_fused  (std_msgs/Float32，单位：度，±180)
  /tmp/heading_fuse.csv：t,psi_radar,fcy,d,fused,innov,acc,air

环境变量：FUSE_GATE=8 FUSE_K=0.05 FUSE_MAX_RATE=0.5 FUSE_GZ_SCALE=16 FUSE_STILL_DPS=1 FUSE_REACQ_S=3
用法：python3 ~/diag/heading_fuse.py
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int16, UInt8MultiArray
from geometry_msgs.msg import Vector3
import tf2_ros

GATE = float(os.environ.get('FUSE_GATE', '8.0'))            # deg
K = float(os.environ.get('FUSE_K', '0.05'))                 # 1/s
MAX_RATE = float(os.environ.get('FUSE_MAX_RATE', '0.5'))    # deg/s
GZ_SCALE = float(os.environ.get('FUSE_GZ_SCALE', '16.0'))   # LSB per deg/s
STILL_DPS = float(os.environ.get('FUSE_STILL_DPS', '1.0'))  # deg/s
REACQ_S = float(os.environ.get('FUSE_REACQ_S', '3.0'))      # s
ARM_H = float(os.environ.get('FUSE_ARM_H', '60'))           # cm
CSV = os.environ.get('FUSE_CSV', '/tmp/heading_fuse.csv')
STALE_S = float(os.environ.get('FUSE_STALE_S', '0.5'))      # 输入过期阈值（供下游判断）
TRUST_BAD = float(os.environ.get('FUSE_TRUST_BAD', '20.0'))    # deg，超它判"位姿不可信"
TRUST_GOOD = float(os.environ.get('FUSE_TRUST_GOOD', '15.0'))  # deg，低于它才恢复
TRUST_BAD_T = float(os.environ.get('FUSE_TRUST_BAD_T', '0.3'))  # s
TRUST_GOOD_T = float(os.environ.get('FUSE_TRUST_GOOD_T', '2.0'))  # s


def wrap(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


class HeadingFuse(Node):
    def __init__(self):
        super().__init__('heading_fuse')
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(Float32, '/heading_fused', 10)
        self.pub_trust = self.create_publisher(Bool, '/pose_trust', 10)
        self.pub_innov = self.create_publisher(Float32, '/heading_innov', 10)
        self.create_subscription(Float32MultiArray, '/fc_attitude', self.on_att, 10)
        self.create_subscription(Vector3, '/fc_gyro', self.on_gyro, 10)
        self.create_subscription(Int16, '/height', self.on_h, 10)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 10)

        self.fcy = None          # 飞控航向（度）
        self.gz = None           # 陀螺 Z（LSB）
        self.h = None
        self.unlock = None
        self.d = None
        self.cal_buf = []        # 标定缓冲：需要连续稳定才锁定 d
        self.bad_run = 0.0
        self.still_run = 0.0
        self.n_rej = 0
        self.n_reacq = 0
        self.n_frame = 0
        self.trusted = False
        self.bad_t = 0.0
        self.good_t = 0.0
        self.last = None         # (psi, fcy, fused, innov, acc, air)
        self.t_print = 0.0
        self.prev_t = None

        need_hdr = (not os.path.exists(CSV)) or os.path.getsize(CSV) == 0
        self.csv = open(CSV, 'a', buffering=1)
        if need_hdr:
            self.csv.write('t,psi_radar,fcy,d,fused,innov,acc,air\n')
        self.line('=' * 62)
        self.line('heading_fuse 启动 | GATE=%.1f° K=%.2f MAX_RATE=%.1f°/s GZ_SCALE=%.0f REACQ=%.1fs'
                  % (GATE, K, MAX_RATE, GZ_SCALE, REACQ_S))
        self.create_timer(0.02, self.tick)      # 50Hz

    def line(self, s):
        print('[%s] %s' % (time.strftime('%H:%M:%S'), s), flush=True)

    # ---------- 输入 ----------
    def on_att(self, m):
        if len(m.data) >= 3:
            self.fcy = float(m.data[2])        # 飞控航向（度）；实测与陀螺自洽

    def on_gyro(self, m):
        self.gz = float(m.z)

    def on_h(self, m):
        self.h = int(m.data)

    def on_fc(self, m):
        try:
            self.unlock = int(m.data[1])
        except Exception:
            pass

    def psi_radar(self):
        try:
            tr = self.buf.lookup_transform('map', 'dr_base', rclpy.time.Time())
            q = tr.transform.rotation
            return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        except Exception:
            return None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- 主循环 ----------
    def tick(self):
        t = self.now()
        psi = self.psi_radar()
        if psi is None or self.fcy is None:
            return
        dt = 0.02 if self.prev_t is None else max(1e-3, min(0.5, t - self.prev_t))
        self.prev_t = t

        air = (self.unlock == 1) and (self.h is not None and self.h >= ARM_H)
        gz_dps = (self.gz / GZ_SCALE) if self.gz is not None else 0.0
        self.still_run = self.still_run + dt if abs(gz_dps) < STILL_DPS else 0.0

        if self.d is None:
            # ★2026-10-16 实测教训：d 对"物理转动"是免疫的（飞机转时 psi 与 fcYaw 一起转），
            #   所以 d 只该在开机标定时确定，之后只允许"慢速"修正，绝不能中途重新捕获
            #   （否则雷达一次 40° 的位姿跳变会被当成真值请进来）。
            #   标定条件：连续 2 秒（50Hz×100）偏移散度 < 2°，否则清空重来。
            self.cal_buf.append(wrap(psi - self.fcy))
            if len(self.cal_buf) >= 200:          # 连续 4 秒稳定才锁定（避开开机时 SLAM 还在收敛）
                if (max(self.cal_buf) - min(self.cal_buf)) < 2.0:
                    self.d = sum(self.cal_buf) / len(self.cal_buf)
                    self.line('标定完成 d = %+.1f°（雷达 − 飞控，%d 帧）' % (self.d, len(self.cal_buf)))
                    self.cal_buf = []
                else:
                    self.cal_buf = self.cal_buf[-100:]
            return                      # 未锁定前不发布 → 下游自动回退到雷达航向（保守）

        fused = wrap(self.fcy + self.d)
        innov = wrap(psi - fused)
        if abs(innov) < GATE:
            step = max(-MAX_RATE * dt, min(MAX_RATE * dt, K * innov * dt))
            self.d = wrap(self.d + step)
            acc = 1
            self.bad_run = 0.0
        else:
            acc = 0
            self.n_rej += 1
            self.bad_run += dt

        self.n_frame += 1
        self.last = (psi, self.fcy, fused, innov, acc, air)
        self.pub.publish(Float32(data=float(fused)))
        # ---- 位姿可信度（给 pos_hold / auto_guard 用）----
        # 依据：2026-10-17 实测，起飞后 1.5s 雷达航向 1 秒内跳 +58°（陀螺纹丝未动），
        #       而漂移在 8 秒后才开始 → 这个信号有 8 秒提前量。
        if abs(innov) > TRUST_BAD:
            self.bad_t += dt
            self.good_t = 0.0
        else:
            self.good_t += dt
            self.bad_t = 0.0
        if self.trusted and self.bad_t >= TRUST_BAD_T:
            self.trusted = False
            self.line('!! 位姿不可信：innov=%+.1f° 持续 %.1fs → /pose_trust=false' % (innov, self.bad_t))
        elif (not self.trusted) and self.good_t >= TRUST_GOOD_T and abs(innov) < TRUST_GOOD:
            self.trusted = True
            self.line('位姿恢复可信：innov=%+.1f° 稳定 %.1fs → /pose_trust=true' % (innov, self.good_t))
        self.pub_trust.publish(Bool(data=self.trusted))
        self.pub_innov.publish(Float32(data=float(innov)))
        try:
            self.csv.write('%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d\n'
                           % (t, psi, self.fcy, self.d, fused, innov, acc, 1 if air else 0))
        except Exception:
            pass

        if (t - self.t_print) >= 1.0:
            self.t_print = t
            self.line('psi=%+7.1f fcy=%+7.1f d=%+6.1f fused=%+7.1f innov=%+6.1f %s trust=%s | 拒绝 %d/%d'
                      % (psi, self.fcy, self.d, fused, innov,
                         '拒绝' if not acc else '  ok', 'T' if self.trusted else 'F',
                         self.n_rej, self.n_frame))


def main():
    rclpy.init()
    n = HeadingFuse()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
