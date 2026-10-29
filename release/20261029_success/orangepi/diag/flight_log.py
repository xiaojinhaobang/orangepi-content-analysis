#!/usr/bin/env python3
"""试飞记录（定时采样版）—— 修正 hover_test 的日志积压问题。

hover_test.py 用单线程 spin_once 每 0.2s 跑一次，却订阅了 6 个高频话题
（约 600 msg/s），结果记录出来的是积压的旧数据：raw 列会连续几秒完全一样。
本脚本改成：所有订阅只做「缓存最新值」，由 20Hz 定时器统一采样写盘。

用法: flight_log.py [秒数,默认60] [输出文件,默认 ~/traces/flight_<时间>.log]
记录列: t x y psi | filt(注入飞控) raw(雷达原始) tvd(位置环指令) | h mode unlock
        | fcYaw(飞控四元数反算) fcYawEuler(飞控欧拉角,2Hz) fcGyroZ(IMU原始陀螺Z) fcYawCmd(飞控目标的偏航角速度)
        | fcRol fcPit fcThr fcTgtVx fcTgtVy（飞控 0x41 实时控制帧：它自己的姿态指令 + 它自己的速度目标）
        | fcRoll fcPitch（飞控 0x01 四元数反算的真实姿态角，200Hz —— 用来量"颠簸"）
        | fcAccX fcAccY（飞控 0x01 原始加速度 LSB，200Hz —— 用来独立量"真实运动 → 注入速度"的延迟）
        | fcVelX fcVelY（飞控 0x07 它自己的速度估计 cm/s —— 判定"飞控知不知道自己在漂"）
        | fcPwm=(4 路电机输出, % )（飞控 0x09 —— 控制链最末端，判定慢摆是飞控驱动还是外部扰动）
          注意：它是四轴，IMU 的 0x20 帧只填 4 路，407 那侧一次读 8 路会把帧外内存读出来，
          所以 m5~m8 是垃圾 —— 只记前 4 路。
        | fcTarA=(目标姿态 rol/pit/yaw, 度)（飞控 0x0A）
        | fcTarV=(目标速度 x,y, cm/s)（飞控 0x0B —— 飞控内部位置环的输出）
        | fcCtlA=(姿态控制量 r/p/t/y, ±5000)（飞控 0x21 —— 姿态环 PID 输出）
        | fcVFwd=(x,y)（飞控 0x0C —— 转发验证帧，内容应和 fcVel 相同；有值=转发函数有效）
判读: 把 raw 和 tvd 对齐看 —— raw 是"看到的速度"，tvd 是"我们的反应"。
      psi(SLAM航向) 对 fcYaw(飞控航向) —— 两者差多少就是"机体真转了多少"。
      fcGyroZ 摆 = 机身在转（和 SLAM 无关）；fcGyroZ 不动而 psi 摆 = 位姿估计在摆。
      fcYawCmd 也在摆 = 飞控自己在命令这个摆动（环路/估计侧）；fcYawCmd 平 = 外部扰动。
      ★2026-10-09 新增 fcRol/fcPit/fcTgtVx/fcTgtVy 用来回答"漂移时飞控在干什么"：
        fcRol/fcPit 有恒定分量 = 飞控在用倾角顶（环在动，静差来自扰动）；
        fcRol/fcPit≈0 而飞机在漂 = 飞控认为没误差（它没在用我们喂的速度）；
        fcTgtVx/Vy≈0 而实测速度≈1cm/s = 飞控"看不见"这个漂移。
"""
import math
import os
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray, UInt8MultiArray


class FlightLog(Node):
    def __init__(self):
        super().__init__('flight_log')
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.filt = (0.0, 0.0)
        self.raw = (0.0, 0.0)
        self.tvd = None
        self.h = None
        self.fc = None
        self.fc_att = None
        self.fc_att_eu = None
        self.fc_gyro = None
        self.fc_acc = None
        self.fc_vel = None
        self.fc_pwm = None
        self.fc_tar_a = None
        self.fc_tar_v = None
        self.fc_ctl_a = None
        self.fc_vfwd = None
        self.fc_target = None
        self.echo = (0, 0)
        self.create_subscription(Twist, '/velocity_map_filtered', self.on_filt, 1)
        self.create_subscription(Twist, '/velocity_map', self.on_raw, 1)
        self.create_subscription(Float32MultiArray, '/target_velocity', self.on_tvd, 1)
        self.create_subscription(Int16, '/height', self.on_h, 1)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 1)
        self.create_subscription(Int16MultiArray, '/fc_vel_echo', self.on_echo, 1)
        self.create_subscription(Float32MultiArray, '/fc_attitude', self.on_fc_att, 1)
        self.create_subscription(Float32MultiArray, '/fc_attitude_euler', self.on_fc_att_eu, 1)
        self.create_subscription(Vector3, '/fc_gyro', self.on_fc_gyro, 1)
        self.create_subscription(Vector3, '/fc_acc', self.on_fc_acc, 1)
        self.create_subscription(Vector3, '/fc_vel', self.on_fc_vel, 1)
        self.create_subscription(Float32MultiArray, '/fc_pwm', self.on_fc_pwm, 1)
        self.create_subscription(Float32MultiArray, '/fc_target_atti', self.on_fc_tar_a, 1)
        self.create_subscription(Vector3, '/fc_target_vel', self.on_fc_tar_v, 1)
        self.create_subscription(Float32MultiArray, '/fc_ctrl_amt', self.on_fc_ctl_a, 1)
        self.create_subscription(Vector3, '/fc_vel_fwd', self.on_fc_vfwd, 1)
        self.create_subscription(Float32MultiArray, '/fc_target', self.on_fc_target, 1)

    def on_filt(self, m):
        self.filt = (m.linear.x * 100.0, m.linear.y * 100.0)

    def on_raw(self, m):
        self.raw = (m.linear.x * 100.0, m.linear.y * 100.0)

    def on_tvd(self, m):
        if len(m.data) >= 2:
            self.tvd = (float(m.data[0]), float(m.data[1]),
                        float(m.data[2]) if len(m.data) > 2 else 0.0,
                        float(m.data[3]) if len(m.data) > 3 else 0.0)

    def on_h(self, m):
        self.h = int(m.data)

    def on_fc(self, m):
        if len(m.data) >= 2:
            self.fc = (int(m.data[0]), int(m.data[1]))

    def on_echo(self, m):
        if len(m.data) >= 2:
            self.echo = (int(m.data[0]), int(m.data[1]))

    def on_fc_att(self, m):
        if len(m.data) >= 3:
            self.fc_att = (float(m.data[0]), float(m.data[1]), float(m.data[2]))

    def on_fc_att_eu(self, m):
        if len(m.data) >= 3:
            self.fc_att_eu = (float(m.data[0]), float(m.data[1]), float(m.data[2]))

    def on_fc_gyro(self, m):
        self.fc_gyro = (float(m.x), float(m.y), float(m.z))

    def on_fc_acc(self, m):
        self.fc_acc = (float(m.x), float(m.y), float(m.z))

    def on_fc_vel(self, m):
        self.fc_vel = (float(m.x), float(m.y), float(m.z))

    def on_fc_pwm(self, m):
        # 注意：四轴只发 4 路（0x09 帧 dataLen=8）—— 判 >=4，别再写 8
        if len(m.data) >= 4:
            self.fc_pwm = [float(v) for v in m.data[:4]]

    def on_fc_tar_a(self, m):
        if len(m.data) >= 3:
            self.fc_tar_a = [float(v) for v in m.data[:3]]

    def on_fc_tar_v(self, m):
        self.fc_tar_v = (float(m.x), float(m.y))

    def on_fc_ctl_a(self, m):
        if len(m.data) >= 4:
            self.fc_ctl_a = [float(v) for v in m.data[:4]]

    def on_fc_vfwd(self, m):
        self.fc_vfwd = (float(m.x), float(m.y))

    def on_fc_target(self, m):
        if len(m.data) >= 7:
            self.fc_target = [float(v) for v in m.data[:7]]

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


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    outdir = os.path.expanduser('~/traces')
    os.makedirs(outdir, exist_ok=True)
    path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        outdir, time.strftime('flight_%Y%m%d_%H%M%S.log'))

    rclpy.init()
    n = FlightLog()
    fh = open(path, 'w')
    print('>>> 定时采样记录到', path, flush=True)
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.02)
        now = time.time() - t0
        if now - last >= 0.10:                    # 20Hz 定时采样，不等回调
            last = now
            tv = n.tvd if n.tvd else (float('nan'),) * 4
            fc = n.fc if n.fc else (None, None)
            fa = n.fc_att if n.fc_att else (float('nan'),) * 3
            fe = n.fc_att_eu if n.fc_att_eu else (float('nan'),) * 3
            fg = n.fc_gyro[2] if n.fc_gyro else float('nan')
            ft = n.fc_target if n.fc_target else [float('nan')] * 7
            px, py, psi = n.pose()
            line = ('t=%6.2f x=%7.2f y=%7.2f psi=%7.1f | filt=(%6.1f,%6.1f) raw=(%6.1f,%6.1f) '
                    'tvd=(%6.1f,%6.1f) z=%6.1f yaw=%6.1f | fc回显=(%4d,%4d) h=%s mode=%s unlock=%s'
                    ' | fcYaw=%7.1f fcYawEuler=%7.1f fcGyroZ=%7.0f fcYawCmd=%6.0f'
                    ' | fcRol=%7.2f fcPit=%7.2f fcThr=%5.0f fcTgtVx=%6.1f fcTgtVy=%6.1f'
                    ' | fcRoll=%7.2f fcPitch=%7.2f'
                    ' | fcAccX=%7.0f fcAccY=%7.0f'
                    ' | fcVelX=%6.1f fcVelY=%6.1f'
                    ' | fcPwm=(%5.1f,%5.1f,%5.1f,%5.1f)'
                    ' | fcTarA=(%6.1f,%6.1f,%6.1f)'
                    ' | fcTarV=(%6.1f,%6.1f)'
                    ' | fcCtlA=(%6.0f,%6.0f,%6.0f,%6.0f)'
                    ' | fcVFwd=(%6.1f,%6.1f)'
                    ' | fcGyroXY=(%6.0f,%6.0f)'
                    % (now, px, py, psi, n.filt[0], n.filt[1], n.raw[0], n.raw[1],
                       tv[0], tv[1], tv[2], tv[3], n.echo[0], n.echo[1], n.h, fc[0], fc[1],
                       fa[2], fe[2], fg, ft[3], ft[0], ft[1], ft[2], ft[4], ft[5],
                       fa[0], fa[1], (n.fc_acc[0] if n.fc_acc else float('nan')),
                       (n.fc_acc[1] if n.fc_acc else float('nan')),
                       (n.fc_vel[0] if n.fc_vel else float('nan')),
                       (n.fc_vel[1] if n.fc_vel else float('nan')),
                       *([n.fc_pwm[i] if n.fc_pwm else float('nan') for i in range(4)]),
                       *([n.fc_tar_a[i] if n.fc_tar_a else float('nan') for i in range(3)]),
                       (n.fc_tar_v[0] if n.fc_tar_v else float('nan')),
                       (n.fc_tar_v[1] if n.fc_tar_v else float('nan')),
                       *([n.fc_ctl_a[i] if n.fc_ctl_a else float('nan') for i in range(4)]),
                       (n.fc_vfwd[0] if n.fc_vfwd else float('nan')),
                       (n.fc_vfwd[1] if n.fc_vfwd else float('nan')),
                       (n.fc_gyro[0] if n.fc_gyro else float('nan')),
                       (n.fc_gyro[1] if n.fc_gyro else float('nan'))))
            fh.write(line + '\n')
            fh.flush()
    fh.close()
    print('>>> done ->', path, flush=True)


if __name__ == '__main__':
    main()
