#!/usr/bin/env python3
"""20Hz 记录飞控状态字节 + 油门 + 高度 -> CSV。

用途：抓"飞控自己降落"瞬间的 mode_cmd / unlock_cmd / cmd_fun 变化，
      区分"遥控侧触发的降落"和"飞控内部逻辑"。

2026-10-11 追加 thr：`ANO_LX.c:191` 里 `rt_tar.thr = 遥控油门通道 - 1000`，
      也就是说油门**不是飞控接管的**，永远是遥控杆的值。所以 thr 突降 =
      遥控油门通道瞬断（链路问题），thr 缓降 = 手在收杆。必须记下来才能区分。

2026-10-14 追加 ms / pwm 两列，用来定"自降"的源头：
  · **ms** = `/mission_step` = 0xF1 帧的 byte[0] = **F407 收到的最后一条 0x66 子命令**。
    判据（10-11 用过这条路）：
      - `cmd1` 变 6 时 `ms` 仍是 1（我们只发过起飞）→ **F407 没收到降落帧，
        是 IMU 核心板自己决定降落的**；
      - `ms` 跟着变成 2 → 是"有帧被解析成降落"（串口噪声凑出合法帧）。
    这一列必须一直在 CSV 里，否则每次都要重新查一遍。
  · **pwm** = `/fc_pwm` 四路里的最大值 = 电机输出（推力代理量）。
    判据：PWM 已经顶到 90%+ 而高度还起不来 → 电池/动力不足，不是软件。

用法: python3 fcstat_log.py [输出路径]   默认 /tmp/fcstat.csv
列:   t, h, mode_sta, unlock_sta, mode_cmd, unlock_cmd, cid, cmd0, cmd1, thr, ms, pwm
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, UInt8, UInt8MultiArray


class FcStatLog(Node):
    def __init__(self, path):
        super().__init__('fcstat_log')
        self.f = open(path, 'w', buffering=1)
        # ★ 2026-10-14 深夜：**记录 4 路电机输出**（pwm0..pwm3）。
        #   现场"高 PWM + 不起升 + 自旋" ⇒ 某一角没推力。只看最大值看不出是哪个角，
        #   记 4 列就能直接锁定是哪个电机/桨 —— 改完下次解锁瞬间就能出结论。
        self.f.write('t,h,mode_sta,unlock_sta,mode_cmd,unlock_cmd,cid,cmd0,cmd1,'
                     'thr,ms,pwm,pwm0,pwm1,pwm2,pwm3\n')
        self.h = None
        self.st = None
        self.thr = None
        self.ms = None
        self.pwm = None
        self.pwm4 = None
        self.t0 = time.time()
        # CHANGE_ONLY：20Hz 全量写盘吃掉半个核，改成"状态变化才写 + 1s 心跳"，
        # 诊断需要的字段（mode/unlock/cmd_fun/thr/h）一条不少。
        self._last_key = None
        self._last_hb = 0.0
        self.create_subscription(Int16, '/height', self.on_h, 10)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_st, 10)
        self.create_subscription(Float32MultiArray, '/fc_target', self.on_tgt, 10)
        self.create_subscription(UInt8, '/mission_step', self.on_ms, 10)
        self.create_subscription(Float32MultiArray, '/fc_pwm', self.on_pwm, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('fcstat_log -> %s' % path)

    def on_h(self, m):
        self.h = int(m.data)

    def on_st(self, m):
        if len(m.data) >= 7:
            self.st = [int(x) for x in m.data[:7]]

    def on_tgt(self, m):
        # /fc_target = [rol, pit, thr, yaw_dps, vx, vy, vz]
        if len(m.data) >= 3:
            self.thr = int(m.data[2])

    def on_ms(self, m):
        # F407 收到的最后一条 0x66 子命令（1=起飞 / 2=降落 / 3=返航 / 4=锁定）
        self.ms = int(m.data)

    def on_pwm(self, m):
        # /fc_pwm = 四路电机输出（%），取最大值当推力代理量
        if len(m.data) >= 4:
            self.pwm4 = [round(float(x), 1) for x in m.data[:4]]
            self.pwm = round(max(self.pwm4), 1)
        elif len(m.data) >= 1:
            self.pwm = round(max(float(x) for x in m.data), 1)

    def tick(self):
        t = time.time() - self.t0
        key = (tuple(self.st or ()), self.thr, self.ms, self.pwm,
               tuple(self.pwm4 or ()))
        now = time.time()
        heartbeat = (now - self._last_hb) >= 1.0
        if key == self._last_key and not heartbeat:
            return
        self._last_key = key
        if heartbeat:
            self._last_hb = now
        if self.st:
            self.f.write('%.2f,%s,%s,%s,%s,%s,%s\n'
                         % (t, self.h, ','.join(str(x) for x in self.st),
                            self.thr, self.ms, self.pwm,
                            ','.join(str(x) for x in (self.pwm4 or []))))
        else:
            self.f.write('%.2f,%s,,,,,,,,%s,,%s\n' % (t, self.h, self.thr, self.pwm))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/fcstat.csv'
    rclpy.init()
    n = FcStatLog(path)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
