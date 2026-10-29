#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""auto_guard — 飞行中自动保护守护（只监视 + 只发轻降，唯一会上锁的是坠落规则）。

为什么存在：2026-10-16 铁柱旁那轮，飞机悬停中单方向漂 30cm，
飞行员看到漂就拉油门到底 -> 1.4m 自由落体。人来不及喊，守护来得及。

生效条件：已解锁(unlock_sta=1) 且 高度 >= ARM_H(默认 60cm)。
地面、未解锁、低空 一律不动作。

触发规则
  R1 横移超限 : 离 ARM 时锁定的基准点水平 > OFF_LIMIT(25cm) 持续 OFF_HOLD(0.4s)
  R2 速度超限 : 0.5s 位移窗速度 > SPD_LIMIT(20cm/s) 持续 SPD_HOLD(0.6s)
  R3 定位丢失 : tf map->dr_base 连续 TF_LOST(1.0s) 取不到
  R4 自由落体 : 半个窗口内高度掉 > FALL_DH(50cm) -> 发 0x66[4] 停桨（唯一会上锁的规则）

触发后动作
  发 /mission_cmd [2,0] = 0x66[2] 飞控自动轻降 -> 等 h<10cm（最多 LAND_WAIT 25s）
  -> 发 [4,0] 上锁 -> 进入 done，再次解锁前不再触发

手动快停
  板上 touch /tmp/guard_land -> 下个周期(50ms)执行同样的轻降流程

日志
  /tmp/auto_guard.log  事件
  /tmp/auto_guard.csv  20Hz 轨迹 t,h,unlock,x,y,dist,spd,state

用法（板上）
  export ROS_DOMAIN_ID=6
  source /opt/ros/humble/setup.bash
  python3 ~/diag/auto_guard.py
临时改阈值：
  GUARD_OFF_LIMIT=20 GUARD_SPD_LIMIT=15 python3 ~/diag/auto_guard.py
"""
import collections
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int16, Int16MultiArray, UInt8MultiArray
import tf2_ros

ARM_H = float(os.environ.get('GUARD_ARM_H', '60'))            # cm
OFF_LIMIT = float(os.environ.get('GUARD_OFF_LIMIT', '25'))    # cm
OFF_HOLD = float(os.environ.get('GUARD_OFF_HOLD', '0.4'))     # s
SPD_LIMIT = float(os.environ.get('GUARD_SPD_LIMIT', '20'))    # cm/s
SPD_HOLD = float(os.environ.get('GUARD_SPD_HOLD', '0.6'))     # s
TF_LOST = float(os.environ.get('GUARD_TF_LOST', '1.0'))       # s
FALL_DH = float(os.environ.get('GUARD_FALL_DH', '50'))        # cm
FALL_DT = float(os.environ.get('GUARD_FALL_DT', '0.3'))       # s
LAND_WAIT = float(os.environ.get('GUARD_LAND_WAIT', '25'))    # s
HEAD_MAX = float(os.environ.get('GUARD_HEAD_MAX', '400'))     # cm 超过即视为遥测假值，丢弃
MANUAL = os.environ.get('GUARD_MANUAL_FILE', '/tmp/guard_land')
LOG = os.environ.get('GUARD_LOG', '/tmp/auto_guard.log')
CSV = os.environ.get('GUARD_CSV', '/tmp/auto_guard.csv')


class Guard(Node):
    def __init__(self):
        super().__init__('auto_guard')
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(Int16MultiArray, '/mission_cmd', 10)
        self.create_subscription(Int16, '/height', self.on_h, 10)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 10)
        # ★2026-10-17：位姿可信度。不可信时暂停偏移/速度判据（它们吃雷达位置，
        #   会被假位姿骗去降落 —— 10/17 两次误触发都是这个原因）。
        self.create_subscription(Bool, '/pose_trust', self.on_trust, 10)
        # ★2026-10-17：航点飞行时 waypoint_nav 会持续发 /target_position。
        #   这时 R1/R2 的“离起飞点过远/速度过快”会把正常航段当成漂移，
        #   实测 32cm 正常左前位移就触发了 0x66[2]。航点模式只保留
        #   R3 定位丢失和 R4 自由落体。
        self.create_subscription(Float32MultiArray, '/target_position',
                                 self.on_target, 10)
        self.route_t = 0.0
        self.route_warned = False
        self.pose_ok = True
        self.blind_since = None
        self.blind_warned = False

        self.h = None
        self.h_win = collections.deque(maxlen=7)     # 高度中值滤波窗
        self.h_med = None
        self.h_bad = 0
        self.unlock = None
        self.base = None
        self.win = collections.deque(maxlen=12)      # ~0.6s @ 20Hz
        self.state = 'idle'                          # idle | watch | landing | done
        self.t_off = None
        self.t_spd = None
        self.t_nan = None
        self.t_land = None
        self.t_print = 0.0

        need_hdr = (not os.path.exists(CSV)) or os.path.getsize(CSV) == 0
        self.fcsv = open(CSV, 'a', buffering=1)
        if need_hdr:
            self.fcsv.write('t,h,unlock,x,y,dist,spd,state\n')
        self.flog = open(LOG, 'a', buffering=1)
        self.line('=' * 58)
        self.line('auto_guard 启动 | 偏移%.0fcm/%.1fs | 速度%.0fcm/s/%.1fs | 定位丢%.1fs | 掉落%.0fcm'
                  % (OFF_LIMIT, OFF_HOLD, SPD_LIMIT, SPD_HOLD, TF_LOST, FALL_DH))
        self.create_timer(0.05, self.tick)

    def line(self, s):
        msg = '[%s] %s' % (time.strftime('%H:%M:%S'), s)
        print(msg, flush=True)
        try:
            self.flog.write(msg + '\n')
        except Exception:
            pass

    def on_h(self, m):
        v = int(m.data)
        # 2026-10-16 实测：高度遥测会冒出 8019cm 这种假值，把"自由落体"规则骗到
        # 直接发停桨（飞机从 85cm 掉下来）。所以：离谱值直接丢，其余走中值滤波。
        if v < 0 or v > HEAD_MAX:
            self.h_bad += 1
            return
        self.h = v
        self.h_win.append(v)
        self.h_med = sorted(self.h_win)[len(self.h_win) // 2]

    def on_fc(self, m):
        try:
            self.unlock = int(m.data[1])
        except Exception:
            pass

    def on_trust(self, m):
        self.pose_ok = bool(m.data)

    def on_target(self, m):
        if len(m.data) >= 4:
            self.route_t = self.now()

    def send(self, sub, arg=0):
        m = Int16MultiArray()
        m.data = [int(sub), int(arg)]
        self.pub.publish(m)

    def pose(self):
        try:
            tr = self.buf.lookup_transform('map', 'dr_base', rclpy.time.Time())
            return (tr.transform.translation.x * 100.0,
                    tr.transform.translation.y * 100.0)
        except Exception:
            return (float('nan'), float('nan'))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def csvrow(self, t, x, y, dist, spd):
        try:
            self.fcsv.write('%.2f,%s,%s,%.1f,%.1f,%.1f,%.1f,%s\n'
                            % (t, self.h_med if self.h_med is not None else -1,
                               self.unlock if self.unlock is not None else -1,
                               x, y, dist, spd, self.state))
        except Exception:
            pass

    def trigger(self, why, dist, spd):
        self.state = 'landing'
        self.t_land = self.now()
        self.t_off = self.t_spd = self.t_nan = None
        self.line('!! 触发：%s (h=%s dist=%.1fcm spd=%.1fcm/s) -> 发 0x66[2] 轻降'
                  % (why, self.h_med, dist, spd))
        self.send(2, 0)

    def finish(self):
        self.send(4, 0)
        self.state = 'done'
        self.line('落地（h=%s）-> 发 0x66[4] 上锁' % self.h_med)

    def tick(self):
        t = self.now()
        x, y = self.pose()
        nan = math.isnan(x)
        h = self.h_med if self.h_med is not None else -1

        if os.path.exists(MANUAL):
            try:
                os.remove(MANUAL)
            except Exception:
                pass
            if self.state in ('idle', 'watch'):
                self.trigger('手动快停', 0.0, 0.0)

        if self.state == 'landing':
            if h < 10 or (t - self.t_land) > LAND_WAIT:
                self.finish()
            self.csvrow(t, 0.0 if nan else x, 0.0 if nan else y, 0.0, 0.0)
            return

        if self.state == 'done':
            if self.unlock == 0:
                self.state = 'idle'
                self.base = None
                self.win.clear()
                self.line('飞控已上锁 -> 守护复位（下次解锁重新武装）')
            return

        air = (self.unlock == 1) and (h >= ARM_H)
        if not air:
            if self.state == 'watch':
                self.line('离开空中状态（unlock=%s h=%s）-> 解除武装' % (self.unlock, h))
            self.state = 'idle'
            self.base = None
            self.win.clear()
            self.t_off = self.t_spd = self.t_nan = None
            if (t - self.t_print) >= 5.0:
                self.t_print = t
                self.line('地面待机 h=%-4s unlock=%-4s pos=(%.1f, %.1f) 假值%d'
                          % (h, self.unlock,
                             0.0 if nan else x, 0.0 if nan else y, self.h_bad))
            return

        if nan:
            if self.t_nan is None:
                self.t_nan = t
            elif (t - self.t_nan) > TF_LOST:
                self.trigger('定位丢失 >%.1fs' % TF_LOST, 0.0, 0.0)
            return
        self.t_nan = None

        if self.base is None:
            self.base = (x, y)
            self.state = 'watch'
            self.line('ARM：基准 (%.1f, %.1f) h=%d' % (x, y, h))

        dist = math.hypot(x - self.base[0], y - self.base[1])

        # R4 自由落体（用窗口最老一帧）
        if len(self.win) >= 2:
            t0 = self.win[0][0]
            h0 = self.win[0][3]
            if (t - t0) >= FALL_DT and (h0 - h) > FALL_DH:
                self.line('!! 自由落体：%.2fs 内掉 %dcm（%d->%d）-> 发 0x66[4] 停桨'
                          % (t - t0, h0 - h, h0, h))
                self.send(4, 0)
                self.state = 'done'
                return

        self.win.append((t, x, y, h))

        spd = 0.0
        if len(self.win) >= 2:
            t0, x0, y0, _ = self.win[0]
            dt = t - t0
            if dt >= 0.3:
                spd = math.hypot(x - x0, y - y0) / dt

        # ---- 位姿不可信：暂停偏移/速度判据（R1/R2），保留自由落体那条 ----
        if not self.pose_ok:
            if self.blind_since is None:
                self.blind_since = t
                self.line('位姿不可信 → 暂停偏移/速度判据（改为盲飞监测）')
            elif (t - self.blind_since) > 20.0 and not self.blind_warned:
                self.blind_warned = True
                self.line('!! 已盲飞 %.0fs（位置环冻结中，靠飞控自身定点）' % (t - self.blind_since))
            self.csvrow(t, x, y, dist, spd)
            return
        if self.blind_since is not None:
            self.line('位姿恢复可信 → 偏移/速度判据恢复（盲飞 %.0fs）' % (t - self.blind_since))
            self.blind_since = None
            self.blind_warned = False

        # 航点飞行：目标点在持续刷新 → 位移/速度是任务本身，不是漂移。
        route_active = (self.route_t > 0.0 and (t - self.route_t) < 0.5)
        if route_active:
            if not self.route_warned:
                self.route_warned = True
                self.line('航点模式 → 暂停偏移/速度判据（保留掉落/定位丢失保护）')
            self.t_off = None
            self.t_spd = None
            self.csvrow(t, x, y, dist, spd)
            return
        if self.route_warned:
            self.route_warned = False
            self.line('航点模式结束 → 恢复偏移/速度判据')

        if dist > OFF_LIMIT:
            if self.t_off is None:
                self.t_off = t
            elif (t - self.t_off) >= OFF_HOLD:
                self.trigger('横移 %.0fcm > %.0fcm' % (dist, OFF_LIMIT), dist, spd)
                return
        else:
            self.t_off = None

        if spd > SPD_LIMIT:
            if self.t_spd is None:
                self.t_spd = t
            elif (t - self.t_spd) >= SPD_HOLD:
                self.trigger('水平速度 %.0fcm/s > %.0f' % (spd, SPD_LIMIT), dist, spd)
                return
        else:
            self.t_spd = None

        self.csvrow(t, x, y, dist, spd)
        if (t - self.t_print) >= 1.0:
            self.t_print = t
            self.line('监视 h=%-4s dist=%5.1fcm spd=%5.1fcm/s' % (h, dist, spd))


def main():
    rclpy.init()
    n = Guard()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.line('auto_guard 退出')
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
