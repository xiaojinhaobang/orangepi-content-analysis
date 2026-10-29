#!/usr/bin/env python3
"""解锁即自动发 /gs/start —— 消除"人解锁 → 上位机发 START"之间的 SSH 往返延迟。

背景（2026-10-14 实测）：IMU 核心板会在解锁后约 15 秒自己跑一次"一键降落"
（cmd1=6，香橙派侧全程没发过降落帧，是串口噪声被 F407 误解析成合法 0x66[2]），
约 3.6 秒后自锁。所以从"解锁"到"我们的 0x66[1,156] 起飞"必须抢在 ~15 秒窗口内。
手动在 PC 上敲 ros2 topic pub，光 SSH 往返就 3~5 秒，15:24 那次正好差 0.2 秒失败。
改成在板上盯着：解锁一旦稳定，立刻发。

判据：/fc_status mode_sta==2（定点）+ unlock_sta==1（已解锁）+ cmd1!=6（没在自降），
      且 /fc_target 的 thr>=MIN_THR（油门杆真的推起来了），连续保持 HOLD_S 秒
      → 连发 3 次 /gs/start 后退出。

用法: python3 -u ~/diag/arm_trigger.py [hold_s] [timeout_s] [mode]
      默认 hold_s=0.8  timeout_s=300  mode=start
      mode=start   : 解锁稳定后发 /gs/start（走 mission_node 全流程）
      mode=takeoff : 解锁稳定后**直接**发 /mission_cmd[1,156]（只测一键起飞，
                     不跑任务节点；用于把"起飞"单独拎出来验证）
      mode=hover   : 解锁稳定后**调用 hover_auto.py --h H --hov T --reach 0.95**（已知可用的
                     悬停自测链，单帧），H/T 由第 4/5 个参数给（默认 150 / 20）
前提: mode=start 时 mission_node 已在待命（等 /gs/start）。
"""
import subprocess
import sys
import time
import fcntl

import rclpy
from rclpy.node import Node
from std_msgs.msg import (Float32MultiArray, Int16, Int16MultiArray, UInt8,
                          UInt8MultiArray)

MIN_THR = 430.0          # 解锁后推到中位即可触发
MAX_THR = 620.0          # 允许中位以上；仍要求先解锁再推，避免地面高油门起转
TAKEOFF_H = 156          # 与 mission_node 的 TAKEOFF_MIN_H 一致
CLIMB_WAIT_S = 5.0       # 发完等这么久还不离地就重发（同 mission_node）


def owner_busy():
    """单主控锁是否被占（hover_test 或 mission 正在跑）。

    ★ 2026-10-19：autostart 每 15 秒会重新拉起本脚本，光 kill 挡不住。
      这里在**即将发 START 的那一刻**检查：有人持有 /tmp/flight_owner.lock
      就退出，避免自动 START 把正在跑的悬停测试踢掉。
    """
    try:
        fd = open('/tmp/flight_owner.lock', 'a')
    except OSError:
        return False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(fd, fcntl.LOCK_UN)
        return False
    except OSError:
        return True
    finally:
        fd.close()


class ArmTrigger(Node):
    def __init__(self, mode):
        super().__init__('arm_trigger')
        self.mode = mode
        self.st = None
        self.thr = None
        self.h = None
        self.t_ok = None
        self.last_report = 0.0
        self.fired = False
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_st, 10)
        self.create_subscription(Float32MultiArray, '/fc_target', self.on_tgt, 10)
        self.create_subscription(Int16, '/height', self.on_h, 10)
        # ★ 2026-10-14 深夜：解锁瞬间就把"外部速度源"打开，别让它真空。
        #   证据：今晚 9 次 cmd1=6（飞控自己一键降落）100% 落在"已解锁"且
        #   /target_velocity 静默的窗口里；而白天两条直线测试（位置环一直在发速度）
        #   各飞 77 / 80 秒，cmd1=6 分别是 0 次和 1 次（收尾那次）。
        #   10-12 悬停 31 秒也有 pos_hold 在发（还带 -1.8cm/s 的常值 trim）。
        #   → 猜：IMU 核心板有"外部速度源超时"保护，0x31 一断它就降落。
        #   这里发 3（`pid_controller.cpp:212-215` → 持续发 {0,0,0,0}，20Hz），
        #   速度流新鲜但不干涉飞控；任务挂环时会自己切到 2。
        self.pub_act = self.create_publisher(UInt8, '/active_controller', 10)
        if mode == 'takeoff':
            self.pub = self.create_publisher(Int16MultiArray, '/mission_cmd', 10)
        else:
            self.pub = self.create_publisher(Int16, '/gs/start', 10)

    def on_st(self, m):
        if len(m.data) >= 7:
            self.st = [int(x) for x in m.data[:7]]

    def on_tgt(self, m):
        if len(m.data) >= 3:
            self.thr = float(m.data[2])

    def on_h(self, m):
        self.h = int(m.data)

    def keep_velocity_alive(self):
        """发 /active_controller=3：位置环进入"停止"态，但每拍仍发 {0,0,0,0}。"""
        m = UInt8()
        m.data = 3
        for _ in range(5):
            self.pub_act.publish(m)
            time.sleep(0.05)

    def wait_reason(self):
        """返回 None 表示"条件已满足"，否则返回还没满足的原因。"""
        if self.st is None:
            return '等 /fc_status'
        if self.thr is None:
            return '等 /fc_target（油门）'
        if self.st[0] != 2:
            return 'mode_sta=%d（请 CH5 打中档）' % self.st[0]
        if self.st[1] != 1:
            return 'unlock_sta=0（等遥控解锁）'
        if self.st[6] == 6:
            return '飞控正在一键降落（cmd1=6）→ 等它过去'
        if self.thr < MIN_THR:
            return '油门 %.0f < %.0f（解锁后缓慢推油门）' % (self.thr, MIN_THR)
        if self.thr > MAX_THR:
            return '油门 %.0f > %.0f（不要超过中位，回中位）' % (self.thr, MAX_THR)
        return None


def main():
    hold_s = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
    timeout_s = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
    mode = sys.argv[3] if len(sys.argv) > 3 else 'start'
    arg4 = sys.argv[4] if len(sys.argv) > 4 else None
    arg5 = sys.argv[5] if len(sys.argv) > 5 else None
    rclpy.init()
    node = ArmTrigger(mode)
    print('arm_trigger 待命(模式=%s)：解锁稳定 %.1fs 后自动触发（超时 %.0fs）'
          % (mode, hold_s, timeout_s), flush=True)
    t_end = time.monotonic() + timeout_s
    ok = 1
    try:
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            why = node.wait_reason()
            if why is not None:
                node.t_ok = None
                if time.monotonic() - node.last_report > 3.0:
                    node.last_report = time.monotonic()
                    print('  等待: %s' % why, flush=True)
                continue
            if node.t_ok is None:
                node.t_ok = time.monotonic()
                continue
            if time.monotonic() - node.t_ok < hold_s:
                continue
            print('✅ 已稳定解锁（mode=2 unlock=1 cmd1=%d 油门=%.0f）'
                  % (node.st[6], node.thr), flush=True)
            if owner_busy():
                print('!! /tmp/flight_owner.lock 被占用（hover_test/任务在跑）'
                      '→ 不自动发 START，退出', flush=True)
                break
            if mode == 'hover':
                # 10-12 悬停链的 pos_hold 已经在起飞前置 active=2；
                # 这里再置 3 会让 C++ 环每拍发 0，覆盖 pos_hold 的速度源。
                print('  hover 模式：保留 pos_hold 的 /active_controller=2（不置 3）',
                      flush=True)
            else:
                node.keep_velocity_alive()
                print('  已置 /active_controller=3（速度流保鲜，避免"外部速度源超时"自降）',
                      flush=True)
            if mode == 'hover':
                h = arg4 or '150'
                hov = arg5 or '20'
                print('  → 启动 hover_auto.py --h %s --hov %s --reach 0.95'
                      '（到 95%% 高度再计时，单帧）' % (h, hov), flush=True)
                # hover_auto 一进来就检查解锁，所以先把它拉起来再执行
                rc = subprocess.call(['python3', '-u',
                                      '/home/orangepi/diag/hover_auto.py',
                                      '--h', h, '--hov', hov,
                                      '--reach', '0.95'])
                print('  hover_auto 退出码 %d' % rc, flush=True)
                # 只跑一次就退出 —— 自动重复起飞太危险，要重测由人重新启动本脚本
                ok = rc
                break
            if mode == 'takeoff':
                ok = do_takeoff(node)
            else:
                msg = Int16()
                msg.data = 0
                for _ in range(3):
                    node.pub.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.05)
                print('  /gs/start 已发出', flush=True)
                ok = 0
            break
        else:
            print('超时未解锁 → 退出（未发 START）', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return ok


def do_takeoff(node):
    """直接发 /mission_cmd[1,156] 测一键起飞；没离地就重发，最多 3 次。"""
    msg = Int16MultiArray()
    msg.data = [1, TAKEOFF_H]
    for attempt in range(1, 4):
        if node.st is None or node.st[1] != 1:
            print('✗ 起飞前复核：飞控已上锁（%s）→ 停，不发 0x66' % (node.st,),
                  flush=True)
            return 1
        if node.st[6] == 6:
            print('✗ 起飞前复核：飞控正在一键降落（cmd1=6）→ 停，不发 0x66',
                  flush=True)
            return 1
        print('起飞 #%d：0x66[1,%d]（解锁位=%d cmd1=%d 油门=%.0f h=%s）'
              % (attempt, TAKEOFF_H, node.st[1], node.st[6], node.thr, node.h),
              flush=True)
        node.pub.publish(msg)
        t0 = time.monotonic()
        got_air = False
        while time.monotonic() - t0 < CLIMB_WAIT_S:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.h is not None and node.h >= 25:
                got_air = True
                break
            if node.st[1] != 1:
                print('✗ 起飞 #%d 过程中飞控上锁 → 停' % attempt, flush=True)
                return 1
            if node.st[6] == 6:
                print('✗ 起飞 #%d 过程中飞控跑一键降落（cmd1=6）→ 停' % attempt,
                      flush=True)
                return 1
        if got_air:
            print('✅ 已离地：h=%s cm（目标 %d）→ 看一眼悬停就收' % (node.h, TAKEOFF_H),
                  flush=True)
            return 0
        print('  #%d 没离地（h=%s）→ 重发' % (attempt, node.h), flush=True)
    print('✗ 三次都没离地 → 交操作手', flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
