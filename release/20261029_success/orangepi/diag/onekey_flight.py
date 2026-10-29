#!/usr/bin/env python3
"""一键起飞：激活位置环 -> 0x66 起飞 -> 等航线走完 -> 降落。

用法:
  python3 ~/diag/onekey_flight.py                       # 高度默认 150cm
  python3 ~/diag/onekey_flight.py --ros-args -p height_cm:=120

安全设计:
  1) 起飞前强制检查 /fc_status：mode_sta 必须是 2(定点)，unlock_sta 必须是 1(已解锁)
     不满足就原地等待并打印提示（不满足绝不起飞）
  2) 飞行中若 unlock_sta 变 0 -> 判定被人接管，立即转降落
  3) 航线长时间不推进(默认 60s) -> 自动降落
  4) Ctrl-C 随时中断 -> 自动发降落
  5) 飞控若没收到 0x66（/mission_step 不变），不会进入飞行状态，超时后退出

参数:
  height_cm    起飞高度，默认 150
  assume_ready 跳过 mode/unlock 检查（仅台架调试用，默认 false）
  route_stall_s 航线不推进多久算卡住，默认 60
  mission_s    整个任务超时，默认 300
  land_at_end  结束后是否自动降落，默认 true
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Int16MultiArray, String, UInt8, UInt8MultiArray

S_WAIT_LINK, S_CHECK, S_ARM, S_TAKEOFF, S_CLIMB, S_FLY, S_LAND, S_DONE = range(8)
NAMES = ['WAIT_LINK', 'CHECK', 'ARM', 'TAKEOFF', 'CLIMB', 'FLY', 'LAND', 'DONE']


class OneKeyFlight(Node):
    def __init__(self):
        super().__init__('onekey_flight')
        self.height = int(self.declare_parameter('height_cm', 150).value)
        self.assume_ready = bool(self.declare_parameter('assume_ready', False).value)
        # 注意：这两个用【整数秒】声明，避免 -p xxx:=60 传整数时类型不匹配报错
        self.stall_s = float(self.declare_parameter('route_stall_s', 60).value)
        self.mission_s = float(self.declare_parameter('mission_s', 300).value)
        self.land_at_end = bool(self.declare_parameter('land_at_end', True).value)
        self.manual_takeoff_s = float(self.declare_parameter('manual_takeoff_s', 45).value)

        self.fc = None            # [mode_sta, unlock_sta, mode_cmd, unlock_cmd, ...]
        self.idx = None           # /route_current_index
        self.reached = None
        self.wp_status = ''
        self.height_cm = None
        self.mission_step = None

        self.state = S_WAIT_LINK
        self.t_state = time.time()
        self.t_last_idx = time.time()
        self.last_idx = None
        self.sent_takeoff = 0.0
        self.sent_active = 0.0
        self.sent_land = 0.0

        self.pub_active = self.create_publisher(UInt8, '/active_controller', 10)
        self.pub_cmd = self.create_publisher(Int16MultiArray, '/mission_cmd', 10)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 10)
        self.create_subscription(UInt8, '/route_current_index', self.on_idx, 10)
        self.create_subscription(UInt8, '/route_reached_index', self.on_reached, 10)
        self.create_subscription(String, '/waypoint_nav/status', self.on_status, 10)
        self.create_subscription(Int16, '/height', self.on_height, 10)
        self.create_subscription(UInt8, '/mission_step', self.on_step, 10)
        self.create_timer(0.2, self.step)
        self.create_timer(2.0, self.heartbeat)
        self.get_logger().info('一键起飞: 目标高度 %d cm, 等待链路...' % self.height)

    # ---------- 回调 ----------
    def on_fc(self, m):
        if len(m.data) >= 7:
            self.fc = list(m.data)

    def on_idx(self, m):
        self.idx = int(m.data)

    def on_reached(self, m):
        self.reached = int(m.data)

    def on_status(self, m):
        self.wp_status = m.data

    def on_height(self, m):
        self.height_cm = int(m.data)

    def on_step(self, m):
        self.mission_step = int(m.data)

    # ---------- 工具 ----------
    def go(self, st):
        self.get_logger().info('%s -> %s' % (NAMES[self.state], NAMES[st]))
        self.state = st
        self.t_state = time.time()

    def heartbeat(self):
        mode = self.fc[0] if self.fc else '?'
        lock = self.fc[1] if self.fc else '?'
        self.get_logger().info(
            '[%s] mode=%s unlock=%s idx=%s h=%s | %s'
            % (NAMES[self.state], mode, lock, self.idx, self.height_cm, self.wp_status[:60]))

    def send_cmd(self, sub, h=0):
        m = Int16MultiArray()
        m.data = [sub, h]
        self.pub_cmd.publish(m)

    def abort_to_land(self, why):
        self.get_logger().warn('!! 转降落: %s' % why)
        self.go(S_LAND)

    # ---------- 状态机 ----------
    def step(self):
        now = time.time()

        if self.state == S_WAIT_LINK:
            # 先反复激活位置环：否则航点执行器不会开始发布 /route_current_index，
            # 而 WAIT_LINK 又在等它 —— 会互等死锁。
            if now - self.sent_active > 1.0:
                self.sent_active = now
                self.pub_active.publish(UInt8(data=2))
            if self.fc is not None and self.idx is not None:
                self.go(S_CHECK)
            return

        if self.state == S_CHECK:
            if self.assume_ready:
                self.get_logger().warn('assume_ready=true：跳过 mode/unlock 检查（仅台架）')
                self.go(S_ARM)
                return
            if self.fc is None:
                return
            mode, unlock = self.fc[0], self.fc[1]
            if mode != 2:
                if now - self.t_state > 3.0:
                    self.t_state = now
                    self.get_logger().warn(
                        'mode_sta=%d 不是定点(2) -> 请把 CH5 打到中档，我等着' % mode)
                return
            if unlock != 1:
                if now - self.t_state > 3.0:
                    self.t_state = now
                    self.get_logger().warn('还没解锁 -> 请手动内八解锁，我等着')
                return
            self.go(S_ARM)
            return

        if self.state == S_ARM:
            for _ in range(3):
                self.pub_active.publish(UInt8(data=2))
            self.get_logger().info('已激活位置环 (/active_controller=2)')
            self.go(S_TAKEOFF)
            return

        if self.state == S_TAKEOFF:
            if now - self.sent_takeoff > 1.0:
                self.sent_takeoff = now
                self.send_cmd(1, self.height)
                self.get_logger().info('发 0x66 起飞 -> %d cm' % self.height)
            if self.mission_step == 1:
                self.get_logger().info('飞控回执 mission_step=1，起飞指令已接受')
                self.climb_ok = False
                self.get_logger().warn(
                    '若飞控没有自动起飞：请【手动推油门】把它拉起来，我最多等 %d 秒'
                    % int(self.manual_takeoff_s))
                self.go(S_CLIMB)
            elif now - self.t_state > 20.0:
                self.get_logger().error('20 秒内飞控没回执 mission_step=1，放弃')
                self.go(S_DONE)
            return

        if self.state == S_CLIMB:
            # 只收到回执不算成功，必须看到高度真的涨起来
            if self.height_cm is not None and self.height_cm >= 30:
                self.get_logger().info('已离地（高度 %d cm），进入航线监控' % self.height_cm)
                self.t_last_idx, self.last_idx = now, self.idx
                self.go(S_FLY)
            elif now - self.t_state > self.manual_takeoff_s:
                self.get_logger().error('%d 秒内没离地（高度仍是 %s cm）-> 判起飞失败'
                                        % (int(self.manual_takeoff_s), self.height_cm))
                self.abort_to_land('起飞失败，原地降落')
            return

        if self.state == S_FLY:
            if self.fc and self.fc[1] == 0:
                self.abort_to_land('飞行中 unlock_sta 变 0（被人接管）')
                return
            if self.idx != self.last_idx:
                self.last_idx, self.t_last_idx = self.idx, now
            if 'done' in self.wp_status:
                self.get_logger().info('航线完成: %s' % self.wp_status)
                if self.land_at_end:
                    self.go(S_LAND)
                else:
                    self.go(S_DONE)
                return
            if now - self.t_last_idx > self.stall_s:
                self.abort_to_land('航线 %ds 没推进' % self.stall_s)
                return
            if now - self.t_state > self.mission_s:
                self.abort_to_land('任务超时 %ds' % self.mission_s)
            return

        if self.state == S_LAND:
            if now - self.sent_land > 1.0:
                self.sent_land = now
                self.send_cmd(2, 0)
                self.get_logger().info('发 0x66 降落')
            if self.height_cm is not None and self.height_cm < 12:
                self.get_logger().info('已落地（高度 %d cm）' % self.height_cm)
                self.go(S_DONE)
            elif now - self.t_state > 40.0:
                self.get_logger().warn('降落指令已发 40s，仍未检测到落地')
                self.go(S_DONE)
            return

        if self.state == S_DONE:
            self.get_logger().info('=== 全流程结束 ===')
            raise SystemExit


def main():
    rclpy.init()
    node = OneKeyFlight()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().warn('收到 Ctrl-C -> 发降落')
        for _ in range(5):
            node.send_cmd(2, 0)
            time.sleep(0.2)
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
