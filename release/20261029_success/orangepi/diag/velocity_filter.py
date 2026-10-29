#!/usr/bin/env python3
"""速度滤波：/velocity_map -> 死区 + 一阶低通 -> /velocity_map_filtered

目的：静止时 SLAM 速度有 ±2~3 cm/s 假值（实测），直接进飞控定点环会造成游走。
     策略：|v| 小于死区就报 0（不确定就报静止，而不是把噪声当运动）。

参数 (--ros-args -p ...):
  deadband_cms  默认 5.0   合速度小于它 → 输出 0
  alpha         默认 0.15  一阶低通系数（输入 145Hz 时 τ≈46ms）
  max_cms       默认 55.0  限幅（飞控门限是严格 <60）
  log_period_s  默认 2.0   统计打印周期
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class VelocityFilter(Node):
    def __init__(self):
        super().__init__('velocity_filter')
        # 2026-09-27：deadband 10 太狠 —— 飞行中 5~10cm/s 的慢漂会被当成"静止"
        # 上报给飞控，飞控就完全不修正。静止噪声实测只有 ±2~3cm/s，4 足够。
        self.deadband = self.declare_parameter('deadband_cms', 1.0).value
        # 2026-09-28：alpha 0.15 → 0.35。
        # /velocity_map 是 ~15Hz 发的，α=0.15 在这个速率下 τ≈0.44s —— 这条速度是
        # **灌进飞控速度环当反馈**的，反馈里塞 0.4 秒滞后会把飞控自己的环推到
        # 低频极限环（实测 0.38Hz 左右摆 + 机体倾斜摆动 + 航向跟着摆）。
        # 削尖峰交给下面的 slew 限幅，不靠"压平整个信号"。
        self.alpha = self.declare_parameter('alpha', 0.35).value
        self.max_cms = self.declare_parameter('max_cms', 55.0).value
        self.invalid_cms = self.declare_parameter('invalid_cms', 60.0).value
        # 尖峰阈值：20 太低（飞行中真实速度就 30~50cm/s，会被误判成尖峰而冻结注入）
        self.spike_cms = self.declare_parameter('spike_cms', 45.0).value
        self.slew = self.declare_parameter('slew_cms', 3.0).value
        # ★ 2026-10-06：降采样。实测本节点吃掉 ~35% 一个核（/velocity_map 有
        #   ~160Hz，逐帧在 Python 里做滤波）；而飞控侧只要求 ≥5Hz 的有效注入值，
        #   50Hz 完全够。限到 50Hz 后 CPU 降到约 1/3。
        self.max_rate_hz = self.declare_parameter('max_rate_hz', 50.0).value
        self.last_t = 0.0
        self.open_loop = bool(self.declare_parameter('open_loop', False).value)
        period = self.declare_parameter('log_period_s', 2.0).value

        self.fx = 0.0
        self.fy = 0.0
        self.n = 0
        self.n_zero = 0
        self.n_rej = 0
        self.n_spike_run = 0
        self.sum_raw = 0.0
        self.sum_f = 0.0

        self.pub = self.create_publisher(Twist, '/velocity_map_filtered', 10)
        self.create_subscription(Twist, '/velocity_map', self.on_vel, 10)
        self.create_timer(period, self.report)
        if self.open_loop:
            # 方案1：不依赖输入，固定 100Hz 发"有效 0"，飞控看门狗始终满足
            self.create_timer(0.01, self.publish_zero)
        self.get_logger().info(
            'velocity_filter: deadband=%.1f alpha=%.2f max=%.1f invalid=%.1f OPEN_LOOP=%s'
            % (self.deadband, self.alpha, self.max_cms, self.invalid_cms, self.open_loop))

    def on_vel(self, msg):
        # 降采样：距上次处理不足 1/max_rate_hz 秒的帧直接丢掉（不滤波也不发布）
        if self.max_rate_hz > 0.0:
            now = time.monotonic()
            if now - self.last_t < 1.0 / self.max_rate_hz:
                return
            self.last_t = now
        # ===== 方案1：打开飞控的位置环 =====
        # 注入【有效的 0 速度】：告诉飞控"我没在动"，它的定点环就不做自主修正，
        # 退化成纯速度执行器；位置闭环全交给香橙派的 PID（用雷达位姿）。
        # 必须"发有效值0"，不能"不发帧"——不发会让 200ms 看门狗把测量置无效(0x8000)。
        if self.open_loop:
            # 由定时器统一发 0（见 __init__），这里不重复处理
            return

        vx = msg.linear.x * 100.0
        vy = msg.linear.y * 100.0
        raw_mag = math.hypot(vx, vy)

        # 1) 尖峰剔除：超过阈值判为不可信 -> 本周期目标=保持上一次输出
        #    （不灌新值，也不丢帧——丢帧会让飞控看门狗报"传感器无效"）
        if raw_mag >= self.spike_cms:
            # 2026-10-06 修正：原来只要超阈值就"冻结输出（保持上一次）"。
            # 这会形成正反馈：真开始漂 → 雷达测到 >45 → 冻结成 0 → 飞控速度环失去反馈
            # （指令退化成加速度指令）→ 越飘越快（实测飞走 3.5m、以及"起飞很飘"）。
            # 现在：单帧超阈值仍保持上一次（抗噪声尖峰），但**连续 5 帧都超阈值**
            # （≈0.33s，真实运动的尺度）→ 判定为真实高速，**限幅通过**。
            self.n_rej += 1
            self.n_spike_run += 1
            if self.n_spike_run >= 5:
                scale = self.max_cms / raw_mag
                tx, ty = vx * scale, vy * scale
            else:
                tx, ty = self.fx, self.fy
        elif raw_mag < self.deadband:
            # 2) 死区：小于死区当作静止
            self.n_spike_run = 0
            tx, ty = 0.0, 0.0
        else:
            self.n_spike_run = 0
            tx, ty = vx, vy

        # 3) 一阶低通
        fx_new = self.fx + self.alpha * (tx - self.fx)
        fy_new = self.fy + self.alpha * (ty - self.fy)

        # 4) ★ 变化率限制（最关键）：每个周期最多变化 slew_cms
        #    静止时实测注入速度尖峰到 37cm/s，这一步把它削平，
        #    不让瞬时跳变直接灌进飞控定点环。
        self.fx += max(-self.slew, min(self.slew, fx_new - self.fx))
        self.fy += max(-self.slew, min(self.slew, fy_new - self.fy))

        # 5) 限幅
        self.fx = max(-self.max_cms, min(self.max_cms, self.fx))
        self.fy = max(-self.max_cms, min(self.max_cms, self.fy))

        out = Twist()
        out.linear.x = self.fx / 100.0
        out.linear.y = self.fy / 100.0
        self.pub.publish(out)

        self.n += 1
        if raw_mag < self.deadband:
            self.n_zero += 1
        self.sum_raw += raw_mag
        self.sum_f += math.hypot(self.fx, self.fy)

    def publish_zero(self):
        self.pub.publish(Twist())
        self.n += 1
        self.n_zero += 1

    def report(self):
        if self.n == 0:
            return
        self.get_logger().info(
            'raw|v| avg=%.2f filtered|v| avg=%.2f | deadband %.0f%% | spikes(>=%.0f) %d of %d'
            % (self.sum_raw / self.n, self.sum_f / self.n, 100.0 * self.n_zero / self.n,
               self.spike_cms, self.n_rej, self.n))
        self.n = self.n_zero = self.n_rej = 0
        self.sum_raw = self.sum_f = 0.0


def main():
    rclpy.init()
    node = VelocityFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
