#!/usr/bin/env python3
"""简易定位（航位推算）：速度积分 -> 发布 tf map -> base_link

阶段三"简易地图"方案：不建图、不用雷达位姿，只把速度积出来当坐标。
位置环 (pid_control_pkg) 只需要 tf map->laser_link，所以本节点一发 tf 就能直接用。

输入（source 参数）：
  raw       /velocity_map          (Twist, m/s)  ← 方案1 用这个
  filtered  /velocity_map_filtered (Twist, m/s)  默认
  fc_echo   /fc_vel_echo           (Int16, cm/s)

★ 2026-09-27 修复：方案1（velocity_filter open_loop=true）时
  /velocity_map_filtered 是**恒定 0**，再拿它积分等于位姿冻结 →
  位置环以为飞机没动，误差永远不为 0。所以方案1 必须 source=raw。
航向：由 /target_velocity[3]（我们下发的 yaw dps）积分，或不积分（yaw_hold）
输出：
  tf: map -> base_link
  /dead_reckon/pose (PoseStamped)  便于观察/记录
参数：
  source, rate_hz, yaw_from_cmd, init_yaw_deg, vx_sign, vy_sign, max_v_cms
  axis_swap  默认 false。★ 已废弃：固定旋转只在"机头朝向不变"时成立，
             实测机头转过 90° 后同一次推机就落到另一根轴上。
             现在改成：导航系 = 地图系（不旋转），由 body_vel_adapter.py
             按 tf 里的实时航向把速度转到机体系。
  yaw_from_tf 默认 true：dr_base 的 yaw 直接取 tf map->heading_frame 的航向
             （即雷达 SLAM 测到的机头朝向），供 body_vel_adapter 旋转用。
             以前是积分我们的 yaw 指令，开机就偏（初始朝向没对齐）。
"""
import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Empty, Float32MultiArray, Int16MultiArray


class DeadReckon(Node):
    def __init__(self):
        super().__init__('dead_reckon')
        self.source = self.declare_parameter('source', 'filtered').value
        rate = self.declare_parameter('rate_hz', 50.0).value
        self.yaw_from_cmd = self.declare_parameter('yaw_from_cmd', True).value
        self.init_yaw = math.radians(self.declare_parameter('init_yaw_deg', 0.0).value)
        self.vx_sign = self.declare_parameter('vx_sign', 1.0).value
        self.vy_sign = self.declare_parameter('vy_sign', 1.0).value
        self.axis_swap = self.declare_parameter('axis_swap', False).value
        self.yaw_from_tf = self.declare_parameter('yaw_from_tf', True).value
        self.heading_frame = self.declare_parameter('heading_frame', 'laser_link').value
        # A/B: passthrough=true 时不再积分速度，直接把 cartographer 的
        # map->base_link 镜像成 map->dr_base；下游话题名保持不变。
        self.passthrough = bool(self.declare_parameter('passthrough', False).value)
        self.passthrough_frame = self.declare_parameter(
            'passthrough_frame', 'base_link').value
        self.off_x = 0.0
        self.off_y = 0.0
        self.off_yaw = 0.0
        self.offset_valid = False
        # 水平位姿跳变门控：20Hz 下 5cm/帧 = 100cm/s，
        # 高于正常飞行速度；超过它按坏帧丢弃，不传给位置环。
        self.max_step_cm = float(self.declare_parameter('max_step_cm', 5.0).value)
        self.last_pose = None
        self.last_jump_log = 0.0
        # 航向低通：SLAM 航向有跳变，直接进 PID 会被追着放大（曾导致自转）。
        # 2026-09-28：α=0.03 @50Hz → τ≈0.67s（当时最好成绩那一版用的就是它）。
        # 2026-09-30：改成 α=0.10（τ≈0.2s）试了两场，航向峰峰反而从 12° 涨到 21°/26°，
        # 于是**回退到 0.03** —— 结论：这条滞后不是主因，动了反而更差。
        self.yaw_lpf_alpha = self.declare_parameter('yaw_lpf_alpha', 0.03).value
        self.max_v_cms = self.declare_parameter('max_v_cms', 60.0).value
        self.child_frame = self.declare_parameter('child_frame', 'dr_base').value

        self.x_cm = 0.0
        self.y_cm = 0.0
        self.yaw = self.init_yaw
        self.vyaw_dps = 0.0
        self.have_vel = False

        self.tfb = tf2_ros.TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/dead_reckon/pose', 10)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)

        if self.source == 'fc_echo':
            self.create_subscription(Int16MultiArray, '/fc_vel_echo', self.on_fc, 10)
        elif self.source == 'raw':
            self.create_subscription(Twist, '/velocity_map', self.on_twist, 10)
        else:
            self.create_subscription(Twist, '/velocity_map_filtered', self.on_twist, 10)
        self.create_subscription(Float32MultiArray, '/target_velocity', self.on_cmd, 10)
        # ★ 2026-10-06：归零话题。原来"归零"靠 reset_origin.sh 杀进程再重启，
        #   那条路要走 6s sleep + 两次 ros2 CLI 诊断打印 ≈ 15 秒（任务起飞前白等）；
        #   而且 dead_reckon 现在归 ros2 launch 托管，单杀它还可能被 launch 连坐。
        #   现在任务节点直接发一条 Empty 就完成归零（<50ms）。
        self.create_subscription(Empty, '/dead_reckon/reset', self.on_reset, 10)

        self.last_t = self.get_clock().now()
        self.create_timer(1.0 / rate, self.step)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            'dead_reckon: source=%s rate=%.0fHz yaw_from_cmd=%s yaw_lpf_alpha=%.3f (τ≈%.2fs)' %
            (self.source, rate, self.yaw_from_cmd, self.yaw_lpf_alpha,
             1.0 / max(self.yaw_lpf_alpha * rate, 1e-6)))
        if self.passthrough:
            self.get_logger().info(
                'dead_reckon passthrough: map->%s mirrored to map->%s'
                % (self.passthrough_frame, self.child_frame))

    def on_twist(self, m):
        vx = m.linear.x * 100.0
        vy = m.linear.y * 100.0
        if self.axis_swap:
            vx, vy = vy, -vx
        self._set_vel(vx, vy)

    def on_fc(self, m):
        if len(m.data) >= 2:
            self._set_vel(float(m.data[0]), float(m.data[1]))

    def _set_vel(self, vx_cms, vy_cms):
        if abs(vx_cms) > self.max_v_cms or abs(vy_cms) > self.max_v_cms:
            return  # 超速判为不可信，丢掉这一帧
        self.vx_cms = self.vx_sign * vx_cms
        self.vy_cms = self.vy_sign * vy_cms
        self.have_vel = True

    def on_cmd(self, m):
        if len(m.data) >= 4 and self.yaw_from_cmd:
            self.vyaw_dps = float(m.data[3])

    def on_reset(self, m):
        """把积分位置清到 0（= 当前飞机位置当作原点），不重启进程。"""
        if self.passthrough:
            try:
                tr = self.buf.lookup_transform(
                    'map', self.passthrough_frame, rclpy.time.Time())
                self.off_x = tr.transform.translation.x
                self.off_y = tr.transform.translation.y
                q = tr.transform.rotation
                self.off_yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.offset_valid = True
                self.last_pose = None
                self.have_vel = False
                self.get_logger().info(
                    'passthrough reset: map 原点偏移已记录，dr_base 将从 0 开始')
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(
                    'passthrough reset 失败：%s' % e)
            return
        self._set_vel(0.0, 0.0)
        self.x_cm = 0.0
        self.y_cm = 0.0
        # ★ 2026-10-06（关键修复）：yaw 也**直接对齐**到 tf 的当前值，不走低通。
        #   坑：本节点的 yaw 是 tf yaw 的一阶低通（α=0.03，收敛慢几秒）。原来
        #   "归零"只清位置不清航向 → 归零瞬间残留的旧航向被任务当成 ψ0 去生成
        #   整条航线 → **航线整体被转掉一个角度**（实测 28.5°：归零时 28.5°，
        #   10 秒后自己收敛回 0°，说明真实航向是 0°，28.5° 是残留）。
        #   现象：本该"直上直下"的段变成"又往下又往右"，而且到点 dyaw 报 28~32°。
        if self.yaw_from_tf:
            try:
                tr = self.buf.lookup_transform('map', self.heading_frame, rclpy.time.Time())
                q = tr.transform.rotation
                self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.have_vel = False
            except Exception:  # noqa: BLE001
                pass
        self.get_logger().info(
            '收到 /dead_reckon/reset → 位置已归零，yaw 已对齐 tf (%.1f°)' % math.degrees(self.yaw))

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if self.passthrough:
            self.publish_passthrough(now)
            return
        if dt <= 0.0 or dt > 0.5:
            return
        if self.have_vel:
            self.x_cm += getattr(self, 'vx_cms', 0.0) * dt
            self.y_cm += getattr(self, 'vy_cms', 0.0) * dt
        if self.yaw_from_tf:
            try:
                tr = self.buf.lookup_transform('map', self.heading_frame, rclpy.time.Time())
                q = tr.transform.rotation
                yaw_raw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                dyaw = ((yaw_raw - self.yaw + math.pi) % (2.0 * math.pi)) - math.pi
                self.yaw += self.yaw_lpf_alpha * dyaw
            except Exception:
                pass
        elif self.yaw_from_cmd:
            self.yaw += math.radians(self.vyaw_dps) * dt

        tr = TransformStamped()
        tr.header.stamp = now.to_msg()
        tr.header.frame_id = 'map'
        tr.child_frame_id = self.child_frame
        tr.transform.translation.x = self.x_cm / 100.0
        tr.transform.translation.y = self.y_cm / 100.0
        tr.transform.translation.z = 0.0
        tr.transform.rotation.z = math.sin(self.yaw / 2.0)
        tr.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tfb.sendTransform(tr)

        ps = PoseStamped()
        ps.header = tr.header
        ps.pose.position.x = tr.transform.translation.x
        ps.pose.position.y = tr.transform.translation.y
        ps.pose.orientation.z = tr.transform.rotation.z
        ps.pose.orientation.w = tr.transform.rotation.w
        self.pose_pub.publish(ps)

    def publish_passthrough(self, now):
        try:
            src = self.buf.lookup_transform(
                'map', self.passthrough_frame, rclpy.time.Time())
            p = src.transform.translation
            q = src.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if not self.offset_valid:
                self.off_x = p.x
                self.off_y = p.y
                self.off_yaw = yaw
                self.offset_valid = True
            x = p.x - self.off_x
            y = p.y - self.off_y
            yaw -= self.off_yaw
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            if self.max_step_cm > 0.0 and self.last_pose is not None:
                step = math.hypot(
                    x - self.last_pose[0], y - self.last_pose[1])
                if step > self.max_step_cm:
                    now_s = time.time()
                    if now_s - self.last_jump_log > 1.0:
                        self.last_jump_log = now_s
                        self.get_logger().warn(
                            '拒绝水平位姿跳变：单步 %.2fcm > %.2fcm（保持上一帧）'
                            % (step, self.max_step_cm))
                    return
            tr = TransformStamped()
            tr.header.stamp = src.header.stamp
            tr.header.frame_id = 'map'
            tr.child_frame_id = self.child_frame
            tr.transform.translation.x = x
            tr.transform.translation.y = y
            tr.transform.translation.z = 0.0
            tr.transform.rotation.z = math.sin(yaw / 2.0)
            tr.transform.rotation.w = math.cos(yaw / 2.0)
            self.tfb.sendTransform(tr)

            ps = PoseStamped()
            ps.header = tr.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = tr.transform.rotation.z
            ps.pose.orientation.w = tr.transform.rotation.w
            self.pose_pub.publish(ps)
            self.last_pose = (x, y, yaw)
        except Exception:
            pass

    def report(self):
        self.get_logger().info('dead_reckon pose: x=%.1f y=%.1f yaw=%.1f deg' %
                               (self.x_cm, self.y_cm, math.degrees(self.yaw)))


def main():
    rclpy.init()
    node = DeadReckon()
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
