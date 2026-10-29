#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""imu_pub — 把飞控 IMU 打包成 sensor_msgs/Imu，给 cartographer 用（开 use_imu_data 用）。

为什么：cartographer 现在 `use_imu_data = false`，位姿纯靠扫描匹配；在"一面墙 + 空旷侧"
        的退化几何里航向(yaw)最弱 → 会卡住/跳 58°/跳几米（2026-10-16~17 实测）。
        陀螺对 yaw 是强约束，喂进去能从根上改善。

输入（飞控透传，实测频率）：
  /fc_gyro       geometry_msgs/Vector3           200Hz  原始陀螺 LSB（18.4 LSB = 1 deg/s）
  /fc_acc        geometry_msgs/Vector3           200Hz  原始加计 LSB（静止 az≈1366 → 约 139 LSB/(m/s^2)）
  /fc_attitude   std_msgs/Float32MultiArray      185Hz  (roll, pitch, yaw) 单位：度

输出：
  /imu           sensor_msgs/Imu                 ← 直接转发陀螺/加计，姿态由欧拉角算四元数
  /tf_static     imu_link → base_link（安装偏置未知，先用单位变换；2D 只关心转动）

注意（上机前务必确认，否则比不开更差）：
  1. 静止时 linear_acceleration 的模 ≈ 9.81
  2. 静止时 angular_velocity 的模 ≈ 0（零偏多少）
  3. frame 能变换到 tracking_frame（amphi.lua 里是 laser_link）

环境变量：IMU_GYRO_SCALE=18.4  IMU_ACC_SCALE=139  IMU_FRAME=imu_link  IMU_PARENT=base_link
用法：python3 ~/diag/imu_pub.py
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import StaticTransformBroadcaster

GYRO_SCALE = float(os.environ.get('IMU_GYRO_SCALE', '18.4'))   # LSB per deg/s
ACC_SCALE = float(os.environ.get('IMU_ACC_SCALE', '139.0'))    # LSB per m/s^2
FRAME = os.environ.get('IMU_FRAME', 'imu_link')
PARENT = os.environ.get('IMU_PARENT', 'base_link')


class ImuPub(Node):
    def __init__(self):
        super().__init__('imu_pub')
        self.pub = self.create_publisher(Imu, '/imu', 50)
        self.create_subscription(Vector3, '/fc_gyro', self.on_gyro, 50)
        self.create_subscription(Vector3, '/fc_acc', self.on_acc, 50)
        self.create_subscription(Float32MultiArray, '/fc_attitude', self.on_att, 20)

        self.g = (0.0, 0.0, 0.0)
        self.a = (0.0, 0.0, 0.0)
        self.rpy = (0.0, 0.0, 0.0)        # 度
        self.n = 0
        self.ng = 0
        self.na = 0
        self.t_print = 0.0
        self.gsum = [0.0, 0.0, 0.0]
        self.asum = [0.0, 0.0, 0.0]

        self.static = StaticTransformBroadcaster(self)
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = PARENT
        ts.child_frame_id = FRAME
        ts.transform.rotation.w = 1.0
        self.static.sendTransform(ts)

        self.line('imu_pub 启动 | gyro %.1f LSB/(deg/s)  acc %.0f LSB/(m/s^2)  %s -> %s'
                  % (GYRO_SCALE, ACC_SCALE, PARENT, FRAME))
        self.create_timer(1.0, self.report)

    def line(self, s):
        self.get_logger().info(s)

    def on_att(self, m):
        if len(m.data) >= 3:
            self.rpy = (float(m.data[0]), float(m.data[1]), float(m.data[2]))

    def on_acc(self, m):
        self.a = (float(m.x), float(m.y), float(m.z))
        self.na += 1
        for i, v in enumerate(self.a):
            self.asum[i] += v

    def on_gyro(self, m):
        self.g = (float(m.x), float(m.y), float(m.z))
        self.ng += 1
        for i, v in enumerate(self.g):
            self.gsum[i] += v
        self.publish_imu()

    def publish_imu(self):
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = FRAME

        # (roll,pitch,yaw) 度 → 四元数（ZYX，与 ROS 一致）
        r = math.radians(self.rpy[0])
        p = math.radians(self.rpy[1])
        y = math.radians(self.rpy[2])
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        m.orientation.w = cr * cp * cy + sr * sp * sy
        m.orientation.x = sr * cp * cy - cr * sp * sy
        m.orientation.y = cr * sp * cy + sr * cp * sy
        m.orientation.z = cr * cp * sy - sr * sp * cy
        m.orientation_covariance[0] = 0.01
        m.orientation_covariance[4] = 0.01
        m.orientation_covariance[8] = 0.01

        # 陀螺：LSB → deg/s → rad/s
        k = math.radians(1.0) / GYRO_SCALE
        m.angular_velocity.x = self.g[0] * k
        m.angular_velocity.y = self.g[1] * k
        m.angular_velocity.z = self.g[2] * k
        for i in range(3):
            m.angular_velocity_covariance[i * 4] = 1e-3

        # 加计：LSB → m/s^2
        ka = 1.0 / ACC_SCALE
        m.linear_acceleration.x = self.a[0] * ka
        m.linear_acceleration.y = self.a[1] * ka
        m.linear_acceleration.z = self.a[2] * ka
        for i in range(3):
            m.linear_acceleration_covariance[i * 4] = 1e-2

        self.pub.publish(m)
        self.n += 1

    def report(self):
        gx, gy, gz = (self.gsum[i] / max(1, self.ng) for i in range(3))
        ax, ay, az = (self.asum[i] / max(1, self.na) for i in range(3))
        anorm = math.sqrt(ax * ax + ay * ay + az * az) / ACC_SCALE
        gnorm = math.sqrt(gx * gx + gy * gy + gz * gz) / GYRO_SCALE * math.pi / 180.0
        self.line('pub %d 帧 | 加计模 %.2f m/s^2（静止应≈9.81） | 陀螺零偏模 %.2f °/s | rpy=%.1f/%.1f/%.1f'
                  % (self.n, anorm, math.degrees(gnorm), self.rpy[0], self.rpy[1], self.rpy[2],))
        self.ng = self.na = 0
        self.gsum = [0.0, 0.0, 0.0]
        self.asum = [0.0, 0.0, 0.0]


def main():
    rclpy.init()
    n = ImuPub()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
