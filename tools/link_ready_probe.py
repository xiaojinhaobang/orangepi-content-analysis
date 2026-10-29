#!/usr/bin/env python3
"""Measure real cold-start readiness of the flight link, item by item."""

import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, UInt8


class Probe(Node):
    def __init__(self):
        super().__init__('link_ready_probe')
        self.t_fc = None
        self.t_tv = None
        self.t_vel = None
        self.t_act = None
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(Int16, '/fc_link_len', self._fc, 1)
        self.create_subscription(Float32MultiArray, '/target_velocity', self._tv, 1)
        self.create_subscription(Twist, '/velocity_map', self._vel, 1)
        self.pub = self.create_publisher(UInt8, '/active_controller', 10)

    def _fc(self, m):
        if self.t_fc is None:
            self.t_fc = time.time()

    def _tv(self, m):
        if self.t_tv is None:
            self.t_tv = time.time()

    def _vel(self, m):
        if self.t_vel is None:
            self.t_vel = time.time()

    def tf_ok(self):
        try:
            self.buf.lookup_transform('map', 'dr_base', rclpy.time.Time())
            return True
        except Exception:
            return False


def main():
    rclpy.init()
    n = Probe()
    t0 = time.time()
    t_tf = None
    while time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.05)
        el = time.time() - t0
        if t_tf is None and n.tf_ok():
            t_tf = time.time()
        if (t_tf and n.t_fc and n.t_vel and n.t_act is None and n.t_tv is None):
            m = UInt8()
            m.data = 2
            for _ in range(10):
                n.pub.publish(m)
                rclpy.spin_once(n, timeout_sec=0.02)
            n.t_act = time.time()
        if t_tf and n.t_fc and n.t_vel and n.t_tv:
            break
    print('t(s)  tf   fc   vel  tv')
    for name, t in (('tf map->dr_base', t_tf), ('fc link len', n.t_fc),
                    ('radar velocity', n.t_vel), ('activate sent', n.t_act),
                    ('target_velocity', n.t_tv)):
        print('  %-16s %s' % (name, ('%.1f s' % (t - t0)) if t else 'NOT READY'))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
