"""Passive ROS event recorder. No publishers, services, or control commands."""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray, UInt8, UInt8MultiArray, Int32MultiArray


TOPICS = [
    ('/fc_height_diag', Int32MultiArray),
    ('/height', Int16),
    ('/target_velocity', Float32MultiArray),
    ('/target_velocity_nav', Float32MultiArray),
    ('/fc_target', Float32MultiArray),
    ('/fc_target_vel', Vector3),
    ('/fc_vel', Vector3),
    ('/fc_status', UInt8MultiArray),
    ('/active_controller', UInt8),
    ('/mission_cmd', Int16MultiArray),
]


class Observer(Node):
    def __init__(self, stream):
        super().__init__('takeoff_observer')
        self.stream = stream
        self.origin = time.monotonic()
        self.subscriptions_kept = [
            self.create_subscription(kind, topic, self.callback(topic), 100)
            for topic, kind in TOPICS
        ]
        self.flush_timer = self.create_timer(1.0, stream.flush)

    def callback(self, topic):
        def receive(message):
            if hasattr(message, 'data'):
                value = message.data
                if not isinstance(value, (int, float)):
                    value = list(value)
            else:
                value = [message.x, message.y, message.z]
            self.stream.write(json.dumps({
                'wall_ns': time.time_ns(),
                'elapsed_s': time.monotonic() - self.origin,
                'topic': topic, 'value': value,
            }, separators=(',', ':')) + '\n')
        return receive


def main():
    rclpy.init()
    with open(sys.argv[1], 'x', encoding='utf8') as stream:
        stream.write(json.dumps({'kind': 'metadata', 'wall_ns': time.time_ns(),
                                 'scope': 'ROS receipt events; not UART delivery or internal FC height target',
                                 'topics': [topic for topic, _ in TOPICS]}) + '\n')
        node = Observer(stream)
        deadline = time.monotonic() + 1800
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
