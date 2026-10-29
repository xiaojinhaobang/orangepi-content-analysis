"""Passive ROS receipt timing: no publishers, services or control commands."""
import json,sys,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16,Int32MultiArray,Float32MultiArray,UInt8MultiArray,UInt8,Int16MultiArray

class Observer(Node):
    def __init__(self,stream):
        super().__init__('velocity_height_observer')
        self.stream=stream
        self.last={}
        self.subs=[]
        topics=[('/velocity_map',Twist),('/velocity_map_filtered',Twist),('/scan',LaserScan),('/height',Int16),('/fc_height_diag',Int32MultiArray),('/fc_vel',Vector3),('/fc_mag',Float32MultiArray),('/target_velocity',Float32MultiArray),('/fc_target',Float32MultiArray),('/fc_status',UInt8MultiArray),('/active_controller',UInt8),('/mission_cmd',Int16MultiArray)]
        for topic,kind in topics:
            self.subs.append(self.create_subscription(kind,topic,self.callback(topic),100))
        self.create_timer(1.0,stream.flush)
    def callback(self,topic):
        def receive(m):
            now=time.monotonic_ns()
            gap=(now-self.last[topic])/1e9 if topic in self.last else None
            self.last[topic]=now
            if isinstance(m,Twist):
                value=[m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]
            elif isinstance(m,LaserScan):
                value={'stamp_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'frame':m.header.frame_id,'range_count':len(m.ranges)}
            elif isinstance(m,Vector3): value=[m.x,m.y,m.z]
            else: value=list(m.data) if hasattr(m.data,'__iter__') else m.data
            self.stream.write(json.dumps({'wall_ns':time.time_ns(),'mono_ns':now,'gap_s':gap,'topic':topic,'value':value})+'\n')
        return receive

def main():
    rclpy.init()
    with open(sys.argv[1],'x',encoding='utf-8') as stream:
        node=Observer(stream)
        deadline=time.monotonic()+1800
        try:
            while rclpy.ok() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=0.1)
        finally:
            node.destroy_node()
            rclpy.shutdown()
if __name__=='__main__': main()
