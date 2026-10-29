#!/usr/bin/env python3
"""悬停测试（方案1：只用香橙派的位置环）。
用法: hover_test.py [秒数,默认300]
日志: ~/traces/hover_<时间>.log  （持久，重启不丢）

2026-09-27 修复（事故原因：三个开关同时处于"关"的状态 → 完全开环 → 必飘）:
  1) /active_controller 由 1 改成 2。pid_controller.cpp 里
     1 -> is_active_controller_=false，PID 算出速度但**不发布**；2 才是无人机激活。
  2) 新增发布 /target_position = [当前位置 x, y, 150, 0]。
     pid_controller.cpp 的 controlTimerCallback 第一行就是
     `if (!has_target_position_) return;` —— 没有目标点，位置环什么都不发。
  3) 二者必须在 0x66 起飞之前发布，否则离地瞬间没有水平闭环。
配套要求: 起飞前确认 speed_source=雷达 且 velocity_filter 处于 open_loop=true
          （飞控只看我们的 0x31 速度指令，不再自己做定点修正）。
"""
import math, sys, time
import rclpy, tf2_ros
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray, UInt8, UInt8MultiArray
from geometry_msgs.msg import Twist

def q2yaw(q):
    return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

class H(Node):
    def __init__(self):
        super().__init__('hover_test')
        self.fc=None; self.h=None; self.tacc=None; self.slam=(0.,0.); self.raw=(0.,0.)
        self.buf=tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf,self)
        self.pa=self.create_publisher(UInt8,'/active_controller',10)
        self.pc=self.create_publisher(Int16MultiArray,'/mission_cmd',10)
        self.pt=self.create_publisher(Float32MultiArray,'/target_position',10)
        self.create_subscription(UInt8MultiArray,'/fc_status',self.on_fc,10)
        self.create_subscription(Int16,'/height',self.on_h,10)
        self.create_subscription(Int16MultiArray,'/fc_vel_echo',self.on_t,10)
        self.create_subscription(Twist,'/velocity_map_filtered',self.on_s,10)
        self.create_subscription(Twist,'/velocity_map',self.on_r,10)
    def on_fc(self,m):
        if len(m.data)>=7: self.fc=list(m.data)
    def on_h(self,m): self.h=int(m.data)
    def on_t(self,m):
        if len(m.data)>=2: self.tacc=(int(m.data[0]),int(m.data[1]))
    def on_s(self,m): self.slam=(m.linear.x*100.,m.linear.y*100.)
    def on_r(self,m): self.raw=(m.linear.x*100.,m.linear.y*100.)
    def pose(self):
        try:
            tr=self.buf.lookup_transform('map','dr_base',rclpy.time.Time(seconds=0))
            return (tr.transform.translation.x*100., tr.transform.translation.y*100.,
                    q2yaw(tr.transform.rotation))
        except Exception:
            return None

secs=float(sys.argv[1]) if len(sys.argv)>1 else 300.0
rclpy.init(); n=H()
for _ in range(12):
    n.pa.publish(UInt8(data=2)); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.15)
print('>>> 位置环已激活 (/active_controller=2)', flush=True)

# 目标点 = 当前位置悬停，高度 150cm。没有它位置环一行速度都不会发。
hold=n.pose()
if hold is None:
    print('!! 拿不到 tf map->dr_base，无法设定目标点：先确认链路起来了', flush=True)
    sys.exit(2)
for _ in range(8):
    # 目标 yaw 必须用**当前航向**，不能写死 0：导航系是地图系，
    # 机头朝向不一定是 0°，写 0 会让飞控先把飞机转 90° 再飞（危险）。
    m0=Float32MultiArray(); m0.data=[float(hold[0]), float(hold[1]), 150.0, float(hold[2])]
    n.pt.publish(m0); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.15)
print('>>> 目标点已发: x=%.1f y=%.1f z=150cm yaw=%.1f (悬停)' % (hold[0], hold[1], hold[2]), flush=True)

m=Int16MultiArray(); m.data=[1,150]
for _ in range(6):
    n.pc.publish(m); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.15)
print('>>> 0x66 起飞已发(150cm)', flush=True)

import os
os.makedirs('/home/orangepi/traces', exist_ok=True)
path='/home/orangepi/traces/hover_%s.log' % time.strftime('%Y%m%d_%H%M%S')
t0=time.time(); ref=None; out=open(path,'w')
print('>>> 记录到', path, flush=True)
while time.time()-t0 < secs:
    rclpy.spin_once(n,timeout_sec=0.05)
    p=n.pose()
    if p:
        if n.h is not None and n.h>=30 and ref is None:
            ref=p; print('>>> 已离地 h=%d 参考点(%.0f,%.0f)'%(n.h,p[0],p[1]), flush=True)
        drift=math.hypot(p[0]-ref[0], p[1]-ref[1]) if ref else 0.0
        dyaw=((p[2]-ref[2]+180)%360)-180 if ref else 0.0
        fc=n.fc or [None]*7
        line=('t=%6.1f x=%7.1f y=%7.1f yaw=%6.1f drift=%6.1f dyaw=%6.1f '
              '| filt=(%6.1f,%6.1f) raw=(%6.1f,%6.1f) tacc=%s | h=%s mode=%s unlock=%s'
              % (time.time()-t0, p[0], p[1], p[2], drift, dyaw,
                 n.slam[0], n.slam[1], n.raw[0], n.raw[1], n.tacc, n.h, fc[0], fc[1]))
        print(line, flush=True); out.write(line+'\n'); out.flush()
    time.sleep(0.2)
out.close(); print('>>> done ->', path, flush=True)
