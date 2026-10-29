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
import fcntl, math, os, sys, time
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
        self.act=None
        self.create_subscription(UInt8,'/active_controller',self.on_act,10)
        self.tv_n=0
        self.create_subscription(Float32MultiArray,'/target_velocity_nav',self.on_tv,20)
    def on_fc(self,m):
        if len(m.data)>=7: self.fc=list(m.data)
    def on_h(self,m): self.h=int(m.data)
    def on_t(self,m):
        if len(m.data)>=2: self.tacc=(int(m.data[0]),int(m.data[1]))
    def on_s(self,m): self.slam=(m.linear.x*100.,m.linear.y*100.)
    def on_r(self,m): self.raw=(m.linear.x*100.,m.linear.y*100.)
    def on_act(self,m): self.act=int(m.data)
    def on_tv(self,m): self.tv_n+=1
    def pose(self):
        try:
            tr=self.buf.lookup_transform('map','dr_base',rclpy.time.Time(seconds=0))
            return (tr.transform.translation.x*100., tr.transform.translation.y*100.,
                    q2yaw(tr.transform.rotation))
        except Exception:
            return None

secs=float(sys.argv[1]) if len(sys.argv)>1 else 300.0
h_tgt=float(sys.argv[2]) if len(sys.argv)>2 else 150.0
rclpy.init(); n=H()
# ★ 2026-10-19：单主控锁。mission_node 跑任务时持有同一把锁；
#   抢不到就退出，避免 20:02 那种"旧任务 + 悬停测试"同时写控制话题。
_lock=open('/tmp/flight_owner.lock','w')
try:
    fcntl.flock(_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except OSError:
    print('!! 任务链正在跑（/tmp/flight_owner.lock 被占用）→ 拒绝启动悬停测试', flush=True)
    sys.exit(3)
_lock.write(str(os.getpid())); _lock.flush()
for _ in range(12):
    n.pa.publish(UInt8(data=2)); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.15)
print('>>> 位置环已激活 (/active_controller=2)', flush=True)

# WAIT_UNLOCK: 先起脚本、等操作手解锁，解锁后立刻发目标+0x66（抢 15s 自降窗口）
print('>>> 等待解锁（CH5 中位 + CH6 高位）…', flush=True)
tw = time.time()
while time.time() - tw < 180:
    rclpy.spin_once(n, timeout_sec=0.05)
    if n.fc and len(n.fc) >= 2 and n.fc[1] == 1 and n.fc[0] == 2:
        break
# ★ 2026-10-19：超时没解锁就**不能**继续。原代码不管有没有解锁都会往下发
#   目标点和 0x66[1,H] —— 在地面/上锁状态下发一键起飞是危险的。
if not (n.fc and len(n.fc) >= 2 and n.fc[1] == 1 and n.fc[0] == 2):
    print('!! 180s 内没等到解锁（mode=2 unlock=1）→ 退出，不发目标/0x66', flush=True)
    sys.exit(4)
print('>>> 已解锁（等待 %.1fs），开始发目标+0x66' % (time.time() - tw), flush=True)
time.sleep(0.2)

# ★ 2026-10-19：解锁后**重新**置一次 active=2。
#   实测 21:43 那轮：起飞前 53 秒的等待里 active 被清掉，PID 收到目标却一帧不发，
#   整场只有飞控自己定点 → 悬停漂 5~18cm。这里在解锁瞬间补发，确保 PID 是活的。
for _ in range(8):
    n.pa.publish(UInt8(data=2)); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.1)

# 目标点 = 当前位置悬停，高度 150cm。没有它位置环一行速度都不会发。
hold=n.pose()
if hold is None:
    print('!! 拿不到 tf map->dr_base，无法设定目标点：先确认链路起来了', flush=True)
    sys.exit(2)
for _ in range(8):
    # 目标 yaw 必须用**当前航向**，不能写死 0：导航系是地图系，
    # 机头朝向不一定是 0°，写 0 会让飞控先把飞机转 90° 再飞（危险）。
    m0=Float32MultiArray(); m0.data=[float(hold[0]), float(hold[1]), h_tgt, float(hold[2])]
    n.pt.publish(m0); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.15)
print('>>> 目标点已发: x=%.1f y=%.1f z=%.0fcm yaw=%.1f (悬停)'
      % (hold[0], hold[1], h_tgt, hold[2]), flush=True)

# ★ 2026-10-19：发 0x66 之前必须确认 PID 真的在出速度。
#   21:43 那轮就是没确认：PID 收到目标但不发布，飞机靠飞控硬撑，漂 5~18cm。
_tv0=n.tv_n
t_chk=time.time()
while time.time()-t_chk < 1.5 and n.tv_n == _tv0:
    rclpy.spin_once(n,timeout_sec=0.05)
if n.tv_n == _tv0:
    print('!! 1.5s 内 /target_velocity_nav 没有任何消息 → 位置环没接管，不发 0x66',
          flush=True)
    sys.exit(5)
print('>>> 位置环已确认在出速度（%.1f 帧/1.5s）' % (n.tv_n - _tv0), flush=True)

m=Int16MultiArray(); m.data=[1,int(h_tgt)]
for _ in range(6):
    n.pc.publish(m); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.15)
print('>>> 0x66 起飞已发(%.0fcm)' % h_tgt, flush=True)

os.makedirs('/home/orangepi/traces', exist_ok=True)
path='/home/orangepi/traces/hover_%s.log' % time.strftime('%Y%m%d_%H%M%S')
t0=time.time(); ref=None; out=open(path,'w')
print('>>> 记录到', path, flush=True)
while time.time()-t0 < secs:
    rclpy.spin_once(n,timeout_sec=0.05)
    # ★ 2026-10-19：**不再每 0.5s 强抢 active=2**。
    #   原来那样会 0.5s 内推翻 mission/guard 的 active=3，等于测试脚本能否决安全停止。
    #   现在只观察：被置 3（停止）或 0（关环）就立即中止，把控制权交回去。
    if n.act == 3:
        print('!! /active_controller=3（有人要求停止/接管）→ 中止悬停测试', flush=True)
        break
    if n.act == 0:
        print('!! /active_controller=0（位置环被关闭）→ 中止悬停测试', flush=True)
        break
    p=n.pose()
    if p:
        if n.h is not None and n.h>=30 and ref is None:
            ref=p; print('>>> 已离地 h=%d 参考点(%.0f,%.0f)'%(n.h,p[0],p[1]), flush=True)
        drift=math.hypot(p[0]-ref[0], p[1]-ref[1]) if ref else 0.0
        dyaw=((p[2]-ref[2]+180)%360)-180 if ref else 0.0
        fc=n.fc or [None]*7
        line=('t=%6.1f x=%7.1f y=%7.1f yaw=%6.1f drift=%6.1f dyaw=%6.1f '
              '| filt=(%6.1f,%6.1f) raw=(%6.1f,%6.1f) tacc=%s | h=%s mode=%s unlock=%s act=%s'
              % (time.time()-t0, p[0], p[1], p[2], drift, dyaw,
                 n.slam[0], n.slam[1], n.raw[0], n.raw[1], n.tacc, n.h, fc[0], fc[1],
                 n.act))
        print(line, flush=True); out.write(line+'\n'); out.flush()
    time.sleep(0.2)
out.close(); print('>>> done ->', path, flush=True)
