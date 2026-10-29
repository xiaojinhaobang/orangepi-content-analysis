#!/usr/bin/env python3
"""导航系速度 -> 机体系速度（位置环与飞控之间的坐标适配层）。

为什么需要它：
  position_pid_controller 的 calculateErrors() 只在导航系里算误差，
  算完直接把结果当 /target_velocity 发出去，而飞控的 0x31 要求**机体速度**
  （x=机头, y=机身左侧）。中间少了「按当前机头朝向旋转」这一步。
  实测：机头朝向从 89° 变到 0° 后，同一次"往前推"的位移从 x 轴跑到 y 轴，
  证明不旋转的话修正方向会整体错 90°，飞机一转向就变成正反馈。

本节点：
  输入  /target_velocity_nav  [vx_nav, vy_nav, vz, vyaw]   （位置环输出，导航系）
  输出  /target_velocity      [vx_body, vy_body, vz, vyaw] （uart_to_stm32 消费）
  旋转量: 航向 psi = tf map->dr_base 的 yaw（dead_reckon 用雷达 SLAM 的机头朝向填）
  公式:   v_body = R(-psi) * v_nav

★ 2026-10-14 追加：**偏航阻尼 + 角度保持**（把测试链路的成果搬进任务链路）
  为什么：任务链路的 C++ 位置环是 kp_yaw=0（只剩 ki_yaw=0.04），实测平移段
  航向峰峰 21°，而测试链路（pos_hold/leg_test 里同样的阻尼器）只有 2.4~5.6°。
  控制律：vyaw_out = w_nav + damp
          damp = -( k*(带通 0.05~0.5Hz + 超前预测) + kyaw*(psi - psi_target) )
  航向目标 psi_target 来自 /target_position[3]（waypoint_nav 发的目标航向；
  掉头时它会整跳 +45°，我们的角度项正好**帮它转**，不会跟它对着干）。
  环境变量（默认全开）：
    BVA_DAMP=0/1  BVA_K=2.2  BVA_KYAW=1.2  BVA_DEAD=0.5  BVA_TLEAD=0.2
    BVA_FHP=0.05  BVA_FLP=0.5  BVA_VMAX=5.0  BVA_CSV=/tmp/bva_damp.csv
"""
import math
import os
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int16, UInt8

GYRO_SCALE = 18.4          # LSB/(deg/s)，和 pos_hold / leg_test 一致


def _envf(name, default):
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return float(default)


def wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0


class YawDamper:
    """0.05~0.5Hz 带通 + 相位超前（速率阻尼）+ 角度保持，量化成整数 °/s。

    和 pos_hold.py / leg_test.py 里的同一套：带通看不见 DC（恒定偏航力矩），
    所以必须再加一个角度项；量化 1°/s ⇒ 增益要够（kyaw≥1 才能用 1° 误差发出 1°/s）。
    """

    def __init__(self, f_hp, f_lp, t_lead, k, vmax, dead, kyaw=0.0):
        self.f_hp, self.f_lp, self.t_lead = f_hp, f_lp, t_lead
        self.k, self.vmax, self.dead, self.kyaw = k, vmax, dead, kyaw
        self.x_prev = None
        self.hp = self.lp = self.lp_prev = self.dlp = 0.0
        self.raw = self.q = 0.0

    def update(self, x, dt, err_deg=0.0):
        dt = min(max(dt, 1e-3), 0.2)
        if self.x_prev is None:
            self.x_prev = x
            return 0.0
        rc_hp = 1.0 / (2.0 * math.pi * self.f_hp)
        self.hp = (rc_hp / (rc_hp + dt)) * (self.hp + x - self.x_prev)
        self.x_prev = x
        rc_lp = 1.0 / (2.0 * math.pi * self.f_lp)
        self.lp += (dt / (rc_lp + dt)) * (self.hp - self.lp)
        d = (self.lp - self.lp_prev) / dt
        self.lp_prev = self.lp
        self.dlp += 0.25 * (d - self.dlp)
        cmd = -(self.k * (self.lp + self.t_lead * self.dlp) + self.kyaw * err_deg)
        cmd = max(-self.vmax, min(self.vmax, cmd))
        self.raw = cmd
        if abs(cmd) < self.dead:
            self.q = 0.0
        else:
            q = int(round(cmd))
            self.q = float(q if q else (1 if cmd > 0 else -1))
        return self.q


class BodyVelAdapter(Node):
    def __init__(self):
        super().__init__('body_vel_adapter')
        self.nav_frame = self.declare_parameter('nav_frame', 'dr_base').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(Float32MultiArray, '/target_velocity', 10)
        self.create_subscription(Float32MultiArray, '/target_velocity_nav', self.on_cmd, 10)
        self.n = 0
        # ---- 偏航阻尼 / 角度保持 ----
        self.damp_on = os.environ.get('BVA_DAMP', '1').strip().lower() not in (
            '0', 'off', 'false', 'no')
        self.damp = YawDamper(
            f_hp=float(os.environ.get('BVA_FHP', 0.05)),
            f_lp=float(os.environ.get('BVA_FLP', 0.5)),
            t_lead=float(os.environ.get('BVA_TLEAD', 0.2)),
            k=float(os.environ.get('BVA_K', 2.2)),
            vmax=float(os.environ.get('BVA_VMAX', 5.0)),
            dead=float(os.environ.get('BVA_DEAD', 0.5)),
            kyaw=float(os.environ.get('BVA_KYAW', 1.2)))
        self.gz = None
        self.psi_tgt = None
        self.tgt = None              # /target_position 的 [x, y, z]（地图系 cm）
        self.last_t = None
        self.create_subscription(Vector3, '/fc_gyro', self.on_gyro, 10)
        # ★2026-10-16：融合航向（heading_fuse 发布：陀螺为主 + 雷达慢速校准）
        self.create_subscription(Float32, '/heading_fused', self.on_hf, 10)
        self.hf = None
        self.hf_t = 0.0
        self.create_subscription(Float32MultiArray, '/target_position', self.on_tgt, 10)
        # ---- 速度前馈 + 低增益 P（方案二，2026-10-14；默认关）----
        #   为什么：任务用位置闭环，位置来自雷达 SLAM（滞后 0.35~0.5s、带噪）
        #   → 减速段过冲、来回修，现场看就是"直线也晃"。改成：
        #     远段  v = 前馈(恒速,近了带刹车) + 低增益P      ← 不用滞后的位置判断"到没到"
        #     过渡  两者线性混合
        #     近段  v = 原位置环输出                        ← 停位精度仍交给原环
        #   z / yaw 一律不动（z 走原环，yaw 走上面那套阻尼+角度保持）。
        self.ff_on = os.environ.get('BVA_FF', '0').strip().lower() not in (
            '0', 'off', 'false', 'no')
        # ★ 2026-10-14 深夜：20 → 13。实测（20:52 那轮）指令上限 22cm/s 而
        #   **实测峰速 31.9cm/s（超 45%）** —— 超速后位置环看到误差掉太快就反向刹，
        #   来回就是"震"（现场："从悬停点到第一个码点很震"，那段 90cm 是最长直线，
        #   前馈满速巡航，问题最明显）。先把巡航速度压下来，让"指令 ≈ 实际"。
        #   要回退：把 13 改回 20。
        self.vff = _envf('BVA_VFF', 20.0)          # 前馈巡航速度 cm/s（直线测试 20 就稳，回到 20）
        # ★ 2026-10-14 深夜：0.5 → 0.30（现场反馈"减速段收敛过程一直拖、扫完还在继续"）。
        #   0.5 时：距离 40cm 还在 20cm/s 巡航、到 18cm 才降到 9cm/s —— 飞机速度有滞后，
        #   等它真减速已经冲过头，位置环再反着拉回来，就是那一顿一顿的收敛。
        #   0.30 时：巡航速度不变（仍是 vff=20），但**从 66cm 就开始收**，
        #   末段交回位置环时的速度 9 → 5.5cm/s，超调应当明显变小。
        #   要回退：把下面的 0.30 改回 0.5，或者 systemd drop-in 里加
        #   Environment=BVA_KBRAKE=0.5 覆盖（需要 root，普通账号 tee 不进去）。
        # ★ 2026-10-14 深夜：0.30 → 0.5（**撤回**，现场"还是不稳定，改回最开始的"）。
        self.kbrake = _envf('BVA_KBRAKE', 0.5)     # 刹车曲线: v_ff = min(vff, kbrake*d)
        # ★ 2026-10-14 深夜：0.15 → 0.05。现场关键观察："之前直线测试飞 20cm/s 也挺稳"。
        #   差别在**闭环**：leg_test 是开环发速度（位置交给飞控定点环），任务是
        #   追**滞后 0.35~0.5s 的雷达位置** → 冲过头、反向拉 → 横向来回摆（"飘"）。
        #   把位置误差项压到 0.05，任务航段的行为就接近直线测试那套开环；
        #   留一点用来消慢漂。极端情况可设 0（纯开环）。要回退：改回 0.15。
        # ★ 2026-10-14 深夜：0.05 → 0.15（**撤回**，同上）。
        self.kp_low = _envf('BVA_KP_LOW', 0.15)    # 低增益 P（消漂）
        self.ff_far = _envf('BVA_FF_FAR', 35.0)    # ≥ 这个距离：纯前馈+低P
        self.ff_near = _envf('BVA_FF_NEAR', 18.0)  # ≤ 这个距离：交回位置环
        # ★ 2026-10-14 深夜（现场要求"确保这一段不会震动"）：
        #   悬停点→A进-0 是最长的直线（90cm），前馈上限原来跟着 max_linear_velocity=22，
        #   而实测**实际跑到 31.9cm/s（超 45%）** → 冲过头、位置环反向拉 → 来回震。
        #   把前馈上限压到 15：长直线段不再冲，扫描段（36~40cm，速度本来只有 10~18）
        #   基本不受影响。要回退：改回 22。
        self.vmax_ff = _envf('BVA_VMAX_FF', 15.0)  # 前馈段总限幅
        # ★ 2026-10-14 深夜：**过路模式（匀速直穿，不刹车）**。
        #   现场要求："扫完这一面到降落点，线路一样，但是匀速、不要刹车了。"
        #   扫完最后一面之后剩下的航点是"绕行 → 绕行 → land"，只要飞过去，
        #   不需要在每个过路点收口（收口 = 刹到零再加速 = 一顿一顿）。
        #   由 /bva_no_brake（Bool）运行时切换：mission_node 在到达最后一个扫描点后置 True。
        #   True 时：v = vff 恒定（不随距离衰减），且**不交回位置环**，一路匀速穿过去；
        #            过路点的"到达"由 waypoint_nav 的单点 tol_cm 判定，不需要精确停位。
        self.no_brake = _envf('BVA_NO_BRAKE', 0.0) > 0.5
        self.create_subscription(Bool, '/bva_no_brake', self.on_no_brake, 10)
        # ★ 2026-10-15：位置环是否已接管（=2 才算）。用来决定"要不要注入偏航"，
        #   见 on_cmd 末尾的说明。
        self.active = 0
        self.create_subscription(UInt8, '/active_controller', self.on_active, 10)
        # ★ 2026-10-15：**离地门**（照抄悬停自测链 pos_hold 的 h>60 才介入）。
        #   现场判断："不会是硬件问题吧，会不会是角度逻辑为了到达某个角度才这样" ——
        #   对：我们注入的水平速度在离地前会让飞控**倾斜机身**去产生那个速度，
        #   地上表现就是"侧翻趋势"。而 pos_hold 有 h>60 才介入的门，所以它从不侧翻；
        #   我们这条链原来没有门，且起飞前 /target_position 往往是上一轮的**残留目标**
        #   （距离很大）→ 一解锁就发水平速度 → 立刻歪。加门后：低空一律只发 0。
        self.height = None
        self.create_subscription(Int16, '/height', self.on_height, 10)
        self.air_min = _envf('BVA_AIR_MIN', 40.0)   # 低于这个高度不发任何控制量
        self.csv = None
        try:
            self.csv = open(os.environ.get('BVA_CSV', '/tmp/bva_damp.csv'), 'w')
            self.csv.write('t,psi,psi_tgt,err,gz_dps,bp,pred,damp_raw,damp_q,w_nav,vyaw_out\n')
        except Exception as e:                 # noqa: BLE001
            self.get_logger().warn('阻尼 CSV 打不开: %s' % e)
        self.rows = 0
        self.get_logger().info('body_vel_adapter: %s -> body, heading from tf %s->%s'
                               % ('/target_velocity_nav', self.map_frame, self.nav_frame))
        self.get_logger().info(
            '偏航阻尼: %s k=%.2f kyaw=%.2f dead=%.2f t_lead=%.2fs（目标来自 /target_position[3]）'
            % ('开' if self.damp_on else '关', self.damp.k, self.damp.kyaw,
               self.damp.dead, self.damp.t_lead))
        self.get_logger().info(
            '速度前馈: %s vff=%.0f kbrake=%.2f kp_low=%.2f 过渡 %.0f~%.0fcm（近段交回位置环）'
            % ('开' if self.ff_on else '关', self.vff, self.kbrake, self.kp_low,
               self.ff_near, self.ff_far))

    def on_gyro(self, m):
        self.gz = float(m.z)

    def on_tgt(self, m):
        if len(m.data) >= 4:
            self.tgt = (float(m.data[0]), float(m.data[1]), float(m.data[2]))
            # ★ 2026-10-14 深夜：记住"起飞/巡航高度"（挂上环后第一个目标点的 z=142），
            #   用来判断"这一段要不要让 z 环介入"（见 on_cmd 末尾的分档）。
            if getattr(self, 'z_home', None) is None:
                self.z_home = float(m.data[2])
                self.get_logger().warn('z 环基准高度 = %.0fcm（目标≈它时 vz 归零，交给飞控）'
                                       % self.z_home)

    def on_no_brake(self, m):
        """过路模式开关（见 __init__ 里 no_brake 的说明）。"""
        self._nb_apply(m)

    def on_active(self, m):
        """位置环接管状态（=2 才算接管）。见 on_cmd 末尾"偏航只在接管后注入"。"""
        self.active = int(m.data)

    def on_height(self, m):
        self.height = float(m.data)

    def _nb_apply(self, m):
        new = bool(m.data)
        if new != self.no_brake:
            self.no_brake = new
            self.nb_dir = None          # 每次开启都重新锁方向（见 on_cmd 里的说明）
            self.get_logger().warn('过路模式 %s（%s）'
                                   % ('ON：匀速直穿、不刹车'
                                      if new else 'OFF：恢复正常前馈刹车 + 交回位置环',
                                      '扫完最后一面，飞往降落点'))
            # ★ 2026-10-15：这里原来残留一行 `self.psi_tgt = float(m.data[3])`，
            #   但 /bva_no_brake 是 **Bool**（1 字节）→ 每次开关到达都会抛异常，
            #   回调直接崩掉 → 过路模式实际上从没正常生效过（"开了反而不动"）。
            #   已删除。

    def heading(self):
        p = self.pose()
        return None if p is None else p[2]

    def on_hf(self, m):
        self.hf = float(m.data)
        self.hf_t = time.time()

    def pose(self):
        """返回 (x_cm, y_cm, psi_rad)（地图系）；拿不到返回 None。"""
        try:
            tr = self.buf.lookup_transform(self.map_frame, self.nav_frame, rclpy.time.Time())
            q = tr.transform.rotation
            psi = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            # ★2026-10-16：航向优先用融合值（陀螺为主 + 雷达慢速校准），过期才回退雷达航向
            if self.hf is not None and (time.time() - self.hf_t) < 0.5:
                psi = math.radians(self.hf)
            return (tr.transform.translation.x * 100.0,
                    tr.transform.translation.y * 100.0,
                    psi)
        except Exception:
            return None

    def on_cmd(self, m):
        if len(m.data) < 4:
            return
        now = time.time()
        dt = (now - self.last_t) if self.last_t else 0.05
        self.last_t = now
        vx, vy, vz, w = float(m.data[0]), float(m.data[1]), float(m.data[2]), float(m.data[3])
        p = self.pose()
        psi = None if p is None else p[2]
        # ---- 速度前馈 + 低增益 P（BVA_FF=1 才启用；默认关）----
        #   远段不用滞后的位置判断"到没到"，由前馈把位移带走，低增益 P 只消漂；
        #   近段交回原位置环收口（保 8cm 停位精度）。
        ff_tag = '-'
        # ★ 2026-10-15（现场关键修复）：**只有位置环接管（active_controller==2）时才允许
        #   改造速度**。原来这里没有这道门 → 手飞/待命时（active≠2）它照样拿
        #   **上一轮的残留目标**算出速度注入飞控 → 飞机朝旧目标飘
        #   （现场："刚才手飞了一次，一开始可以后面直接飘了"）。
        if (self.ff_on and getattr(self, 'active', 0) == 2
                and p is not None and self.tgt is not None):
            # ★ 2026-10-15（现场方案）：**走廊模式** —— 每到一个新航点就重新锁方向，
            #   整个航段用"固定方向 + 恒速"飞（位置环少干预），只在近段（≤ff_near）
            #   交回位置环收口。这样扫描段的每条 36~40cm 也基本是匀速直滑，
            #   和直线测试手感一致（现场："直线飞得好，航向就不会偏"）。
            if self.tgt != getattr(self, 'nb_last_tgt', None):
                self.nb_last_tgt = self.tgt
                self.nb_dir = None
                self.nb_tgt0 = None
                self._cor_psi = None      # 新航段：机体系旋转基准航向也重取
                self._cor_left = None     # 新航段：巡航计时重算（见下面"速度×时间"）
            ex, ey = self.tgt[0] - p[0], self.tgt[1] - p[1]
            d = math.hypot(ex, ey)
            if d > 1e-3:
                ux, uy = ex / d, ey / d
                # ★ 2026-10-15：交接阈值 18cm → **8cm**。
                #   现场"结束一面扫描后飞往降落点，飞一下顿一下" —— 过路点容差是 30cm，
                #   但走廊模式在 18cm 就交回位置环收口 → 每个过路点都减速一次。
                #   收到 8cm 后：过路点（容差 30cm）根本到不了 8cm → 一路匀速穿过。
                # ★ 2026-10-15：8.0 → **14.0**（折中）。
                #   收 8cm 的代价：扫描点也要到 8cm 才开始收 → **冲过头**（现场：
                #   "飞到最左边那个飞过头了"，那里 A1 距 A出-0 只有 11cm，一冲全过）。
                #   14cm 时：扫描点（容差 8~25cm）提前 6cm 进收口区、不再冲过头；
                #   过路点（容差 30cm）根本到不了 14cm 内 → 过渡段仍是匀速直穿。
                # ★ 2026-10-15（现场方案）：**按"速度 × 时间"飞，和直线测试一模一样**。
                #   本段巡航时间 T = 距离 / 巡航速度；前 T 秒恒速直飞（固定方向），
                #   T 用完就交回位置环收口。好处：不依赖滞后的位置判断，
                #   和 leg_test 的手感一致；而每个格点都会停下来收口，
                #   所以航段误差不会累积（每格重新对准一次）。
                if getattr(self, '_cor_left', None) is None:
                    self._cor_left = min(d / max(self.vff, 1e-3), 8.0)
                if self._cor_left > 0:
                    self._cor_left -= dt
                    # ★ 2026-10-14 深夜：**方向锁死**（这才是"直线逻辑"的本质）。
                    #   原来这里还用 `ux, uy = ex/d, ey/d` —— 即每帧用**滞后 0.35~0.5s 的
                    #   雷达位置**重算方向，等于还在闭环追位置 → 还是走弧线（现场：
                    #   "还是没走直线逻辑"）。直线测试之所以稳，是因为它整段发**同一个
                    #   固定方向**的速度。所以这里在过路模式开启的那一刻把方向锁住，
                    #   之后一直用这个方向（直到关掉过路模式）。
                    if getattr(self, 'nb_dir', None) is None:
                        # 目标一变就重新锁方向（见 on_cmd 开头），所以这里直接锁：
                        # 方向来自**新目标**，不会再锁到残留目标。
                        self.nb_dir = (ux, uy)
                        # ★ 2026-10-15：**连机体系旋转的基准航向一起锁死**。
                        #   现场"飞直线时还是略歪" —— 原因是方向虽在地图系锁了，
                        #   但旋转到机体系用的是**实时航向**，航向飘一点路就弯一点。
                        #   锁住这一刻的 psi 后，机身系里也完全固定 → 绝对直线。
                        self._cor_psi = psi
                    if getattr(self, 'nb_dir', None) is None:
                        # 还没锁到方向（或还没等到新目标）→ 本拍输出 0，绝不乱走
                        nx = ny = 0.0
                    else:
                        ux, uy = self.nb_dir
                        nx = self.vff * ux
                        ny = self.vff * uy
                    # 过路段：匀速直穿（不衰减、不交回位置环）。
                    # ★ 2026-10-14：加 `d > ff_far(35cm)` 条件 —— 过路模式只在**远段**生效，
                    #   近了（<35cm）自动落回下面的正常逻辑（刹车 + 18cm 交回位置环收口）。
                    #   否则"恒速 20cm/s 冲一个 4cm 容差的点"会直接冲过去，下一目标在背后，
                    #   又变成反向拉回来（飘）。
                    # ★ 2026-10-15：纯开环 —— 固定方向 + 固定速度，**不叠加位置误差项**
                    #   （叠了就等于把滞后带噪的雷达位置又接回闭环，还是会抖）。
                    if getattr(self, 'nb_dir', None) is not None:
                        ff_tag = 'cru%3.0f' % d
                else:
                    # ★ 2026-10-16：走廊计时走完（交回位置环）→ **清掉锁定的基准航向**。
                    #   原来 `_cor_psi` 只在"航段切换"时重置，走廊结束后仍生效 →
                    #   末段收口时还用旧航向旋转机身系 → 最后几厘米方向会偏。
                    #   交回位置环就该用**实时航向**（和 pos_hold 一致）。
                    self._cor_psi = None
                    vmag = min(self.vff, self.kbrake * d)  # 巡航恒速 + 就近刹车
                    nx = vmag * ux + self.kp_low * ex
                    ny = vmag * uy + self.kp_low * ey
                    if d >= self.ff_far:
                        ff_tag = 'ff%3.0f' % d
                    elif d <= self.ff_near:
                        nx, ny, ff_tag = vx, vy, 'nav%3.0f' % d
                    else:
                        t = (d - self.ff_near) / (self.ff_far - self.ff_near)
                        nx = (1.0 - t) * vx + t * nx
                        ny = (1.0 - t) * vy + t * ny
                        ff_tag = 'mix%3.0f' % d
                mm = math.hypot(nx, ny)
                if mm > self.vmax_ff:
                    nx, ny = nx * self.vmax_ff / mm, ny * self.vmax_ff / mm
                vx, vy = nx, ny
        psi_c = getattr(self, '_cor_psi', None)
        psi_u = psi if (psi_c is None or psi is None) else psi_c
        if psi is None:
            bx, by = vx, vy                       # 拿不到航向就先不旋转（保守）
        else:
            c, s = math.cos(psi_u), math.sin(psi_u)
            bx = c * vx + s * vy
            by = -s * vx + c * vy
        # ---- 偏航阻尼 + 角度保持（把测试链路的成果搬进任务链路）----
        psi_deg = math.degrees(psi) if psi is not None else None
        err = 0.0
        if psi_deg is not None and self.psi_tgt is not None:
            err = wrap_deg(psi_deg - self.psi_tgt)
        w_out = w
        if self.damp_on and self.gz is not None and psi_deg is not None:
            dq = self.damp.update(self.gz / GYRO_SCALE, dt, err)
            w_out = max(-30.0, min(30.0, w + dq))     # 飞控侧上限 30°/s
        if self.csv is not None:
            try:
                self.csv.write(
                    '%.3f,%.2f,%s,%+.2f,%+.3f,%+.3f,%+.3f,%+.3f,%+.1f,%+.3f,%+.3f\n'
                    % (now, psi_deg if psi_deg is not None else float('nan'),
                       self.psi_tgt if self.psi_tgt is not None else float('nan'),
                       err, (self.gz or 0.0) / GYRO_SCALE, self.damp.lp,
                       self.damp.lp + self.damp.t_lead * self.damp.dlp,
                       self.damp.raw, self.damp.q, w, w_out))
                self.rows += 1
                if self.rows % 50 == 0:
                    self.csv.flush()
            except Exception:                     # noqa: BLE001
                pass
        out = Float32MultiArray()
        out.data = [bx, by, vz, w_out]
        # ★ 2026-10-14 深夜：**z 环分档**（治"悬停抖"）。
        #   现场："起飞悬停完全跟悬停测试一样好了，现在悬停还是抖"。
        #   悬停自测链（pos_hold）**完全不碰高度**（vz 恒 0，高度交给飞控自己），
        #   而任务链的 C++ 环一直在调 z → 两个环同时控高度，上下一拱一拱。
        #   但 z 不能整个关掉：下排要降到 103cm，而飞控不接受"降高度"命令。
        #   所以按目标高度分档：**目标离基准高度 <8cm → vz 归零**（交给飞控），
        #   需要换排/变高度时才让 z 环介入。
        if (self.tgt is not None and getattr(self, 'z_home', None) is not None
                and len(out.data) >= 3
                and abs(self.tgt[2] - self.z_home) < 8.0):
            out.data[2] = 0.0
        # ★ 2026-10-15：**换排（目标高度明显不同）时把下降速度放大**。
        #   现场"下降太慢了"：C++ 环 kp_z=0.3 → 39cm 的换排只给 11.7cm/s，
        #   还要跟飞控自己的 142 定高环抢 → 慢得离谱。C++ 参数在 install 目录里、
        #   改它要 sudo 密码（这台板子没放行），所以就在这一层把 vz 放大：
        #   |z_target − z_home| > 10cm（即换排/降落）时 ×3，并限幅 ±40cm/s。
        #   等价于把 kp_z 提到 ~0.9，但不动 C++/launch。
        elif (self.tgt is not None and getattr(self, 'z_home', None) is not None
                and len(out.data) >= 3 and abs(self.tgt[2] - self.z_home) > 10.0):
            zmax = _envf('BVA_VZ_BOOST_MAX', 40.0)
            out.data[2] = max(-zmax, min(zmax, out.data[2] * 3.0))
            # ★ 2026-10-15（现场方案①）：**竖直下降/上升期间冻结 xy**。
            #   换排是"同一个 x,y、只变高度"的一段；期间还发 xy 就会一边降一边横漂
            #   （现场："竖直下降没下到高度又开始向下飘到下面的航点"）。
            #   判据：目标 xy 与上一目标基本不变、而目标高度变化 >15cm → 纯竖直段。
            if (getattr(self, 'tgt_prev', None) is not None
                    and math.hypot(self.tgt[0] - self.tgt_prev[0],
                                   self.tgt[1] - self.tgt_prev[1]) < 5.0
                    and abs(self.tgt[2] - self.tgt_prev[2]) > 15.0):
                out.data[0] = 0.0
                out.data[1] = 0.0
        # ★ 2026-10-15：**离地门** —— 高度低于 air_min（默认 40cm）时，
        #   xy / z / 偏航**全部输出 0**（速度流照常发，保鲜不破）。
        #   这条是"不再侧翻"的关键：离地前不发水平速度，飞控就不需要倾斜机身。
        if (self.height is not None and self.height < self.air_min
                and len(out.data) >= 4):
            out.data[0] = 0.0
            out.data[1] = 0.0
            out.data[2] = 0.0
            out.data[3] = 0.0
        # ★ 2026-10-15：**给注入的偏航率加硬上限**（治"自旋"）。
        #   我们注进去的第 4 个分量是 vyaw，而 C++ 环的偏航只有积分（ki_yaw=0.04，
        #   没有 P/D）→ 积分一旦绕起来就会变成持续转速，现场表现就是"自旋"。
        #   正常阻尼只需要 ±3°/s，所以超过就截断（并打日志，便于确认是不是它在发）。
        # ★ 2026-10-15：**偏航只在位置环接管（/active_controller==2）之后才注入**。
        #   现场："起飞为什么有偏航量呢？起飞不是完全按悬停自测吗" —— 对，起飞段不该有。
        #   原因是这里的偏航阻尼/角度保持是**无条件叠加**的，它会拿
        #   `/target_position[3]` 当目标航向；而起飞时 waypoint_nav 还没起，
        #   那个航向是**上一轮的残留值** → 角度项去追一个错航向 → 起飞自己转头/自旋。
        #   所以：接管前 vyaw 恒 0（速度流照样发、保鲜不断，只是不带偏航指令）。
        #   接管后仍加 ±3°/s 硬上限，防积分绕起来变成持续自旋。
        if len(out.data) >= 4:
            # ★ 2026-10-15：±3 → **±10°/s**。现场指出"要保证第一个二维码前方向正确" ——
            #   进入扫描前机头可能偏 ~10°（第一段不发偏航、C++ 环又只有积分），
            #   而 ±3°/s 要 3 秒以上才拉正，飞机早开始扫了 → 整条扫描线是斜的。
            #   放到 ±10°/s：进 `A进-0`（hold 1 秒）那一下就基本摆正，扫描线才是直的。
            #   要回退：把这个 10.0 改回 3.0。
            vmax = _envf('BVA_VYAW_MAX', 10.0)
            # ★ 2026-10-15：**vz 合理性校验**。实测 /height 有毛刺（15483 个样本里
            #   h 从 1 到 3288cm），C++ 的 z 环生吃它 → 可能算出几千 cm/s 的 vz
            #   → 飞机突然一顿/掉一段（现场"中途突然降落一段高度"的可能来源）。
            #   超过 ±60cm/s 的一律按不可信处理：限到 ±40（换排需要的最大也就 ~40）。
            if len(out.data) >= 3 and abs(out.data[2]) > 60.0:
                self.get_logger().warn('vz 异常（%.0f cm/s）→ 按毛刺处理，限到 ±40'
                                       % out.data[2])
                out.data[2] = 40.0 if out.data[2] > 0 else -40.0
            # ★ 2026-10-15：**第一段（过路模式开启时）也不要注偏航** ——
            #   现场"到第一个二维码前还是抖"：走廊模式只把方向开环了，但挂环那刻
            #   偏航阻尼就启用，它在第一段还在纠航向 → 抖。等过了第一段再启用。
            if getattr(self, 'active', 0) != 2 or self.no_brake:
                if out.data[3] != 0.0:
                    out.data[3] = 0.0
            elif abs(out.data[3]) > vmax:
                self.get_logger().warn('vyaw 截断：%.1f → %.1f °/s'
                                       % (out.data[3], vmax if out.data[3] > 0 else -vmax))
                out.data[3] = vmax if out.data[3] > 0 else -vmax
        self.pub.publish(out)
        self.tgt_prev = self.tgt      # 供"纯竖直段"判据用（见上面的冻结 xy）
        self.n += 1
        if self.n % 250 == 0:
            self.get_logger().info(
                'psi=%.1f tgt=%s err=%+.1f damp=%+.1f ff=%s | nav(%+.1f,%+.1f) -> body(%+.1f,%+.1f) | vyaw %.2f->%.2f'
                % (psi_deg if psi_deg is not None else float('nan'),
                   ('%.1f' % self.psi_tgt) if self.psi_tgt is not None else 'nan',
                   err, self.damp.q, ff_tag, vx, vy, bx, by, w, w_out))


def main():
    rclpy.init()
    node = BodyVelAdapter()
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
