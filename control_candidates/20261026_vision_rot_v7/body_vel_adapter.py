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
from std_msgs.msg import (
    Bool, Float32, Float32MultiArray, Int16, UInt8, UInt8MultiArray)

GYRO_SCALE = 18.4          # LSB/(deg/s)，和 pos_hold / leg_test 一致


def _envf(name, default):
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return float(default)


def wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0


def add_cross_track(vx, vy, x, y, line_start, ux, uy, kp, max_corr):
    """沿锁定直线速度叠加受限横向纠偏。

    line_start / (x,y) 均为地图系 cm。cross>0 表示当前位置在直线左侧，
    因此沿直线左侧法向量的反方向修正。
    """
    if line_start is None or max_corr <= 0.0:
        return (vx, vy, 0.0, 0.0)
    cross = (-uy) * (x - line_start[0]) + ux * (y - line_start[1])
    corr = max(-max_corr, min(max_corr, -kp * cross))
    return (vx - corr * uy, vy + corr * ux, cross, corr)


def bounded_position_hold(ex, ey, kp, max_speed, deadband):
    """有界地图系定点速度：死区内为零，方向朝向目标。"""
    d = math.hypot(ex, ey)
    if not math.isfinite(d) or d <= deadband:
        return (0.0, 0.0)
    speed = min(max_speed, max(0.0, kp) * d)
    return (speed * ex / d, speed * ey / d)


def is_upward_row_change(prev, new, height, air_min, active,
                         min_dz=20.0, max_xy=12.0):
    """True only for an airborne, near-stationary XY, upward row change."""
    if prev is None or new is None or height is None:
        return False
    if float(height) < float(air_min) or int(active) != 2:
        return False
    try:
        dz = float(new[2]) - float(prev[2])
        dxy = math.hypot(float(new[0]) - float(prev[0]),
                         float(new[1]) - float(prev[1]))
    except (TypeError, ValueError, IndexError):
        return False
    return dz >= float(min_dz) and dxy <= float(max_xy)


def planned_line(prev_tgt, tgt, pose, min_len=1.0):
    """Return the planned segment anchor and direction.

    A waypoint may be accepted several centimetres before its true position.
    Using the live pose as the corridor anchor turns that residual into a
    permanent diagonal. The previous route target is the segment start; the
    live pose is only a fallback for the first target or a position-invariant
    target change.
    """
    if tgt is None:
        return (None, None)
    target = (float(tgt[0]), float(tgt[1]))
    if prev_tgt is not None:
        start = (float(prev_tgt[0]), float(prev_tgt[1]))
        dx, dy = target[0] - start[0], target[1] - start[1]
        length = math.hypot(dx, dy)
        if length >= min_len:
            return (start, (dx / length, dy / length))
    if pose is None:
        return (None, None)
    return ((float(pose[0]), float(pose[1])), None)


def heading_mark_ok(height, air_min, gz_dps, psi_tgt, err,
                    mark_gyro_max, mark_heading_err_max):
    """Physical heading gate for a laser mark.

    Controller selection is not part of this predicate. The mission owns mark
    requests; this gate only reports whether the aircraft is airborne and its
    heading is settled enough for a short pulse.
    """
    return bool(
        height is not None
        and height >= air_min
        and gz_dps is not None
        and psi_tgt is not None
        and abs(gz_dps) <= mark_gyro_max
        and abs(err) <= mark_heading_err_max)


class LaserZGuard:
    """Single, testable laser-Z authority and latched fail-stop policy."""

    def __init__(self, kp, deadband, up_max, down_max, freeze_err,
                 height_age, status_age, max_err):
        self.kp = float(kp)
        self.deadband = float(deadband)
        self.up_max = float(up_max)
        self.down_max = float(down_max)
        self.freeze_err = float(freeze_err)
        self.height_age = float(height_age)
        self.status_age = float(status_age)
        self.max_err = float(max_err)
        self.fault_reason = None

    def reset(self):
        self.fault_reason = None

    def evaluate(self, enabled, active, fc_status, fc_age, height,
                 height_age, target_z):
        result = {
            'vz': 0.0, 'freeze_xy': False,
            'fault': self.fault_reason, 'new_fault': None,
        }
        if not enabled:
            return result
        if self.fault_reason is not None:
            return result
        if active != 2:
            return result

        def fail(reason):
            self.fault_reason = reason
            result['fault'] = reason
            result['new_fault'] = reason
            return result

        if fc_status is None or not math.isfinite(fc_age) or fc_age > self.status_age:
            return fail('fc_status 过期')
        if fc_status != (3, 1):
            return fail('飞控不再处于mode3+unlock1（当前 %s）' % (fc_status,))
        if (height is None or not math.isfinite(height)
                or not math.isfinite(height_age) or height_age > self.height_age):
            return fail('激光 /height 过期')
        if height < 0.0 or height > 500.0:
            return fail('激光 /height 越界')
        if target_z is None or not math.isfinite(target_z):
            return result

        err = float(target_z) - float(height)
        if not math.isfinite(err) or abs(err) > self.max_err:
            return fail('激光/目标高度差异常 %.1fcm' % err)
        if abs(err) > self.freeze_err:
            result['freeze_xy'] = True
        if abs(err) <= self.deadband:
            result['vz'] = 0.0
        else:
            result['vz'] = max(
                -self.down_max, min(self.up_max, self.kp * err))
        return result


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

    def reset(self):
        """Clear filter memory when crossing a control-mode boundary."""
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


class ScanYawPD:
    """Bounded heading PD used only during locked straight scan legs.

    The legacy YawDamper is a band-pass/lead rate damper.  In the recorded
    scan it spent long intervals saturated at +/-2 deg/s while the heading
    error kept crossing the target.  For a straight scan leg the heading
    target is constant, so use a direct PD:

        yaw_rate = -Kp * (psi - psi_target) - Kd * gyro_z
    """

    def __init__(self, kp, kd, vmax, dead):
        self.kp = float(kp)
        self.kd = float(kd)
        self.vmax = float(vmax)
        self.dead = float(dead)
        self.raw = 0.0

    def update(self, err_deg, gyro_z_dps):
        cmd = -self.kp * err_deg - self.kd * gyro_z_dps
        self.raw = max(-self.vmax, min(self.vmax, cmd))
        if abs(self.raw) < self.dead:
            return 0.0
        q = int(round(self.raw))
        return float(q if q else (1 if self.raw > 0.0 else -1))


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
        # ★ 2026-10-20：默认关闭。100cm 悬停实测它以 ~1Hz 打 ±5°/s，
        #   把偏航激成 ±10° 的极限环（fcYawCmd 跟着翻）。先让飞控自己保航向。
        # ★ 2026-10-20：轻量重开。5°/s 太猛会自激；改为 vmax=2、kyaw=0.4、k=1.0，
        #   只压位置修正耦合出来的小偏航，不去抢飞控的主航向保持。
        self.damp_on = os.environ.get('BVA_DAMP', '1').strip().lower() not in (
            '0', 'off', 'false', 'no')
        self.damp = YawDamper(
            f_hp=float(os.environ.get('BVA_FHP', 0.05)),
            f_lp=float(os.environ.get('BVA_FLP', 0.5)),
            t_lead=float(os.environ.get('BVA_TLEAD', 0.2)),
            k=float(os.environ.get('BVA_K', 1.0)),
            vmax=float(os.environ.get('BVA_VMAX', 2.0)),
            dead=float(os.environ.get('BVA_DEAD', 0.5)),
            kyaw=float(os.environ.get('BVA_KYAW', 0.4)))
        # 2026-10-24 scan-yaw candidate.  Only active on a locked straight
        # scan leg (active=2, FF direction locked, no_brake=false).
        self.scan_yaw_on = os.environ.get(
            'BVA_SCAN_YAW', '1').strip().lower() not in (
                '0', 'off', 'false', 'no')
        self.scan_yaw = ScanYawPD(
            kp=float(os.environ.get('BVA_SCAN_YAW_KP', 0.50)),
            kd=float(os.environ.get('BVA_SCAN_YAW_KD', 0.28)),
            vmax=float(os.environ.get('BVA_SCAN_YAW_MAX', 6.0)),
            dead=float(os.environ.get('BVA_SCAN_YAW_DEAD', 0.5)))
        self.rotate_yaw = ScanYawPD(
            kp=float(os.environ.get('BVA_ROTATE_YAW_KP', 0.80)),
            kd=float(os.environ.get('BVA_ROTATE_YAW_KD', 0.20)),
            vmax=float(os.environ.get('BVA_ROTATE_YAW_MAX', 30.0)),
            dead=float(os.environ.get('BVA_ROTATE_YAW_DEAD', 0.5)))
        self.rotate_yaw_active = False
        self.rotate_hold_xy = None
        self.rotate_xy_kp = _envf('BVA_ROTATE_XY_KP', 0.80)
        # 本轮自转实测峰值横漂约 24cm；旧 8cm/s 的上限让回位拖了 2.2s。
        # 现在自转保持点与 C++ 使用同一个计划航点，不再互相拉扯，
        # 12cm/s 仅用于回到该点，仍低于扫描段 24cm/s。
        self.rotate_xy_max = _envf('BVA_ROTATE_XY_MAX', 12.0)
        self.rotate_xy_dead = _envf('BVA_ROTATE_XY_DEAD', 0.5)
        # B lower-row -> upper-row must be a vertical translation.  Keep the
        # live XY fixed while the commanded Z rises, then release at height.
        self.zchange_hold_xy = None
        self.zchange_xy_kp = _envf('BVA_ZCHANGE_XY_KP', 0.60)
        self.zchange_xy_max = _envf('BVA_ZCHANGE_XY_MAX', 8.0)
        self.zchange_xy_dead = _envf('BVA_ZCHANGE_XY_DEAD', 0.5)
        self.zchange_min_dz = _envf('BVA_ZCHANGE_MIN_DZ', 20.0)
        self.zchange_max_xy = _envf('BVA_ZCHANGE_MAX_XY', 12.0)
        self.zchange_release_cm = _envf('BVA_ZCHANGE_RELEASE_CM', 2.0)
        self.leg_yaw = ScanYawPD(
            kp=float(os.environ.get('BVA_LEG_YAW_KP', 0.50)),
            kd=float(os.environ.get('BVA_LEG_YAW_KD', 0.10)),
            vmax=float(os.environ.get('BVA_LEG_YAW_MAX', 3.0)),
            dead=float(os.environ.get('BVA_LEG_YAW_DEAD', 0.5)))
        self.yaw_mode_prev = None
        self.gz = None
        self.psi_tgt = None
        self.tgt = None              # /target_position 的 [x, y, z]（地图系 cm）
        self.tgt_prev_route = None   # 上一条不同位置的航线目标
        self.nb_dir = None
        self.nb_plan_dir = None
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
        self.vff = _envf('BVA_VFF', 24.0)          # 首段/两面过渡前馈速度
        self.scan_vff = _envf('BVA_SCAN_VFF', 24.0)
        # ★ 2026-10-14 深夜：0.5 → 0.30（现场反馈"减速段收敛过程一直拖、扫完还在继续"）。
        #   0.5 时：距离 40cm 还在 20cm/s 巡航、到 18cm 才降到 9cm/s —— 飞机速度有滞后，
        #   等它真减速已经冲过头，位置环再反着拉回来，就是那一顿一顿的收敛。
        #   0.30 时：巡航速度不变（仍是 vff=20），但**从 66cm 就开始收**，
        #   末段交回位置环时的速度 9 → 5.5cm/s，超调应当明显变小。
        #   要回退：把下面的 0.30 改回 0.5，或者 systemd drop-in 里加
        #   Environment=BVA_KBRAKE=0.5 覆盖（需要 root，普通账号 tee 不进去）。
        # ★ 2026-10-14 深夜：0.30 → 0.5（**撤回**，现场"还是不稳定，改回最开始的"）。
        self.kbrake = _envf('BVA_KBRAKE', 0.8)     # 刹车曲线: v_ff = min(vff, kbrake*d)
        self.scan_kbrake = _envf('BVA_SCAN_KBRAKE', 1.0)
        # ★ 2026-10-14 深夜：0.15 → 0.05。现场关键观察："之前直线测试飞 20cm/s 也挺稳"。
        #   差别在**闭环**：leg_test 是开环发速度（位置交给飞控定点环），任务是
        #   追**滞后 0.35~0.5s 的雷达位置** → 冲过头、反向拉 → 横向来回摆（"飘"）。
        #   把位置误差项压到 0.05，任务航段的行为就接近直线测试那套开环；
        #   留一点用来消慢漂。极端情况可设 0（纯开环）。要回退：改回 0.15。
        # ★ 2026-10-14 深夜：0.05 → 0.15（**撤回**，同上）。
        self.kp_low = _envf('BVA_KP_LOW', 0.15)    # 低增益 P（消漂）
        self.ff_far = _envf('BVA_FF_FAR', 35.0)    # ≥ 这个距离：纯前馈+低P
        # 短段（尤其 A进-0→A3 仅15cm）提前交回位置环，避免前馈冲过码位。
        self.ff_near = _envf('BVA_FF_NEAR', 5.0)   # ≤ 这个距离：交回位置环
        self.short_seg_cm = _envf('BVA_SHORT_SEG_CM', 25.0)
        self.short_ff_near = _envf('BVA_SHORT_FF_NEAR', 3.0)
        self.seg_ff_near = None
        # 起飞后的 90cm 首段惯性大；20cm 时实际仍会越过 A进-0 约 10cm。
        # 提前到 28cm 交回位置环，前馈巡航速度不变。
        self.ff_near_nb = _envf('BVA_FF_NEAR_NO_BRAKE', 18.0)
        # ★ 2026-10-14 深夜（现场要求"确保这一段不会震动"）：
        #   悬停点→A进-0 是最长的直线（90cm），前馈上限原来跟着 max_linear_velocity=22，
        #   而实测**实际跑到 31.9cm/s（超 45%）** → 冲过头、位置环反向拉 → 来回震。
        #   把前馈上限压到 15：长直线段不再冲，扫描段（36~40cm，速度本来只有 10~18）
        #   基本不受影响。要回退：改回 22。
        self.vmax_ff = _envf('BVA_VMAX_FF', 26.0)  # 过渡段总限幅
        self.scan_vmax_ff = _envf('BVA_SCAN_VMAX_FF', 24.0)
        self.xy_accel = _envf('BVA_XY_ACCEL', 30.0)  # 正常加速/转向变化率 cm/s²
        self.xy_prev = (0.0, 0.0)
        self.xy_t = None
        # 直线段低速横向纠偏。只修横向偏差，不改前馈主方向。
        self.cross_kp = _envf('BVA_CROSS_KP', 0.20)
        self.cross_max = _envf('BVA_CROSS_MAX', 3.0)
        self.scan_cross_kp = _envf('BVA_SCAN_CROSS_KP', 0.20)
        self.scan_cross_max = _envf('BVA_SCAN_CROSS_MAX', 4.0)
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
        # 程控模式激光 Z 唯一外环。
        # 只有 mission 显式 enable、飞控 mode_sta==3/unlock==1 且激光新鲜时才输出。
        # 任一安全前置条件失效即锁存故障，直到 mission 明确 disable 后再次 enable。
        # 历史复现默认关闭 LaserZGuard，让 C++ PID 的 Z 原样通过。
        self.laser_z_enabled = os.environ.get(
            'BVA_LASER_Z', '0').strip().lower() not in (
                '0', 'off', 'false', 'no')
        self.laser_z_engaged = False
        self.laser_z_fault = False
        self.lz_kp = _envf('BVA_LZ_KP', 0.50)
        self.lz_dead = _envf('BVA_LZ_DEAD', 2.0)
        self.lz_up = _envf('BVA_LZ_UP', 15.0)
        self.lz_down = _envf('BVA_LZ_DOWN', 12.0)
        self.lz_freeze_err = _envf('BVA_LZ_FREEZE_ERR', 5.0)
        self.lz_height_age = _envf('BVA_LZ_HEIGHT_AGE', 0.25)
        self.lz_status_age = _envf('BVA_LZ_STATUS_AGE', 0.50)
        self.lz_max_err = _envf('BVA_LZ_MAX_ERR', 200.0)
        self.lz_guard = LaserZGuard(
            self.lz_kp, self.lz_dead, self.lz_up, self.lz_down,
            self.lz_freeze_err, self.lz_height_age, self.lz_status_age,
            self.lz_max_err)
        self.height_t = 0.0
        self.fc_status = None
        self.fc_status_t = 0.0
        self.pub_lz_fault = self.create_publisher(
            Bool, '/laser_z_fault', 10)
        self.create_subscription(
            Bool, '/laser_z_enable', self.on_laser_z_enable, 10)
        self.create_subscription(
            UInt8MultiArray, '/fc_status', self.on_fc_status, 10)
        # 打点稳定窗：视觉瞄准之外，还要求机头角速度/航向误差在限内。
        # 上一轮把门放到 5°/15°/s 后，A3 仍在 err≈5.1° 的 0.96s
        # 关闭段跨过瞄准线。再留 1° 和 3°/s 的测量/量化余量；
        # 激光仍有 0.10 横向瞄准窗与 mission running 所有权门约束。
        self.mark_gyro_max = _envf('BVA_MARK_GYRO_MAX', 18.0)
        # A2 was centered while err sat at 6.0deg; the inclusive gate flapped
        # every frame and A2 never fired. 7deg keeps the gyro/height gates.
        self.mark_heading_err_max = _envf('BVA_MARK_HEADING_ERR_MAX', 7.0)
        self.mark_ok_prev = None
        self.pub_mark_ok = self.create_publisher(Bool, '/mark_ok', 10)
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
        self.lz_csv = None
        try:
            self.lz_csv = open(
                os.environ.get('BVA_LZ_CSV', '/tmp/bva_laser_z.csv'),
                'w')
            self.lz_csv.write(
                't,enable,active,mode,unlock,h,h_age,z_tgt,z_err,'
                'vz_out,freeze,fault\n')
        except Exception as e:                 # noqa: BLE001
            self.get_logger().warn('激光Z CSV 打不开: %s' % e)
        self.rows = 0
        self.get_logger().info('body_vel_adapter: %s -> body, heading from tf %s->%s'
                               % ('/target_velocity_nav', self.map_frame, self.nav_frame))
        self.get_logger().info(
            '偏航阻尼: %s k=%.2f kyaw=%.2f dead=%.2f t_lead=%.2fs（目标来自 /target_position[3]）'
            % ('开' if self.damp_on else '关', self.damp.k, self.damp.kyaw,
               self.damp.dead, self.damp.t_lead))
        self.get_logger().info(
            '速度前馈: %s 首段vff=%.0f/近段%.0fcm；'
            '扫描vff=%.0f/近段%.0fcm；kbrake=%.2f kp_low=%.2f'
            % ('开' if self.ff_on else '关', self.vff, self.ff_near_nb,
               self.scan_vff, self.ff_near, self.kbrake, self.kp_low))
        self.get_logger().info(
            '直线横向纠偏: 首段kp=%.2f/max=%.1f；扫描kp=%.2f/max=%.1fcm/s'
            % (self.cross_kp, self.cross_max,
               self.scan_cross_kp, self.scan_cross_max))
        self.get_logger().info('XY正常加速/转向平滑: %.1fcm/s²（减速与反向纠偏立即执行）'
                               % self.xy_accel)

    def on_gyro(self, m):
        self.gz = float(m.z)

    def on_tgt(self, m):
        if len(m.data) >= 4:
            new_tgt = (float(m.data[0]), float(m.data[1]), float(m.data[2]))
            prev_tgt = self.tgt
            if (self.tgt is None
                    or math.hypot(new_tgt[0] - self.tgt[0],
                                  new_tgt[1] - self.tgt[1]) > 0.2):
                self.tgt_prev_route = self.tgt
            self.tgt = new_tgt
            if (new_tgt != prev_tgt
                    and is_upward_row_change(
                        prev_tgt, new_tgt, self.height, self.air_min,
                        self.active, self.zchange_min_dz,
                        self.zchange_max_xy)):
                p = self.pose()
                self.zchange_hold_xy = (
                    (p[0], p[1]) if p is not None
                    else (new_tgt[0], new_tgt[1]))
                self.get_logger().warn(
                    '换高竖直锁：%.0f→%.0fcm，锁住XY (%.1f, %.1f)cm'
                    % (prev_tgt[2], new_tgt[2],
                       self.zchange_hold_xy[0], self.zchange_hold_xy[1]))
            elif (new_tgt != prev_tgt
                  and self.zchange_hold_xy is not None):
                self.zchange_hold_xy = None
            # ★ 2026-10-19：偏航角度保持的目标。
            #   原代码 psi_tgt 从未赋值（永远 None）→ YawDamper 的 kyaw 项是死代码，
            #   只剩带通阻尼，没有"回到目标航向"的能力。
            self.psi_tgt = wrap_deg(float(m.data[3]))
            # ★ 2026-10-14 深夜：记住"起飞/巡航高度"（挂上环后第一个目标点的 z=142），
            #   旧版本曾据此把上排 Z 归零；程控候选不再用它决定 Z。
            if getattr(self, 'z_home', None) is None:
                self.z_home = float(m.data[2])
                self.get_logger().warn('航点基准高度 = %.0fcm'
                                       % self.z_home)

    def on_no_brake(self, m):
        """过路模式开关（见 __init__ 里 no_brake 的说明）。"""
        self._nb_apply(m)

    def on_active(self, m):
        """位置环接管状态（=2 才算接管）。见 on_cmd 末尾"偏航只在接管后注入"。"""
        self.active = int(m.data)

    def on_height(self, m):
        h = float(m.data)
        now = time.monotonic()
        if not math.isfinite(h) or h < 0.0 or h > 500.0:
            return
        if (self.height is not None and self.height_t > 0.0
                and now - self.height_t <= self.lz_height_age
                and abs(h - self.height) > 100.0):
            return
        self.height = h
        self.height_t = now

    def on_fc_status(self, m):
        if len(m.data) >= 2:
            self.fc_status = (int(m.data[0]), int(m.data[1]))
            self.fc_status_t = time.monotonic()

    def _set_laser_z_fault(self, reason):
        if self.laser_z_fault:
            return
        self.laser_z_fault = True
        self.get_logger().error('激光 Z 故障锁存：%s；输出全零，等待任务中止/人工接管' % reason)
        msg = Bool()
        msg.data = True
        self.pub_lz_fault.publish(msg)

    def on_laser_z_enable(self, m):
        enabled = bool(m.data)
        if enabled == self.laser_z_engaged:
            return
        self.laser_z_engaged = enabled
        self.laser_z_fault = False
        self.lz_guard.reset()
        msg = Bool()
        msg.data = False
        self.pub_lz_fault.publish(msg)
        self.get_logger().warn('激光 Z %s（仅允许飞控程序模式3）'
                               % ('接管' if enabled else '释放'))

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

    def smooth_xy(self, vx, vy):
        """只缓和正常加速/转向；减速、小幅反向纠偏和无效输入不延续旧速度。"""
        now = time.monotonic()
        dt = 0.02 if self.xy_t is None else min(max(now - self.xy_t, 0.0), 0.1)
        self.xy_t = now
        px, py = self.xy_prev
        if not (math.isfinite(vx) and math.isfinite(vy)):
            self.xy_prev = (0.0, 0.0)
            return self.xy_prev
        # 保留原收口的减速幅度和反向纠偏，不平滑掉制动需求。
        if (math.hypot(vx, vy) < math.hypot(px, py) - 1e-6
                or (px * vx + py * vy < 0.0
                    and math.hypot(vx, vy) <= math.hypot(px, py) + 1e-6)):
            self.xy_prev = (vx, vy)
            return self.xy_prev
        dx, dy = vx - px, vy - py
        change = math.hypot(dx, dy)
        limit = max(0.0, self.xy_accel) * dt
        if change > limit and change > 0.0:
            scale = limit / change
            vx, vy = px + dx * scale, py + dy * scale
        self.xy_prev = (vx, vy)
        return self.xy_prev

    def on_cmd(self, m):
        if len(m.data) < 4:
            return
        now = time.time()
        dt = (now - self.last_t) if self.last_t else 0.05
        self.last_t = now
        vx, vy, vz, w = float(m.data[0]), float(m.data[1]), float(m.data[2]), float(m.data[3])
        p = self.pose()
        # 航向缺失或非有限时，下面禁止输出水平速度。
        psi = None if p is None or not math.isfinite(p[2]) else p[2]
        # ---- 速度前馈 + 低增益 P（BVA_FF=1 才启用；默认关）----
        #   远段不用滞后的位置判断"到没到"，由前馈把位移带走，低增益 P 只消漂；
        #   近段交回原位置环收口（保 8cm 停位精度）。
        ff_tag = '-'
        ff_near = self.ff_near_nb if self.no_brake else self.ff_near
        leg_vff = self.vff if self.no_brake else self.scan_vff
        leg_vmax = self.vmax_ff if self.no_brake else self.scan_vmax_ff
        leg_kbrake = self.kbrake if self.no_brake else self.scan_kbrake
        leg_cross_kp = self.cross_kp if self.no_brake else self.scan_cross_kp
        leg_cross_max = self.cross_max if self.no_brake else self.scan_cross_max
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
                self.nb_plan_dir = None
                self.nb_tgt0 = None
                self._cor_left = None     # 新航段：巡航计时重算（见下面"速度×时间"）
                self.nb_start, self.nb_plan_dir = planned_line(
                    self.tgt_prev_route, self.tgt, p)
                self.seg_ff_near = None
            ex, ey = self.tgt[0] - p[0], self.tgt[1] - p[1]
            d = math.hypot(ex, ey)
            if self.seg_ff_near is None:
                self.seg_ff_near = (
                    min(ff_near, self.short_ff_near)
                    if d <= self.short_seg_cm else ff_near)
            ff_near = self.seg_ff_near
            # 进入减速区就耗尽本航段巡航计时；回摆到远区也不重新巡航。
            # 只有上面的目标切换才重置计时，原35/18cm混合与PID收口照常执行。
            # 直线前馈保持到近段阈值；旧代码在35cm就提前结束，
            # 导致剩余20–35cm全部交给低增益位置环，肉眼表现为突然减速。
            if d <= ff_near:
                self._cor_left = 0.0
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
                    # 时间预算必须使用实际输出限幅，并计入从0加速到该速度的时间。
                    v_eff = max(1e-3, min(leg_vff, leg_vmax))
                    ramp_s = v_eff / max(self.xy_accel, 1e-3)
                    # 给近端位置环预留 8cm，避免固定方向巡航把中心点冲过去。
                    travel = max(0.0, d - ff_near)
                    self._cor_left = min(
                        travel / v_eff + 0.5 * ramp_s, 8.0)
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
                        # 优先用上一目标→当前目标的计划线，而不是带着到点
                        # 残差从实际位姿重新画一条斜线。
                        self.nb_dir = (self.nb_plan_dir or (ux, uy))
                        # 只锁地图方向；机体系指令随当前有效航向转换。
                    if getattr(self, 'nb_dir', None) is None:
                        # 还没锁到方向（或还没等到新目标）→ 本拍输出 0，绝不乱走
                        nx = ny = 0.0
                    else:
                        ux, uy = self.nb_dir
                        # 接近目标时按 kbrake 平滑降速，同时仍沿原直线飞行。
                        speed = min(leg_vff, leg_vmax,
                                    max(0.0, leg_kbrake * d))
                        nx = speed * ux
                        ny = speed * uy
                        nx, ny, _, _ = add_cross_track(
                            nx, ny, p[0], p[1], getattr(self, 'nb_start', None),
                            ux, uy, leg_cross_kp, leg_cross_max)
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
                    vmag = min(leg_vff, leg_vmax,
                               leg_kbrake * d)  # 巡航恒速 + 就近刹车
                    nx = vmag * ux + self.kp_low * ex
                    ny = vmag * uy + self.kp_low * ey
                    if d >= self.ff_far:
                        ff_tag = 'ff%3.0f' % d
                    elif d <= ff_near:
                        nx, ny, ff_tag = vx, vy, 'nav%3.0f' % d
                    else:
                        t = (d - ff_near) / (self.ff_far - ff_near)
                        nx = (1.0 - t) * vx + t * nx
                        ny = (1.0 - t) * vy + t * ny
                        ff_tag = 'mix%3.0f' % d
                mm = math.hypot(nx, ny)
                if mm > leg_vmax:
                    nx, ny = nx * leg_vmax / mm, ny * leg_vmax / mm
                vx, vy = nx, ny
        if (psi is None or getattr(self, 'active', 0) != 2
                or (self.height is not None and self.height < self.air_min)):
            # 原失效/低高度/未接管分支优先，不保留上次平滑状态。
            self.xy_prev, self.xy_t = (0.0, 0.0), None
        else:
            vx, vy = self.smooth_xy(vx, vy)
        if psi is None:
            bx = by = 0.0                         # 无有效航向，不透传地图系速度
        else:
            c, s = math.cos(psi), math.sin(psi)
            bx = c * vx + s * vy
            by = -s * vx + c * vy
        # ---- 偏航阻尼 + 角度保持（把测试链路的成果搬进任务链路）----
        psi_deg = math.degrees(psi) if psi is not None else None
        err = 0.0
        if psi_deg is not None and self.psi_tgt is not None:
            err = wrap_deg(psi_deg - self.psi_tgt)
        w_out = w
        gz_dps = (self.gz / GYRO_SCALE) if self.gz is not None else None
        mark_ok = heading_mark_ok(
            self.height, self.air_min, gz_dps, self.psi_tgt, err,
            self.mark_gyro_max, self.mark_heading_err_max)
        if mark_ok != self.mark_ok_prev:
            self.get_logger().info(
                '打点姿态门 %s（h=%s gz=%s err=%.1f active=%d）'
                % ('放行' if mark_ok else '关闭',
                   'nan' if self.height is None else '%.0f' % self.height,
                   'nan' if gz_dps is None else '%.1f' % gz_dps,
                   err, self.active))
            self.mark_ok_prev = mark_ok
        self.pub_mark_ok.publish(Bool(data=mark_ok))
        heading_pd_active = bool(
            self.scan_yaw_on
            and self.active == 2
            and psi_deg is not None
            and self.psi_tgt is not None
            and gz_dps is not None
            and self.height is not None
            and self.height >= self.air_min)
        rotate_was_active = self.rotate_yaw_active
        if not heading_pd_active:
            self.rotate_yaw_active = False
        elif abs(err) >= 30.0:
            self.rotate_yaw_active = True
        elif abs(err) <= 3.0:
            self.rotate_yaw_active = False
        if self.rotate_yaw_active and not rotate_was_active and p is not None:
            self.rotate_hold_xy = (
                (self.tgt[0], self.tgt[1]) if self.tgt is not None
                else (p[0], p[1]))
            self.get_logger().info(
                '自转定点：锁住进入时位置 (%.1f, %.1f)cm' % self.rotate_hold_xy)
        elif not self.rotate_yaw_active:
            self.rotate_hold_xy = None
        if self.rotate_yaw_active:
            yaw_mode = 'rotate'
            heading_ctl = self.rotate_yaw
        else:
            yaw_mode = 'leg' if self.no_brake else 'scan'
            heading_ctl = self.leg_yaw if self.no_brake else self.scan_yaw
        if heading_pd_active:
            if self.yaw_mode_prev != yaw_mode:
                self.damp.reset()
                self.get_logger().info(
                    '航向PD[%s]：kp=%.2f kd=%.2f max=%.1f°/s'
                    % (yaw_mode, heading_ctl.kp, heading_ctl.kd,
                       heading_ctl.vmax))
            w_out = heading_ctl.update(err, gz_dps)
        else:
            if self.yaw_mode_prev is not None:
                self.damp.reset()
                self.get_logger().info('航向PD退出，恢复BVA航向阻尼')
            if self.damp_on and gz_dps is not None and psi_deg is not None:
                dq = self.damp.update(gz_dps, dt, err)
                w_out = max(-30.0, min(30.0, w + dq))  # 飞控侧上限 30°/s
        self.yaw_mode_prev = yaw_mode if heading_pd_active else None
        if (self.rotate_yaw_active and p is not None
                and self.rotate_hold_xy is not None):
            hx, hy = bounded_position_hold(
                self.rotate_hold_xy[0] - p[0],
                self.rotate_hold_xy[1] - p[1],
                self.rotate_xy_kp, self.rotate_xy_max, self.rotate_xy_dead)
            vx, vy = self.smooth_xy(hx, hy)
            c, s = math.cos(psi), math.sin(psi)
            bx = c * vx + s * vy
            by = -s * vx + c * vy
        if self.zchange_hold_xy is not None:
            z_tgt = self.tgt[2] if self.tgt is not None else None
            if (p is None or self.height is None or z_tgt is None
                    or abs(self.height - z_tgt) <= self.zchange_release_cm):
                self.get_logger().warn(
                    '换高竖直锁释放：h=%s target=%s'
                    % ('nan' if self.height is None else '%.1f' % self.height,
                       'nan' if z_tgt is None else '%.1f' % z_tgt))
                self.zchange_hold_xy = None
            elif (self.active == 2 and self.height >= self.air_min
                  and psi is not None):
                hx, hy = bounded_position_hold(
                    self.zchange_hold_xy[0] - p[0],
                    self.zchange_hold_xy[1] - p[1],
                    self.zchange_xy_kp, self.zchange_xy_max,
                    self.zchange_xy_dead)
                vx, vy = self.smooth_xy(hx, hy)
                c, s = math.cos(psi), math.sin(psi)
                bx = c * vx + s * vy
                by = -s * vx + c * vy
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
        now_mono = time.monotonic()
        target_z = None if self.tgt is None else self.tgt[2]
        height_age = ((now_mono - self.height_t)
                      if self.height_t > 0.0 else float('inf'))
        fc_age = ((now_mono - self.fc_status_t)
                  if self.fc_status_t > 0.0 else float('inf'))
        lz_requested = bool(self.laser_z_engaged and self.laser_z_enabled)
        lz = self.lz_guard.evaluate(
            enabled=lz_requested,
            active=self.active,
            fc_status=self.fc_status,
            fc_age=fc_age,
            height=self.height,
            height_age=height_age,
            target_z=target_z)
        if lz['new_fault'] is not None:
            self._set_laser_z_fault(lz['new_fault'])
        if self.lz_csv is not None:
            try:
                z_err = (target_z - self.height
                         if target_z is not None and self.height is not None
                         else float('nan'))
                self.lz_csv.write(
                    '%.3f,%d,%d,%s,%s,%s,%.3f,%s,%.3f,%.3f,%d,%s\n'
                    % (time.time(), int(self.laser_z_engaged), self.active,
                       'nan' if self.fc_status is None else self.fc_status[0],
                       'nan' if self.fc_status is None else self.fc_status[1],
                       'nan' if self.height is None else '%.1f' % self.height,
                       height_age, 'nan' if target_z is None else '%.1f' % target_z,
                       z_err, float(lz['vz']), int(lz['freeze_xy']),
                       '' if lz['fault'] is None else '1'))
                self.lz_csv.flush()
            except Exception:  # noqa: BLE001
                pass
        if lz_requested:
            if lz['fault'] is not None:
                out.data[0] = 0.0
                out.data[1] = 0.0
                out.data[2] = 0.0
                out.data[3] = 0.0
            else:
                out.data[2] = float(lz['vz'])
                if lz['freeze_xy']:
                    out.data[0] = 0.0
                    out.data[1] = 0.0
                    out.data[3] = 0.0
        # ★ 2026-10-15：**离地门** —— 高度低于 air_min（默认 40cm）时，
        #   xy / z / 偏航**全部输出 0**（速度流照常发，保鲜不破）。
        #   这条是"不再侧翻"的关键：离地前不发水平速度，飞控就不需要倾斜机身。
        if (self.height is not None and self.height < self.air_min
                and len(out.data) >= 4):
            out.data[0] = 0.0
            out.data[1] = 0.0
            out.data[3] = 0.0
            # Program-mode laser-Z may lift off from the ground. Keep only
            # positive Z below air_min; never permit descent or XY motion.
            if not (self.laser_z_engaged and self.active == 2
                    and not self.laser_z_fault and out.data[2] > 0.0):
                out.data[2] = 0.0
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
            vmax = _envf('BVA_VYAW_MAX', 30.0)
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
            if getattr(self, 'active', 0) != 2:
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
