#!/usr/bin/env python3
"""任务节点：10/14 双高度环复现 -> 跑遍历航线 -> 自动降落。

本候选只用于复现 2026-10-14 16:34 的垂直振荡：
  CH5=2 定点模式；0x66[1,142] 由核心原生起飞；
  到达 142cm 后启用 C++ Z PID，且保留核心自身融合高度环。
  香橙派激光 Z 外环必须保持关闭。

订阅 /gs/start (Int16)   0=遍历盘点(已实现)  1=定点盘点(待接)
执行遍历:
  1. 检查飞控处于 mode2 且已由遥控解锁
  2. 归零（reset_origin.sh，把当前位姿当原点）
  3. 读当前机头朝向 ψ0，按方案基线几何**实时生成**货架航线
     （机体位置 = 相机位置 − 相机前伸 23.5cm；机体高度 = 相机高度 + 6.5cm）
  4. 核心原生 OneKey_Takeoff 到上排高度，再交给 C++ 激光高度 PID
  5. 监视 /route_reached_index，全部到点后发 0x66[2] 降落，落地后 0x66[4] 锁定
识别事件: 订阅 /qr/code + /route_current_index -> 发 /gs/event [num, face, pos]
"""
import math
import os
import re
import shutil
import subprocess
import threading
import time
import json
import fcntl

import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import (Bool, Empty, Float32MultiArray, Int16, Int16MultiArray,
                          String, UInt8,
                          UInt8MultiArray)

# 2026-10-24：双环不抢高度已验证。核心原生起飞实测会停在约123–129cm，
# 因此改为最低高度放行；C++ Z目标仍为142cm，交棒后继续补到目标。
TAKEOFF_RELEASE_MIN_CM = 100.0
TAKEOFF_RELEASE_MAX_CM = 180.0
TAKEOFF_HEIGHT_HOLD_S = 0.3
TAKEOFF_HEIGHT_MAX_AGE_S = 1.0
# 2026-10-24 程控候选：0x66 只负责触发离地；起飞后由香橙派在 mode 3
# 用激光 /height 直接控制 Z，最终目标和航线高度统一为 142cm。
CORE_TAKEOFF_TEST_ROUTE_CM = 142.0
CORE_TAKEOFF_TEST_TARGET_CM = 142.0
LASER_Z_TAKEOFF_FLOOR_CM = 40.0
LASER_Z_ACQUIRE_TOL_CM = 4.0
LASER_Z_ACQUIRE_HOLD_S = 0.5
LASER_Z_ACQUIRE_TIMEOUT_S = 20.0
LASER_Z_HEIGHT_MAX_AGE_S = 0.25
PROGRAM_START_SUB = 5
PROGRAM_ARM_TIMEOUT_S = 5.0
PROGRAM_START_MAX_GROUND_H_CM = 15
POSHOLD_READY_TIMEOUT_S = 6.0


class TakeoffReleaseGate:
    """Release when laser height is inside a safe floor window."""

    def __init__(self, min_cm, max_cm, hold_s):
        self.min_cm = min_cm
        self.max_cm = max_cm
        self.hold_s = hold_s
        self.since = None
        self.last_sample_at = None

    def update(self, sample, now):
        if sample is None:
            self.since = None
            return False
        height, sampled_at = sample
        if (not math.isfinite(height) or not math.isfinite(sampled_at)
                or not 0.0 <= now - sampled_at <= TAKEOFF_HEIGHT_MAX_AGE_S
                or not self.min_cm <= height <= self.max_cm):
            self.since = None
            return False
        if (self.last_sample_at is not None
                and sampled_at - self.last_sample_at > TAKEOFF_HEIGHT_MAX_AGE_S):
            self.since = None
        self.last_sample_at = sampled_at
        if self.since is None:
            self.since = sampled_at
        # Re-reading one cached sample cannot satisfy the continuous hold.
        return sampled_at - self.since >= self.hold_s


# ---- 方案基线几何（cm，场地下方左角为原点、x 向右、y 向上）----
# ★ 2026-10-07 现场重新量场地：420×420（7×7 块 60cm 砖），原点=场地左下角。
#   起飞点 (60,60) = 左下角那格砖的中心；板面 x=180；结束点 (400,400)。
TAKEOFF_FIELD = (60.0, 60.0)
LANDING_FIELD = (400.0, 400.0)
FIELD_W, FIELD_H = 420.0, 420.0     # 7×7 块 60cm 砖
PANEL_X = 180.0
# 板子含框仍 210cm（100cm 板面 + 框）。现场口径："从 A 面看的右侧到 x 轴 90cm、
# 到 y 轴 180cm" → 近端（A 面看是右侧）y=90，板面 x=180，故板子占 y=90~300。
# 三列中心仍按"离板两端各 55cm、间距 50cm"排 → 145 / 195 / 245。
PANEL_Y = (90.0, 300.0)
COL_Y = (145.0, 195.0, 245.0)       # 三列中心
ROW_CAM_Z = (150.0, 111.5)          # 上排/下排：**相机**要对准的高度
B_LOWER_SCAN_DOWN_CM = 5.0          # B下排码比A下排低约5cm，仅下移B下排相机目标
CAM_FWD, CAM_DOWN = 23.5, 6.5       # 相机在机体前 23.5cm、低 6.5cm
# 2026-10-06：30 → 40。实测连续飞行时飞机会往板面方向漂（起飞后 24cm 级定位误差
# + 环路滞后），30cm 余量不够用（操作手因"快撞板"接管）。加到 40cm 留裕度。
OBS_DIST = 40.0                     # 相机到板面
Z_TRIM = -15.0                      # 高度修正 cm（2026-09-30 实测：飞机比目标高约 10cm）
DETOUR_Y = 355.0                    # 绕板端部的横向通道（板远端 y=300，留 55cm）
# 2026-10-07：两面之间的过渡走**板近端**（现场顺序 A3..A6 → 过渡 → B4..B1）。
# 换面通道的 y。约束两边都有：
#   · 板子含框近端 y=90 → 往上是板子
#   · 场地边/墙 y=0      → 往下是墙
# 实测演进：45（离板 45cm，现场"差点撞上板子"）→ 10（离墙只有 10cm，现场"离墙过近"）。
# 可用带宽只有 90cm。现场在 50 的基础上又反馈"离板子还是近，再加 7~8cm"，
# 所以取 42：离板子 48cm、离墙 42cm。
TRANSIT_Y = 42.0
# 换面"原地掉头"之后的稳定停留（秒）：现场要求先停一下确认转向/数据再进 B 面
STABILIZE_HOLD_S = 0.5

# 《D题货架盘点-香橙派方案基线》§1.1 离线回归样本：位置 -> 货号。
# **现场货号是随机的**，这张表只做"参考对照"（写进航线注释、记进任务 CSV），
# 绝不参与识别逻辑——货号一律现场解码。
REFERENCE = {
    # ★ 2026-10-07 现场实测（用户口述）：A 面 = 起飞点这一侧
    #   飞行实测佐证：飞向 A3 时解到货号 9 → A3=9 与该表一致
    "A": {1: 8, 2: 11, 3: 9, 4: 7, 5: 5, 6: 6},
    "B": {1: 13, 2: 14, 3: 17, 4: 15, 5: 12, 6: 16},
}
TRACE_DIR = os.path.expanduser('~/traces')


# ---- 两面的名字与列号（现场口径 2026-10-07）----
# · **A 面 = 起飞点这一侧（场地 x 小的一侧，图上左边）**
# · B 面 = 另一侧（场地 x 大的一侧，图上右边）
# · 列号：站在该面**外侧**正对板子看，上排从左到右 1,2,3，下排 4,5,6
#   - A 面（站在 x 小侧朝 +x 看）：左手边 = y 大 → A1 在 y=255
#   - B 面（站在 x 大侧朝 −x 看）：左手边 = y 小 → B1 在 y=155
# 所以 COL_Y 下标 c(1..3) 换算成"列号"时，A 面要翻过来。
NEAR_X = PANEL_X - OBS_DIST - CAM_FWD      # 86.5  A 面一侧（起飞点侧）
FAR_X = PANEL_X + OBS_DIST + CAM_FWD       # 213.5 B 面一侧


def label_col(face, c):
    """COL_Y 下标 c(1..3) → 该面的货位列号(1..3)。"""
    return c if face == 'B' else 4 - c


def body_for_camera(face, code_y, cam_z):
    """相机要到位 -> 反推机体航点（场地系 cm）。"""
    if face == "A":                     # A 面在起飞点这一侧
        body_x = PANEL_X - OBS_DIST - CAM_FWD
    else:
        body_x = PANEL_X + OBS_DIST + CAM_FWD
    return (body_x, code_y, cam_z + CAM_DOWN + Z_TRIM)


def row_camera_z(face, row):
    """该面该排应让相机对准的高度；只补偿B下排的实测竖直偏差。"""
    z = ROW_CAM_Z[row]
    if face == 'B' and row == 1:
        z -= B_LOWER_SCAN_DOWN_CM
    return z


def reference_of(label):
    """航点标签(如 'B3') -> 参考货号；'takeoff'/'land' 或表里没有返回 '-'。"""
    if not label or len(label) != 2 or label[0] not in REFERENCE:
        return '-'
    return REFERENCE[label[0]].get(int(label[1]), '-')


# ---- 连续扫描（边走边识别，不停顿）----
PASS_SPACING = 10.0     # 观察线上打点间隔 cm：够密 → 位置环不停顿，匀速滑过去
PASS_MARGIN = 15.0      # 扫描线两端各留的余量 cm（让码提前进画面、走完再出画面）

# ---- 2026-10-14：快滑 + 到点打点（用户要求"边飞边扫、利索干净"）----
# 做法：格与格之间**不停**（全局 hold_s=0），只在**列中心短停 CELL_HOLD_S**；
#   识别在滑行/接近过程中完成（相机连续出帧），停下只为"码居中 + 打点"。
#   激光不在"采纳那一刻"打，而是**等飞机停到该格中心**再打（否则边走边打会拖成一条线）。
# ★ 2026-10-14：保持**今天这套**（现场："先试试停顿吧，把停顿飞的差不多再尝试
#   一直直线" —— 指的是今天这种"飞一段、在格点收一下"的自然停走，不是 10-06 的
#   逐格 hold 1.2s）。True = 格间不停、只有列中心短停 CELL_HOLD_S。
#   现场观察：因为前馈到 18cm 会交回位置环收口，实际表现就是"每个格点收一下"。
SCAN_SLIDE = True       # False = 回到 10-06 的逐格停 1.2s（备用/回退）
CELL_HOLD_S = 0.0       # 连续扫描：码点到达后不驻留，未打中由 pending 继续等
LASER_PULSE_S = 0.15    # 顺手打点：短脉冲，边走边打也不会拖成长线
ROW_SPLIT_H = 122.0     # cm：/height 低于它算下排（上排 142 / 下排 103）
# ★ 2026-10-14：起飞后先在目标高度**稳 4 秒**再开始扫描。
#   现场反馈"起飞是好几段的、悬停不稳、转到第一个码太急"——一键起飞自己分几段爬，
#   爬完马上就被位置环拖着走，机体还没稳。这里在 takeoff 航点上加单点 hold。
# ★ 2026-10-14 深夜：4.0 → 1.0。起飞现在有了独立的"等稳定 3s"（见 run_traverse），
#   而 waypoint_nav 还要在这个航点上再等 hold 秒才往下走。两段叠加就是"悬停好久
#   才开始飞"（实测 16:34 那轮：从 START 到开始横移共 19.4 秒，其中这 4 秒纯属重复）。
# ★ 2026-10-14 深夜：1.0 → 0.3。起飞点这个航点只是一个"进航线"的标记，
#   悬停稳不稳已经由下面那段"等稳定"独立确认过了，不需要在这里再停 1 秒。
TAKEOFF_HOLD_S = 0.3

# ★ 2026-10-18：起飞前位姿稳定性闸门（连续 N 帧位移 < 容差才发 0x66）
# ★ 2026-10-19（按现场要求）：**起飞段完全复制悬停链**。
#   10/16 验证过的那套 pos_hold（kp=0.35 ki=0.05 trim=-1.8 vmax=12
#   + 偏航阻尼）重新启用；它只负责"解锁 → 爬到航线高度 → 稳住"这一段，
#   交棒给 waypoint_nav 前会被停掉，因此不会和 body_vel_adapter 抢
#   /target_velocity（起飞段 C++ 环没有 /target_position，本来就不发）。
USE_POS_HOLD = True

POSE_STABLE_N = 5
POSE_STABLE_TOL_CM = 2.0
POSE_STABLE_TIMEOUT_S = 8.0

# ★ 2026-10-14 深夜（现场要求）：**扫完之后飞往降落点那段不要"减速收敛"**。
#   现场描述："扫完到降落点的逻辑就不用减速收敛了，然后爬高也不用了，多高就多高。"
#   做法：给"绕行/land"这些**过路点**单独放宽容差（waypoint_nav 支持单点 tol_cm）——
#   还差 30cm 就算到达、立刻换下一个目标，前馈不用把速度收到零收口，
#   飞机保持巡航速度直接穿过去；高度也不再改（land 航点沿用上一个点的高度）。
TRANSIT_TOL_CM = 30.0
# ★ 2026-10-14 深夜（现场踩坑，"直接往右飞、差点撞墙"）：**航向基准检查**。
#   场地坐标是锚在 SLAM 地图系上的，而地图朝向会随 cartographer 重启重新对齐
#   （实测同一机头朝向，psi0 从 −132° 变成 0.1°）→ 整条航线在房间里转了 132°。
#   所以起飞前先比一下：偏离参考值超过容差就中止，不发 0x66。
PSI0_REF_DEG = 0.0        # ★ 2026-10-16：改成 0°
PSI0_TOL_DEG = 25.0
# ★ 2026-10-15（现场方案"第二条"）：**场地坐标改为相对起飞时机头方向**。
#   依据：`simple_map/waypoints_site1.yaml` 里写明的约定 ——
#   "原点=场地左下角、起飞点 (0,0)、**机头 yaw = 0**"。
#   也就是说场地坐标本来就是以"起飞时机头方向"为 +x 定义的，
#   那么位置换算里就不该乘 `psi`（地图系的机头朝向）。
#   现在地图帧每次开机随机（psi 实测出现过 -163/-0.3/37/56/160），
#   乘进去等于给整条航线随机转一个角度 —— 这就是"方向有时对有时不对"的根源。
#   注意：**航向目标（yaw）仍用地图系的值**（C++ 环比较的是地图系航向）。
#   True = 位置换算用 0（相对起飞点）；False = 回到用 psi（旧行为）。
# ★ 2026-10-16（**回退**）：证据表明这条前提是错的 ——
#   10-14 16:34（psi0=−91.7）和 10-15 20:52（psi0 正常区间）用**旧逻辑**（位置乘 psi）
#   都跑对了；而 `FIELD_ROT_ZERO=True`（位置不乘 psi）**一次都没成功起飞验证过**。
#   如果场地坐标真是"相对起飞点机头"的，旧逻辑应该**每一轮都算错** ✗ —— 可它是对的 ✅。
#   → 真正的乱源是"**每次开机地图帧朝向随机**"，不是换算公式。
#   所以：回到旧逻辑，并**保留 psi0 守卫**（不在惯常区间就拒绝起飞，绝不朝错方向飞）。
#   根治办法仍是"固定地图帧"（加载标定地图 / 重锚），见交接文档 §2。
# ★ 2026-10-16（**改回 True**，实验验证后）：实测"转 90° + 重启链路"后
#   `psi0 ≈ 0.00°` → 说明 **cartographer 的 map 帧就锚定在"启动那一刻飞机的位姿"**上
#   （`map` 的 +x = 启动时机头方向）。所以只要**先摆位、再上电**，
#   基准就等于摆放朝向 → `psi0 ≈ 0` → 换算应当用"相对起飞点"（即 psi_pos = 0）。
#   这也正是 `simple_map/waypoints_site1.yaml` 写的约定："起飞点 yaw = 0"。
#   配合 `PSI0_REF_DEG = 0.0`（守卫放行 ≤25°）→ 每次开机摆好位就能飞，方向恒定。
#   历史注：之前以为它错，是因为**从没成功起飞验证过**（那两次都被飞控切电中止）；
#   而 −91.7/−132 那几轮"好用"是 SLAM 启动时飞机不在起飞位、旧逻辑恰好补偿上了。
# ★ 2026-10-16（下午，实测后回退到 False）：位置换算**必须**乘 ψ0。
#   依据是数学而不是经验：`to_flight()` 把场地向量旋转 ψ0 才得到地图向量
#   ⇒ 场地 +x = 机头 = 地图角 ψ0（与 simple_map/waypoints_site1.yaml 的约定一致）。
#   乘 0 只在 ψ0≈0 时等价，而 ψ0 并不保证是 0 —— amphi.lua 里
#   `use_odometry=false` + `use_imu_data=false` + 启动时 `load_state`（加载旧图）
#   ⇒ 地图帧朝向由扫描匹配决定，会锁到 90°/180° 的错误对齐。
#   实测反证：摆放完全没动，ψ0 在 −15.4 / −155.2 / −93.9 之间跳；
#   而 10/14 16:34（ψ0=−91.7）用 False 是飞出正确结果的。
#   ⇒ 回到 False，方向就与"起飞那一刻机头"恒定对齐，ψ0 取什么值都无所谓。
FIELD_ROT_ZERO = False
# ψ0 守卫：这个约定下 ψ0 本来就任意（乘进去就自动补偿了），拦它只会误拦合法飞行
# （10/15 实测踩过：psi0=71.4° → 被挡）。保留记录、不再拦停。
PSI0_GUARD_ON = False
# ★ 2026-10-14 深夜：**自转/过渡点**的航向容差（全局是 20°，太松）。
#   现场目标："前往过渡点**精准自转**然后重复" —— 自转不准，下一面的扫描起点就歪，
#   所以这些点单独收紧到 3°（只影响它们，扫描段的 20° 不动）。
ROTATE_YTOL_DEG = 3.0
# ★ 2026-10-14 深夜：**撤回 TAKEOFF_MIN_H=156**（那条是今晚自己挖的坑）。
#   原意是"给飞控一个 ≥150 的起飞高度以规避自降"，实际后果是：
#     · 飞控一口气爬到 **156** 并自己稳在那里（它自己的高度环目标就是 156）；
#     · 而航线里 takeoff 航点 z=**142**，waypoint_nav 的到达判据是 `dz ≤ z_tol(8cm)`
#       → dz=14cm **永远不满足** → 它一直卡在航点 0（状态文本 "flying takeoff:
#       dx=0 dy=2 dz=139 dyaw=-0"），水平指令≈0，飞机原地悬停一步不走。
#     · 我们的 z 环也降不下去：kp_z 今晚被调成 0.3（0.3×14≈4cm/s），
#       打不过飞控自己那条"稳在 156"的强高度环。
#   实测（15:58 那轮）：h 在 156 稳了 15 秒，wpn.log 里没有"到达航点 0"，
#   /route_reached_index 从没发布过。
#   → 起飞高度**必须等于航线高度**（原来就是这样，任务以前能跑）。
#   自降是另一个问题（只在"已解锁"时出现，unlock 后 1~6s 内就可能发生，
#   和高度无关；见 2026-10-14 的数据），不要再用抬高起飞高度来规避。
TAKEOFF_MIN_H = 0.0
# 扫描采纳速度：只要不是过快就允许识别。打点不等待停稳，由视觉瞄准窗
# 决定时机；短脉冲把行进中的点控制在较小长度。
# 2026-10-14 实测：末段接近是 8~12cm/s 的慢爬，只用 8 会把码整段挡掉（A1/A2 漏读）。
SCAN_ACCEPT_SPEED = 30.0
# ★ 2026-10-19：激光器在相机左侧 1.5 cm → 要打准，"码"必须出现在画面中心偏左 1.5 cm。
#   aim_dx 是带符号的归一化偏移（正=码在画面右侧），目标是 -0.06（约 1.5cm/24cm 半幅）。
AIM_TARGET_DX = -0.06
AIM_TOL = 0.35          # 约 7 cm；仍要求码在可打中的中心窗口
MARK_AIM_TOL = 0.12     # 过线前后两帧都必须在激光中线附近
MARK_WINDOW_TOL = 0.22  # 未跨线时的窄瞄准窗兜底，避免上排整排漏打
MARK_DY_TOL = AIM_TOL   # 上下位置沿用原门，只收紧横向“过边”判断
MARK_CROSS_GAP_S = 1.20
MARK_CROSS_MIN_STEP = 0.02
MARK_CROSS_EXTRA_DX = 0.02  # 过线后沿运动方向再前进约 0.5cm 才打
MARK_RETRY_WINDOW_S = 0.80  # 过线后等待相邻 QR 帧，允许姿态状态短暂不同步
AIM_FRESH_S = 0.4
LASER_PENDING_TTL_S = 8.0  # 本轮 A1 过线距采纳约 6.1s，5s 会过早丢弃
MARK_STATE_MAX_AGE_S = 0.25
MARK_SPEED_MAX = 20.0

LASER_CENTER_TOL = 0.35    # 打点时的居中要求（比采纳的 center_tol 严）


def face_pass(face, row, cols, reverse=False, spacing=PASS_SPACING, margin=PASS_MARGIN):
    """把"某面某排的若干列"变成一条**连续扫描线**。

    reverse=True → 从 y 大的那一端往 y 小的飞（现场说的"从右往左"）。
    ★ 2026-10-07 修：原来这里写死 sorted(cols)，调用方传进来的顺序被无视 →
      mission_node 里那句"蛇形折返"实际是空转，两排永远都是 y 小 → y 大。

    ★ 2026-10-14：SCAN_SLIDE=True 时，扫描线**两端各加一个 hold=0 的进出点**
      （让码提前进画面/走完再出画面），列中心保持一格一个航点 —— 但它们的
      hold 由 run_traverse 写成 CELL_HOLD_S（0.5s），格间不再停。

    ★ 2026-10-06：**每格只放一个航点（列中心）**，配合 hold_s=1.2s 让飞机
      在每格中心停一下再识别（格与格之间照旧飞）。

    为什么 10-06 曾改回"逐格停"（原设计是每 10cm 一个点、hold_s=0 连续滑行）：
      · 连续滑行时码永远不在画面中心（实测归一化偏移 0.34~0.53）→ 激光沿机头
        打，会偏十几厘米，**打不到码上**；
      · 画面里会同时出现相邻格的码 → **串码**（B3 读到 8/11/5/7）。
    停一下正好解决这两个问题：码居中 → 能打准；一帧一个码 → 不串。
    提速放在"格与格之间"（调 max_linear_velocity），不在识别瞬间。

    2026-10-14 的折中：**滑行的间距不变（还是按列来）、但格间不停**，
    仍靠 center_tol 与"同码只绑一次"防串码；打点推迟到"停稳"那一刻。
    """
    z = row_camera_z(face, row)
    cl = sorted(cols, reverse=reverse)
    out = []
    if SCAN_SLIDE and cl:
        y_in = (COL_Y[cl[0] - 1] + margin) if reverse else (COL_Y[cl[0] - 1] - margin)
        bx, by, bz = body_for_camera(face, y_in, z)
        out.append(('%s进-%d' % (face, row), bx, by, bz))
    for c in cl:
        y = COL_Y[c - 1]                     # 列中心（码就在这里）
        bx, by, bz = body_for_camera(face, y, z)
        out.append(('%s%d' % (face, label_col(face, c) + row * 3), bx, by, bz))
    if SCAN_SLIDE and cl:
        y_out = (COL_Y[cl[-1] - 1] - margin) if reverse else (COL_Y[cl[-1] - 1] + margin)
        bx, by, bz = body_for_camera(face, y_out, z)
        out.append(('%s出-%d' % (face, row), bx, by, bz))
    return out


def aim_crossed(prev_dx, dx, target=AIM_TARGET_DX,
                band=MARK_AIM_TOL, min_step=MARK_CROSS_MIN_STEP):
    """连续两帧跨过激光中线时返回 True，信号跳变不算过线。"""
    try:
        a = float(prev_dx) - target
        b = float(dx) - target
    except (TypeError, ValueError):
        return False
    if not (math.isfinite(a) and math.isfinite(b)):
        return False
    if abs(a) > band or abs(b) > band:
        return False
    if abs(b - a) < min_step:
        return False
    return (a > 0.0 and b <= 0.0) or (a < 0.0 and b >= 0.0)


def mark_after_cross_ok(
        age_s, dx, dy, direction, window_s, band, dy_tol, extra_dx):
    """过线候选已沿运动方向多走一小段，且仍在中心窗口内。"""
    try:
        age_s = float(age_s)
        dx = float(dx)
        dy = float(dy)
        direction = int(direction)
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(v) for v in (age_s, dx, dy)):
        return False
    advance = (dx - AIM_TARGET_DX) * direction
    return (0.0 <= age_s <= window_s
            and extra_dx <= advance <= band
            and abs(dx - AIM_TARGET_DX) <= band
            and abs(dy) <= dy_tol)


# ---------- 航线结构（2026-10-07 现场口述定稿）----------
# 现场指定的顺序：A3 A2 A1 → A4 A5 A6 → 过渡 → B4 B5 B6 → B3 B2 B1 → 结束点
#   · A 面（起飞点侧）：上排 A3→A2→A1（y155→y255）→ 换高度 → 下排 A4→A5→A6（y255→y155）
#   · 过渡：**板近端**两个过渡点（先沿 A 面侧走到通道、再横穿到 B 面），第二点掉头 180°
#   · B 面（另一侧）：下排 B4→B5→B6（y155→y255）→ 换高度 → 上排 B3→B2→B1（y255→y155）
#   · 降落：直线飞往结束点
#
# 为什么必须插过渡点：A 面收在 (86.5, 155)，B 面起于 (213.5, 155)，
# 两点直连会**穿过板面 x=150 @ y=155**（板子含框 y=100~310 范围内）→ 撞板。
FACE_ORDER = ('A', 'B')                          # 先飞起飞点这一侧
# (排, 该排是否从 y 大往 y 小飞)
FACE_ROW_ORDER = {'A': ((0, False), (1, True)),
                  'B': ((1, False), (0, True))}
TRANSIT_Z = ROW_CAM_Z[0] + CAM_DOWN + Z_TRIM


def plan_points(subset=''):
    """生成整条航线（含两面之间的绕行过渡点）。

    返回 [(label, 场地x, 场地y, 场地z), ...]；label 以 '过渡-' 开头的不算货位。
    """
    want = {s.strip().upper() for s in str(subset).split(',') if s.strip()}

    def cols_of(face, row):
        out = []
        for col in (1, 2, 3):
            label = '%s%d' % (face, label_col(face, col) + row * 3)
            if want and label.upper() not in want:
                continue
            out.append(col)
        return out

    has_b = any(cols_of('B', row) for row in (0, 1))
    # 过渡点高度 = **下面要扫的那一面的第一排高度**，这样换面时高度不变
    # （2026-10-07 现场：A6 在下排 103，过渡点原来写死顶层 142 → 白爬 39cm，
    #   到 B4 又降回 103。改成跟下一排一致后，A6→过渡→B4 全程 103 不变。）
    b_first_row = next((r for r, _ in FACE_ROW_ORDER['B'] if cols_of('B', r)), None)
    b_transit_z = (row_camera_z('B', b_first_row) + CAM_DOWN + Z_TRIM
                   if b_first_row is not None else TRANSIT_Z)
    pts = []
    for face in FACE_ORDER:
        if face == 'B' and pts and has_b:
            pts.append(('过渡-离开板', NEAR_X, TRANSIT_Y, b_transit_z))
            pts.append(('过渡-到达B面侧', FAR_X, TRANSIT_Y, b_transit_z))
            # ★ 2026-10-07 现场要求："到第二个过渡点**原地**旋转，别边飞边转"。
            #   所以第二个过渡点保持旧航向(psi0)，紧跟**同位置、只改航向**的航点。
            #   ★★ 再改：一次 180° 太狠 —— 实测 scan matcher 在大角度自转后会跳变，
            #   出现"它以为转正了、机体其实是斜的"。改成 **4 步 ×45°**，每步转到位
            #   由 waypoint_nav 的 hold 机制停一下，给小角度匹配留时间。
            pts.append(('过渡-掉头', FAR_X, TRANSIT_Y, b_transit_z))
            # ★ 再跟一个"过渡-稳定"：同位置同航向，但**单独停留 3 秒**
            #   （现场要求：转完先停一下，确认转向正确、位姿数据稳定，再进 B 面）。
            pts.append(('过渡-稳定', FAR_X, TRANSIT_Y, b_transit_z))
        for row, right_to_left in FACE_ROW_ORDER[face]:
            cols = cols_of(face, row)
            if cols:
                pts += face_pass(face, row, cols, reverse=right_to_left)
    # 只飞 B 面时起飞点在 A 面侧，直连过去会穿板 → 先绕板子近端过去
    if pts and pts[0][1] > PANEL_X:
        pts = [('引导-绕板近端', NEAR_X, TRANSIT_Y, b_transit_z),
               ('引导-进B面侧', FAR_X, TRANSIT_Y, b_transit_z),
               ('引导-掉头', FAR_X, TRANSIT_Y, b_transit_z),
               ('引导-稳定', FAR_X, TRANSIT_Y, b_transit_z)] + pts
    return pts


def face_cells(face, start_from_left=True):
    """某面 6 个机体航点（场地系 cm）: 上排 1,2,3 再下排 4,5,6。

    列号用 label_col() 反查到的 COL_Y 下标，保证和 face_pass()/_label_by_pose()
    同一套口径。start_from_left=False 时每排反过来。
    """
    out = []
    for row in (0, 1):
        labs = (1, 2, 3) if start_from_left else (3, 2, 1)
        for lab in labs:
            c = lab if face == 'B' else 4 - lab
            out.append(('%s%d' % (face, lab + row * 3),
                        ) + body_for_camera(face, COL_Y[c - 1], row_camera_z(face, row)))
    return out


def to_flight(fx, fy, psi_deg):
    """场地系(cm) -> 地图/SLAM 系(cm)：以起飞点为原点，按起飞时的机头朝向 ψ0 旋转。

    ★ 2026-09-30 修正符号：原来写成 (dx·cos + dy·sin, −dx·sin + dy·cos)，
      那是**反向旋转**，等于把目标点飞到镜像的另一侧。
      ψ0 小的时候看不出来（9/27 那次正方形 psi0=12°，转错 24° 还是正方形）；
      ψ0=−88.5° 时就暴露了：目标被送到反方向（实测飞机往板子的反方向飞）。
      正确关系：场地 +x = 机头 = 地图角 ψ0；场地 +y = 机头左侧 = ψ0+90°
      ⇒ 地图向量 = dx·(cosψ0, sinψ0) + dy·(−sinψ0, cosψ0)
                  = (dx·cosψ0 − dy·sinψ0, dx·sinψ0 + dy·cosψ0)
    """
    dx, dy = fx - TAKEOFF_FIELD[0], fy - TAKEOFF_FIELD[1]
    a = math.radians(psi_deg)
    return (round(dx * math.cos(a) - dy * math.sin(a), 1),
            round(dx * math.sin(a) + dy * math.cos(a), 1))


def to_field(mx, my, psi_deg):
    """地图/SLAM 系(cm) -> 场地系(cm)：to_flight() 的逆变换。

    ★ 2026-10-14：**必须的**。`_pose()` 拿的是地图系坐标，而场地几何
    （PANEL_X / COL_Y）是场地系 —— 两者差一个 ψ0 旋转。psi0≈0 时看不出来
    （历史成绩全是小 ψ0），psi0=-101° 时就暴露：A 面被判成 B 面、货号全绑错格。
    """
    a = math.radians(psi_deg)
    return (mx * math.cos(a) + my * math.sin(a) + TAKEOFF_FIELD[0],
            -mx * math.sin(a) + my * math.cos(a) + TAKEOFF_FIELD[1])


class Mission(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.route_file = self.declare_parameter('route_file', '/tmp/route_mission.yaml').value
        # subset：只飞指定货位，例 -p subset:=B1,B2,B3（空=整条 12 货位）
        # 半面/单格测试用，避免每次都飞完整航线。
        self.subset = str(self.declare_parameter('subset', '').value).strip()
        # 10/14 复现只允许 auto：CH5=2、遥控解锁、0x66[1,142] 核心原生起飞。
        self.takeoff_mode = str(
            self.declare_parameter('takeoff_mode', 'auto').value
        ).strip().lower()
        self.last_idx = None
        self.qr_num = None
        self.qr_center = None
        self.qr_label = None
        self.bound = {}          # {货位标签: 货号}，用于去重（现场货号互不相同）
        self.pending_laser = None   # 已采纳、等飞机停到该格中心再打点的货位
        self.pending_laser_at = None
        self.mark_ok = False
        self.mark_ok_t = 0.0
        # (货位, 货号, 上一帧 dx, 单调时钟)，只用于判断是否跨过激光中线
        self._mark_track = None
        # (货位, 货号, dx, dy, 过线方向, 过线单调时钟)
        self._mark_retry = None
        self._mark_cross_dir = 1
        self.psi0 = 0.0             # 起飞瞬间的机头朝向（场地系→地图系的旋转量）
        # 画面尺寸 + "中心区"容差（归一化，0.35 ≈ 画面中间 70% 宽/高）
        self.img_w = float(self.declare_parameter('img_w', 1280.0).value)
        self.img_h = float(self.declare_parameter('img_h', 720.0).value)
        # 2026-10-06：0.35 → 0.5。0.35（画面中间 70%）把该认的码也挡掉了
        # （实测 B1/B3 一个事件都没有）；上一场不筛又会串码。先放宽到画面中间一半。
        # ★ 2026-10-07：现场要求"激光要打在码上"。激光与相机光轴平行 ⇒ 码必须
        #   落在画面中央附近，激光点才落在码上。0.75 太松（点会偏到码外），
        #   收到 0.5：按 40cm 观察距离、HFOV/2≈30° 估，最大横向偏 ≈ 40*tan(15°)
        #   ≈ 11cm，正好是 20cm 码的半宽边界；再松就打不上了。
        self.center_tol = float(self.declare_parameter('center_tol', 0.60).value)
        self.running = False
        self._start_lock = threading.Lock()
        # ★ 2026-10-19：飞控自降/上锁时由 on_fc 置位，主流程在每个阶段检查。
        self.abort_requested = False
        self.laser_z_control_active = False
        self.laser_z_fault = False
        self._owner_fd = None
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.pub_cmd = self.create_publisher(Int16MultiArray, '/mission_cmd', 10)
        # ★ 2026-10-14 深夜：扫完最后一面后打开"过路模式"（body_vel_adapter 匀速直穿、不刹车）
        self.pub_nb = self.create_publisher(Bool, '/bva_no_brake', 10)
        self.pub_event = self.create_publisher(Int16MultiArray, '/gs/event', 10)
        # 激光打点：只有"该格货号被采纳"时才打（laser_driver 低电平点亮 GPIO40）
        self.pub_laser = self.create_publisher(Bool, '/laser/trigger', 10)
        self.pub_laser_z = self.create_publisher(Bool, '/laser_z_enable', 10)
        # 自己也能激活位置环（不再依赖控制台 HTTP —— 控制台偶发段错误会连累任务）
        self.pub_act = self.create_publisher(UInt8, '/active_controller', 10)
        # ★ 2026-10-16（晚）：C++ 位置环的目标点。没有它位置环一行速度都不发
        #   （`pid_controller.cpp` 第一行 `if (!has_target_position_) return;`）——
        #   悬停测试靠它在爬升段保持位置，任务链原来没人发，见 _set_target_position。
        self.pub_pos = self.create_publisher(Float32MultiArray, '/target_position', 10)
        # ★ 2026-10-19：**改到起飞段才挂 pos_hold**（不再开机常驻）。
        #   原因：pos_hold 直接写 /target_velocity，开机常驻会让该话题出现
        #   两个 publisher（pos_hold + body_vel_adapter），体检和"单发布者"假设都失效。
        #   19:26 的证据表明：open_loop=false 时 0x31 静默 13 秒也不会自降，
        #   所以不再需要"解锁前就挂上"。起飞段见 run_traverse 里的 _ensure_pos_hold。
        # /target_velocity 计数：起飞前确认 pos_hold 真的在发（2026-10-19 新增）
        self.tv_n = 0
        # 归零：给 dead_reckon 发一条 Empty，让它把积分位置清 0（见下方注释）
        self.pub_reset = self.create_publisher(Empty, '/dead_reckon/reset', 10)
        self.create_subscription(Int16, '/gs/start', self.on_start, 10)
        self.create_subscription(UInt8, '/route_current_index', self.on_idx, 10)
        # ★ 2026-10-07："已到达"索引。任务结束判据原来用"当前航点索引"，
        #   于是最后一格（B1）一到位、waypoint_nav 刚切到 land 航点时就判定
        #   结束并**原地降落**（实测："为什么是原地降落"）。改成等 **already
        #   reached 到 land 那一点** 才发降落，飞机才会真的飞到 (400,400) 再落。
        self.reached_idx = None
        self.create_subscription(UInt8, '/route_reached_index', self.on_reached, 10)
        # ★ 用 /qr/result（JSON，带像素中心）而不是 /qr/code，才能判断码在不在画面中心
        self.create_subscription(String, '/qr/result', self.on_qr, 10)
        self.create_subscription(UInt8MultiArray, '/fc_status', self.on_fc, 5)
        self.create_subscription(
            Bool, '/laser_z_fault', self.on_laser_z_fault, 10)
        # ★ 2026-10-07(晚)：降落关位置环改成"看高度"（原来无条件 sleep 6s）。
        self.height_sample = None       # atomic (accepted height, monotonic receipt)
        self.height = None               # /height 飞控高度(cm)，带两道剔除
        self.create_subscription(Int16, '/height', self.on_height, 5)
        self.create_subscription(Bool, '/mark_ok', self.on_mark_ok, 10)
        self.create_subscription(Float32MultiArray, '/target_velocity',
                                 lambda m: setattr(self, 'tv_n', self.tv_n + 1), 20)
        self.labels = []                 # 航点标签（与 route 顺序一致）
        self.trace_file = None           # 本次任务的 位置-货号 记录（CSV）
        self.fc = None                   # (模式, 解锁)
        self._cmd1_sample = None  # atomic (receipt sequence, command, monotonic time)
        self.get_logger().info('mission_node 就绪，等 /gs/start（0=遍历）')
        self.get_logger().info(
            '10/14历史复现：仅接受CH5=2；原生0x66[1,142]起飞，'
            'C++ Z PID 1.0/0/0.2，BVA激光Z关闭')

    # ---------- 事件 ----------
    def on_qr(self, m):
        """★ 2026-10-06：改用 /qr/result（JSON，带像素中心），只认"画面中心区"的码。

        旧实现订阅 /qr/code（只有一个货号数字），**丢掉了二维码在画面里的位置**，
        于是连续飞行时视野边缘扫到相邻货位的码也会被绑到"当前航点"上 ——
        实测 B3 读成 8/11/5/7（串码）。

        激光是沿机头方向、与相机同轴的，所以**只有落在画面中心的那一个码**才是
        激光能打到的目标。这里按归一化偏移过滤：|dx|、|dy| 都小于 center_tol 才采纳。
        """
        try:
            info = json.loads(m.data)
        except Exception:  # noqa: BLE001
            return
        code = info.get('code')
        c = info.get('center') or []
        if code is None or len(c) < 2:
            return
        nx = abs((float(c[0]) - self.img_w / 2.0) / (self.img_w / 2.0))
        self.aim_dx = (float(c[0]) - self.img_w / 2.0) / (self.img_w / 2.0)
        self.aim_dy = (float(c[1]) - self.img_h / 2.0) / (self.img_h / 2.0)
        self.aim_t = time.time()
        ny = abs((float(c[1]) - self.img_h / 2.0) / (self.img_h / 2.0))
        # ★ 2026-10-14：还要求"飞机基本停住"才采纳 —— 码居中(center_tol) + 停稳，
        #   两个条件同时满足，激光点才落在码上（边走边打会拖成线/打偏）。
        spd = self._speed()
        lbl_now = self._label_by_pose()
        # ★ 2026-10-15：**本排进入点（A进-0/A进-1）到达之前，只记不采纳、不打点**。
        #   现场："到第一个二维码前，在途中还没开始飞就打点" —— 实测 A3 那次
        #   打点比到达 A进-0 还早 2.7 秒，也就是在过渡段就打掉了。
        #   过渡段不是扫描段，码在画面里的几何关系也不对，不该打。
        if self.reached_idx is None or self.reached_idx < 1:
            self.get_logger().info(
                'QR候选: 货号=%s 归一化偏移=(%.2f,%.2f) 速度=%.0fcm/s '
                '丢弃(还没到 A进-0，过渡段不采纳/不打点)'
                % (code, nx, ny, spd))
            return
        # ---- (1) 补打：已采纳但当时还在动的那一格，等"码居中 + 基本停住"再打 ----
        if (self.pending_laser and self.pending_laser_at is not None
                and time.time() - self.pending_laser_at > LASER_PENDING_TTL_S):
            self.get_logger().warn(
                '待打点 %s 超过%.0fs仍未进入瞄准窗，跳过'
                % (self.pending_laser, LASER_PENDING_TTL_S))
            self.pending_laser = None
            self.pending_laser_at = None
        crossed = self._on_qr_center_cross(
            lbl_now, int(code), self.aim_dx, self.aim_dy)
        if crossed and self.pending_laser == lbl_now:
            self._mark_retry = (
                lbl_now, int(code), self.aim_dx, self.aim_dy,
                self._mark_cross_dir, time.monotonic())
            self.get_logger().info(
                '过线后待额外偏移%+.2f：%s dx=%+.2f dy=%+.2f'
                % (MARK_CROSS_EXTRA_DX, lbl_now, self.aim_dx, self.aim_dy))
            self._try_mark_retry()
            return
        if (not crossed and self._mark_retry is not None
                and lbl_now == self._mark_retry[0]
                and int(code) == self._mark_retry[1]):
            label, code0, _, _, direction, crossed_at = self._mark_retry
            self._mark_retry = (
                label, code0, self.aim_dx, self.aim_dy,
                direction, crossed_at)
            if self._try_mark_retry():
                return
        if (self.pending_laser == lbl_now
                and self.bound.get(lbl_now) == int(code)
                and abs(self.aim_dx - AIM_TARGET_DX) <= MARK_WINDOW_TOL
                and abs(self.aim_dy) <= MARK_DY_TOL
                and self._mark_stable()):
            self.get_logger().info(
                '窄瞄准窗打点：%s dx=%+.2f dy=%+.2f（%.0fcm/s，货号%d）'
                % (lbl_now, self.aim_dx, self.aim_dy, spd, int(code)))
            self._mark_retry = None
            self.pending_laser = None
            self.pending_laser_at = None
            self._fire_laser(LASER_PULSE_S)
            return
        stopped = spd <= SCAN_ACCEPT_SPEED
        # 诊断：每次检测都记一行（含被丢弃的），用于用真实像素数据定阈值
        self.get_logger().info(
            'QR候选: 货号=%s center=(%.0f,%.0f) 归一化偏移=(%.2f,%.2f) 速度=%.0fcm/s %s'
            % (code, float(c[0]), float(c[1]), nx, ny,
               spd,
               '采纳' if (nx <= self.center_tol and ny <= self.center_tol and stopped)
               else ('丢弃(在动)' if (nx <= self.center_tol and ny <= self.center_tol)
                     else '丢弃(偏)')))
        if nx > self.center_tol or ny > self.center_tol:
            return                     # 不在中心区 → 不是激光要打的那个码
        if not stopped:
            return                     # 还没停稳 → 等停住那一帧再采纳（打点时机）
        self.qr_num = int(code) & 0xFF
        self.qr_center = (float(c[0]), float(c[1]))
        # ★ 2026-10-06：绑定货位改用"飞机当前实际位置"，不再用航点索引。
        #   实测：航点间距 10cm、但"到达"容差 25cm → 索引会提前约 25cm 跳，
        #   于是 B2 的码被算到 B3 头上（表现为 B3 串码 + B2 漏读，实为一件事）。
        self.qr_label = self._label_by_pose()
        self._emit_event()

    def _label_by_pose(self):
        """按飞机当前位置选货位（比航点索引准）。

        三列中心在机体坐标里是 COL_Y[i]-TAKEOFF_Y；面由 x 判定（A 面在
        x < 板面-起飞点 的一侧，即起飞点这侧；B 面在另一侧）；列号用
        label_col() 换算（A 面左右翻）；上下排沿用当前航点标签里的排
        （高度不同，从 y 分不出来，而索引虽然会提前跳，排内是不会错的）。
        """
        p = self._pose()
        if not p:
            return None
        # ★ 2026-10-14：先把手里的**地图系**位姿转回**场地系**再判面/列
        #   （原来直接拿地图系坐标跟场地几何比 → psi0≠0 时全判错，见 to_field 注释）
        fx, fy = to_field(p[0], p[1], self.psi0)
        face = 'A' if fx < PANEL_X else 'B'
        centers = [(i + 1, COL_Y[i]) for i in range(3)]
        col = min(centers, key=lambda t: abs(t[1] - fy))[0]
        # ★ 2026-10-14：上下排改用**飞控高度**判（原来按"当前航点标签"判）。
        #   原因：快滑模式下航线里多了"进/出"扫描点（名字不是两字符货位），
        #   按标签判会把下排的码绑到上排货位；高度才是真值（上排 142 / 下排 103）。
        row = 1 if (self.height is not None and self.height < ROW_SPLIT_H) else 0
        return '%s%d' % (face, label_col(face, col) + row * 3)

    def _speed(self):
        """用位姿差分粗估水平速度（cm/s，低通）。拿不到位姿就返回 0（不拦）。"""
        p = self._pose()
        now = time.time()
        if not p:
            return 0.0
        if getattr(self, '_last_pose', None) is None:
            self._last_pose = (p[0], p[1], now)
            self._speed_lp = 0.0
            return 0.0
        x0, y0, t0 = self._last_pose
        dt = now - t0
        if dt < 0.02:
            return self._speed_lp
        v = math.hypot(p[0] - x0, p[1] - y0) / dt
        self._last_pose = (p[0], p[1], now)
        self._speed_lp += 0.15 * (v - self._speed_lp)
        return self._speed_lp

    def on_idx(self, m):
        idx = int(m.data)
        if self.last_idx is not None and idx != self.last_idx and self.pending_laser:
            # 检测通常比航点判定早 1~2 秒；不要因换格立即丢弃。
            # 仍需随后看到同一货号并进入瞄准窗，不会凭旧坐标乱打。
            age = (time.time() - self.pending_laser_at
                   if self.pending_laser_at is not None else 1e9)
            if age > LASER_PENDING_TTL_S:
                self.get_logger().warn(
                    '离开航点 %d 时 %s 已等待%.1fs，跳过打点'
                    % (self.last_idx, self.pending_laser, age))
                self.pending_laser = None
                self.pending_laser_at = None
            else:
                self.get_logger().info(
                    '离开航点 %d，保留 %s 待打点（等待视觉瞄准窗）'
                    % (self.last_idx, self.pending_laser))
        self.last_idx = idx

    def _on_qr_center_cross(self, label, code, dx, dy):
        """同一货位同一码的连续帧跨过激光中线时返回 True。"""
        now = time.monotonic()
        if label is None or abs(float(dy)) > MARK_DY_TOL:
            self._mark_track = None
            return False
        prev = self._mark_track
        crossed = False
        prev_dx = None
        if (prev is not None and prev[0] == label and prev[1] == code
                and now - prev[2] <= MARK_CROSS_GAP_S):
            prev_dx = prev[3]
            crossed = aim_crossed(prev_dx, dx)
        self._mark_track = (label, code, now, float(dx))
        if crossed:
            self._mark_cross_dir = 1 if dx > prev_dx else -1
            self.get_logger().info(
                '过激光中线：%s dx=%+.2f→%+.2f 目标%+.2f dy=%+.2f'
                % (label, prev_dx, dx, AIM_TARGET_DX, dy))
        return crossed

    def _fire_laser(self, hold_s=1.2):
        """打点：点亮激光 hold_s 秒后自动熄灭（只有采纳该格货号时才调用）。"""
        try:
            self.pub_laser.publish(Bool(data=True))
            self.get_logger().info('激光打点：亮 %.1fs' % hold_s)

            def _off():
                try:
                    self.pub_laser.publish(Bool(data=False))
                except Exception:             # noqa: BLE001
                    pass

            threading.Timer(hold_s, _off).start()
        except Exception as e:                # noqa: BLE001
            self.get_logger().warn('激光触发失败: %s' % e)

    def on_reached(self, m):
        self.reached_idx = int(m.data)
        # 到点事件只推进状态，不再直接触发激光。
        # 激光必须由 on_qr 中新鲜的“视觉居中 + 停稳”条件触发；
        # 否则航点到了但相机里码仍偏时会被打歪。

    def on_fc(self, m):
        if len(m.data) >= 2:
            self.fc = (int(m.data[0]), int(m.data[1]))
            if (self.laser_z_control_active
                    and (self.fc[0] != 3 or self.fc[1] != 1)):
                self.get_logger().error(
                    '激光Z中止：飞控不再处于mode3+unlock1（当前 %s）'
                    % (self.fc,))
                self.abort_requested = True
                try:
                    stop = UInt8()
                    stop.data = 3
                    self.pub_act.publish(stop)
                except Exception:  # noqa: BLE001
                    pass
        # ★ 2026-10-14：盯 cmd1（IMU 板最后执行的命令编号）。变 6 = 飞控在跑一键降落
        #   （"自降"）—— 我们从来没发过 0x66[2]，所以它只可能来自 IMU 核心板内部。
        #   发现就打 ERROR（操作手立刻知道是"自降"而不是我们的环坏了）。
        if len(m.data) >= 7:
            c1 = int(m.data[6])
            if c1 == 6 and getattr(self, '_cmd1_last', 4) != 6 and self.running:
                self.get_logger().error(
                    '⚠⚠ 飞控自己开始降落（cmd1=6）—— 这是"自降"，不是我们发的指令')
                # ★ 2026-10-19：不仅要停速度输出，还要让主流程**真的退出**。
                #   19:57 那轮只发了 active=3，主流程随后仍启动了 waypoint_nav →
                #   任务卡在 running 5 分钟，20:02 的新 START 全被"任务已在跑"吞掉。
                self.abort_requested = True
                # ★ 2026-10-15：**立刻停掉位置环输出**（切 3 = 只发 {0,0,0,0}）。
                #   现场规律："每次自降后会怠速然后自旋" —— 自降后飞控已在上锁/怠速，
                #   而我们还在继续注速度（尤其偏航率 vyaw）→ 飞控照执行 → 在地上自旋。
                #   所以一发现自降就马上不再注任何控制量（速度流仍保鲜，不影响自降判定）。
                try:
                    m = UInt8()
                    m.data = 3
                    self.pub_act.publish(m)
                    self.get_logger().warn('  → 已置 /active_controller=3（停止注入速度，含偏航）')
                except Exception:  # noqa: BLE001
                    pass
            self._cmd1_last = c1
            previous = self._cmd1_sample
            self._cmd1_sample = (previous[0] + 1 if previous else 1, c1, time.monotonic())

    def on_laser_z_fault(self, m):
        if not bool(m.data):
            return
        self.laser_z_fault = True
        if self.laser_z_control_active:
            self.get_logger().error(
                '激光Z外环故障 → 中止航线并保持零速流，交操作手降落')
            self.abort_requested = True
            try:
                stop = UInt8()
                stop.data = 3
                self.pub_act.publish(stop)
            except Exception:  # noqa: BLE001
                pass

    def _takeoff_ack_after(self, before_seq, sent_at):
        sample = self._cmd1_sample
        now = time.monotonic()
        return (sample is not None and sample[0] > before_seq
                and sample[1] == 5 and sent_at <= sample[2] <= now
                and now - sample[2] <= TAKEOFF_HEIGHT_MAX_AGE_S)

    def _fresh_airborne(self):
        sample = self.height_sample
        now = time.monotonic()
        return (sample is not None and 0 <= now - sample[1] <= TAKEOFF_HEIGHT_MAX_AGE_S
                and sample[0] >= 25)

    def _wait_takeoff_floor(self, timeout_s=8.0):
        """Wait until core-only launch is safely above the external-Z floor."""
        started = time.monotonic()
        while time.monotonic() - started < timeout_s:
            if self.abort_requested or not self.fc or self.fc[1] != 1:
                return False
            sample = self.height_sample
            if (sample is not None
                    and 0.0 <= time.monotonic() - sample[1]
                    <= LASER_Z_HEIGHT_MAX_AGE_S
                    and sample[0] >= LASER_Z_TAKEOFF_FLOOR_CM):
                return True
            time.sleep(0.05)
        self.get_logger().error(
            '起飞后未在%.0fs内稳定越过%.0fcm，拒绝启动激光Z'
            % (timeout_s, LASER_Z_TAKEOFF_FLOOR_CM))
        return False

    def _wait_program_arm(self, timeout_s=PROGRAM_ARM_TIMEOUT_S):
        """Wait for F407 0x66[5] to unlock and release program velocity."""
        started = time.monotonic()
        while time.monotonic() - started < timeout_s:
            if self.abort_requested:
                return False
            if not self.fc or self.fc[0] != 3:
                self.get_logger().error(
                    '程控启动中止：模式不再是3（当前 %s）' % (self.fc,))
                return False
            if self.fc[1] == 1:
                return True
            time.sleep(0.05)
        self.get_logger().error(
            '程控启动超时：F407 未在%.1fs内完成解锁（/fc_status=%s）'
            % (timeout_s, self.fc))
        return False

    def on_height(self, m):
        """Laser height, with range/spike rejection and receipt freshness."""
        h = int(m.data)
        if h < 0 or h > 500:
            return
        now = time.monotonic()
        previous = self.height_sample
        if (previous is not None
                and now - previous[1] <= TAKEOFF_HEIGHT_MAX_AGE_S
                and abs(h - previous[0]) > 100):
            return
        self.height_sample = (h, now)
        self.height = h

    def on_mark_ok(self, m):
        self.mark_ok = bool(m.data)
        self.mark_ok_t = time.monotonic()
        self._try_mark_retry()

    def _try_mark_retry(self):
        """过线后等额外偏移与相邻 /mark_ok；同码、几何和速度门不变。"""
        retry = self._mark_retry
        if retry is None:
            return False
        label, code, dx, dy, direction, crossed_at = retry
        now = time.monotonic()
        age = now - crossed_at
        if age > MARK_RETRY_WINDOW_S:
            self._mark_retry = None
            if self.pending_laser == label:
                self.get_logger().warn(
                    '过线后%.2fs未达到额外偏移，等待下一次过线：%s'
                    % (MARK_RETRY_WINDOW_S, label))
            return False
        if self.pending_laser != label:
            self._mark_retry = None
            return False
        if not mark_after_cross_ok(
                age, dx, dy, direction, MARK_RETRY_WINDOW_S,
                MARK_AIM_TOL, MARK_DY_TOL, MARK_CROSS_EXTRA_DX):
            return False
        if not self._mark_stable():
            return False
        spd = self._speed()
        self._mark_retry = None
        self.pending_laser = None
        self.pending_laser_at = None
        self.get_logger().info(
            '过线偏置打点：%s %.2fs 前进%+.2f（%.0fcm/s，货号%d）'
            % (label, age, (dx - AIM_TARGET_DX) * direction, spd, code))
        self._fire_laser(LASER_PULSE_S)
        return True

    def _mark_stable(self):
        """Require fresh heading-stability state and moderate scan speed."""
        if not self.mark_ok:
            return False
        if time.monotonic() - self.mark_ok_t > MARK_STATE_MAX_AGE_S:
            return False
        return self._speed() <= MARK_SPEED_MAX

    def _wait_takeoff_release(self, timeout_s, hold_s):
        gate = TakeoffReleaseGate(
            TAKEOFF_RELEASE_MIN_CM, TAKEOFF_RELEASE_MAX_CM, hold_s)
        started = time.monotonic()
        while time.monotonic() - started < timeout_s:
            if self.abort_requested or not self.fc or self.fc[1] != 1:
                self.get_logger().error('起飞高度等待中止：自降或飞控未解锁，不挂航线')
                return False
            now = time.monotonic()
            if gate.update(self.height_sample, now):
                return True
            time.sleep(0.05)
        self.get_logger().error(
            '起飞放行等待超时：要求 %.0f–%.0fcm，持续=%.1fs，'
            '必须使用最近%.1fs内的激光数据；不挂航线'
            % (TAKEOFF_RELEASE_MIN_CM, TAKEOFF_RELEASE_MAX_CM, hold_s,
               TAKEOFF_HEIGHT_MAX_AGE_S))
        return False

    def _wait_laser_z_height(self, target, timeout_s, hold_s):
        """Wait for the BVA laser loop to acquire the real route height."""
        since = None
        last_sample_at = None
        started = time.monotonic()
        while time.monotonic() - started < timeout_s:
            if self.abort_requested or self.laser_z_fault:
                self.get_logger().error(
                    '激光Z高度获取中止：abort=%s fault=%s'
                    % (self.abort_requested, self.laser_z_fault))
                return False
            if not self.fc or self.fc != (3, 1):
                self.get_logger().error(
                    '激光Z高度获取中止：飞控不是mode3+unlock1（当前 %s）'
                    % (self.fc,))
                return False
            sample = self.height_sample
            now = time.monotonic()
            if sample is None:
                since = last_sample_at = None
            else:
                h, sampled_at = sample
                fresh = 0.0 <= now - sampled_at <= LASER_Z_HEIGHT_MAX_AGE_S
                in_band = fresh and abs(h - target) <= LASER_Z_ACQUIRE_TOL_CM
                if not in_band:
                    since = last_sample_at = None
                else:
                    if (last_sample_at is not None
                            and sampled_at - last_sample_at
                            > LASER_Z_HEIGHT_MAX_AGE_S):
                        since = None
                    last_sample_at = sampled_at
                    if since is None:
                        since = sampled_at
                    if sampled_at - since >= hold_s:
                        return True
            time.sleep(0.05)
        self.get_logger().error(
            '激光Z高度获取超时：目标%dcm，允许±%.0fcm，持续%.1fs，'
            '当前h=%s mode=%s'
            % (target, LASER_Z_ACQUIRE_TOL_CM, hold_s,
               self.height, self.fc))
        return False

    def _wait_landed(self, timeout=15.0, thr=25):
        """等飞机落到 thr cm 以下（落地判据）；最多等 timeout 秒。

        返回"关位置环"那一刻的 /height（None=全程没读到）。
        ★ 2026-10-07(晚)：原来这里是无条件 sleep 6s ——
          降得慢就**在空中**把位置环关掉（危险），降得快则落地后还在发指令。
        判据要求"连续 5 次（约 0.5s）都在 thr 以下"，防单个野值在半空就把环关掉。
        """
        last = None
        n_low = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            h = self.height
            if h is not None:
                last = h
                if h <= thr:
                    n_low += 1
                    if n_low >= 5:
                        return h
                else:
                    n_low = 0
            time.sleep(0.1)
        self.get_logger().warn('等落地超时 %.0fs，当前高度 %s cm —— 仍然关位置环'
                               % (timeout, '未知' if last is None else '%.0f' % last))
        return last

    def _save_logs(self):
        """任务结束把 /tmp 的关键日志复制到 NVMe 上（/tmp 是内存盘，重启就没了）。

        ★ 2026-10-07：下一场要靠 /tmp/qr.log 里的
          `QR候选: 货号=.. 归一化偏移=(..)` 行来定 center_tol（"扫全" vs "激光打准"），
          所以每次任务结束都留一份，别指望 /tmp 撑到下次开机。
        """
        dst_dir = os.path.expanduser('~/traces')
        try:
            os.makedirs(dst_dir, exist_ok=True)
        except Exception:                     # noqa: BLE001
            return
        stamp = time.strftime('%Y%m%d_%H%M%S')
        for src in ('/tmp/qr.log', '/tmp/mission.log', '/tmp/wpn.log'):
            try:
                if os.path.exists(src):
                    dst = os.path.join(dst_dir, 'fl_%s_%s' % (stamp, os.path.basename(src)))
                    shutil.copyfile(src, dst)
                    self.get_logger().info('日志已存 NVMe: %s' % dst)
            except Exception as e:            # noqa: BLE001
                self.get_logger().warn('存日志失败 %s: %s' % (src, e))

    def _emit_event(self):
        """把识别到的货号绑定到"当前正在飞的那个货位"。"""
        if self.qr_num is None or not self.labels:
            return
        label = self.qr_label
        if label is None:
            if self.last_idx is None or self.last_idx >= len(self.labels):
                return
            label = self.labels[self.last_idx]
        # 起飞/降落航点的标签不是货位（'takeoff'/'land'），别去解析它，
        # 否则 int('a') 直接抛异常（2026-09-30 实测：起飞阶段识别到码就崩在这）。
        if len(label) != 2 or label[0] not in ('A', 'B') or not label[1].isdigit():
            self.qr_num = None
            return
        # ★ 2026-10-06：去重（串码的最后一道防线）。
        #   现场 12 个货号互不相同 → "同一货号出现两次" 或 "同一货位已绑过"
        #   都说明是相邻格串过来的。实测：B5 读到 6（其实是 B4 的）、
        #   B3 读到 7（其实是 B6 的）—— 都被这一步挡掉。
        if label in self.bound or self.qr_num in self.bound.values():
            self.get_logger().info(
                '忽略重复：%s=货号%d（已绑定 %s）' % (label, self.qr_num, self.bound))
            self.qr_num = None
            return
        self.bound[label] = int(self.qr_num)
        # ★ 2026-10-14：打点**推迟到飞机停到该格中心**（快滑模式下边走边打会拖成线）
        #   停稳了立刻打；还在动就记 pending，等 on_qr 里"码居中+停稳"那一帧补打。
        self.pending_laser = label
        self.pending_laser_at = time.time()
        self.get_logger().info(
            '采纳 %s → 待连续帧跨过激光中线且姿态稳定时打点' % label)
        face, pos = label[0], int(label[1])
        ev = Int16MultiArray()
        ev.data = [int(self.qr_num), 0 if face == "A" else 1, pos]
        self.pub_event.publish(ev)
        ref = reference_of(label)
        mark = '✓' if ref == self.qr_num else ('≠' if ref != '-' else '')
        self.get_logger().info('EVENT %s = 货号 %d（参考 %s %s）'
                               % (label, self.qr_num, ref, mark))
        self._trace(label, ref, self.qr_num)
        self.qr_num = None

    def _trace(self, label, ref, num):
        """把 位置-货号 落到 CSV：位置,参考货号,现场识别货号,时间。"""
        if not self.trace_file:
            return
        try:
            with open(self.trace_file, 'a') as fh:
                fh.write('%s,%s,%s,%.1f\n' % (label, ref, num, time.time()))
                fh.flush()
                os.fsync(fh.fileno())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn('写记录失败: %s' % e)

    # ---------- 触发 ----------
    def _plan_points(self):
        return plan_points(self.subset)

    def on_start(self, m):
        mode = int(m.data)
        self.get_logger().info('收到 START mode=%d' % mode)
        if mode != 0:
            self.get_logger().warn('定点盘点暂未实现（只支持遍历）')
            return
        if not self._start_lock.acquire(blocking=False):
            self.get_logger().warn('任务已在跑，忽略')
            return
        self.running = True
        try:
            threading.Thread(target=self.run_traverse, daemon=True).start()
        except Exception:
            self.running = False
            self._start_lock.release()
            raise

    def _set_active(self, val, n=5):
        """把 /active_controller 设成 val（0=关位置环 / 2=接管）。

        ★ 2026-10-14 深夜：`position_pid_controller` 的激活是**锁存**的
          （`pid_controller.cpp:206-219`：收到 2 就一直发速度，只有收到 0 才关）。
          而本节点原来只在 `finally`（任务结束）发 0，**开始前从不发 0** ——
          所以只要上一轮留在 2、或控制台/链路启动时激活过，**起飞爬升那一段
          位置环也在发 vz**，和飞控自己的高度环互相顶（现场："起飞一顿一顿、
          速度慢"，而悬停自测链爬起来是一口气）。
          所以起飞前必须强制置 0，爬完再置 2。
        """
        m = UInt8()
        m.data = int(val)
        for _ in range(int(n)):
            self.pub_act.publish(m)
            time.sleep(0.05)

    def _set_laser_z(self, enabled, n=5):
        """Enable/disable the BVA laser-Z owner with repeated delivery."""
        m = Bool()
        m.data = bool(enabled)
        for _ in range(int(n)):
            self.pub_laser_z.publish(m)
            time.sleep(0.05)
        self.laser_z_control_active = bool(enabled)
        if enabled:
            self.laser_z_fault = False
        self.get_logger().warn(
            '激光Z控制%s（mode3唯一Z外环）'
            % ('接管' if enabled else '释放'))

    # ★ 2026-10-16（下午，按现场要求）：**起飞段完全照搬悬停链**。
    #   悬停链（hover_auto + pos_hold）的实况：
    #     · 起飞前就把 pos_hold 挂上、并把 /active_controller 置 2
    #       （hover_test.py 注释原文："二者必须在 0x66 起飞之前发布，
    #         否则离地瞬间没有水平闭环"）
    #     · pos_hold 在地面/低空每拍发 [0,0,0,0]；**一过 60cm** 立刻记住悬停点
    #       并接上水平 PI（kp=0.40 ki=0.04 trim=(0,-1.8) vmax=12），vz 恒 0，
    #       所以它只做水平保持、不和飞控高度环抢 z。
    #   任务链原来用 /active_controller=3（每拍发 0）一直等到 h≈130 才接管，
    #   中间那 2 秒多没有任何水平闭环 → 飞控拿着带偏置的 SLAM 速度自己定点，
    #   实测漂 ~10cm/s（2026-10-16 16:21 那轮：`fcTgtVx/Vy` 全程 0 却漂了 16cm）。
    #   所以这里改成：起飞前起 pos_hold + active=2；爬完悬停稳了再交棒给航线。
    # ★ 2026-10-16（晚）：参数换成 10-12 晚定档那套（`交接…-2026-10-12-晚.md:29-32`）——
    #   现场实测：同一套 pos_hold，默认 (0.40/0.04、阻尼器只在 log) 一起飞就往一边窜；
    #   换成这套 + 阻尼器 on 后"效果很好"（用户原话）。9 个位置参数：
    #     kp ki trim_vx trim_vy vmax  kd_yaw kyaw vmax_dps kiyaw
    POSHOLD_ARGS = ['0.35', '0.05', '0.0', '-1.8', '12', '0.40', '0.25', '12', '0']
    POSHOLD_ENV = {                      # 偏航阻尼器：10-12 晚用的就是 on
        'POSHOLD_DAMP': 'on', 'POSHOLD_DAMP_K': '2.2', 'POSHOLD_DAMP_DEAD': '0.5',
        'POSHOLD_DAMP_TLEAD': '0.2', 'POSHOLD_E_DEAD': '1.0', 'POSHOLD_PROBE': 'off',
    }

    def _stop_pos_hold(self):
        """停掉外挂位置环（进程名拼开，避免 pkill 匹配到自己的命令行）。"""
        subprocess.run(['pkill', '-f', '[p]os_hold'], capture_output=True)
        time.sleep(0.3)

    def _pos_hold_running(self):
        return subprocess.run(['pgrep', '-f', '[p]os_hold'],
                              capture_output=True).returncode == 0

    def _set_target_position(self, z_cm):
        """发布 `/target_position = [当前x, 当前y, 起飞高度, 当前航向]`。

        ★ 2026-10-16（晚，现场定位出的**最本质**一处差异）：
          C++ 位置环（`position_pid_controller`）的目标来自 `/target_position`，
          它回调第一行就是 `if (!has_target_position_) return;`
          —— **没有目标点，位置环一行速度都不发**（`hover_test.py:9-11` 原话）。

          悬停自测链（`hover_test.py` / `hover_auto.py` 那条）起飞前显式发了
          `[x, y, 150, yaw]`，所以爬升段是**位置环在保持位置 → 直上直下**。
          而任务链里没人发（`ros2 topic info /target_position` 的 publisher = 0），
          位置环全程静默 → 只剩 `pos_hold` 的速度指令在推，飞控把它当外部速度源，
          实测一起飞就往右漂、`pos_hold` 命令顶到上限也按不住。
          这里补上这一步，任务链的爬升段才真正和悬停链一致。
        """
        p = self._pose()
        if p is None:
            self.get_logger().warn('  ✗ 拿不到位姿 tf，无法设置起飞目标点 → 位置环不会动')
            return False
        m = Float32MultiArray()
        m.data = [float(p[0]), float(p[1]), float(z_cm), float(p[2])]
        for _ in range(8):
            self.pub_pos.publish(m)
            time.sleep(0.05)
        self.get_logger().info(
            '起飞目标点已发 /target_position = [%.1f, %.1f, %.0f, %.1f°]'
            '（与悬停链一致：爬升段由位置环保持水平）' % (p[0], p[1], z_cm, p[2]))
        return True

    def _ensure_pos_hold(self, force=False):
        """确保外挂位置环在跑（起飞段用它，和悬停链 10-12 晚那套一致）。

        ★ 2026-10-16（晚，现场踩坑）：**必须在解锁之前就挂好。**
          飞控有一条"外部速度源超时 → 自己降落+自锁"的保护：
          `/target_velocity`（0x31）静默约 2 秒它就自降。16:47 那轮的实际链条：
            START → 生成航线 2.4s → 起飞段里 kill+重启 pos_hold（静默 2.3s）
            → 飞控已自锁（复核 mode=1 unlock=0）→ 起飞帧打在上锁状态上 → 只有怠速。
          所以：**节点启动时**就把 pos_hold 挂上（那时还没解锁，静默无害），
          起飞段只做"没有才补"；补的时候也先用 active=3 保住速度流。
        """
        if self._pos_hold_running() and not force:
            self.get_logger().info('pos_hold 已在跑（起飞段保持不动，避免速度流静默）')
            return False
        # 关键：先把速度流保住（pid 每拍发 0），再动 pos_hold，避免出现静默窗口
        self._set_active(3)
        subprocess.run(['pkill', '-f', '[p]os_hold'], capture_output=True)
        time.sleep(0.3)
        # ★ 必须用 `export`：只写 `A=B;` 是普通 shell 赋值，子进程（pos_hold）继承不到 —— 
        #   16:47 那轮就栽在这里：参数对了、但阻尼器还是默认的 log 模式。
        envs = ' '.join('%s=%s' % kv for kv in sorted(self.POSHOLD_ENV.items()))
        cmd = ('export ROS_DOMAIN_ID=6; source /opt/ros/humble/setup.bash; '
               'source ~/ros2_ws/install/setup.bash; cd ~/diag; '
               'export ' + envs + '; '
               'setsid nohup python3 -u pos_hold.py ' + ' '.join(self.POSHOLD_ARGS) +
               ' > /tmp/pos_hold.log 2>&1 < /dev/null &')
        subprocess.Popen(['bash', '-lc', cmd], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
        # 2.0→0.8s：后面仍以真实 /target_velocity 帧作为就绪判据，
        # 不用固定长等待替代证据。
        time.sleep(0.8)
        self.get_logger().info(
            '已挂 pos_hold（10-12 晚定档：kp=0.35 ki=0.05 trim=(0,-1.8) vmax=12 '
            '+ kd=0.40 kyaw=0.25，阻尼器 on k=2.2；vz 恒 0）')
        return True

    def _start_waypoint_nav(self):
        """提前拉起航点导航节点（它要等 /active_controller==2 才开始跑）。

        ★ 2026-10-14 深夜：原来这段是在"悬停已稳"之后才做（pkill + sleep0.5 +
          Popen + sleep2 + 密集激活 3s ≈ 5.7 秒），全串行堆在"开始飞"前面。
          实测 16:34 那轮：START→开始横移 19.4 秒，这 5.7 秒是纯浪费。
          waypoint_nav 只在收到 2 之后才动（`waypoint_nav.py:92-95`），
          所以起飞前就把它拉起来、让 DDS 先匹配好，是安全且省时的。
        """
        subprocess.run(['pkill', '-f', '[w]aypoint_nav.py'], capture_output=True)
        time.sleep(0.3)
        # Discard the previous run's readiness marker before launching.
        with open('/tmp/wpn.log', 'w', encoding='utf-8'):
            pass
        cmd = ('export ROS_DOMAIN_ID=6; source /opt/ros/humble/setup.bash; '
               'source ~/ros2_ws/install/setup.bash; cd ~; '
               'setsid nohup timeout 1800 python3 -u ~/diag/waypoint_nav.py --ros-args '
               '-p file:=%s -p body_frame:=dr_base > /tmp/wpn.log 2>&1 < /dev/null &'
               % self.route_file)
        subprocess.Popen(['bash', '-lc', cmd], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self.abort_requested:
                return False
            try:
                with open('/tmp/wpn.log', encoding='utf-8', errors='replace') as f:
                    if 'waypoints=' in f.read():
                        return True
            except OSError:
                pass
            time.sleep(0.05)
        self.get_logger().warn('未见航点就绪日志，沿用原2秒启动等待并继续重复激活')
        return True

    def _waypoint_nav_running(self):
        return subprocess.run(['pgrep', '-f', '[w]aypoint_nav.py'],
                              capture_output=True).returncode == 0

    def _cmd(self, sub, height=0, reps=1):
        """0x66 任务帧（起飞/降落/返航/锁定）。**默认只发 1 帧**。

        ★ 2026-10-14（重要修复）：这里原来是 `for _ in range(8)`，一次起飞发 8 帧，
          三次重试共 24 帧。10-11 的实测记录（`flight_console.py:470-478`）写明：
          "连发 6 帧会让 IMU 板的任务步骤（cmd_fun 5→6）被反复推进，**爬到目标高度后
          直接跳到'降落' —— 飞机自己落下**"。今天 15:16 / 15:29 / 15:34 三轮失败
          全部符合这个签名（cmd1 从 4 跳到 6、油门窜一下就回怠速、随后自锁），而
          用单帧的 flight_console 路径昨天飞得动。
          所以单帧就够 —— 407 侧收到一帧会把 mission_tk_req 置 1，然后每毫秒重试
          OneKey_Takeoff() 直到被接受（`AnoDTRasp.c:68-77`），**不要为了"保险"加发数**。

        reps>1 仅留给"确认没收到时人工重发"的场景，默认不要用。
        """
        msg = Int16MultiArray()
        msg.data = [int(sub), int(height)]
        for i in range(int(reps)):
            self.pub_cmd.publish(msg)
            if int(reps) > 1 and i + 1 < int(reps):
                time.sleep(0.1)

    def _qr_start(self):
        """按需启动识别节点。

        ★ 2026-10-06：qr_vision 常驻实测吃 **116% CPU（一个半核）**，是整条链路里
        最重的进程；而它只在"扫描货位"这段时间有用。这里改成任务开始时起、
        任务结束时停（`finally` 保证一定停）。看门狗里对应的常驻补启已删掉。
        """
        if subprocess.run(['pgrep', '-f', '[q]r_vision_node'],
                          capture_output=True).returncode == 0:
            return
        cmd = ('export ROS_DOMAIN_ID=6; source /opt/ros/humble/setup.bash; '
               'source /home/orangepi/ros2_ws/install/setup.bash; '
               'setsid nohup python3 -u '
               '/home/orangepi/ros2_ws/install/qr_vision/lib/qr_vision/qr_vision_node.py '
               # laser_enabled:=false —— 激光改由本节点"采纳货号"时打点，
               # 避免 qr_vision 每解到码就闪（会打到画面边上的码）
               '--ros-args -p mode:=camera -p device:=/dev/video0 '
               '-p laser_enabled:=false -p cooldown_sec:=0.12 '
               '> /tmp/qr.log 2>&1 < /dev/null &')
        subprocess.Popen(['bash', '-lc', cmd],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # 标志文件：看门狗靠它判断"任务真的在跑"（mission_node 本身是常驻的，
        # 一直在等 /gs/start，所以不能拿"进程存在"当判据）
        try:
            open('/tmp/mission_active', 'w').close()
        except Exception:  # noqa: BLE001
            pass
        self.get_logger().info('按需启动 qr_vision（任务期间才开，省 ~1.2 核）')
        # ★ 2026-10-14 深夜：这里原来是 `time.sleep(2.5)`（写死等 2.5 秒）。
        #   实测 16:34 那轮 START→航线生成 = 3.07 秒，其中 2.5 秒就是这一句，
        #   另外 0.5 秒是下面归零后的 sleep。而 qr_vision 真正被用到是
        #   **扫描开始**（起飞+爬升+稳定之后，约 10 秒后），所以这 2.5 秒纯属白等。
        #   降到 0.3 秒只为让进程真的 spawn 出来（日志/异常能落地）。
        time.sleep(0.3)

    def _qr_stop(self):
        subprocess.run(['pkill', '-f', '[q]r_vision_node'], capture_output=True)
        try:
            os.remove('/tmp/mission_active')
        except Exception:  # noqa: BLE001
            pass
        self.get_logger().info('已停 qr_vision（释放 ~1.2 核）')

    def _log_backup_loop(self):
        """任务期间每 15 秒把 /tmp/mission.log、/tmp/wpn.log 拷到 ~/traces/。

        ★ 2026-10-14 深夜：板子会随机重启（P3），重启后 /tmp 全丢，
          `_save_logs()`（只在 finally 跑）根本来不及 → 现场数据全没了
          （20:52 那轮就是，无法查"为什么漏了 3 个货位"）。
          这个后台线程保证最多丢 15 秒。
        """
        dest = os.path.expanduser('~/traces/live_%d' % int(time.time()))
        try:
            os.makedirs(dest, exist_ok=True)
        except Exception:  # noqa: BLE001
            return
        while self.running:
            for src in ('/tmp/mission.log', '/tmp/wpn.log'):
                try:
                    if os.path.exists(src):
                        shutil.copyfile(src, os.path.join(
                            dest, os.path.basename(src)))
                except Exception:  # noqa: BLE001
                    pass
            time.sleep(15)

    def _pose(self):
        try:
            tr = self.buf.lookup_transform('map', 'dr_base', rclpy.time.Time())
            q = tr.transform.rotation
            yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            return (tr.transform.translation.x * 100.0,
                    tr.transform.translation.y * 100.0, yaw)
        except Exception:
            return None

    def _wait_pose_stable(self):
        """连续 POSE_STABLE_N 帧的相邻帧位移都 < POSE_STABLE_TOL_CM 才算稳。"""
        deadline = time.monotonic() + POSE_STABLE_TIMEOUT_S
        last = None
        steps = []
        while time.monotonic() < deadline:
            p = self._pose()
            if p is None:
                last = None
                steps = []
                time.sleep(0.05)
                continue
            if last is not None:
                steps.append(math.hypot(p[0] - last[0], p[1] - last[1]))
                if len(steps) > POSE_STABLE_N:
                    steps.pop(0)
                if len(steps) >= POSE_STABLE_N and max(steps) < POSE_STABLE_TOL_CM:
                    self.get_logger().info(
                        '位姿稳定闸门通过：最近 %d 帧最大位移 %.2f cm（阈值 %.1f cm）'
                        % (POSE_STABLE_N, max(steps), POSE_STABLE_TOL_CM))
                    return True
            last = p
            time.sleep(0.05)
        self.get_logger().error(
            '✗ 位姿不稳定：%.0fs 内没能连续 %d 帧位移 <%.1f cm（最近最大帧位移 %s）→ 拒绝起飞，不发 0x66'
            % (POSE_STABLE_TIMEOUT_S, POSE_STABLE_N, POSE_STABLE_TOL_CM,
               ('%.1f cm' % max(steps)) if steps else '无数据'))
        return False

    def run_traverse(self):
        # on_start 已原子占用；直到原任务完成清理才允许下一次触发。
        try:
            self._run_traverse_owned()
        finally:
            self.running = False
            self.abort_requested = False
            self._start_lock.release()

    def _run_traverse_owned(self):
        # ★ 2026-10-19：单主控锁。hover_test 抢同一把锁；两边不能同时写
        #   /target_position /active_controller（20:02 就是这么飘的）。
        fd = None
        try:
            fd = open('/tmp/flight_owner.lock', 'w')
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fd.write(str(os.getpid()))
            fd.flush()
            self._owner_fd = fd
        except OSError:
            self.get_logger().error(
                '✗ /tmp/flight_owner.lock 被占用（hover_test 或另一份任务在跑）'
                '→ 拒绝执行，先停掉对方')
            if fd is not None:
                fd.close()
            return
        self.running = True
        self.abort_requested = False
        self.laser_z_fault = False
        self._set_laser_z(False, n=2)
        self.bound = {}          # 每次任务重新开始去重
        # ★ 2026-10-14 深夜：**周期性把日志落到 NVMe**。
        #   20:52 那轮飞完板子随机重启（P3 老问题），`/tmp/mission.log` / `wpn.log`
        #   全丢 → 没法查"为什么漏了 3 个货位"。`_save_logs()` 只在 finally 跑，
        #   重启就来不及。这里起个后台线程每 15 秒往 ~/traces/ 拷一份。
        threading.Thread(target=self._log_backup_loop, daemon=True).start()
        try:
            self.get_logger().info('遍历任务开始')
            if self.takeoff_mode != 'auto':
                self.get_logger().error(
                    '历史复现候选只支持 auto，当前=%s'
                    % self.takeoff_mode)
                return
            if not self.fc:
                self.get_logger().error('没有 /fc_status，拒绝执行遍历')
                return
            if self.fc[0] != 2:
                self.get_logger().error(
                    '飞控模式 %d（历史复现要求 CH5=2 定点模式）→ 拒绝起任务'
                    % self.fc[0])
                return
            if self.fc[1] != 1:
                self.get_logger().error(
                    '历史复现要求遥控先解锁（/fc_status=%s）→ 拒绝起任务'
                    % (self.fc,))
                return
            # ★ 2026-10-14 深夜（现场踩坑）：**任务期间必须停掉网页控制台**。
            #   实测 `/target_position` 有两个发布者：`waypoint_nav`（我们的航线）
            #   和 `flight_console`（8090 网页控制台，它一直在发一个默认点
            #   (-55.4, 70.8)）→ 两个目标互相打架，飞机原地不动（现场："怎么不前进"）。
            subprocess.run(['pkill', '-f', '[f]light_console'], capture_output=True)
            self._qr_start()
            # ★ 2026-10-06：归零改成发 /dead_reckon/reset（<50ms）。
            #   原来跑 reset_origin.sh：里面有 sleep 6 + 两次 `ros2 topic echo` 诊断打印
            #   （每次启动 ROS CLI + DDS 发现要 3~6 秒）→ **从触发到起飞白等 15 秒**。
            #   而且 dead_reckon 现在归 ros2 launch 托管，脚本里那句 pkill dead_reckon.py
            #   还可能被 launch 连坐（实测出现过链路只剩 3 个节点）。
            self.pub_reset.publish(Empty())
            time.sleep(0.2)          # 归零是话题一发就走（<50ms），0.5 → 0.2
            pose = None
            for _ in range(20):
                pose = self._pose()
                if pose:
                    break
                time.sleep(0.5)
            if not pose:
                self.get_logger().error('拿不到位姿 tf，任务中止')
                return
            if not self._wait_pose_stable():
                return
            psi = pose[2]
            self.psi0 = psi             # ★ 2026-10-14：货位绑定要用它做坐标反变换
            # ★ 2026-10-14 深夜：**起飞前查航向基准**（见 PSI0_REF_DEG 的说明）。
            #   偏离参考值 → 中止，绝不发 0x66（否则整条航线在房间里是转过的，会撞东西）。
            # ★ 2026-10-14 深夜（**纠正**）：这条检查原来是"偏离 −132° 就中止"，**是错的**。
            #   场地坐标并不是绝对房间坐标，而是"以起飞点为原点、以归零那一刻机头方向
            #   为 +x"的机体相对坐标（`simple_map/` 那张简易地图就是这么标的，板子在 −y
            #   = 飞机右侧）。所以 psi0 本来就随每次会话变化，使命里的 `to_field()` 已经
            #   补偿了地图系↔场地系的旋转 —— 不该拿它当"对齐好坏"的判据（会误拦合法飞行）。
            #   现在只记录、不再中止。
            # ★ 2026-10-15（**改回拦停**）：实测 psi0 落到 0.1° / 100.4° 时，
            #   航线在房间里是"转过的"（目标算成"往后左"）→ 飞行方向完全错（危险）。
            #   而飞对的那几轮 psi0 都在 −90° ~ −133°。所以这一条必须拦：
            #   方向错比"不飞"危险得多。
            _dpsi = (psi - PSI0_REF_DEG + 180.0) % 360.0 - 180.0
            # ★ 2026-10-15：**只在 FIELD_ROT_ZERO=False（老模式）时才拦**。
            #   现在位置换算已经不用 psi 了（场地坐标相对起飞点），
            #   psi 落在哪都不影响方向 —— 再拦就把正常飞行也挡掉了
            #   （实测刚被挡：psi0=71.4° → 中止，和方向无关）。
            if PSI0_GUARD_ON and abs(_dpsi) > PSI0_TOL_DEG:
                self.get_logger().error(
                    '✗ 航向基准异常：psi0=%.1f°（参考 %.0f°，差 %.1f°）→ 中止，不发 0x66。'
                    '这是"建图时飞机没按惯常位置/朝向摆好"导致的 —— 把飞机摆回惯常起飞位、'
                    '机头照平时朝向，重启链路（cartographer 重建图）后再飞。'
                    % (psi, PSI0_REF_DEG, _dpsi))
                return
            self.get_logger().info('航向基准正常：psi0=%.1f°（参考 %.0f°）'
                                   % (psi, PSI0_REF_DEG))
            # ★ 2026-10-06 保险：位姿航向（dead_reckon，低通）与 tf 航向直接对一下。
            #   实测踩坑：归零瞬间残留旧航向（28.5°）而真实航向是 0°，航线被整体
            #   转了 28.5°。dead_reckon 已改成归零时直接对齐 tf；这里再做一次校验，
            #   差得多就报警（宁可看到告警，也不要带着错航向起飞）。
            try:
                tr = self.buf.lookup_transform('map', 'dr_base', rclpy.time.Time())
                q = tr.transform.rotation
                psi_tf = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
                dpsi = abs((psi - psi_tf + 180.0) % 360.0 - 180.0)
                if dpsi > 5.0:
                    self.get_logger().warn(
                        '⚠ 位姿航向 %.1f° 与 tf 航向 %.1f° 相差 %.1f°（低通残留？）'
                        % (psi, psi_tf, dpsi))
            except Exception:  # noqa: BLE001
                pass
            pts = self._plan_points()
            if not pts:
                self.get_logger().error('subset=%s 没匹配到任何货位，任务中止' % self.subset)
                return
            self.get_logger().info('连续扫描航线：%d 个航点，覆盖 %s'
                                   % (len(pts), ' '.join(sorted({p[0] for p in pts}))))
            # 绕板端部（实测：从板子这侧扫完直接连降落点会蹭到白板端头 → 撞板）
            # 航点标签必须和 route 顺序一一对应，所以 detour 也要进 labels
            detour = []
            if pts[-1][1] < PANEL_X:          # 扫完还在 A 侧（起飞点侧）→ 必须绕板远端
                # 巡航高度沿用"最后一格的高度"，避免扫完下排还要先爬回顶层
                cruise_z = pts[-1][3]
                detour = [('绕行-A侧到板远端', NEAR_X, DETOUR_Y, cruise_z),
                          ('绕行-横穿到降落侧', LANDING_FIELD[0], DETOUR_Y, cruise_z)]
            self.labels = (['takeoff'] + [p[0] for p in pts]
                           + [d[0] for d in detour] + ['land'])
            try:
                os.makedirs(TRACE_DIR, exist_ok=True)
                self.trace_file = os.path.join(
                    TRACE_DIR, 'mission_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
                with open(self.trace_file, 'w') as fh:
                    fh.write('# 单板双面遍历盘点 位置-货号 记录\n')
                    fh.write('# 参考货号来自方案基线 §1.1 离线样本；现场货号随机\n')
                    fh.write('位置,参考货号,现场识别货号,时间戳\n')
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn('建记录文件失败: %s' % e)
                self.trace_file = None
            with open(self.route_file, 'w') as fh:
                fh.write('# 由 mission_node 实时生成，psi0=%.2f 度\n' % psi)
                fh.write('# 遍历流程: 起飞 → %s → 降落\n'
                         % ' → '.join(p[0] for p in pts))
                fh.write('# 参考货号(方案基线 §1.1 离线样本，现场随机): %s\n'
                         % ' '.join('%s=%s' % (p[0], reference_of(p[0])) for p in pts))
                # ★ 2026-10-14（快滑模式）：全局 hold_s 0（格间不停），
                #   只有"列中心"那一格航点带 hold: CELL_HOLD_S（见下面的 hold 写法）；
                #   识别在滑行/接近过程中完成，停下只为"码居中 + 打点"。
                #   SCAN_SLIDE=False 时回到 10-06 的逐格停 1.2s。
                # ★ 2026-10-14 深夜：z_cm 8.0 → 25.0。
                #   实测 16:03 那轮：起飞到 142 很干净（268~279s 稳在 142~148），
                #   但 IMU 板在 t=279.5 又跑了一次 cmd1=6，把高度从 146 砸到 123；
                #   等 waypoint_nav（晚 2.5s 才挂上）开始判"到达"时 h≈126，
                #   dz=|142-126|=16 > 8 → 永远不满足 → 又卡在航点 0。
                #   也就是说"自降"抖的量（20~25cm）比容差还大，容差必须比它宽，
                #   否则这条链路在自降抖动存在时不可能跑起来。
                #   注意：容差只影响"算不算到达"，目标高度仍是 142，z 环照旧往 142 拉。
                fh.write('tolerances: {pos_cm: 8.0, z_cm: 25.0, yaw_deg: 20.0, hold_s: %.1f}\n'
                         % (0.0 if SCAN_SLIDE else 1.2))
                # 边界用机体坐标（= 场地坐标 − 起飞点）；场地 420×420、起飞点 (60,60)
                fh.write('loop: false\nfield: {x_min: %.1f, x_max: %.1f, '
                         'y_min: %.1f, y_max: %.1f}\n'
                         % (-TAKEOFF_FIELD[0], FIELD_W - TAKEOFF_FIELD[0],
                            -TAKEOFF_FIELD[1], FIELD_H - TAKEOFF_FIELD[1]))
                fh.write('obstacles: []\nwaypoints:\n')
                # ★ 2026-09-30 踩坑：原来所有航点写 yaw:0.0 —— 位置环就会把机头
                #   强行转到"SLAM 系 0 度"，起飞瞬间直接转了 90°（实测）。
                #   正确做法：航向目标 = 起飞时的实际机头 ψ0（A 面正对板子）。
                # ★ 2026-10-07：航向改成**按点名判**（点坐标相同、没法用坐标区分）：
                #   B 面各格与"原地掉头"点用 ψ0+180，其余（A 面各格、两个过渡点
                #   里的第一个、"到达B面侧"）都用 ψ0。于是飞机先保持机头飞到
                #   第二个过渡点，"原地掉头"那一点只改航向 → 原地转 180°。
                def yaw_for(name):
                    """航向按**点名**判，不按坐标——因为"到达B面侧"和"原地掉头"
                    两点坐标相同，坐标判法无法区分先直飞、再原地转。
                    '掉头N' → psi0+N（步进旋转）；'稳定' → psi0+180。"""
                    if name[:1] == 'B':
                        return psi + 180.0
                    m = re.search(r'掉头(\d+)$', name)
                    if m:
                        return psi + float(m.group(1))
                    if '掉头' in name:
                        return psi + 180.0
                    if '稳定' in name:
                        return psi + 180.0
                    return psi

                fh.write('  - {name: takeoff, x: 0.0, y: 0.0, z: %.0f, yaw: %.1f, hold: %.1f}\n'
                         % (round(pts[0][3]), psi, TAKEOFF_HOLD_S))
                land_yaw = psi
                # ★ 2026-10-15：位置换算的旋转基准（见 FIELD_ROT_ZERO 的说明）
                psi_pos = 0.0 if FIELD_ROT_ZERO else psi
                for name, fx, fy, fz in list(pts) + list(detour):
                    x, y = to_flight(fx, fy, psi_pos)
                    yaw_t = yaw_for(name)
                    land_yaw = yaw_t
                    comment = ('   # %s 参考货号 %s' % (name, reference_of(name))
                               if len(name) == 2 else '')
                    # 换面"稳定"点多停一会儿（秒）→ 写成单点 hold；
                    # ★ 2026-10-14：列中心货位点写 CELL_HOLD_S（打点窗口），其余不停
                    hold = ''
                    if '稳定' in name:
                        hold = ', hold: %.1f' % STABILIZE_HOLD_S
                    elif len(name) == 2 and name[1].isdigit():
                        hold = ', hold: %.1f' % CELL_HOLD_S
                    # ★ 2026-10-14 深夜（现场指出）：扫描线两端的**进出点**（A进-*/A出-*）
                    #   必须先停住再动作 —— "最左边扫完竖直下降就可以继续扫"，
                    #   而原来这两点 hold=0、容差 8cm，前一点（A1/A6）离它只 11cm，
                    #   判定圈几乎重叠 → 还在冲就切下一个目标，下降是在飘着的状态里发生的
                    #   （现场："到最左边二维码飞过头，导致前往下一个航点是飘下去的"）。
                    # ★ 2026-10-15 修正：`A出-0` 是 **4 个字符**（A/出/-/0），
                    #   原来判 `len(name) == 3` 从来没匹配上 → 两端进出点根本没加
                    #   hold/tol（现场："最左边还是没有竖直降落"）。
                    # ★ 2026-10-15：`tol_cm` **4.0 → 8.0**。4cm 不可达 —— 位置环
                    #   `ki_xy=0`，实测静差 ~5cm（10-12 记录"接受 ~5cm 静差"），
                    #   所以 4cm 容差会让飞机在 `A进-0` 前**永远收不进去**：
                    #   现场"一直停在第一个二维码前"（日志里速度 4~5cm/s、码很居中
                    #   却一直不放行）。8cm = 全局容差，可达；hold 1 秒保留（先停再动作）。
                    if len(name) >= 3 and name[0] == 'A' and name[1] in ('进', '出'):
                        # First A entry is only a scan-start marker.  Keeping its
                        # historical 1s hold stops 15cm before the first QR code
                        # and forces a second position-loop crawl.  Enter the
                        # scan continuously; keep holds for exits/row changes.
                        first_entry = (name == 'A进-0')
                        hold += ', hold: 0.0, tol_cm: 8.0' if first_entry \
                            else ', hold: 1.0, tol_cm: 8.0'
                        # ★ 2026-10-15：**换排点必须真的降到目标高度才放行**。
                        #   实测 `A进-1 dz=-17.0`（目标 103、它到 ~120 就算"到了"）
                        #   → 接着往下排飞，就是现场"竖直下降没下到高度又开始向下飘"。
                        #   全局 z 容差 25cm 对换排太宽，这里单独收到 8cm。
                        if '-1' in name:
                            hold += ', ztol_cm: 8.0'
                    # ★ 2026-10-14 深夜：扫完之后的过路点（绕行…）放宽容差，
                    #   不要求精确收口 —— 配合 /bva_no_brake 的匀速直穿。
                    if name.startswith('绕行'):
                        hold += ', tol_cm: %.0f' % TRANSIT_TOL_CM
                    # 自转/过渡点：航向收紧到 3°（"精准自转"，下一面起点才不歪）
                    if '稳定' in name or '掉头' in name or name[:1] == 'B':
                        hold += ', ytol_deg: %.0f' % ROTATE_YTOL_DEG
                    fh.write('  - {name: %s, x: %.1f, y: %.1f, z: %.0f, yaw: %.1f%s}%s\n'
                             % (name, x, y, round(fz), yaw_t, hold, comment))
                # 降落航点沿用"上一个航点的朝向"，避免落地前原地转 180°。
                # ★ 2026-10-14 深夜：高度改成"多高就多高"——沿用最后一个航点的高度
                #   （原来写死 z=120，比扫描高度 103 高 17cm，落地前还要多爬一段）。
                cruise_z = round((list(pts) + list(detour))[-1][3])
                lx, ly = to_flight(LANDING_FIELD[0], LANDING_FIELD[1], psi_pos)
                fh.write('  - {name: land, x: %.1f, y: %.1f, z: %.0f, yaw: %.1f,'
                         ' tol_cm: %.0f}\n'
                         % (lx, ly, cruise_z, land_yaw, TRANSIT_TOL_CM))
            self.get_logger().info('航线已生成: %d 个货位 (psi0=%.1f)，记录 %s'
                                   % (len(pts), psi, self.trace_file))
            self.last_idx = None
            self.reached_idx = None
            # ★★ 2026-10-14：**先一键起飞、等它自己爬到位，再挂航线**。
            #   原来顺序是反的（waypoint_nav 先起 → 2 秒后才发 0x66）：
            #   爬升过程中位置环同时在调 z（到 142）和 xy，等于两个控制器抢高度，
            #   现场表现就是"起飞分好几段、悬停不稳、转头一个码太急"。
            #   悬停自测那条链路（hover_auto + pos_hold）**不碰高度**（vz 恒 0、
            #   且 h<60 不介入），所以那种起飞是一口气竖直爬、很稳 —— 这里对齐它。
            h_tgt = round(pts[0][3])
            # ★ 2026-10-14 深夜：起飞高度 = 航线高度（TAKEOFF_MIN_H 已撤成 0）。
            #   两者必须一致，否则 waypoint_nav 的 dz≤8cm 判据过不去，卡在航点 0。
            h_goto = h_tgt if TAKEOFF_MIN_H <= 0 else max(h_tgt, TAKEOFF_MIN_H)
            # 只改变送进核心的一键起飞载荷；h_goto 继续作为返航点/激光交棒目标。
            core_takeoff_h = (
                int(CORE_TAKEOFF_TEST_TARGET_CM)
                if abs(h_goto - CORE_TAKEOFF_TEST_ROUTE_CM) < 0.5 else h_goto)
            # 爬升期间预加载航点节点；active=3 时它只等待、不发布目标。
            self._set_active(3)
            if not self._start_waypoint_nav():
                self.get_logger().error('waypoint_nav预加载失败 → 不起飞')
                return
            # ★ 2026-10-14 深夜：**起飞爬升期间强制关掉位置环**。
            #   C++ 位置环的激活是锁存的（收到 2 就一直发速度，直到收到 0），
            #   而本节点原来从不在这之前发 0 —— 上一轮留在 2 的话，爬升全程
            #   位置环都在发 vz，和飞控高度环互相顶（"起飞一顿一顿、速度慢"）。
            #   先在航点列表里把"爬升段不许发速度"这件事定死，再发 0x66。
            # ★ 2026-10-14 深夜（第二轮修正）：爬升段用 **3** 而不是 0。
            #   · 0 = 位置环完全停发 → /target_velocity 静默。
            #     今晚的对照说明"静默"本身就危险：悬停自测单跑（一条速度都没有）
            #     2 秒就自降；任务卡在航点 0 那次也是在挂环之前（速度静默期）自降；
            #     而白天两条**直线测试**（位置环一直在发速度）各飞了 77 / 80 秒，
            #     cmd1=6 分别是 0 次 和 1 次（那一次还是收尾）。
            #     → 猜：IMU 核心板有"外部速度源超时"保护，我们的 0x31 流一断它就降落。
            #   · 3 = `pid_controller.cpp:212-215` → `should_stop_=true`，
            #     每拍都发 {0,0,0,0}：**速度流保持新鲜，但不跟飞控抢高度**
            #     （0 会和飞控高度环互相顶，就是之前"起飞一顿一顿"的来源）。
            #   所以：爬升段发 3（保鲜 + 不干涉），爬完再发 2 接管。
            # ★ 2026-10-16（按现场要求）：**照搬悬停链** —— 起飞前就挂 pos_hold
            #   并把 /active_controller 置 2（hover_test.py 注释原文：
            #   "二者必须在 0x66 起飞之前发布，否则离地瞬间没有水平闭环"）。
            #   注意 waypoint_nav 不能提前起：它一见 active==2 就开跑，
            #   会和 pos_hold 抢 /target_velocity，所以挪到交棒那一步。
            if USE_POS_HOLD:
                self._ensure_pos_hold()
            # 保持 active=3 直到高度门通过；pos_hold 不依赖该锁存标志。
            # ★ 2026-10-19：**正向确认** pos_hold 真的在写 /target_velocity。
            #   和 hover_test 一样：只发 active=2 不等于有控制，必须看到速度帧。
            if USE_POS_HOLD:
                tv0 = self.tv_n
                t_tv = time.monotonic()
                while (time.monotonic() - t_tv < POSHOLD_READY_TIMEOUT_S
                       and self.tv_n == tv0):
                    time.sleep(0.05)
                if self.tv_n == tv0:
                    self.get_logger().error(
                        '✗ pos_hold %.0fs 内没有发 /target_velocity'
                        '（起飞段没有水平控制）→ 中止，不发 0x66'
                        % POSHOLD_READY_TIMEOUT_S)
                    self._stop_pos_hold()
                    self._set_active(3)
                    return
                self.get_logger().info(
                    'pos_hold 已确认在发速度（%.0f 帧/≤%.0fs）'
                    % (self.tv_n - tv0, POSHOLD_READY_TIMEOUT_S))
            # ★ 2026-10-16（晚，撤回）：本来想用 `/target_position` 让位置环参与爬升段，
            #   但对照数据（rec_20261016_161704.log t=1502 那次飞得好的悬停）表明：
            #   hover_auto 那条链**同样不发** `/target_position`，所以这不是差异所在；
            #   发了反而可能让位置环和 pos_hold 两个速度源打架。固撤回。
            # self._set_target_position(h_goto)
            self.get_logger().info(
                '起飞前已挂 pos_hold 并置 /active_controller=2'
                '（与悬停链一致：离地 60cm 起水平闭环、vz 恒 0、不抢高度）')
            if self.takeoff_mode == 'manual':
                # ★ 2026-10-14（避开"自降"）：文档记录**手动飞不自降**，0x66 路径会。
                #   这个模式下我们不发起飞指令，只等操作手用遥控把飞机抬到高度。
                self.get_logger().info(
                    '起飞方式=manual：请用遥控把飞机抬到 %.0fcm（不发 0x66，避开自降）' % h_tgt)
                t_wait = time.monotonic()
                while time.monotonic() - t_wait < 60:
                    if self.height is not None and self.height >= h_tgt - 25:
                        break
                    time.sleep(0.2)
                if self.height is None or self.height < h_tgt - 25:
                    self.get_logger().error('✗ 60 秒内没等到手动起飞（h=%s）→ 中止' % self.height)
                    return
                self.get_logger().info('✅ 手动起飞到位（h=%s）→ 现在挂航线' % self.height)
            elif self.takeoff_mode == 'program':
                # F407 0x66[5]: arm and release 0x31 without OneKey_Takeoff.
                if not self.fc or self.fc[0] != 3:
                    self.get_logger().error(
                        '程控启动前置不满足：期望 mode3，当前 %s'
                        % (self.fc,))
                    self._stop_pos_hold()
                    self._set_active(3)
                    return
                self.get_logger().warn(
                    '程控启动：发送 0x66[5]（PC arm，不创建融合高度目标）')
                self._cmd(PROGRAM_START_SUB, 0)
                if not self._wait_program_arm():
                    self._stop_pos_hold()
                    self._set_active(3)
                    self.get_logger().error(
                        '✗ 程控启动未获得解锁，未发任何上升速度')
                    return
                self.get_logger().info(
                    '✓ F407 已完成程控解锁并打开速度通道；准备激光Z离地')
            else:
                # ★ 2026-10-14 实测：0x66 一键起飞约一半的尝试会"收下命令但不给油门"
                #   （cmd1 回到 4、h 不动、thr 停在小值）→ 必须像 hover_auto 一样重发；
                #   三次都没离地就**中止任务**，绝不带着地面上的飞机挂航线。
                got_air = False
                aborted = False
                miss_cnt = 0   # 连续"命令未被飞控受理"的次数（用来提示 CH6 复位）
                for attempt in range(1, 4):
                    # ★ 2026-10-14（晚）：发 0x66 前**最后一刻复核解锁位**。
                    #   任务开头虽然查过一次，但到真正发起飞指令之间还有 3~5 秒
                    #   （起 qr_vision + 等 tf + 生成航线）。15:16 实测就是在这 3 秒里
                    #   飞控自己跑完"一键降落"（cmd1=6）→ 上锁（unlock_sta 1→0），
                    #   三条 0x66[1,156] 全打在锁定状态上：只怠速、不升空。
                    #   锁着就别发 —— 发也不会离地，只是白烧三次重试。
                    if not self.fc or self.fc[1] != 1:
                        self.get_logger().error(
                            '✗ 起飞前复核：飞控已上锁（/fc_status=%s）→ 立即中止，不发 0x66'
                            % (self.fc,))
                        aborted = True
                        break
                    # ★ 2026-10-14（下调）：cmd1==6 **不能一看到就中止**。
                    #   实测（flight_data/2026-10-14/fcstat_run1.csv）：正常悬停 16 秒的
                    #   那一轮里 cmd1 在 4↔6 之间瞬跳了 6 次，每次只有 0.3s，飞机照飞。
                    #   所以 0.3s 的 6 是状态机回显抖动；只有**持续**在 6 才是真的在降落。
                    if getattr(self, '_cmd1_last', 4) == 6:
                        self.get_logger().warn('  复核：cmd1=6（可能是瞬跳）→ 等 2s 看它是否复位')
                        t_w = time.monotonic()
                        while time.monotonic() - t_w < 2.0:
                            if getattr(self, '_cmd1_last', 4) != 6:
                                break
                            if not self.fc or self.fc[1] != 1:
                                break
                            time.sleep(0.1)
                        if getattr(self, '_cmd1_last', 4) == 6 and self.fc and self.fc[1] == 1:
                            self.get_logger().error(
                                '✗ 起飞前复核：cmd1=6 持续 2s 没复位 → 真的在降落，不发 0x66')
                            aborted = True
                            break
                        self.get_logger().info('  cmd1 已复位 → 继续')
                    self.get_logger().info(
                        '起飞 #%d：0x66[1,%d]（核心目标 %d；激光交棒/航线高度 %d；'
                        '位置环还没挂，纯飞控爬升；'
                        '复核 mode=%s unlock=%s cmd1=%s）'
                        % (attempt, core_takeoff_h, core_takeoff_h, h_tgt,
                           self.fc[0], self.fc[1],
                           getattr(self, '_cmd1_last', '?')))
                    receipt = self._cmd1_sample
                    before_seq = receipt[0] if receipt else 0
                    sent_at = time.monotonic()
                    self._cmd(1, core_takeoff_h)
                    # ★ 2026-10-16：一级判据 —— 命令到底有没有到飞控。
                    #   `cmd1` = IMU 核自己回报的"最后收到的 CMD[1]"，5 = 一键起飞
                    #   （AnoDTRasp.c:572-578 把它塞进 0xF1 帧，fc_sta 是 IMU 状态帧解出来的）。
                    #   实测对照：到了 → 飞控自己的爬升剖面 0.9s 把油门顶到 94%，差速只涨 1.3%；
                    #             没到 → 飞控留在定点靠杆慢爬（5.8%/s），差速 3.7%/s 追上来 → 侧翻。
                    #   所以先在 1.5s 内确认 cmd1==5，没到就立刻重发，不再白等 5 秒。
                    seen5 = self._takeoff_ack_after(before_seq, sent_at)
                    t_ack = time.monotonic()
                    while not seen5 and time.monotonic() - t_ack < 1.5:
                        if self._takeoff_ack_after(before_seq, sent_at):
                            seen5 = True
                            break
                        if not self.fc or self.fc[1] != 1:
                            break
                        time.sleep(0.05)
                    if seen5:
                        self.get_logger().info('  ✓ 飞控已受理起飞（cmd1→5）')
                    elif self.fc and self.fc[1] == 1:
                        miss_cnt += 1
                        self.get_logger().warn(
                            '  ✗ 1.5s 内没看到 cmd1=5（现在 %s）'
                            '→ 起飞命令没送到飞控（停在 IMU 之前），重发'
                            % (getattr(self, '_cmd1_last', '?'),))
                        # ★ 2026-10-16 实测：这个状态是"粘住"的，重发根本救不回来。
                        #   13:20:13 / 13:20:53 两轮共 6 次尝试全被拒（cmd1 全程 4）；
                        #   拨一次 CH6 低位→高位后，13:23:29 第一次起飞 171ms 内就被受理。
                        #   所以别让操作手去猜"是不是动力不足"——直接告诉他怎么复位。
                        if miss_cnt >= 2:
                            self.get_logger().error(
                                '⚠ 连续 %d 次起飞命令未被飞控受理（cmd1 一直 4）——'
                                '重发救不回来。请拨一次 CH6 低位停 1~2 秒再回高位复位，'
                                '然后重新起飞' % miss_cnt)
                    t_try = time.monotonic()
                    while time.monotonic() - t_try < 5.0:
                        if self.abort_requested:
                            aborted = True
                            break
                        if self.fc and self.fc[1] == 1 and self._fresh_airborne():
                            got_air = True
                            break
                        if not self.fc or self.fc[1] != 1:
                            self.get_logger().error(
                                '✗ 起飞 #%d 过程中飞控上锁（unlock=%s，cmd1=%s）→ 中止'
                                % (attempt, self.fc[1] if self.fc else None,
                                   getattr(self, '_cmd1_last', '?')))
                            aborted = True
                            break
                        # cmd1==6 同样只看"持续"：见上面的说明。
                        # 真在降落的表现是 h 一直往下掉 + unlock 变 0，那两个判据已经在上面了。
                        # ★ 2026-10-16：命令没被受理时别再等满 5 秒 —— 地面上差速正以
                        #   约 3.7%/s 累积，多待一秒就多一分"歪着窜出去"的风险。
                        if (not seen5) and self.height is not None and self.height < 10:
                            break
                        time.sleep(0.2)
                    if aborted or got_air:
                        break
                    self.get_logger().warn(
                        '  #%d 没离地（h=%s，cmd1=%s%s）→ 重发'
                        % (attempt, self.height, getattr(self, '_cmd1_last', '?'),
                           '' if seen5 else '，命令未受理'))
                if aborted:
                    self.get_logger().error('✗ 起飞中断 → 任务中止（不做任何动作，交操作手）')
                    return
                if not got_air:
                    self.get_logger().error('✗ 三次都没离地 → 任务中止（不做任何动作，交操作手）')
                    if miss_cnt >= 2:
                        self.get_logger().error(
                            '⚠ 其中 %d 次是"命令未被飞控受理"（cmd1 一直 4）→ '
                            '拨一次 CH6 低位停 1~2 秒再回高位复位，再重新起飞' % miss_cnt)
                    return
                # 10/14 路径：等核心原生起飞到目标附近，再把 C++ 激光 Z PID
                # 叠加到仍由核心融合高度闭环保持的机体上。
                if not self._wait_takeoff_release(
                        20.0, TAKEOFF_HEIGHT_HOLD_S):
                    self._stop_pos_hold()
                    self._set_active(3)
                    return
                self.get_logger().info(
                    '核心起飞越过%.0fcm（h=%s，上限%.0f）→ 交给C++补到%dcm'
                    % (TAKEOFF_RELEASE_MIN_CM, self.height,
                       TAKEOFF_RELEASE_MAX_CM, h_goto))

            # ---- 交棒：pos_hold → 程控激光 Z + waypoint_nav ----
            if self.abort_requested or not self.fc or self.fc[1] != 1 \
                    or getattr(self, '_cmd1_last', 4) == 6:
                self.get_logger().error(
                    '✗ 交棒前检测到自降/上锁（unlock=%s cmd1=%s abort=%s）'
                    '→ 中止，不启动 waypoint_nav'
                    % (self.fc[1] if self.fc else None,
                       getattr(self, '_cmd1_last', '?'), self.abort_requested))
                self._stop_pos_hold()
                self._set_active(3)
                return
            if not self._waypoint_nav_running():
                self.get_logger().error('预加载的waypoint_nav已退出，任务中止')
                self._stop_pos_hold()
                self._set_active(3)
                return
            self._stop_pos_hold()
            self._set_active(3)
            # 历史复现的关键：不启用 BVA 激光 Z 外环。只启动 C++ Z PID，
            # 让它输出到 /target_velocity，与核心融合高度环同时存在。
            self._set_laser_z(False, n=2)
            self._set_active(2, n=10)
            self.get_logger().info(
                '已停 pos_hold；mode2 历史复现：C++激光Z + 核心融合高度同时工作'
                '（/active_controller=2，/laser_z_enable=0）')
            # ★ 2026-10-14 深夜（现场要求）：**第一段（悬停点→A进-0，90cm）也走"直线逻辑"**。
            #   现场："起飞点悬停到第一个二维码还是摆，看看有没有办法也做成直线逻辑"。
            #   那一段是最长的直线、又是位置环刚接管的第一段，闭环追滞后的雷达位置最明显。
            #   所以接管的同时就打开过路模式（恒速 + 固定方向、不用位置反馈），
            #   到 A进-0（航点序号 1）再关掉，之后的扫描段恢复闭环。
            # ★ 2026-10-15：**重新打开第一段的"开环直线"**（方向锁死、恒速、不用位置反馈）。
            #   背景：2026-10-14 深夜以为它导致"不往前飞"，实际真凶是 `flight_console`
            #   抢发 `/target_position`（已修：任务开始自动停它）。现场要求
            #   "从起飞悬停的地方往第一个二维码绝不能再抖" —— 而直线测试之所以不抖，
            #   就是它**开环发速度、不追滞后 0.35~0.5s 的雷达位置**。
            #   直线前馈现在保持到近段18cm，并叠加受限横向纠偏；最后18cm
            #   才交回位置环收口，避免剩余大半距离突然掉到低速闭环。
            # ★ 2026-10-15（第三次打开）：第一段"斜着直飞过去"= 纯开环（= 直线测试那套）。
            #   前两次"不前进"的真因已找到并修掉：
            #     · `_nb_apply` 里 `float(m.data[3])` 取 Bool 第 4 字节 → 开关一到回调崩
            #       → BVA 整个挂掉、/target_velocity 断流 → 飞机停住（真凶）；
            #     · 一行 `vmag = self.vff` 覆盖了"等方向"时的 0；
            #     · 锁方向时用的是上一轮残留目标。
            #   现在：方向在**地图系**锁死（所以航向有小偏差也不会把路走弯），
            #   恒速 13~20cm/s 直飞；近了（<35cm）自动交回位置环收口进 8cm 容差。
            nb0 = Bool()
            nb0.data = True
            for _ in range(5):
                self.pub_nb.publish(nb0)
                time.sleep(0.05)
            self.get_logger().info(
                '第一段（→A进-0）：锁定方向直线前馈 + 限幅横向纠偏'
                '（BVA_FF=1，近18cm交回闭环）')
            act = UInt8()
            act.data = 2
            n = len(self.labels)
            # ★ 2026-10-07 踩坑：这里原来用 time.time()，而板子没有 RTC，
            #   chrony 对时时会把系统时间**跳**一大步（实测中途跳了 2.5 小时）
            #   → 判据立刻成立 → 任务在半空自己发降落（现场："怎么自动下降了"）。
            #   改用单调时钟，对时/改时间都不影响。
            t0 = time.monotonic()
            t_act = time.monotonic()
            # ★ 2026-10-14 深夜：扫描段的最后一个航点一到，就打开"过路模式"。
            #   后面的"绕行 → 绕行 → land"只要匀速飞过去（现场要求），
            #   不要在过路点刹车收口再加速（那就是一顿一顿的来源）。
            idx_transit = next((i for i, nm in enumerate(self.labels)
                                if nm.startswith('绕行')), len(self.labels) - 1)
            nb_sent = False
            first_off = False
            while time.monotonic() - t0 < 420:
                if self.abort_requested or self.laser_z_fault:
                    self.get_logger().error(
                        '激光Z/飞控故障导致航线中止：abort=%s lz_fault=%s'
                        % (self.abort_requested, self.laser_z_fault))
                    break
                # 等"已到达"索引走到最后一个点（land）——这样才会真的飞去降落点
                if self.reached_idx is not None and self.reached_idx >= n - 1:
                    break
                # 第一段走完（A进-0，序号 1）→ 关闭“不刹车”标志。
                # BVA 的直线前馈仍由 BVA_FF=1 对每条扫描段逐段生效。
                if (not first_off and self.reached_idx is not None
                        and self.reached_idx >= 1):
                    nb1 = Bool()
                    nb1.data = False
                    for _ in range(5):
                        self.pub_nb.publish(nb1)
                        time.sleep(0.05)
                    first_off = True
                    self.get_logger().info(
                        '已到 A进-0 → 关闭过路不刹车；扫描段继续逐段直线前馈+横向纠偏')
                if (not nb_sent and self.reached_idx is not None
                        and self.reached_idx >= idx_transit - 1):
                    # ★ 2026-10-14 深夜（现场确认）：**过路逻辑保留** ——
                    #   只有"到第一个二维码那一段"回退成航点闭环（见上面 nb0），
                    #   扫完→绕行→降落点这段仍走恒速直穿（不刹车）。
                    nb = Bool()
                    nb.data = True
                    for _ in range(5):
                        self.pub_nb.publish(nb)
                        time.sleep(0.05)
                    nb_sent = True
                    self.get_logger().info(
                        '扫描完成（到达第 %d 个航点）→ 打开过路模式：匀速直穿、不刹车'
                        % (self.reached_idx + 1))
                # 每 2 秒补发 /active_controller=2：谁晚订阅都能被叫醒（见上面的说明）
                if time.monotonic() - t_act >= 2.0:
                    self.pub_act.publish(act)
                    t_act = time.monotonic()
                time.sleep(1.0)
            if self.abort_requested or self.laser_z_fault:
                self._set_laser_z(False)
                self._set_active(3)
                return
            self.get_logger().info('航线结束，发降落')
            self._set_laser_z(False)
            self._set_active(3)
            self._cmd(2)
            # ★ 2026-10-14 深夜：发完降落**立刻**把位置环切到 3（持续发 {0,0,0,0}）。
            #   原来要等 h≤25cm 才关环（实测 16:34 那轮一直开到 3cm）→ 下降段还在发
            #   xy 修正，现场就是"任务结束了还一顿一顿"。切 3 而不是 0：速度流保持
            #   新鲜（防"外部速度源超时"），但不再干涉飞控的降落动作。
            # ★ 2026-10-07 现场踩坑："降落时失控"——飞机落地后位置环**还在发指令**
            #   （记录里能看到 h=13cm、mode=1、unlock=0 时 yaw 仍被命令 10deg/s，
            #    机体在地上转圈）。所以落定后立刻把位置环关掉（/active_controller=0
            #   会让 position_pid_controller 停止发布速度）。
            # ★ 2026-10-07(晚)：不再盲目 sleep 6s —— 改成等 /height ≤ 25cm 就立刻关
            #   （最多等 15s）。降得慢不会在半空关环，降得快不会在地上多磨几秒。
            # ★ 2026-10-16：降落段顺带把起飞段挂的 pos_hold 停掉（它过 60cm 会
            #   发水平保持，active=3 时虽然不会被注入飞控，但没必要留着）。
            self._stop_pos_hold()
            self._set_active(3)
            h_land = self._wait_landed(timeout=15.0, thr=25)
            stops = UInt8()
            stops.data = 3          # 落地后保持"发 0 速度"而不是静默（见 _set_active 说明）
            for _ in range(10):
                self.pub_act.publish(stops)
                time.sleep(0.05)
            self.get_logger().info(
                '降落段已停位置环（active_controller=3，只发 0），落地时高度 %s cm'
                % ('未知' if h_land is None else '%.0f' % h_land))
            time.sleep(3)
            self.get_logger().info('发锁定')
            self._cmd(4)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error('任务异常: %s' % e)
        finally:
            # 无论正常结束还是异常退出，都确保位置环不再输出控制量（防止落地后还在发指令）。
            # ★ 2026-10-14 深夜：发 3 而不是 0 —— 效果一样是"输出全 0"，但 0x31 流不会断，
            #   避免 IMU 核心板因"外部速度源超时"自己降落。
            try:
                self._set_laser_z(False, n=2)
            except Exception:                 # noqa: BLE001
                pass
            try:
                stops = UInt8()
                stops.data = 3
                for _ in range(10):
                    self.pub_act.publish(stops)
                    time.sleep(0.05)
            except Exception:                 # noqa: BLE001
                pass
            self._qr_stop()
            self._save_logs()
            try:
                nb = Bool()
                nb.data = False        # 关掉"过路模式"，下一轮从正常前馈开始
                for _ in range(5):
                    self.pub_nb.publish(nb)
                    time.sleep(0.05)
            except Exception:  # noqa: BLE001
                pass
            # ★ 2026-10-19：释放单主控锁，让悬停测试/下一轮任务能接手。
            if self._owner_fd is not None:
                try:
                    fcntl.flock(self._owner_fd, fcntl.LOCK_UN)
                    self._owner_fd.close()
                except Exception:          # noqa: BLE001
                    pass
                self._owner_fd = None


def run_check():
    """离线自检（不连飞控/不起 ROS）: 打印 12 个货位、参考货号、机体航点。"""
    pts = plan_points('')
    print('单板双面遍历盘点 · 干跑（未连飞控，坐标=以起飞点为原点、+x 为起飞时机头）')
    print('起飞点 场地(%.0f,%.0f) | 降落点 场地(%.0f,%.0f) | 相机前伸 %.1fcm 低 %.1fcm '
          '| 观察距板面 %.1fcm'
          % (TAKEOFF_FIELD + LANDING_FIELD + (CAM_FWD, CAM_DOWN, OBS_DIST)))
    print('%-8s %-8s %-8s %-8s %-8s' % ('货位', '机体x', '机体y', '机体z', '参考货号'))
    for label, fx, fy, fz in pts:
        x, y = to_flight(fx, fy, 0.0)
        print('%-8s %-8.1f %-8.1f %-8.0f %-8s' % (label, x, y, fz, reference_of(label)))
    lx, ly = to_flight(LANDING_FIELD[0], LANDING_FIELD[1], 0.0)
    print('%-8s %-8.1f %-8.1f %-8.0f %-8s' % ('land', lx, ly, 120, '-'))
    n_slot = sum(1 for p in pts
                 if len(p[0]) == 2 and p[0][0] in 'AB' and p[0][1].isdigit())
    print('顺序: 起飞 → %s → 降落；共 %d 个货位 + %d 个过渡点'
          % (' → '.join(p[0] for p in pts), n_slot, len(pts) - n_slot))


def main():
    import argparse
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument('--check', action='store_true',
                    help='离线干跑：只打印货位/参考货号/机体坐标，不起 ROS')
    args, _ = ap.parse_known_args()
    if args.check:
        run_check()
        return
    rclpy.init()
    node = Mission()
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
