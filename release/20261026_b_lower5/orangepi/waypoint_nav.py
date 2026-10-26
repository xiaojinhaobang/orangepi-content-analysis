#!/usr/bin/env python3
"""航点执行器：读 YAML 航点表，依次发布 /target_position。

不依赖地图：靠 tf(map -> body_frame) 判断到点。位置源是 SLAM 还是速度积分，本节点不关心。
默认等 /active_controller == 2 才开跑（避免上电就发目标点）。

输出：
  /target_position      Float32MultiArray [x_cm, y_cm, z_cm, yaw_deg]
  /route_current_index  UInt8  当前航点序号
  /route_reached_index  UInt8  刚到达的航点序号
  /waypoint_nav/status  String 人类可读状态（给地面站）
参数：
  file / map_frame / body_frame / start_topic / rate_hz / dry_run
"""
import math
import time

import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Int16, String, UInt8


def quat_to_yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class WaypointNav(Node):
    def __init__(self):
        super().__init__('waypoint_nav')
        self.file = self.declare_parameter('file', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.body_frame = self.declare_parameter('body_frame', 'dr_base').value
        self.rate = self.declare_parameter('rate_hz', 10.0).value
        self.dry_run = self.declare_parameter('dry_run', False).value

        with open(self.file) as fh:
            cfg = yaml.safe_load(fh)
        tol = cfg.get('tolerances', {})
        self.pos_tol = float(tol.get('pos_cm', 15.0))
        self.z_tol = float(tol.get('z_cm', 8.0))
        self.yaw_tol = float(tol.get('yaw_deg', 10.0))
        self.hold_s = float(tol.get('hold_s', 2.0))
        # ★ 2026-10-07：航向卡死保护。实测"位置已到位、只有航向差太多"时
        #   会永久悬停（SLAM 航向在掉头后跳变，位置环以为自己已经转正，
        #   指令 yaw=0，飞机却斜着 → waypoint_nav 一直等 dyaw≤20° 等不到）。
        #   这里给个上限：位置到位但只有航向不满足超过 yaw_timeout 秒 →
        #   打印告警并跳过该点，任务继续（不会永远悬停在半空）。
        self.yaw_timeout_s = float(tol.get('yaw_timeout', 25.0))
        self.yaw_wait_from = None
        self.loop = bool(cfg.get('loop', False))
        self.field = cfg.get('field', {})
        self.obstacles = cfg.get('obstacles', [])
        self.wps = cfg.get('waypoints', [])
        assert self.wps, 'waypoints 为空'

        self.idx = 0
        self.started = False
        self.hold_from = None
        self.height_cm = 0.0
        self.have_height = False
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.pos_pub = self.create_publisher(Float32MultiArray, '/target_position', 10)
        # ★ 2026-10-07：位置环订阅 /route_current_index 用的是 TRANSIENT_LOCAL，
        #   默认的 VOLATILE 发布者与它**不兼容** → 位置环一条都收不到 →
        #   route_current_index_ 永远 = 0 → 起飞模式恒成立 → cmd[3](航向角速度)
        #   被强制 0 → **飞机永远转不了向**（实测：卡在过渡点等 dyaw≤20° 等不到）。
        #   TRANSIENT_LOCAL 的发布者同时满足 VOLATILE 和 TRANSIENT_LOCAL 订阅者，
        #   而且晚加入的订阅者还能立刻拿到当前索引。
        idx_qos = QoSProfile(depth=10,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cur_pub = self.create_publisher(UInt8, '/route_current_index', idx_qos)
        self.rea_pub = self.create_publisher(UInt8, '/route_reached_index', idx_qos)
        self.sta_pub = self.create_publisher(String, '/waypoint_nav/status', 10)

        self.create_subscription(UInt8, '/active_controller', self.on_active, 10)
        self.create_subscription(Int16, '/height', self.on_height, 10)
        self.create_timer(1.0 / self.rate, self.step)

        obs = ' | '.join(o.get('name', '?') for o in self.obstacles)
        self.get_logger().info(
            'waypoints=%d field=%s obstacles=[%s] tol=%.0f/%.0fcm %.0fdeg hold=%.1fs'
            % (len(self.wps), self.field, obs, self.pos_tol, self.z_tol, self.yaw_tol, self.hold_s))

    def on_active(self, msg):
        if msg.data == 2 and not self.started:
            self.started = True
            self.get_logger().info('active_controller=2 -> 开始执行航点')

    def on_height(self, msg):
        # ★ 2026-10-15：**与 mission_node 同款的高度毛刺过滤**。
        #   实测 /height 有毛刺（15483 样本里 h 到 3288cm），而这里原来是生吃
        #   → 毛刺会让"到没到"误判（z 误差瞬间几千 cm）。两道剔除：
        #   超过 500cm 丢掉；单步跳变超过 100cm 丢掉。
        h = float(msg.data)
        if h < 0 or h > 500:
            return
        if self.have_height and abs(h - self.height_cm) > 100:
            return
        self.height_cm = h
        self.have_height = True

    def pose(self):
        try:
            tr = self.buf.lookup_transform(self.map_frame, self.body_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform.translation
        yaw = quat_to_yaw_deg(tr.transform.rotation)
        return (t.x * 100.0, t.y * 100.0, self.height_cm, yaw)

    def step(self):
        if not self.started:
            self.publish_status('waiting for /active_controller == 2')
            return
        if self.idx >= len(self.wps):
            self.publish_status('done (%d waypoints)' % len(self.wps))
            return

        wp = self.wps[self.idx]
        x, y, z, yaw = float(wp['x']), float(wp['y']), float(wp['z']), float(wp.get('yaw', 0.0))
        if not self.dry_run:
            m = Float32MultiArray()
            m.data = [x, y, z, yaw]
            self.pos_pub.publish(m)
        self.cur_pub.publish(UInt8(data=self.idx))
        self.publish_status('-> %s [%d/%d] target=(%.0f,%.0f,%.0f) yaw=%.0f'
                            % (wp.get('name', '?'), self.idx + 1, len(self.wps), x, y, z, yaw))

        cur = self.pose()
        if cur is None:
            self.publish_status('no tf %s->%s' % (self.map_frame, self.body_frame))
            return
        ex, ey, ez, eyaw = x - cur[0], y - cur[1], z - cur[2], yaw - cur[3]
        dyaw = (eyaw + 180.0) % 360.0 - 180.0
        dist = math.hypot(ex, ey)
        # ★ 2026-10-14 深夜：支持**单点容差** tol_cm / ztol_cm。
        #   用途：扫完之后"绕行→降落点"那段只要飞过去，不需要在过路点精确收口
        #   （收口 = 前馈刹到零再加速 = 一顿一顿）。给这些点写 tol_cm: 30，
        #   还差 30cm 就算到达、立刻换下一个目标，飞机保持匀速穿过去。
        ptol = float(wp.get('tol_cm', self.pos_tol))
        ztol = float(wp.get('ztol_cm', self.z_tol))
        # ★ 2026-10-14 深夜：航向也支持单点容差 ytol_deg。
        #   用途：**过渡点/自转点**要求"精准自转"（写 ytol_deg: 3），
        #   而扫描段保持全局 20°（飞过时不需要停准航向，停了反而慢）。
        ytol = float(wp.get('ytol_deg', self.yaw_tol))
        ok = (dist <= ptol and abs(ez) <= ztol and abs(dyaw) <= ytol)

        # ★ 2026-10-07：用单调时钟计时（不用 ROS/系统时钟）——板子无 RTC，
        #   chrony 对时会跳时间，用系统时钟会算出离谱的"已停留时间"。
        now = time.monotonic()
        if ok:
            if self.hold_from is None:
                self.hold_from = now
            held = now - self.hold_from
            # ★ 2026-10-07：支持**单点停留时间** wp.hold（秒）。
            #   现场要求"换面原地转完先停一下，确认转向对、数据稳，再进 B 面"，
            #   所以"过渡-稳定"那个航点单独给 3s，其它点仍用全局 hold_s。
            hold_need = float(wp.get('hold', self.hold_s))
            self.publish_status('reached %s, holding %.1f/%.1fs (d=%.1fcm dz=%.1f dyaw=%.1f)'
                                % (wp.get('name', '?'), held, hold_need, dist, ez, dyaw))
            if held >= hold_need:
                self.get_logger().info('到达航点 %d %s (d=%.1fcm dz=%.1f dyaw=%.1f)'
                                       % (self.idx, wp.get('name', '?'), dist, ez, dyaw))
                self.rea_pub.publish(UInt8(data=self.idx))
                self.idx += 1
                self.hold_from = None
                self.yaw_wait_from = None
                if self.idx >= len(self.wps):
                    if self.loop:
                        self.idx = 0
                    else:
                        self.publish_status('done (%d waypoints)' % len(self.wps))
                        self.get_logger().info('全部航点完成')
        else:
            self.hold_from = None
            # 位置到位、只有航向不满足 → 计时；超时则跳过该点继续任务
            if (dist <= ptol and abs(ez) <= ztol
                    and abs(dyaw) > ytol):
                if self.yaw_wait_from is None:
                    self.yaw_wait_from = now
                waited = now - self.yaw_wait_from
                if waited >= self.yaw_timeout_s:
                    self.get_logger().warn(
                        '航向卡死保护：位置已到位但航向差 %.1f° 超过 %.0fs → '
                        '跳过该点继续（%s）'
                        % (dyaw, self.yaw_timeout_s, wp.get('name', '?')))
                    self.idx += 1
                    self.yaw_wait_from = None
                    if self.idx >= len(self.wps):
                        self.publish_status('done (%d waypoints)' % len(self.wps))
            else:
                self.yaw_wait_from = None
            self.publish_status('flying %s: dx=%.0f dy=%.0f dz=%.0f dyaw=%.0f'
                                % (wp.get('name', '?'), ex, ey, ez, dyaw))

    def publish_status(self, text):
        self.sta_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = WaypointNav()
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
