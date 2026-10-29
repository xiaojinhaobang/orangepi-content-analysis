#!/usr/bin/env python3
"""飞行控制台 —— 点击式应用后端（只用标准库 + rclpy，无需额外依赖）。

浏览器打开 http://<板子IP>:8090 即可操作：
  自检 / 启动链路 / 停止链路 / 起飞 / 降落 / 锁定 / 记录 / 航点表选择

「启动链路」会自动完成：ros2 launch 起链路 → 等 30 秒 → 激活位置环。
除遥控解锁(CH5/CH6)外的初始化都在这一个页面里完成。

用法: python3 flight_console.py [--port 8090]
页面模板: 同目录 console_page.html
"""
import argparse
import json
import math
import os
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import tf2_ros
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import (Float32MultiArray, Int16, Int16MultiArray, String,
                          Bool, UInt8, UInt8MultiArray)

HOME = '/home/orangepi'
DIAG = HOME + '/diag'
ROS_SETUP = '/opt/ros/humble/setup.bash'
WS_SETUP = HOME + '/ros2_ws/install/setup.bash'
# cartographer_ros 在独立工作空间里；不起它 link 会报 package not found
KIAN_SETUP = HOME + '/kian_ws/install/setup.bash'
CARTO_SETUP = HOME + '/cartographer_ws/install/setup.bash'
DOMAIN = '6'
LAUNCH = 'my_launch'
LAUNCH_FILE = 'stage3_simple.launch.py'
DEFAULT_WP = DIAG + '/waypoints_square.yaml'
PAGE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'console_page.html')
LINK_LOG = '/tmp/stage3.log'
REC_LOG = '/tmp/flight_log.out'
# ★ 2026-10-06：记录器自己的路径。控制台"开始记录"以前用 pcount('flight_log.py')>0
#   判"是否在记录"，而 flight-recorder 常驻服务永远在跑一个 flight_log.py
#   → 控制台永远认为自己已在记录，自测从来不写自己的日志；record_stop 还会
#   把常驻服务一起 pkill 掉。改成只认"本次自测写的那个文件路径"。
REC_PATH_FILE = '/tmp/console_rec.path'
PID_FILE = '/tmp/flight_console.pid'
AUTOLINK_FLAG = HOME + '/diag/console_autolink.enable'


def kill_previous_instance():
    """启动时先清掉上一个实例，否则端口被占会直接崩。"""
    try:
        with open(PID_FILE) as fh:
            pid = int(fh.read().strip())
    except Exception:
        return
    if pid == os.getpid():
        return
    try:
        os.kill(pid, signal.SIGKILL)
        time.sleep(1.5)
    except Exception:
        pass


def sh(cmd, timeout=10):
    try:
        p = subprocess.run(['bash', '-lc', cmd], capture_output=True, text=True, timeout=timeout)
        return (p.stdout or '') + (p.stderr or '')
    except Exception as e:
        return 'ERR %s' % e


def pcount(pattern):
    # 首字符包成 [x]，否则 pgrep 会匹配到承载它的那条 bash 命令行（自己）
    pat = '[' + pattern[0] + ']' + pattern[1:]
    out = sh("pgrep -fc '%s' || true" % pat, 5).strip()
    try:
        return int(out.split()[0])
    except Exception:
        return 0


def spawn(target, log):
    """后台起一个 ~/diag 下的 python 脚本（带 ROS 环境），日志写到 log。"""
    cmd = ("export ROS_DOMAIN_ID=%s; source %s; source %s; cd %s; "
           "setsid nohup timeout 7200 python3 -u %s > %s 2>&1 < /dev/null &"
           % (DOMAIN, ROS_SETUP, WS_SETUP, HOME, target, log))
    subprocess.Popen(['bash', '-lc', cmd], stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL)


class Link:
    """ros2 launch / 记录 的子进程管理。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.message = '未启动'

    def running(self):
        return pcount('cartographer_node') > 0

    def start(self, use_route, wp_file, activate=True):
        already = False
        with self.lock:
            if self.running():
                already = True
                self.message = '链路已在运行；直接检查就绪并激活'
            else:
                if BRIDGE is not None:
                    BRIDGE.reset_cache()      # 先把旧数据清掉，避免"刚点就就绪"
                cmd = ("cd %s && export ROS_DOMAIN_ID=%s && source %s && source %s && "
                       "source %s && source %s && setsid nohup timeout 7200 ros2 launch %s %s "
                       "open_loop:=false use_route:=%s use_reanchor:=false "
                       "waypoints_file:=%s > %s 2>&1 < /dev/null &"
                       % (HOME, DOMAIN, ROS_SETUP, KIAN_SETUP, CARTO_SETUP, WS_SETUP,
                          LAUNCH, LAUNCH_FILE,
                          'true' if use_route else 'false', wp_file, LINK_LOG))
                subprocess.Popen(['bash', '-lc', cmd], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                self.message = '启动中…（轮询就绪，通常 ~7 秒）'
        if activate:
            threading.Thread(target=self._late_activate, daemon=True).start()
        return self.message

    def _late_activate(self):
        # 实测 tf/飞控回执/雷达速度约 4 秒齐，这里轮询就绪而不是死等 32 秒。
        # 注意：进程本身要几秒才出现，先等进程、再等数据，两步分开判。
        # 2026-09-28：整段包 try/except —— 这里出过"线程异常退出 → 消息永远停在
        # '启动中…（等进程）'"的假死（实测 18:26）。异常必须落到消息里，别静默。
        try:
            self._late_activate_inner()
        except Exception as e:  # noqa: BLE001
            self.message = '链路就绪检查异常: %s（点「自检」看节点）' % e

    def _late_activate_inner(self):
        t0 = time.time()
        while time.time() - t0 < 15.0 and not self.running():
            self.message = '链路启动中…（等进程）'
            time.sleep(0.5)
        if not self.running():
            self.message = '链路启动异常（进程没起来），请点「自检」'
            return
        while time.time() - t0 < 45.0 and not (BRIDGE is not None and BRIDGE.ready()):
            self.message = '链路启动中…（等飞控回执/雷达/位姿）'
            time.sleep(0.4)
        # 激活位置环：原来 shell 起 `ros2 topic pub` 要 3~4 秒（CLI 启动 + DDS 发现）。
        # 本进程自己就有 /active_controller 的 publisher，直接发快得多。
        if BRIDGE is not None:
            BRIDGE.activate()
        else:
            sh("export ROS_DOMAIN_ID=%s; source %s; source %s; "
               "timeout 1.5 ros2 topic pub -r 10 /active_controller std_msgs/msg/UInt8 "
               "'{data: 2}' >/dev/null 2>&1" % (DOMAIN, ROS_SETUP, WS_SETUP), 20)
        # 只在还在"启动中"时改写消息，避免覆盖自测线程写进去的状态。
        # ★ 2026-09-28：原来写的是 startswith('启动中')，但实际消息是
        #   '链路启动中…（等飞控回执/雷达/位姿）' —— 永远匹配不上，
        #   于是链路 3 秒就真的就绪了，页面上却一直显示"启动中"（实测 26 秒还在）。
        if '启动中' in self.message or '链路已在' in self.message:
            self.message = '链路就绪（%.1f s），位置环已激活' % (time.time() - t0)

    def stop(self):
        with self.lock:
            sh("pkill -f '[s]tage3_simple'; pkill -f '[c]artographer_node'; "
               "pkill -f '[v]elocity_filter.py'; pkill -f '[u]art_to_stm32_node'; "
               "pkill -f '[p]osition_pid_controller'; pkill -f '[d]ead_reckon.py'; "
               "pkill -f '[b]ody_vel_adapter.py'; pkill -f '[w]aypoint_nav.py'; "
               "pkill -f '[m]ission_node.py'", 20)
            time.sleep(2)
            if BRIDGE is not None:
                BRIDGE.reset_cache()
            self.message = '链路已停止'
        return self.message

    # ---------- D 题任务侧（mission_node / gs_bridge） ----------
    def start_mission_node(self):
        """起任务节点：等地面站 START（或本页『跑遍历任务』）后自己跑完整流程。"""
        sh("pkill -f '[m]ission_node.py'", 10)
        time.sleep(0.5)
        spawn('%s/mission_node.py --ros-args -p route_file:=/tmp/route_mission.yaml' % DIAG,
              '/tmp/mission.log')
        time.sleep(1.5)
        self.message = 'mission_node 已启动（等 /gs/start=0）'
        return self.message

    def restart_gs_bridge(self, gs_ip=''):
        """重启地面站桥；填电脑 IP 就立即往上推状态，留空等地面站先发包。"""
        ip = (gs_ip or '').strip()
        sh("pkill -f '[g]s_bridge.py'", 10)
        time.sleep(0.5)
        spawn('%s/gs_bridge.py --ros-args -p gs_ip:=%s' % (DIAG, ip) if ip
              else '%s/gs_bridge.py' % DIAG, '/tmp/gs_bridge.log')
        time.sleep(1.5)
        self.message = ('地面站桥已重启，gs_ip=%s' % ip) if ip else \
            '地面站桥已重启（等地面站先发包再锁定地址）'
        return self.message

    def record_start(self):
        if self.recording():
            return '记录已在运行 → ' + self.rec_path()
        # ★ 2026-10-07：板子无 RTC —— 每次上电时钟回到同一时刻 → 文件名相同 → 旧记录被覆盖。
        #   实测已丢过数据（rec_20261007_191704.log 被同名覆盖）。目标已存在就加 -1/-2。
        _base = '%s/traces/console_%s' % (HOME, time.strftime('%Y%m%d_%H%M%S'))
        path = _base + '.log'
        _k = 1
        while os.path.exists(path):
            path = '%s-%d.log' % (_base, _k)
            _k += 1
        cmd = ("export ROS_DOMAIN_ID=%s; source %s; source %s; cd %s; "
               "setsid nohup timeout 3600 python3 -u %s/flight_log.py 3600 %s "
               "> %s 2>&1 < /dev/null &"
               % (DOMAIN, ROS_SETUP, WS_SETUP, HOME, DIAG, path, REC_LOG))
        subprocess.Popen(['bash', '-lc', cmd], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
        try:
            with open(REC_PATH_FILE, 'w') as fh:
                fh.write(path)
        except Exception:  # noqa: BLE001
            pass
        self.message = '记录中 → ' + path
        return self.message

    def record_stop(self):
        # ★ 只杀本次自测启动的那个 flight_log（按路径匹配），
        #   绝不 pkill '[f]light_log.py' —— 那会把 flight-recorder 常驻服务一起干掉。
        p = self.rec_path()
        if p:
            sh("pkill -f '[%s]%s' || true" % (p[0], p[1:]), 10)
        self.message = '记录已停止'
        return self.message

    def reset_origin(self):
        """只重启积分器（dead_reckon），把当前位姿当原点；不重启整条链路。"""
        sh("bash %s/reset_origin.sh > /tmp/reset_origin.out 2>&1" % DIAG, 30)
        time.sleep(1)
        return '原点已重置到飞机当前位置'

    def load_route(self, wp_file):
        """只重启 waypoint_nav 节点装载航线，不整条链路重启（省 ~6 秒）。"""
        sh("pkill -f '[w]aypoint_nav.py'", 10)
        time.sleep(0.5)
        if not os.path.exists(wp_file):
            return '航点表不存在: %s' % wp_file
        cmd = ("export ROS_DOMAIN_ID=%s; source %s; source %s; cd %s; "
               "setsid nohup timeout 7200 python3 -u %s/waypoint_nav.py --ros-args "
               "-p file:=%s -p body_frame:=dr_base > /tmp/wpn.log 2>&1 < /dev/null &"
               % (DOMAIN, ROS_SETUP, WS_SETUP, HOME, DIAG, wp_file))
        subprocess.Popen(['bash', '-lc', cmd], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
        time.sleep(2)
        n = int(os.path.basename(wp_file).count('yaml'))
        return '已装载航线 %s（%s），等 /active_controller=2 后开跑' % (
            os.path.basename(wp_file), 'OK' if n else '?')

    def hover_test_fc(self, takeoff_h=150, hover_s=20.0):
        """★ 2026-10-07：纯飞控悬停自测（我们的位置环完全不参与）。

        和 hover_test 的唯一区别：**不** set_target/激活我们的环，反而先把它关掉，
        起飞→悬停→降落→锁定全由飞控自己的 0x66[1/2/4] 完成。
        用来单独评价飞控动力参数（ID65 PID ONE POWER）。
        """
        if getattr(self, '_seq', False):
            return '已有自测在跑'
        threading.Thread(target=self._hover_seq_fc,
                         args=(takeoff_h, hover_s), daemon=True).start()
        return ('开始【纯飞控】悬停自测：关我们的位置环 → 起飞 %dcm → 悬停 %.0f 秒 → '
                '自动降落 → 锁定；结束时我们的环保持关闭' % (takeoff_h, hover_s))

    def _hover_seq_fc(self, takeoff_h, hover_s):
        self._seq = True
        try:
            if not self.running():
                self.message = '纯飞控自测中止：链路没在跑'
                return
            if not (BRIDGE and BRIDGE.fc and BRIDGE.fc[1] == 1):
                self.message = '纯飞控自测中止：飞控未解锁（先遥控解锁再点）'
                return
            self.message = '纯飞控 1/5 关掉我们的位置环（/active_controller=0）'
            BRIDGE.loop_off()
            time.sleep(1.0)
            self.record_start()
            self.message = '纯飞控 2/5 起飞 %dcm（飞控自己爬）' % takeoff_h
            BRIDGE.takeoff(takeoff_h)
            t0 = time.time()
            while time.time() - t0 < 25 and (BRIDGE.height or 0) < takeoff_h * 0.85:
                time.sleep(0.3)
            if (BRIDGE.height or 0) < takeoff_h * 0.5:
                self.message = '纯飞控自测中止：没起飞（当前高度 %s cm）' % BRIDGE.height
                return
            self.message = '纯飞控 3/5 悬停 %.0f 秒（观察航向/高度摆动，我们不管）' % hover_s
            time.sleep(hover_s)
            self.message = '纯飞控 4/5 自动降落'
            BRIDGE.mission(2)
            t0 = time.time()
            while time.time() - t0 < 30 and (BRIDGE.height or 99) > 8:
                time.sleep(0.3)
            self.message = '纯飞控 5/5 锁定'
            BRIDGE.mission(4)
            time.sleep(1.0)
            self.record_stop()
            self.message = ('纯飞控自测完成：落地高度 %s cm。我们的位置环仍是关闭状态，'
                            '要继续用我们的环请点「激活位置环」' % BRIDGE.height)
        except Exception as e:  # noqa: BLE001
            self.message = '纯飞控自测异常: %s' % e
        finally:
            self._seq = False

    def hover_test(self, takeoff_h=150, hover_h=118, hover_s=12.0):
        """一键自测：归零 → 起飞 → 降到目标高度 → 悬停 → 自动降落 → 锁定。"""
        if getattr(self, '_seq', False):
            return '已有自测在跑'
        threading.Thread(target=self._hover_seq,
                         args=(takeoff_h, hover_h, hover_s), daemon=True).start()
        return ('开始悬停自测：起飞 %dcm → 降到 %dcm → 悬停 %.0f 秒 → 自动降落'
                % (takeoff_h, hover_h, hover_s))

    def _hover_seq(self, takeoff_h, hover_h, hover_s):
        self._seq = True
        try:
            if not self.running():
                self.message = '自测中止：链路没在跑'
                return
            if not (BRIDGE and BRIDGE.fc and BRIDGE.fc[1] == 1):
                self.message = '自测中止：飞控未解锁（先遥控解锁再点）'
                return
            self.message = '自测 1/6 归零'
            self.reset_origin()
            time.sleep(2)
            self.record_start()
            self.message = '自测 2/6 起飞 %dcm' % takeoff_h
            # ★ 2026-09-30：**起飞前**就把水平位置目标设好并激活位置环。
            #   原来要等爬到 85%(≈127cm) 才 set_target，那段"没人管"的爬升期
            #   实测被带出去 15~20cm 才被拉回（14:36 那场 1.6 秒漂 15cm）。
            #   位置环提前接管后，起飞段也能压住水平漂移。
            pose0 = BRIDGE.pose() or (0.0, 0.0, 0.0)
            BRIDGE.set_target(0.0, 0.0, takeoff_h, pose0[2])
            BRIDGE.set_height(takeoff_h)
            t0 = time.time()
            while time.time() - t0 < 25 and (BRIDGE.height or 0) < takeoff_h * 0.85:
                time.sleep(0.3)
            if (BRIDGE.height or 0) < takeoff_h * 0.5:
                self.message = '自测中止：没起飞（当前高度 %s cm）' % BRIDGE.height
                return
            self.message = '自测 3/6 降到 %dcm' % hover_h
            pose = BRIDGE.pose() or (0.0, 0.0, 0.0)
            BRIDGE.set_target(0.0, 0.0, hover_h, pose[2])
            BRIDGE.set_height(hover_h)
            t0 = time.time()
            while time.time() - t0 < 20 and abs((BRIDGE.height or 0) - hover_h) > 6:
                time.sleep(0.3)
            self.message = '自测 4/6 悬停 %.0f 秒（观察航向/高度摆动）' % hover_s
            time.sleep(hover_s)
            self.message = '自测 5/6 自动降落'
            BRIDGE.mission(2)
            t0 = time.time()
            while time.time() - t0 < 30 and (BRIDGE.height or 99) > 8:
                time.sleep(0.3)
            self.message = '自测 6/6 锁定'
            BRIDGE.mission(4)
            time.sleep(1)
            self.record_stop()
            self.message = '自测完成：落地高度 %s cm，数据在 ~/traces/console_*.log' % BRIDGE.height
        except Exception as e:  # noqa: BLE001
            self.message = '自测异常: %s' % e
        finally:
            self._seq = False

    def recording(self):
        """只认本次自测写的那个文件，别把 flight-recorder 常驻服务算进来。"""
        p = self.rec_path()
        return bool(p) and pcount(p) > 0

    def rec_path(self):
        try:
            with open(REC_PATH_FILE) as fh:
                return fh.read().strip()
        except Exception:  # noqa: BLE001
            return ''


class RosBridge(Node):
    """状态订阅 + 指令发布。"""

    def __init__(self):
        super().__init__('flight_console')
        self.height = None
        self.fc = None
        self.link_len = None
        self.tv = None
        self.wp = ''
        self.scan_t = []
        self._tf_after = 0.0        # 只认这个时刻之后的 tf（停链路时清缓存）
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        # ★ 2026-10-01 修 SIGSEGV：tf2 Buffer 不是线程安全的 —— 原来 HTTP 线程直接
        #   调 pose()→lookup_transform，与 ROS 线程里 TransformListener 写 buffer 并发，
        #   偶发段错误（systemd 日志实测 status=11/SEGV，控制台"莫名消失"就是这个）。
        #   改成：只在 ROS 单线程里查 tf 并缓存，HTTP 线程只读缓存。
        self._pose = None
        self.create_timer(0.05, self._update_pose)      # 20 Hz
        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Int16, '/height', lambda m: setattr(self, 'height', int(m.data)), 5)
        self.create_subscription(UInt8MultiArray, '/fc_status', self._fc, 5)
        self.create_subscription(Int16, '/fc_link_len',
                                 lambda m: setattr(self, 'link_len', int(m.data)), 5)
        self.create_subscription(Float32MultiArray, '/target_velocity', self._tv, 5)
        self.create_subscription(String, '/waypoint_nav/status',
                                 lambda m: setattr(self, 'wp', m.data), 5)
        self.create_subscription(LaserScan, '/scan',
                                 lambda m: self.scan_t.append(time.time()), q)
        self.pub_cmd = self.create_publisher(Int16MultiArray, '/mission_cmd', 10)
        self.pub_act = self.create_publisher(UInt8, '/active_controller', 10)
        self.pub_laser = self.create_publisher(Bool, '/laser/trigger', 10)
        self.pub_pos = self.create_publisher(Float32MultiArray, '/target_position', 10)
        self.pub_gsstart = self.create_publisher(Int16, '/gs/start', 10)

    def _fc(self, m):
        if len(m.data) >= 2:
            self.fc = (int(m.data[0]), int(m.data[1]))

    def _tv(self, m):
        if len(m.data) >= 4:
            self.tv = [round(float(v), 1) for v in m.data[:4]]

    def _update_pose(self):
        """只在 ROS 线程里跑（SingleThreadedExecutor）→ 避免 tf buffer 并发访问。"""
        try:
            tr = self.buf.lookup_transform('map', 'dr_base', rclpy.time.Time())
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            if stamp and stamp < self._tf_after:
                self._pose = None    # 停链路前留下的旧 tf，不算"就绪"
                return
            q = tr.transform.rotation
            yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            self._pose = (round(tr.transform.translation.x * 100, 1),
                          round(tr.transform.translation.y * 100, 1), round(yaw, 1))
        except Exception:
            pass

    def pose(self):
        """HTTP 线程只读缓存（真正的 tf 查询在 ROS 线程里做）。"""
        return self._pose

    def reset_cache(self):
        """停链路时清缓存：否则 ready() 拿旧值骗人（实测 stop 后 3 秒内仍报就绪）。"""
        self.height = None
        self.fc = None
        self.link_len = None
        self.tv = None
        self.wp = ''
        self.scan_t = []
        self._pose = None
        self._tf_after = time.time()

    def scan_hz(self):
        self.scan_t = [v for v in self.scan_t if v > time.time() - 3.0]
        return round(len(self.scan_t) / 3.0, 1)

    def ready(self):
        """链路就绪判据：飞控回执 + 雷达速度 + 位姿 tf 三者都到。"""
        return (self.link_len is not None and self.scan_hz() > 0.0
                and self.pose() is not None)

    def takeoff(self, h):
        """0x66[1,h] 起飞 —— 只发 1 帧。

        407 侧收到一帧后会把 mission_tk_req 置 1，然后每毫秒重试
        OneKey_Takeoff() 直到被接受（AnoDTRasp.c:68-77），所以单帧就够。
        实测 2026-10-11：连发 6 帧会让 IMU 板的任务步骤 (cmd_fun 5→6)
        被反复推进，爬到目标高度后直接跳到"降落" —— 飞机自己落下。
        不要为了"保险"再加发数。
        """
        m = Int16MultiArray()
        m.data = [1, int(h)]
        self.pub_cmd.publish(m)
        return '已发起飞 %dcm（单帧；看高度是否上升，没动就再点一次）' % h

    def mission(self, sub):
        m = Int16MultiArray()
        m.data = [int(sub), 0]
        self.pub_cmd.publish(m)          # 单帧，407 侧会重试到接受
        return {2: '已发降落', 3: '已发返航', 4: '已发锁定'}.get(sub, '已发 %d' % sub)

    def set_height(self, h):
        """0x66[1,h]：飞行中改变高度（也用于起飞）。飞控侧是 OneKey_Takeoff(h)。

        同样只发 1 帧 —— 发多帧等于连续多次调用 OneKey_Takeoff，
        会把 IMU 板的任务步骤推快（见 takeoff 的说明）。
        """
        m = Int16MultiArray()
        m.data = [1, int(h)]
        self.pub_cmd.publish(m)
        return '已发 0x66[1] 改高度 → %d cm（看高度读数是否跟随）' % int(h)

    def laser_pulse(self, ms=500):
        """激光脉冲：识别到码后打点用（当前只有话题，GPIO 驱动待落地）。"""
        on = Bool()
        on.data = True
        off = Bool()
        off.data = False
        self.pub_laser.publish(on)
        time.sleep(ms / 1000.0)
        self.pub_laser.publish(off)
        return '已发激光脉冲 %dms' % ms

    def activate(self):
        m = UInt8()
        m.data = 2
        for _ in range(20):
            self.pub_act.publish(m)
            time.sleep(0.05)
        return '已激活位置环 (/active_controller=2)'

    def loop_off(self):
        """★ 2026-10-07：纯飞控模式——发 /active_controller=0，我们的位置环完全退出。

        依据 pid_controller.cpp:211 — data==2 激活；3=Drone Stop（发 0 速度）；
        其他值（含 0）→ is_active_controller_=false，位置环不再发布任何速度指令，
        飞机完全交给飞控自己（这正是 10-01 测 963→3.7° 的那种配置）。
        """
        m = UInt8()
        m.data = 0
        for _ in range(20):
            self.pub_act.publish(m)
            time.sleep(0.05)
        return '已发 /active_controller=0（我们的位置环退出）'

    def mission_start(self, mode=0):
        """D 题任务触发：和地面站按的『开始任务』同一条通路（/gs/start）。

        mode=0 单板双面遍历盘点：起飞 → B 面 6 格 → 绕白板端部 → A 面 6 格 → 降落。
        """
        m = Int16()
        m.data = int(mode)
        for _ in range(4):
            self.pub_gsstart.publish(m)
            time.sleep(0.1)
        if mode != 0:
            return '已发 /gs/start=%d' % mode
        return ('已发 /gs/start=0：遍历任务开始（机载端会自己检查解锁→归零→生成航线→'
                '起飞→逐格上报→降落锁定；先确认已遥控解锁）')

    def traverse(self, subset=''):
        """★ 一键遍历任务（2026-10-06）：把原来"一条条 SSH 命令"的流程固化成一个按钮。

        顺序：
          1) 查链路关键节点（缺哪个就报出来，让操作手点「自检」/等链路起齐）
          2) 查飞控：必须 **模式2（定点）+ 已解锁** —— 否则直接拒绝，
             避免误发 0x31/0x66（原来这步靠对话确认，容易漏）
          3) 用 subset 重起 mission_node（例 B1,B2,B3 / B1,B2,B3,B4,B5,B6；空=整块 12 格）
          4) 等 6 秒等它打印"就绪" → 发 /gs/start=0
        """
        need = ['velocity_filter.py', 'cartographer_node', 'uart_to_stm32_node',
                'position_pid_controller', 'dead_reckon.py', 'body_vel_adapter.py']
        miss = [n for n in need if pcount(n) == 0]
        if miss:
            self.message = '链路没就绪（缺 %s）→ 等链路起齐或点「自检」' % ','.join(miss)
            return self.message
        steps = ['链路 6 节点 ✓']
        # 看门狗（现在不随控制台自启，这里顺手拉起来：节点掉了它会补）
        if pcount('watchdog_nodes.sh') == 0:
            spawn('bash %s/watchdog_nodes.sh' % DIAG, '/tmp/watchdog.log')
            steps.append('看门狗已起')
        else:
            steps.append('看门狗在跑')
        fc = BRIDGE.fc if BRIDGE else None
        if not fc or fc[1] != 1:
            self.message = '飞控未解锁（fc=%s）→ 先 CH5 中档 + 遥控解锁' % (fc,)
            return self.message
        if fc[0] != 2:
            self.message = '飞控模式=%s（要模式2/定点）→ CH5 打到中档' % fc[0]
            return self.message
        steps.append('飞控 模式2+已解锁 ✓')

        subset = (subset or '').strip()
        sh("pkill -f '[m]ission_node.py'", 10)
        time.sleep(0.8)
        args = '--ros-args -p route_file:=/tmp/route_mission.yaml'
        if subset:
            args += ' -p subset:=%s' % subset
        spawn('%s/mission_node.py %s' % (DIAG, args), '/tmp/mission.log')
        time.sleep(6)                       # 等它打印"mission_node 就绪"
        m = Int16()
        m.data = 0
        for _ in range(4):
            self.pub_gsstart.publish(m)
            time.sleep(0.1)
        steps.append('任务节点已起并触发')
        self.message = '已启动遍历任务（%s）｜%s｜流程：归零→起飞→逐格停靠识别→绕行→降落→锁定' \
            % (subset or '整块 12 格', ' → '.join(steps))
        return self.message

    def set_target(self, x, y, z, yaw_deg):
        """发 /target_position 并激活位置环。"""
        m = Float32MultiArray()
        m.data = [float(x), float(y), float(z), float(yaw_deg)]
        for _ in range(8):
            self.pub_pos.publish(m)
            time.sleep(0.08)
        self.activate()
        return '目标已设 (%.0f, %.0f, %.0f) yaw=%.0f' % (x, y, z, yaw_deg)


LINK = Link()
BRIDGE = None


def list_waypoints():
    try:
        return sorted(f for f in os.listdir(DIAG) if f.endswith('.yaml'))
    except Exception:
        return []


def preflight():
    c = []
    c.append(['香橙派', 'ok', sh('hostname', 5).strip()])
    hz = BRIDGE.scan_hz() if BRIDGE else 0
    c.append(['雷达 /scan', 'ok' if hz >= 8 else 'bad', '%.1f Hz' % hz])
    n_uart = pcount('uart_to_stm32_node')
    c.append(['飞控串口节点', 'ok' if n_uart else 'bad', '%d 进程' % n_uart])
    n_carto = pcount('cartographer_node')
    c.append(['建图节点', 'ok' if n_carto else 'bad', '%d 进程' % n_carto])
    n_ad = pcount('body_vel_adapter.py')
    c.append(['坐标旋转节点', 'ok' if n_ad else 'bad', '%d 进程' % n_ad])
    llen = BRIDGE.link_len if BRIDGE else None
    c.append(['飞控回执', 'ok' if llen else 'bad', 'LEN=%s' % llen])
    if BRIDGE and BRIDGE.fc:
        mode, unlock = BRIDGE.fc
        c.append(['飞控模式', 'ok' if mode == 2 else 'warn',
                  'mode=%d %s' % (mode, '定点' if mode == 2 else '（请把 CH5 打中档）')])
        c.append(['解锁状态', 'ok' if unlock == 1 else 'warn',
                  'unlock=%d %s' % (unlock, '已解锁' if unlock else '上锁（需遥控解锁）')])
    else:
        c.append(['飞控状态', 'bad', '收不到 /fc_status'])
    p = BRIDGE.pose() if BRIDGE else None
    c.append(['雷达位姿 tf', 'ok' if p else 'warn',
              ('x=%.1f y=%.1f psi=%.1f' % p) if p else '无 map→dr_base'])
    return c


def status():
    p = BRIDGE.pose() if BRIDGE else None
    ready = bool(BRIDGE and BRIDGE.ready())
    # 显示兜底：真就绪了就别再挂着"启动中"（消息是异步线程写的，会滞后/漏写）
    msg = LINK.message
    if ready and '启动中' in msg:
        msg = '链路就绪（位置环已激活）'
    return {
        'time': time.strftime('%H:%M:%S'),
        'link': LINK.running(),
        'ready': ready,
        'recording': LINK.recording(),
        'message': msg,
        'nodes': {
            'cartographer': pcount('cartographer_node'),
            'velocity_filter': pcount('velocity_filter.py'),
            'dead_reckon': pcount('dead_reckon.py'),
            'uart_to_stm32': pcount('uart_to_stm32_node'),
            'body_vel_adapter': pcount('body_vel_adapter.py'),
            'position_pid': pcount('position_pid_controller'),
            'waypoint_nav': pcount('waypoint_nav.py'),
            'mission_node': pcount('mission_node.py'),
            'gs_bridge': pcount('gs_bridge.py'),
        },
        'scan_hz': BRIDGE.scan_hz() if BRIDGE else 0,
        'height': BRIDGE.height if BRIDGE else None,
        'fc': BRIDGE.fc if BRIDGE else None,
        'link_len': BRIDGE.link_len if BRIDGE else None,
        'pose': p,
        'tv': BRIDGE.tv if BRIDGE else None,
        'wp': BRIDGE.wp if BRIDGE else '',
    }


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _page(self):
        try:
            with open(PAGE_FILE, 'rb') as fh:
                body = fh.read()
        except Exception as e:
            body = ('页面文件缺失: %s' % e).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._page()
        elif self.path == '/api/status':
            self._send(status())
        elif self.path == '/api/waypoints':
            self._send(list_waypoints())
        else:
            self._send({'error': 'not found'}, 404)

    def _read_wp(self):
        n = int(self.headers.get('Content-Length') or 0)
        if not n:
            return DEFAULT_WP
        try:
            f = (json.loads(self.rfile.read(n).decode()) or {}).get('file') or ''
            if f and '/' not in f:
                f = DIAG + '/' + f
            return f if f and os.path.exists(f) else DEFAULT_WP
        except Exception:
            return DEFAULT_WP

    def _body(self):
        n = int(self.headers.get('Content-Length') or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode()) or {}
        except Exception:
            return {}

    def do_POST(self):
        path = self.path.rstrip('/')
        try:
            if path == '/api/start_link':
                msg = LINK.start(False, DEFAULT_WP)
            elif path == '/api/start_link_route':
                msg = LINK.start(True, self._read_wp())
            elif path == '/api/stop_link':
                msg = LINK.stop()
            elif path == '/api/preflight':
                self._send({'message': '自检完成', 'checks': preflight()})
                return
            elif path == '/api/activate':
                msg = BRIDGE.activate()
            elif path == '/api/takeoff120':
                msg = BRIDGE.takeoff(120)
            elif path == '/api/takeoff150':
                msg = BRIDGE.takeoff(150)
            elif path == '/api/set_height':
                msg = BRIDGE.set_height(int(self._body().get('h') or 120))
            elif path == '/api/laser':
                msg = BRIDGE.laser_pulse()
            elif path == '/api/land':
                msg = BRIDGE.mission(2)
            elif path == '/api/lock':
                msg = BRIDGE.mission(4)
            elif path == '/api/record_start':
                msg = LINK.record_start()
            elif path == '/api/record_stop':
                msg = LINK.record_stop()
            elif path == '/api/reset_origin':
                msg = LINK.reset_origin()
            elif path == '/api/load_route':
                msg = LINK.load_route(self._read_wp())
            elif path == '/api/hover_test':
                body = self._body()
                msg = LINK.hover_test(int(body.get('takeoff') or 150),
                                      int(body.get('hover_z') or 118),
                                      float(body.get('hover_s') or 12.0))
            elif path == '/api/hover_test_fc':
                body = self._body()
                msg = LINK.hover_test_fc(int(body.get('takeoff') or 150),
                                         float(body.get('hover_s') or 20.0))
            elif path == '/api/mission_start':
                msg = BRIDGE.mission_start(int(self._body().get('mode') or 0))
            elif path == '/api/traverse':
                # 注意：traverse 定义在 RosBridge（要读 BRIDGE.fc），所以用 BRIDGE 调。
                # 2026-10-06 踩过：写成 LINK.traverse 报 'Link' object has no attribute 'traverse'。
                msg = BRIDGE.traverse(str(self._body().get('subset') or ''))
            elif path == '/api/mission_node_start':
                msg = LINK.start_mission_node()
            elif path == '/api/gs_bridge':
                msg = LINK.restart_gs_bridge(str(self._body().get('gs_ip') or ''))
            else:
                self._send({'error': 'unknown'}, 404)
                return
            self._send({'message': msg})
        except Exception as e:
            self._send({'message': 'ERR %s' % e}, 500)


def main():
    global BRIDGE
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8090)
    ap.add_argument('--gs-ip', default='',
                    help='开机自启时把地面站桥指向这个电脑 IP（留空则等地面站先发包）')
    args = ap.parse_args()
    kill_previous_instance()
    with open(PID_FILE, 'w') as fh:
        fh.write(str(os.getpid()))
    # ★ 2026-10-01：两点改动
    # 1) 用 SingleThreadedExecutor —— tf2 buffer 只有 ROS 线程在碰，HTTP 线程只读缓存
    # 2) 不接管 SIGTERM（rclpy 默认会接管，导致收到 systemd 的 SIGTERM 后不退出，
    #    服务卡在 "deactivating" 直到 90 秒超时被杀）；自己装一个直接退出
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO_SIGNAL_HANDLERS)
    except Exception:  # noqa: BLE001
        rclpy.init()
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
    BRIDGE = RosBridge()
    ex = SingleThreadedExecutor()
    ex.add_node(BRIDGE)
    threading.Thread(target=ex.spin, daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    ip = sh("hostname -I | awk '{print $1}'", 5).strip()
    print('飞行控制台 → http://%s:%d' % (ip, args.port), flush=True)
    # 开机就把链路起热：比赛流程要求"按一下就快速开始"，冷启动的 ~7 秒不该花在起飞那一刻
    if os.path.exists(AUTOLINK_FLAG):
        print('检测到 %s，开机自动起链路（保持热态）' % AUTOLINK_FLAG, flush=True)

        def _autostart():
            time.sleep(3)
            LINK.start(False, DEFAULT_WP)
            time.sleep(2)
            # D 题任务侧一起起：mission_node 等 START；gs_bridge 转发地面站帧
            if pcount('mission_node.py') == 0:
                LINK.start_mission_node()
            if pcount('gs_bridge.py') == 0:
                LINK.restart_gs_bridge(args.gs_ip)
            LINK.message = '自启完成: 链路 + mission_node + 地面站桥 都在跑（等 START/连接）'

        threading.Thread(target=_autostart, daemon=True).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
