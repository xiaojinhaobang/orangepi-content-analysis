# -*- coding: utf-8 -*-
"""地面站主窗口: 链路选择/连接、主控面板(整合全部功能)、航线图、帧路由、日志区。"""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QPlainTextEdit,
    QPushButton, QSpinBox, QStackedWidget, QVBoxLayout, QWidget,
)

from protocol import (
    TYPE_EVENT, TYPE_STATUS, TYPE_ROUTE, TYPE_REPORT, TYPE_START, FACE_NAMES,
    parse_event, parse_status, parse_route, parse_report,
)
from links import UdpLink, SerialLink
from models import Inventory, DroneState
from led_controller import VirtualLed
from simulator import SimulatorCore
import layout

from .page_control import PageControl, MODE_TRAVERSE, MODE_TARGETED
from .page_targeted import PageTargeted

MODE_SIM = "内置模拟"
MODE_UDP = "UDP (WiFi)"
MODE_SERIAL = "串口 (蓝牙/USB)"


class MainWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("D题 · 立体货架盘点无人机 · 地面站")
        self.resize(1080, 720)

        # ---- 数据模型 ----
        self.inv = Inventory(self)
        self.state = DroneState(self)
        self.led = VirtualLed(self)
        self.link = None
        self.sim = None
        self._route_points = []
        self._sim_cell_map = None       # 遍历盘点获得的货架布局, 定点模式复用
        self._target_num = None         # 定点目标编号(手动输入/随机), None=随机

        # ---- 顶部: 链路控制 ----
        self.cmb_mode = QComboBox()
        self.cmb_mode.addItems([MODE_SIM, MODE_UDP, MODE_SERIAL])
        self.spin_port = QSpinBox()
        self.spin_port.setRange(1024, 65535)
        self.spin_port.setValue(8888)
        self.edit_ip = QLineEdit("127.0.0.1")
        self.edit_ip.setFixedWidth(96)
        self.edit_ip.setToolTip("机载端 IP(本地回路测试填 127.0.0.1)")
        self.edit_com = QLineEdit("COM3")
        self.edit_com.setFixedWidth(80)
        self.edit_baud = QSpinBox()
        self.edit_baud.setRange(9600, 921600)
        self.edit_baud.setValue(115200)
        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self._toggle_connect)

        self.btn_start_mission = QPushButton("▶ 开始任务（遍历盘点）")
        self.btn_start_mission.setStyleSheet("font-weight: bold;")
        self.btn_start_mission.setToolTip(
            "向机载端下发 START 0：起飞 → 扫 B 面 6 格 → 绕白板端部 → 扫 A 面 6 格 → 降落")
        self.btn_start_mission.clicked.connect(self._start_mission)

        cfg = QGroupBox("通信链路")
        row = QHBoxLayout(cfg)
        row.addWidget(QLabel("模式:"))
        row.addWidget(self.cmb_mode)
        row.addWidget(QLabel("UDP 端口:"))
        row.addWidget(self.spin_port)
        row.addWidget(QLabel("机载 IP:"))
        row.addWidget(self.edit_ip)
        row.addWidget(QLabel("串口:"))
        row.addWidget(self.edit_com)
        row.addWidget(self.edit_baud)
        row.addWidget(self.btn_connect)
        row.addWidget(self.btn_start_mission)
        row.addStretch(1)

        # ---- 顶部: 模拟器控制 ----
        self.spin_interval = QSpinBox()
        self.spin_interval.setRange(200, 10000)
        self.spin_interval.setValue(1800)
        self.spin_interval.setSuffix(" ms")
        self.btn_sim = QPushButton("开始模拟")
        self.btn_sim.clicked.connect(self._toggle_sim)

        sim_box = QGroupBox("模拟机载端")
        srow = QHBoxLayout(sim_box)
        srow.addWidget(QLabel("盘点间隔:"))
        srow.addWidget(self.spin_interval)
        srow.addWidget(self.btn_sim)

        top = QHBoxLayout()
        top.addWidget(cfg)
        top.addWidget(sim_box)
        top.addStretch(1)

        # ---- 中间: 两个页面(遍历盘点 / 定点盘点)按钮互切 ----
        self.pages = QStackedWidget()
        self.page_ctrl = PageControl(self.inv, self.state, self.led)
        self.page_targeted = PageTargeted(self.inv, self.state)
        self.pages.addWidget(self.page_ctrl)       # 遍历盘点
        self.pages.addWidget(self.page_targeted)   # 定点盘点

        # ---- 底部: 日志 + 状态栏 ----
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(100)
        self.log_view.setPlaceholderText("日志区")
        self.lbl_link = QLabel("未连接")
        self.lbl_count = QLabel("帧 0 · 错误 0")

        lay = QVBoxLayout(self)
        lay.addLayout(top)
        lay.addWidget(self.pages, 1)
        lay.addWidget(self.log_view)
        sb = QHBoxLayout()
        sb.addWidget(self.lbl_link)
        sb.addStretch(1)
        sb.addWidget(self.lbl_count)
        lay.addLayout(sb)

        # ---- 事件接线 ----
        self.inv.item_added.connect(lambda _n, _name: self.led.blink(1000))
        self.cmb_mode.currentTextChanged.connect(self._on_mode_changed)
        self.page_ctrl.mode_changed.connect(self._on_work_mode)
        self.page_targeted.go_traverse.connect(lambda: self.page_ctrl._set_mode(MODE_TRAVERSE))
        self.page_targeted.start_targeted.connect(self._on_targeted_start)
        self.page_ctrl.cleared.connect(self._on_cleared)
        self.state.changed.connect(self._apply_route)
        self._on_mode_changed(self.cmb_mode.currentText())
        self.log("地面站就绪。选择链路后点“连接”；内置模拟可直接“开始模拟”。")

    # ---------- 链路控制 ----------
    def _on_mode_changed(self, mode):
        self.edit_com.setEnabled(mode == MODE_SERIAL)
        self.edit_baud.setEnabled(mode == MODE_SERIAL)
        self.spin_port.setEnabled(mode == MODE_UDP)
        self.edit_ip.setEnabled(mode == MODE_UDP)

    def _toggle_connect(self):
        if self.link:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        mode = self.cmb_mode.currentText()
        if mode == MODE_UDP:
            self.link = UdpLink(
                listen_port=self.spin_port.value(),
                target=(self.edit_ip.text().strip() or "127.0.0.1",
                        self.spin_port.value()),
                parent=self)
        elif mode == MODE_SERIAL:
            self.link = SerialLink(self.edit_com.text(), self.edit_baud.value(), parent=self)
        else:
            self.link = None
            self.log("内置模拟模式: 不建立真实链路")
            return
        self.link.frame_received.connect(self._on_frame)
        self.link.error.connect(lambda e: self.log(f"[链路错误] {e}"))
        self.link.log.connect(self.log)
        self.link.start()
        self.lbl_link.setText(f"已连接 · {mode}")
        self.btn_connect.setText("断开")
        self._refresh_count()

    def _disconnect(self):
        if self.link:
            self.link.frame_received.disconnect(self._on_frame)
            self.link.stop()
            self.link = None
        self.lbl_link.setText("未连接")
        self.btn_connect.setText("连接")
        self.log("链路已断开")

    # ---------- 工作模式(遍历/定点): 两页按钮互切 ----------
    def _on_work_mode(self, mode):
        self.log(f"工作模式切换: {mode}")
        # ★ 切页不再自动下发 START：切页只是换界面，真机起飞必须由
        #   「▶ 开始任务」按钮显式触发（否则点一下"切回遍历盘点"就会真的起飞）。
        # 只切换页面, 不自动启停模拟(模拟由"开始/停止"按钮手动控制)
        if mode == MODE_TARGETED:
            self.pages.setCurrentWidget(self.page_targeted)
        else:
            self.pages.setCurrentWidget(self.page_ctrl)
        self._apply_route()

    def _start_mission(self):
        """开始任务: 遍历盘点 START 0。

        链路已连接 → 真正下发；未连接 → 只提示（避免误以为已经起飞）。
        机载端（~/diag/mission_node.py）收到 START 0 后自己完成：
        检查解锁 → 归零 → 按当前机头实时生成货架航线 → 起飞 → 逐格上报 → 降落锁定。
        """
        if not self.link:
            self.log("未连接链路: 请先选 UDP(WiFi) 并点『连接』；离线演示请用『开始模拟』")
            return
        self.link.send(TYPE_START, bytes([0]))
        self.log("已下发 START 0 = 单板双面遍历盘点（起飞→B面→换面→A面→降落）")

    def _apply_route(self):
        """定点模式下把规划航线画到定点页航线图(遍历页不显示航线)"""
        if self.page_ctrl.mode != MODE_TARGETED:
            self.page_targeted.route_view.set_route([])
            return
        target = None
        n = self.page_targeted.target_num()      # 手动目标优先, 随机模式跟随上报
        if n is not None:
            names = self.inv.query_number(n)
            if names:
                target = names[0]
        self.page_targeted.route_view.set_route(self._route_points, target_name=target)

    # ---------- 定点目标(手动输入/随机) ----------
    def _on_targeted_start(self, num):
        """定点页"开始定点盘点": 用指定编号(或随机)启动定点模拟。

        指定了编号 -> 目标**立刻**生效: 按遍历结果查坐标、规划航线、显示目标,
        不等无人机扫码(题目第二问的定点盘点只扫目标那一个码, 上报是确认不是决策)。
        留空 -> 随机模式, 目标由无人机上报的编号决定。
        遍历模拟由底部按钮手动控制, 这里只切页 + 重启定点模拟。"""
        self._target_num = num
        if self.page_ctrl.mode != MODE_TARGETED:
            self.page_ctrl._set_mode(MODE_TARGETED)     # 先切页, 否则 _apply_route 不画
        self.page_targeted.set_target(num)
        self.log(f"定点目标: {'随机抽取二维码' if num is None else f'手动输入编号 {num}'}")
        if num is not None:
            self._plan_targeted_route(num)              # 要求2-②: 立刻定位并规划
        if self.sim:
            self._toggle_sim()      # 停旧模拟(可能是遍历/旧定点)
        self._toggle_sim()          # 用新目标启动定点模拟

    def _on_cleared(self):
        """清空/复位: 库存没了, 定点目标和航线一并作废。

        目标现在由输入框决定、不再被扫码刷新, 不清的话定点页会停在过期目标上。
        """
        self._target_num = None
        self._route_points = []
        self.page_targeted.reset_view()
        self._apply_route()

    # ---------- 帧路由 ----------
    def _on_frame(self, typ, payload):
        if typ == TYPE_EVENT:
            d = parse_event(payload)
            self.inv.add_event(d["num"], d["face"], d["pos"], d["ts"])
            self.log(f"盘点事件: 编号 {d['num']}  坐标 "
                     f"{FACE_NAMES[d['face']] if d['face'] < len(FACE_NAMES) else '?'}"
                     f"{d['pos']}")
            # 定点模式下事件到达 = 目标识别成功, 再次按库存校准规划航线。
            # 用**当前目标**而不是事件里的编号: 定点盘点只在目标处扫一次,
            # 两者本来就相同; 用目标是为了万一遍历漏了这格(上报前无坐标、
            # 航线画不出来), 事件补上了这格坐标后还能校准出来。
            if self.page_ctrl.mode == MODE_TARGETED:
                num = self.page_targeted.target_num()
                if num is not None:
                    self._plan_targeted_route(num)
        elif typ == TYPE_STATUS:
            d = parse_status(payload)
            self.state.update(d["state"], d["height"], d["battery"])
            self.log(f"状态: {self.state.state_name} 高度 {d['height']}cm 电量 {d['battery']}%")
        elif typ == TYPE_ROUTE:
            points = parse_route(payload)["points"]
            self.log(f"收到航线: {len(points)} 个航点")
            # 遍历页不画航线图: 遍历航线(30 点蛇形)不进 _route_points, 否则切到
            # 定点页时会被 _apply_route 当成定点航线画出来(一堆无关的线)。
            if self.page_ctrl.mode == MODE_TARGETED:
                self._route_points = points
                self._apply_route()
        elif typ == TYPE_REPORT:
            d = parse_report(payload)
            if d["num"]:
                self.state.set_report(d["num"])
                self.log(f"识别报送: 编号 {d['num']}")
                # 有手动目标时只显示不改航线: 题目第二问只扫目标那一个码,
                # 上报的编号是"确认", 不是"改盘哪个"。随机模式才由上报决定。
                if self._target_num is None:
                    self._plan_targeted_route(d["num"])
        self._refresh_count()

    def _plan_targeted_route(self, num):
        """要求2-②: 识别编号后, 按遍历盘点的结果定位该编号坐标,
        规划定点航线(起飞 -> 目标坐标 -> 降落)并下发到航线图。"""
        names = self.inv.query_number(num)
        if names:
            name = names[0]                     # 如 "D2"
            face_char, pos = name[0], int(name[1:])
            # 绕障航线: 起飞 -> 目标面走廊 -> 目标 -> 回走廊 -> 降落(不穿货架)
            self._route_points = layout.targeted_route(face_char, pos)
            self.log(f"按遍历结果规划定点航线: 起飞 → {name} → 降落")
        else:
            self._route_points = []
            self.log(f"编号 {num} 暂无遍历结果, 无法规划定点航线")
        self._apply_route()

    def _refresh_count(self):
        if self.link:
            p = self.link.parser
            self.lbl_count.setText(f"帧 {p.frame_count} · 错误 {p.error_count}")
        else:
            self.lbl_count.setText("帧 0 · 错误 0")

    # ---------- 模拟器 ----------
    def _toggle_sim(self):
        if self.sim:
            self.sim.stop()
            self.sim = None
            self.btn_sim.setText("开始模拟")
            self.log("模拟已停止")
            return
        if self.link:
            # 真实链路连着时，模拟帧会顺着同一条 UDP 通道发给机载端，
            # 把真实通道污染成"半真半假"。要离线演示必须先断开链路。
            self.log("真实链路已连接：禁止启动模拟（模拟帧会污染真实通道）。"
                     "先点『断开』再开始模拟，或直接用『▶ 开始任务』跑真机。")
            return
        self.sim = SimulatorCore(self)
        self.sim.frame_out.connect(self._on_frame)
        self.sim.finished.connect(self._on_sim_finished)
        mode = "targeted" if self.page_ctrl.mode == MODE_TARGETED else "traverse"
        self.sim.start(interval_ms=self.spin_interval.value(), mode=mode,
                       cell_map=self._sim_cell_map if mode == "targeted" else None,
                       target_num=self._target_num if mode == "targeted" else None)
        if mode == "traverse":
            self._sim_cell_map = self.sim.cell_map      # 保存布局供定点模式复用
        self.btn_sim.setText("停止模拟")
        extra = (f", 目标编号 {self._target_num}" if self._target_num else "")
        self.log(f"开始模拟({mode}), 间隔 {self.spin_interval.value()} ms{extra}"
                 + (" (复用遍历布局)" if mode == "targeted" and self._sim_cell_map else ""))

    def _on_sim_finished(self):
        """模拟播放完毕: 释放模拟器并复位按钮, 否则会一直停在"停止模拟"""
        if self.sim:
            self.sim.stop()
            self.sim = None
        self.btn_sim.setText("开始模拟")
        self.log("模拟完成")

    # ---------- 工具 ----------
    def log(self, msg):
        self.log_view.appendPlainText(msg)
        self.log_view.verticalScrollBar().setValue(
            self.log_view.verticalScrollBar().maximum())

    def closeEvent(self, event):
        if self.link:
            self.link.stop()
        if self.sim:
            self.sim.stop()
        event.accept()
