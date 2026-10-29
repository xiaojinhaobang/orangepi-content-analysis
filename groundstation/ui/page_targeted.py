# -*- coding: utf-8 -*-
"""定点盘点页(题目要求2):
操作员输入目标编号 -> 地面站按遍历盘点结果定位该编号坐标并规划航线 ->
无人机飞抵后扫该货位的二维码(内容即目标编号)上报 -> 传回编号+坐标+高度+时间戳。

题目第二问的定点盘点只扫**目标那一个**码, 所以上报是"确认到达", 不是
"决定目标": 目标以输入框为准(留空则退回随机模式, 由上报编号决定)。
布局: 左=简化航线图(大), 右=二维码显示区 + 目标/阶段/识别传回。"""
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget,
)

import layout
from .page_route import PageRoute

_QR_CACHE = {}      # 编号 -> QPixmap (进程级缓存, 避免重复生成)


def _make_qr_pixmap(data):
    """用 qrcode 生成二维码图; 失败返回 None"""
    try:
        import qrcode
        qr = qrcode.QRCode(box_size=6, border=2)
        qr.add_data(str(data))
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white").convert("RGB")
        b = img.tobytes("raw", "RGB")
        qimg = QImage(b, img.width, img.height, img.width * 3, QImage.Format_RGB888)
        return QPixmap.fromImage(qimg)
    except Exception:
        return None


class PageTargeted(QWidget):
    go_traverse = Signal()              # 请求切回遍历盘点页
    start_targeted = Signal(object)     # 请求开始定点盘点, 参数: int 编号 / None 随机

    def __init__(self, inventory, drone_state, parent=None):
        super().__init__(parent)
        self.inv = inventory
        self.state = drone_state
        self._last_event_num = None     # 最近一次识别传回的编号(EVENT 到达)
        self._target_num = None         # 手动指定的目标编号(None=随机模式)

        # ---- 第 0 行: 标题 + 切换到遍历盘点 ----
        top = QHBoxLayout()
        title = QLabel("定点盘点")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #333;")
        top.addWidget(title)
        top.addStretch(1)
        self.btn_go_traverse = QPushButton("← 切换到遍历盘点")
        self.btn_go_traverse.setStyleSheet("font-size: 14px; padding: 4px 14px;")
        top.addWidget(self.btn_go_traverse)
        self.btn_go_traverse.clicked.connect(self.go_traverse.emit)

        # ---- 第 1 行: 目标输入(输入什么就是什么, 留空随机) ----
        inp = QHBoxLayout()
        inp.addWidget(QLabel("目标编号:"))
        self.edit_num = QLineEdit()
        self.edit_num.setPlaceholderText("1~24 · 空=随机")
        self.edit_num.setFixedWidth(110)
        self.edit_num.setMaxLength(3)
        self.btn_start = QPushButton("开始定点盘点")
        self.btn_start.setStyleSheet("font-size: 14px; padding: 4px 14px;")
        self.lbl_inp_hint = QLabel("输入编号后按遍历结果规划航线; 留空随机抽二维码")
        self.lbl_inp_hint.setStyleSheet("font-size: 12px; color: #888;")
        inp.addWidget(self.edit_num)
        inp.addWidget(self.btn_start)
        inp.addSpacing(8)
        inp.addWidget(self.lbl_inp_hint)
        inp.addStretch(1)

        # ---- 主体: 左=航线图(大) | 右=二维码区+目标/阶段/传回 ----
        self.route_view = PageRoute()

        self.g_qr = QGroupBox("二维码")
        qv = QVBoxLayout(self.g_qr)
        self.lbl_qr = QLabel("等待识别二维码…")
        self.lbl_qr.setAlignment(Qt.AlignCenter)
        self.lbl_qr.setMinimumSize(170, 170)
        self.lbl_qr.setStyleSheet(
            "font-size: 14px; color: #888; background: #fafafa;"
            "border: 1px dashed #ccc; border-radius: 4px;")
        qv.addWidget(self.lbl_qr, 1)

        self.lbl_target = QLabel("等待识别目标货物…")
        self.lbl_target.setAlignment(Qt.AlignCenter)
        self.lbl_target.setStyleSheet(
            "font-size: 20px; font-weight: bold; color: #d93025; padding: 6px;")
        self.lbl_target.setWordWrap(True)

        self.lbl_stage = QLabel("阶段: 待起飞")
        self.lbl_stage.setAlignment(Qt.AlignCenter)
        self.lbl_stage.setStyleSheet("font-size: 14px; color: #555; padding: 1px;")

        self.lbl_info = QLabel("")
        self.lbl_info.setAlignment(Qt.AlignCenter)
        self.lbl_info.setStyleSheet("font-size: 13px; color: #1a73e8; padding: 1px;")

        right = QVBoxLayout()
        right.setSpacing(6)
        right.addWidget(self.g_qr)
        right.addWidget(self.lbl_target, 1)
        right.addWidget(self.lbl_stage)
        right.addWidget(self.lbl_info)
        right.addStretch(0)

        body = QHBoxLayout()
        body.setSpacing(10)
        body.addWidget(self.route_view, 3)
        body.addLayout(right, 1)

        # ---- 提示行 ----
        self.lbl_hint = QLabel("提示: 无人机识别随机二维码 → 上报编号 → 地面站按遍历盘点结果"
                               "规划航线(起飞→目标→降落) → 飞抵识别传回编号+信息, LED 亮 1s")
        self.lbl_hint.setStyleSheet("font-size: 12px; color: #888;")

        lay = QVBoxLayout(self)
        lay.setSpacing(4)
        lay.addLayout(top)
        lay.addLayout(inp)
        lay.addLayout(body, 1)
        lay.addWidget(self.lbl_hint)

        self.inv.changed.connect(self._refresh)
        self.state.changed.connect(self._refresh)
        self.inv.item_added.connect(self._on_item_added)
        self.btn_start.clicked.connect(self._on_start_clicked)
        self.edit_num.returnPressed.connect(self._on_start_clicked)

    # ---- 目标输入: 输入什么就是什么 ----
    def _on_start_clicked(self):
        text = self.edit_num.text().strip()
        if not text:                            # 留空 -> 随机二维码
            self.lbl_inp_hint.setText("随机抽取二维码")
            self.lbl_inp_hint.setStyleSheet("font-size: 12px; color: #888;")
            self.start_targeted.emit(None)
            return
        if text.isdigit() and 1 <= int(text) <= 24:
            self.lbl_inp_hint.setText("输入编号后按遍历结果规划航线")
            self.lbl_inp_hint.setStyleSheet("font-size: 12px; color: #888;")
            self.start_targeted.emit(int(text))
            return
        self.lbl_inp_hint.setText("请输入 1~24 或留空随机")
        self.lbl_inp_hint.setStyleSheet("font-size: 12px; color: #d93025;")

    # ---- 目标: 手动输入优先, 留空才是随机 ----
    def set_target(self, num):
        """主窗口设定定点目标编号(输入框 / 裁判指定); None = 随机模式。"""
        self._target_num = num
        self._refresh()

    def target_num(self):
        """当前目标编号。

        题目第二问的定点盘点只扫目标那一个码, 所以扫码是"飞到那儿确认",
        不是"决定盘哪个" —— 目标以手动输入为准。留空(随机模式)时没有手动
        目标, 才跟随无人机上报的编号, 这套随机流程仍然保留。
        """
        return self._target_num if self._target_num is not None else self.state.report_num

    def reset_view(self):
        """清空/复位: 库存已清, 目标和最近一次识别传回都作废。

        目标现在由输入框决定、不再被扫码刷新, 所以复位后必须显式清掉,
        否则页面会一直停在过期的目标编号上。
        """
        self._target_num = None
        self._last_event_num = None
        self._refresh()

    # ---- 事件 ----
    def _on_item_added(self, num, _name):
        self._last_event_num = num
        self._refresh()

    # ---- 刷新 ----
    def _refresh(self):
        n = self.target_num()
        # 二维码显示区
        self._update_qr(n)
        # 目标货物: 目标编号 -> 按遍历结果显示坐标
        if n is not None:
            names = self.inv.query_number(n)
            if names:
                self.lbl_target.setText(f"目标货物  编号 {n}  @  坐标 {names[0]}")
                self.lbl_target.setStyleSheet(
                    "font-size: 20px; font-weight: bold; color: #188038; padding: 6px;")
            else:
                self.lbl_target.setText(f"目标编号 {n}  ·  未找到遍历结果(请先完成遍历盘点)")
                self.lbl_target.setStyleSheet(
                    "font-size: 20px; font-weight: bold; color: #1a73e8; padding: 6px;")
        else:
            self.lbl_target.setText("等待识别目标货物…")
            self.lbl_target.setStyleSheet(
                "font-size: 20px; font-weight: bold; color: #d93025; padding: 6px;")

        # 识别传回信息(要求2-③): 编号 + 坐标 + 高度 + 时间戳
        if self._last_event_num is not None:
            rows = self.inv.recent(1)
            if rows and rows[0][0] == self._last_event_num:
                num, name, ts = rows[0]
                face_char, pos = name[0], int(name[1:])
                height = layout.ROW_HEIGHTS[(pos - 1) // layout.COLS]
                self.lbl_info.setText(
                    f"识别传回: 编号 {num} · 坐标 {name} · 高度 {height}cm · 时间戳 {ts}ms")
            else:
                self.lbl_info.setText("")
        else:
            self.lbl_info.setText("")

        # 阶段
        self.lbl_stage.setText(f"阶段: {self._stage_text()}")

    def _update_qr(self, n):
        if n is None:
            self.g_qr.setTitle("二维码")
            self.lbl_qr.setText("等待识别二维码…")
            self.lbl_qr.setPixmap(QPixmap())
            return
        pm = _QR_CACHE.get(n)
        if pm is None:
            pm = _make_qr_pixmap(n)
            if pm is not None:
                _QR_CACHE[n] = pm
        self.g_qr.setTitle(f"二维码 · 目标编号 {n}")
        if pm is not None:
            self.lbl_qr.setText("")
            self.lbl_qr.setPixmap(
                pm.scaled(168, 168, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.lbl_qr.setText(f"编号 {n} 二维码")

    def _stage_text(self):
        st = self.state.state
        n = self.target_num()
        if st == 4:
            return "已落地 · 定点盘点完成"
        if n is not None and self._last_event_num == n:
            return "目标识别成功 · 返航降落"
        if st == 2:
            return "飞往目标货物…"
        return {0: "待起飞", 1: "起飞中", 3: "降落中"}.get(st, f"未知({st})")
