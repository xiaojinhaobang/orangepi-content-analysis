# -*- coding: utf-8 -*-
"""遍历盘点页(页面1): 状态行 + 指示灯 / 12 格结果总览(单板双面 A/B) /
   实时盘点+进度 / 货物编号查询->坐标 / 清空·复位。
   右上角按钮切换到定点盘点页。"""
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QProgressBar, QPushButton, QSizePolicy, QVBoxLayout, QWidget,
)

import layout
from protocol import FACE_NAMES, FACE_MAX_POS

MODE_TRAVERSE = "遍历盘点"
MODE_TARGETED = "定点盘点"

# 遍历流程阶段（《D题货架盘点-香橙派方案基线》§2/§6 的任务状态机）
STAGES = ["准备", "起飞", "B面扫描", "换面", "A面扫描", "结果核对", "降落", "完成"]

STYLE_CELL_EMPTY = ("border: 2px solid #ccc; border-radius: 6px;"
                    "font-size: 14px; font-weight: bold;"
                    "background: #fafafa; color: #999;")
STYLE_CELL_FILLED = ("border: 2px solid #1a73e8; border-radius: 6px;"
                     "font-size: 14px; font-weight: bold;"
                     "background: #e8f0fe; color: #1a73e8;")
STYLE_CELL_MATCH = ("border: 2px solid #188038; border-radius: 6px;"
                    "font-size: 14px; font-weight: bold;"
                    "background: #e6f4ea; color: #188038;")
STYLE_CELL_DIFF = ("border: 2px solid #e37400; border-radius: 6px;"
                   "font-size: 14px; font-weight: bold;"
                   "background: #fef7e0; color: #b06000;")

STYLE_STAGE_IDLE = ("font-size: 13px; padding: 4px 6px; border-radius: 6px;"
                    "background: #f1f3f4; color: #80868b;")
STYLE_STAGE_ACTIVE = ("font-size: 13px; font-weight: bold; padding: 4px 6px;"
                      "border-radius: 6px; background: #1a73e8; color: #ffffff;")
STYLE_STAGE_DONE = ("font-size: 13px; padding: 4px 6px; border-radius: 6px;"
                    "background: #e6f4ea; color: #188038;")


class PageControl(QWidget):
    mode_changed = Signal(str)          # MODE_TRAVERSE / MODE_TARGETED
    cleared = Signal()                  # 清空/复位: 盘点结果已作废

    def __init__(self, inventory, drone_state, led, parent=None):
        super().__init__(parent)
        self.inv = inventory
        self.state = drone_state
        self.led = led
        self.mode = MODE_TRAVERSE
        self._grid_labels = {}
        self._stage_labels = []

        root = QVBoxLayout(self)
        root.setSpacing(8)

        # ============ 第 1 行: 状态 + 指示灯 ============
        status_row = QHBoxLayout()
        self.lbl_state = QLabel("状态: --")
        self.lbl_height = QLabel("高度: --")
        self.lbl_batt = QLabel("电量: --")
        self.lbl_report = QLabel("识别报送: --")
        for w in (self.lbl_state, self.lbl_height, self.lbl_batt, self.lbl_report):
            w.setStyleSheet("font-size: 14px; padding: 2px 10px;")
            status_row.addWidget(w)
        status_row.addStretch(1)
        status_row.addWidget(QLabel("指示灯"))
        self.lbl_led = QLabel("●")
        self.lbl_led.setStyleSheet("font-size: 26px; color: #aaa;")
        self.lbl_led.setToolTip("每盘点到一个货物亮 1s")
        status_row.addWidget(self.lbl_led)
        root.addLayout(status_row)

        # ============ 第 2 行: 当前模式 + 切换到定点盘点 ============
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("工作模式:"))
        self.lbl_mode = QLabel(self.mode)
        self.lbl_mode.setStyleSheet("font-size: 14px; font-weight: bold; color: #1a73e8;")
        mode_row.addWidget(self.lbl_mode)
        mode_row.addStretch(1)
        self.btn_go_targeted = QPushButton("切换到定点盘点 →")
        self.btn_go_targeted.setStyleSheet("font-size: 14px; padding: 4px 14px;")
        mode_row.addWidget(self.btn_go_targeted)
        root.addLayout(mode_row)

        # ============ 第 3 行: 遍历流程阶段条（方案基线 §2/§6） ============
        root.addLayout(self._build_stage_bar())

        # ============ 主体: 左(24格总览) | 右(实时+查询) ============
        body = QHBoxLayout()
        body.setSpacing(10)
        body.addWidget(self._build_grid(), 3)
        body.addLayout(self._build_right(), 2)
        root.addLayout(body, 1)

        # ---- 事件接线 ----
        self.inv.changed.connect(self._refresh_grid)
        self.inv.item_added.connect(self._on_item)
        self.state.changed.connect(self._refresh_state)
        self.led.state_changed.connect(self._set_led)
        self.btn_go_targeted.clicked.connect(lambda: self._set_mode(MODE_TARGETED))

    # ---------- 控件构建 ----------
    def _build_stage_bar(self):
        """遍历流程阶段条: 准备 → 起飞 → B面扫描 → 换面 → A面扫描 → 结果核对 → 降落 → 完成

        对应方案基线 §2「整体顺序：起飞 → 扫描 B 面 → 绕过白板端部 → 扫描 A 面 → 指定位置降落」，
        阶段由飞控状态 + 已收到的事件推算（协议里没有独立阶段帧）。
        """
        row = QHBoxLayout()
        row.setSpacing(4)
        row.addWidget(QLabel("流程:"))
        for i, name in enumerate(STAGES):
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(STYLE_STAGE_IDLE)
            row.addWidget(lbl, 1)
            self._stage_labels.append(lbl)
            if i < len(STAGES) - 1:
                arrow = QLabel("›")
                arrow.setStyleSheet("color: #bdc1c6; font-size: 15px;")
                row.addWidget(arrow)
        return row

    def _build_grid(self):
        box = QGroupBox("盘点结果总览（含参考货号对照）")
        outer = QVBoxLayout(box)
        outer.setSpacing(4)
        grid_w = QWidget()
        grid = QGridLayout(grid_w)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(8)
        for face in range(len(FACE_NAMES)):
            for pos in range(1, FACE_MAX_POS + 1):
                lbl = QLabel()
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setMinimumSize(88, 52)
                lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                lbl.setStyleSheet(STYLE_CELL_EMPTY)
                grid.addWidget(lbl, face, pos - 1)
                self._grid_labels[(face, pos)] = lbl
            hdr = QLabel(f"{FACE_NAMES[face]} 面")
            hdr.setStyleSheet("font-size: 14px; font-weight: bold; color: #666;")
            hdr.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(hdr, face, 6)
        # 让各行列平均分摊空间, 网格填满左半区
        for r in range(len(FACE_NAMES)):
            grid.setRowStretch(r, 1)
        for c in range(FACE_MAX_POS):
            grid.setColumnStretch(c, 1)
        outer.addWidget(grid_w, 1)
        hint = QLabel("参考货号来自《D题货架盘点-香橙派方案基线》§1.1 离线样本；"
                      "现场货号随机，实际以现场识别为准（✓=与参考一致，≠=不同）")
        hint.setStyleSheet("font-size: 11px; color: #888;")
        hint.setWordWrap(True)
        outer.addWidget(hint)
        return box

    def _build_right(self):
        right = QVBoxLayout()
        right.setSpacing(8)

        # 实时盘点
        g_info = QGroupBox("实时盘点")
        v = QVBoxLayout(g_info)
        self.lbl_big = QLabel("等待盘点数据…")
        self.lbl_big.setAlignment(Qt.AlignCenter)
        self.lbl_big.setStyleSheet(
            "font-size: 40px; font-weight: bold; color: #1a73e8; padding: 14px;")
        self.progress = QProgressBar()
        total = len(FACE_NAMES) * FACE_MAX_POS          # 单板双面：2×6 = 12
        self.progress.setRange(0, total)
        self.progress.setFormat(f"已盘点 %v / {total}")
        self.progress.setMinimumHeight(24)
        v.addWidget(self.lbl_big, 1)
        v.addWidget(self.progress)
        right.addWidget(g_info)

        # 货物查询
        g_query = QGroupBox("货物查询")
        v = QVBoxLayout(g_query)
        qrow = QHBoxLayout()
        self.edit_num = QLineEdit()
        self.edit_num.setPlaceholderText("输入货物编号(1~255)")
        self.edit_num.setMaxLength(3)
        self.btn_query = QPushButton("查 询")
        qrow.addWidget(self.edit_num, 1)
        qrow.addWidget(self.btn_query)
        v.addLayout(qrow)
        self.lbl_query_result = QLabel("输入编号后点查询，显示位置坐标")
        self.lbl_query_result.setAlignment(Qt.AlignCenter)
        self.lbl_query_result.setStyleSheet(
            "font-size: 22px; font-weight: bold; color: #188038; padding: 10px;")
        v.addWidget(self.lbl_query_result)
        btn_row = QHBoxLayout()
        self.btn_clear = QPushButton("清 空")
        self.btn_reset = QPushButton("复 位")
        btn_row.addWidget(self.btn_clear)
        btn_row.addWidget(self.btn_reset)
        v.addLayout(btn_row)
        right.addWidget(g_query)

        self.btn_query.clicked.connect(self._query)
        self.edit_num.returnPressed.connect(self._query)
        self.btn_clear.clicked.connect(self._clear)
        self.btn_reset.clicked.connect(self._reset)
        return right

    # ---------- 模式 ----------
    def _set_mode(self, mode):
        if mode == self.mode:
            return
        self.mode = mode
        self.lbl_mode.setText(mode)
        self.mode_changed.emit(mode)

    # ---------- 刷新 ----------
    def _on_item(self, num, name):
        self.lbl_big.setText(f"{num:>2}\n{name}")

    def _refresh_grid(self):
        self.progress.setValue(self.inv.count())
        for (face, pos), lbl in self._grid_labels.items():
            face_char = FACE_NAMES[face]
            ref = layout.reference_number(face_char, pos)
            num = self.inv.number_at(face, pos)
            head = f"{face_char}{pos}  参考{ref}" if ref is not None else f"{face_char}{pos}"
            if num is None:
                lbl.setText(f"{head}\n—")
                lbl.setStyleSheet(STYLE_CELL_EMPTY)
            elif ref is None:
                lbl.setText(f"{head}\n{num}")
                lbl.setStyleSheet(STYLE_CELL_FILLED)
            elif num == ref:
                lbl.setText(f"{head}\n{num} ✓")
                lbl.setStyleSheet(STYLE_CELL_MATCH)
            else:
                lbl.setText(f"{head}\n{num} ≠")
                lbl.setStyleSheet(STYLE_CELL_DIFF)
        self._refresh_stage()
        if self.inv.done():
            self.lbl_big.setStyleSheet(
                "font-size: 40px; font-weight: bold; color: #188038; padding: 14px;")
            self.lbl_big.setText("全部盘点完成 ✓")

    def _stage_index(self):
        """按飞控状态 + 已盘点结果推算当前阶段（0..7，见 STAGES）。"""
        st = self.state.state
        n_a = sum(1 for p in range(1, FACE_MAX_POS + 1)
                  if self.inv.number_at(FACE_NAMES.index("A"), p) is not None)
        n_b = sum(1 for p in range(1, FACE_MAX_POS + 1)
                  if self.inv.number_at(FACE_NAMES.index("B"), p) is not None)
        if st == 4:
            return 7                                   # 已落地 = 完成
        if st == 3:
            return 6                                   # 降落中
        if self.inv.done():
            return 5                                   # 12 格齐 = 结果核对
        if n_a > 0:
            return 4                                   # 已在 A 面
        if n_b >= FACE_MAX_POS:
            return 3                                   # B 面 6 格齐 = 绕板换面
        if n_b > 0 or st == 2:
            return 2                                   # 飞行中 = B 面扫描
        if st == 1:
            return 1                                   # 起飞中
        return 0                                       # 准备

    def _refresh_stage(self):
        cur = self._stage_index()
        for i, lbl in enumerate(self._stage_labels):
            if i == cur:
                lbl.setStyleSheet(STYLE_STAGE_ACTIVE)
            elif i < cur:
                lbl.setStyleSheet(STYLE_STAGE_DONE)
            else:
                lbl.setStyleSheet(STYLE_STAGE_IDLE)

    def _refresh_state(self):
        self.lbl_state.setText(f"状态: {self.state.state_name}")
        self.lbl_height.setText(f"高度: {self.state.height} cm")
        self.lbl_batt.setText(f"电量: {self.state.battery} %")
        r = self.state.report_num
        self.lbl_report.setText(f"识别报送: {'编号 ' + str(r) if r is not None else '--'}")
        self._refresh_stage()

    # ---------- 查询 / 清空 / 复位 ----------
    def _query(self):
        text = self.edit_num.text().strip()
        if not text.isdigit():
            self._set_result("请输入数字编号 (1~255)", "#d93025")
            return
        num = int(text)
        if not 1 <= num <= 255:
            self._set_result("编号超出范围 (1~255)", "#d93025")
            return
        names = self.inv.query_number(num)
        if names:
            self._set_result(f"编号 {num} → 坐标 {'、'.join(names)}", "#188038")
        else:
            self._set_result(f"编号 {num} 尚未盘点", "#d93025")

    def _set_result(self, text, color):
        self.lbl_query_result.setText(text)
        self.lbl_query_result.setStyleSheet(
            f"font-size: 22px; font-weight: bold; color: {color}; padding: 10px;")

    def _clear(self):
        self.inv.reset()
        self.lbl_big.setText("等待盘点数据…")
        self.lbl_big.setStyleSheet(
            "font-size: 40px; font-weight: bold; color: #1a73e8; padding: 14px;")
        self._set_result("已清空盘点结果", "#1a73e8")
        self.cleared.emit()             # 让定点页把过期的目标/航线一并作废

    def _reset(self):
        self._clear()
        self.state.reset()
        self.led.set_on(False)
        self._set_result("系统已复位", "#1a73e8")

    def _set_led(self, on):
        self.lbl_led.setStyleSheet(
            "font-size: 26px; color: #d93025;" if on else "font-size: 26px; color: #aaa;")
