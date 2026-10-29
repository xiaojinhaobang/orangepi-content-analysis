# -*- coding: utf-8 -*-
"""实时盘点页: 最新盘点结果大字显示 + 进度 + 状态 + 最近事件列表"""
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QGridLayout, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QProgressBar, QVBoxLayout, QWidget,
)

from models import TOTAL_CELLS


class PageLive(QWidget):
    def __init__(self, inventory, drone_state, parent=None):
        super().__init__(parent)
        self.inv = inventory
        self.state = drone_state

        # --- 状态行 ---
        self.lbl_state = QLabel("状态: --")
        self.lbl_height = QLabel("高度: --")
        self.lbl_batt = QLabel("电量: --")
        self.lbl_report = QLabel("识别报送: --")
        status = QHBoxLayout()
        for w in (self.lbl_state, self.lbl_height, self.lbl_batt, self.lbl_report):
            w.setStyleSheet("font-size: 14px; padding: 2px 8px;")
            status.addWidget(w)
        status.addStretch(1)

        # --- 最新盘点大字 ---
        self.lbl_big = QLabel("等待盘点数据…")
        self.lbl_big.setAlignment(Qt.AlignCenter)
        self.lbl_big.setStyleSheet(
            "font-size: 42px; font-weight: bold; color: #1a73e8; padding: 18px;")

        # --- 进度 ---
        self.progress = QProgressBar()
        self.progress.setRange(0, TOTAL_CELLS)
        self.progress.setFormat(f"已盘点 %v / {TOTAL_CELLS}")
        self.progress.setMinimumHeight(26)

        # --- 最近事件 ---
        self.list_events = QListWidget()
        self.list_events.setMinimumHeight(140)
        lbl_hint = QLabel("最近盘点事件:")
        lbl_hint.setStyleSheet("font-size: 13px; color: #666;")

        lay = QVBoxLayout(self)
        lay.addLayout(status)
        lay.addWidget(self.lbl_big, 2)
        lay.addWidget(self.progress)
        lay.addWidget(lbl_hint)
        lay.addWidget(self.list_events, 1)

        # 订阅
        self.inv.item_added.connect(self._on_item)
        self.inv.changed.connect(self._refresh_progress)
        self.state.changed.connect(self._refresh_state)

    def _on_item(self, num, name):
        self.lbl_big.setText(f"{num:>2}\n{name}")
        item = QListWidgetItem(f"编号 {num:>2}    坐标 {name}")
        item.setForeground(QColor("#1a73e8"))
        self.list_events.insertItem(0, item)
        while self.list_events.count() > 12:
            self.list_events.takeItem(self.list_events.count() - 1)

    def _refresh_progress(self):
        self.progress.setValue(self.inv.count())
        if self.inv.done():
            self.lbl_big.setStyleSheet(
                "font-size: 42px; font-weight: bold; color: #188038; padding: 18px;")
            self.lbl_big.setText("全部盘点完成 ✓")

    def _refresh_state(self):
        self.lbl_state.setText(f"状态: {self.state.state_name}")
        self.lbl_height.setText(f"高度: {self.state.height} cm")
        self.lbl_batt.setText(f"电量: {self.state.battery} %")
        r = self.state.report_num
        self.lbl_report.setText(f"识别报送: {'编号 ' + str(r) if r is not None else '--'}")
