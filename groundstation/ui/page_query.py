# -*- coding: utf-8 -*-
"""编号查询页: 输入 1~24 货物编号 -> 显示对应坐标(要求1-④);
   可一键显示全部盘点结果。"""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget,
)


class PageQuery(QWidget):
    def __init__(self, inventory, parent=None):
        super().__init__(parent)
        self.inv = inventory

        title = QLabel("货物编号查询")
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        hint = QLabel("输入货物编号(1~24)，回车或点查询")
        hint.setStyleSheet("color: #666;")

        self.edit = QLineEdit()
        self.edit.setPlaceholderText("例如 12")
        self.edit.setMaxLength(3)
        self.edit.setFixedHeight(32)
        self.edit.setStyleSheet("font-size: 16px;")
        self.btn_query = QPushButton("查 询")
        self.btn_query.setFixedHeight(32)
        self.btn_clear = QPushButton("清空")
        self.btn_clear.setFixedHeight(32)
        self.btn_all = QPushButton("显示全部盘点结果")
        self.btn_all.setFixedHeight(32)

        row1 = QHBoxLayout()
        row1.addWidget(self.edit, 1)
        row1.addWidget(self.btn_query)
        row1.addWidget(self.btn_clear)

        self.lbl_result = QLabel("—")
        self.lbl_result.setAlignment(Qt.AlignCenter)
        self.lbl_result.setStyleSheet(
            "font-size: 34px; font-weight: bold; color: #188038; padding: 24px;")

        self.lbl_detail = QLabel("")
        self.lbl_detail.setStyleSheet("font-size: 14px; color: #666;")

        lay = QVBoxLayout(self)
        lay.addWidget(title)
        lay.addWidget(hint)
        lay.addLayout(row1)
        lay.addWidget(self.lbl_result, 1)
        lay.addWidget(self.lbl_detail)
        lay.addWidget(self.btn_all)

        self.btn_query.clicked.connect(self.do_query)
        self.edit.returnPressed.connect(self.do_query)
        self.btn_clear.clicked.connect(self.clear)
        self.btn_all.clicked.connect(self.show_all)

    def do_query(self):
        text = self.edit.text().strip()
        if not text.isdigit():
            self.lbl_result.setText("请输入数字编号 (1~24)")
            self.lbl_result.setStyleSheet(
                "font-size: 34px; font-weight: bold; color: #d93025; padding: 24px;")
            return
        num = int(text)
        if not 1 <= num <= 24:
            self.lbl_result.setText("编号超出范围 (1~24)")
            self.lbl_result.setStyleSheet(
                "font-size: 34px; font-weight: bold; color: #d93025; padding: 24px;")
            return
        names = self.inv.query_number(num)
        if names:
            self.lbl_result.setText(f"编号 {num} → 坐标 {'、'.join(names)}")
            self.lbl_result.setStyleSheet(
                "font-size: 34px; font-weight: bold; color: #188038; padding: 24px;")
            self.lbl_detail.setText(
                f"已盘点 {self.inv.count()}/24   当前共找到 {len(names)} 个位置")
        else:
            self.lbl_result.setText(f"编号 {num} 尚未盘点")
            self.lbl_result.setStyleSheet(
                "font-size: 34px; font-weight: bold; color: #d93025; padding: 24px;")
            self.lbl_detail.setText("请等待该货物被盘点，或检查编号是否正确")

    def clear(self):
        self.edit.clear()
        self.lbl_result.setText("—")
        self.lbl_result.setStyleSheet(
            "font-size: 34px; font-weight: bold; color: #188038; padding: 24px;")
        self.lbl_detail.setText("")

    def show_all(self):
        items = self.inv.all_cells()
        filled = [(n, v) for (n, v) in items if v is not None]
        if not filled:
            self.lbl_detail.setText("暂无盘点结果")
            return
        parts = [f"{n}: {v}" for n, v in filled]
        # 每行 6 项
        lines = []
        for i in range(0, len(parts), 6):
            lines.append("    ".join(parts[i:i + 6]))
        self.lbl_result.setText("全部盘点结果")
        self.lbl_result.setStyleSheet(
            "font-size: 22px; font-weight: bold; color: #1a73e8; padding: 24px;")
        self.lbl_detail.setText(
            f"共 {len(filled)} 个货物\n" + "\n".join(lines))
