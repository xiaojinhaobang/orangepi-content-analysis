# -*- coding: utf-8 -*-
"""结果总览页: 12 个坐标位(A1~B6, 单板双面)网格, 每格显示坐标名与盘点到的编号"""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QGridLayout, QLabel, QVBoxLayout, QWidget

from protocol import FACE_NAMES, FACE_MAX_POS


class PageOverview(QWidget):
    def __init__(self, inventory, parent=None):
        super().__init__(parent)
        self.inv = inventory
        self._labels = {}

        lay = QVBoxLayout(self)
        title = QLabel("白板双面全部盘点结果")
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        lay.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(6)
        # len(FACE_NAMES) 行(面) x 6 列(位)
        for face in range(len(FACE_NAMES)):
            for pos in range(1, FACE_MAX_POS + 1):
                cell = QLabel()
                cell.setAlignment(Qt.AlignCenter)
                cell.setMinimumSize(96, 52)
                cell.setStyleSheet(
                    "border: 2px solid #ccc; border-radius: 6px;"
                    "font-size: 16px; font-weight: bold; background: #fafafa;")
                grid.addWidget(cell, face, pos - 1)
                key = (face, pos)
                self._labels[key] = cell
        # 行头(面名)
        for face, name in enumerate(FACE_NAMES):
            hdr = QLabel(f"  {name} 面")
            hdr.setStyleSheet("font-size: 14px; font-weight: bold; color: #666;")
            hdr.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(hdr, face, 6)
        lay.addLayout(grid)
        lay.addStretch(1)

        self.inv.changed.connect(self.refresh)
        self.refresh()

    def refresh(self):
        for (face, pos), lbl in self._labels.items():
            num = self.inv.number_at(face, pos)
            name = f"{FACE_NAMES[face]}{pos}"
            if num is None:
                lbl.setText(f"{name}\n—")
                lbl.setStyleSheet(
                    "border: 2px solid #ccc; border-radius: 6px;"
                    "font-size: 16px; font-weight: bold; background: #fafafa; color: #999;")
            else:
                lbl.setText(f"{name}\n{num}")
                lbl.setStyleSheet(
                    "border: 2px solid #1a73e8; border-radius: 6px;"
                    "font-size: 16px; font-weight: bold; background: #e8f0fe; color: #1a73e8;")
