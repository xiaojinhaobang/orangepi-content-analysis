# -*- coding: utf-8 -*-
"""地面站入口。启动后停在遍历盘点页, 点"开始模拟"才跑模拟。
用法: python main.py"""
import sys

from PySide6.QtWidgets import QApplication

from ui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
