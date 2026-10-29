# -*- coding: utf-8 -*-
"""
LED 控制器抽象。题目要求: 每盘点到一个货物, 地面站 LED 亮灭一次(亮 1s 左右)。

电脑端没有 GPIO, 先用 VirtualLed(UI 指示灯)占位;
以后外接 USB/串口小板驱动真实 LED 时, 实现一个 UsbLed(led_controller 同接口)替换即可。
"""
from PySide6.QtCore import QObject, QTimer, Signal


class LedController(QObject):
    """抽象: set_on(bool) 控制亮灭, blink() 亮起并在 duration 后熄灭。"""
    state_changed = Signal(bool)        # True=亮 False=灭

    def __init__(self, parent=None):
        super().__init__(parent)
        self._timers = []

    def set_on(self, on: bool):
        raise NotImplementedError

    def blink(self, duration_ms: int = 1000):
        """亮 duration_ms 后自动灭。连续触发时重置定时器(重新计时)。"""
        self.set_on(True)
        for tm in self._timers:
            try:
                tm.stop()
            except Exception:
                pass
        self._timers = [QTimer.singleShot(duration_ms, lambda: self.set_on(False))]


class VirtualLed(LedController):
    """软件 LED: 通过 state_changed 通知 UI 点亮/熄灭屏幕指示灯。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._on = False

    def set_on(self, on: bool):
        self._on = on
        self.state_changed.emit(on)

    @property
    def is_on(self):
        return self._on
