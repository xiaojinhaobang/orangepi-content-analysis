# -*- coding: utf-8 -*-
"""航线图(简化): 仓库俯视图 + 两个货架用竖线表示(不区分高度) +
   起飞/降落点 + 航线轨迹(虚线)。去掉 24 个坐标点与目标高亮圈。"""
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPen, QPolygonF, QPainterPath
from PySide6.QtWidgets import (
    QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsPathItem, QGraphicsPolygonItem,
    QGraphicsRectItem, QGraphicsScene, QGraphicsSimpleTextItem, QGraphicsView,
    QLabel, QVBoxLayout, QWidget,
)

import layout

WAREHOUSE = QRectF(0, 0, layout.WAREHOUSE_W, layout.WAREHOUSE_H)
TAKEOFF = layout.TAKEOFF
LANDING = layout.LANDING


class PageRoute(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._route_items = []

        self._scene = QGraphicsScene()
        self._view = QGraphicsView(self._scene)
        self._view.setRenderHint(QPainter.Antialiasing)
        self.lbl_info = QLabel("等待航线数据…")

        lay = QVBoxLayout(self)
        lay.addWidget(self._view, 1)
        lay.addWidget(self.lbl_info)

        self._draw_static()

    # ---- 静态背景 ----
    def _draw_static(self):
        s = self._scene
        s.setSceneRect(WAREHOUSE.adjusted(-30, -30, 40, 40))

        # 仓库地面
        floor = QGraphicsRectItem(WAREHOUSE)
        floor.setBrush(QBrush(QColor("#f7f7f2")))
        floor.setPen(QPen(QColor("#888"), 2))
        s.addItem(floor)

        # 货架: 每个货架一条竖线(仅表示位置, 不区分高度)
        for _name, (x0, x1, y0, y1) in layout.SHELVES.items():
            x = (x0 + x1) // 2
            line = QGraphicsLineItem(x, y0, x, y1)
            line.setPen(QPen(QColor("#8a7a55"), 4))
            s.addItem(line)
            tag = QGraphicsSimpleTextItem("货架")
            tag.setPos(x - 18, (y0 + y1) / 2 - 6)
            s.addItem(tag)

        # 起飞点(方) / 降落点(圆)
        tx, ty = TAKEOFF
        sq = QGraphicsPolygonItem(QPolygonF(
            [QPointF(tx - 10, ty - 10), QPointF(tx + 10, ty - 10),
             QPointF(tx + 10, ty + 10), QPointF(tx - 10, ty + 10)]))
        sq.setBrush(QBrush(QColor("#222")))
        s.addItem(sq)
        tag = QGraphicsSimpleTextItem("起飞")
        tag.setPos(tx - 14, ty + 16)
        s.addItem(tag)

        lx, ly = LANDING
        cir = QGraphicsEllipseItem(lx - 12, ly - 12, 24, 24)
        cir.setBrush(QBrush(QColor("#222")))
        s.addItem(cir)
        tag = QGraphicsSimpleTextItem("降落")
        tag.setPos(lx - 14, ly + 18)
        s.addItem(tag)

    # ---- 航线数据 ----
    def set_route(self, points, target_name=None):
        """画规划航线折线; target_name 仅用于航线信息栏显示"""
        s = self._scene
        for it in self._route_items:
            s.removeItem(it)
        self._route_items.clear()

        if points:
            path = QPainterPath()
            path.moveTo(*points[0])
            for x, y in points[1:]:
                path.lineTo(x, y)
            line = QGraphicsPathItem(path)
            line.setPen(QPen(QColor("#e37400"), 3, Qt.DashLine))
            s.addItem(line)
            self._route_items.append(line)
            # 起点(绿)/终点(红)标记
            sx, sy = points[0]
            start = QGraphicsEllipseItem(sx - 5, sy - 5, 10, 10)
            start.setBrush(QBrush(QColor("#188038")))
            s.addItem(start)
            self._route_items.append(start)
            ex, ey = points[-1]
            end = QGraphicsEllipseItem(ex - 5, ey - 5, 10, 10)
            end.setBrush(QBrush(QColor("#d93025")))
            s.addItem(end)
            self._route_items.append(end)
            self.lbl_info.setText(
                f"航线 {len(points)} 个航点"
                + (f"    目标: {target_name}" if target_name else ""))
        else:
            self.lbl_info.setText("无航线数据")

        self._view.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
