# -*- coding: utf-8 -*-
"""场地与白板几何（统一布局源，simulator 与航线图共用）。

★ 2026-09-28 按《D题货架盘点-香橙派方案基线》改写：
  由原题"双货架 4 面 24 码"改为改编版"单白板双面 12 货位"。

坐标系（沿用本文件的屏幕约定）: 厘米, 原点 = 场地上方左角, X 向右 0~300, Y 向下 0~420。
方案基线用的是"左下角为原点、Y 向上"，故这里 scene_y = 420 - 基线y 翻转。

方案基线关键数据:
  - 场地 300cm(横, x) × 420cm(纵, y)
  - 起飞点 (75, 75) 基线坐标 → 场景 (75, 345)
  - 降落点 (225, 345) 基线坐标 → 场景 (225, 75)
  - 白板面 x = 150cm；板面基线 y = 105~305（含框 100~310）→ 场景 y = 115~315
  - B 面朝 -x（起飞点一侧），A 面朝 +x（降落点一侧）
  - 每面 2 排 × 3 列 = 6 个货位；单张码 20×20cm
  - 三列中心沿板面等距（相邻中心间距 50cm）
  - 上排中心离地 150cm，下排 110cm，两排间距 40cm

编号规则（方案基线原文）：**面对某一面观察时**，上排 1-3、下排 4-6。
⇒ A 面与 B 面的观察者左右相反，所以 A1 与 B1 在**相反的物理端**，必须分别处理
  （见 cell_coord 里由 side 决定的列序）。
"""
from protocol import FACE_NAMES

WAREHOUSE_W = 300
WAREHOUSE_H = 420
TAKEOFF = (75, 345)             # 起飞区中心（场景坐标）
LANDING = (225, 75)             # 降落区中心（场景坐标）

# 白板: 名 -> (x0, x1, y0, y1) 俯视图范围（板厚按 4cm 画）
PANEL_X = 150
PANEL_Y0, PANEL_Y1 = 115, 315
SHELVES = {
    1: (PANEL_X - 2, PANEL_X + 2, PANEL_Y0, PANEL_Y1),
}

# 面 -> (白板, 朝向): A 朝东(+X, 降落点一侧), B 朝西(-X, 起飞点一侧)
FACE_SHELF = {
    "A": (1, "east"),
    "B": (1, "west"),
}

ROWS, COLS = 2, 3               # 每面 2 排 × 3 列

# 板面尺寸与二维码高度（方案基线）:
#   板面 200×100cm，含框 210cm；上排中心离地 150cm、下排 110cm
SHELF_HEIGHT = 100
POLE_HEIGHT = 190
ROW_HEIGHTS = {0: 150, 1: 110}

# 实际盘点顺序（方案基线 §6 建议的蛇形）:
#   起飞 → B 面 B1..B3 再 B6..B4 → 绕板端 → A 面 A1..A3 再 A6..A4 → 降落
TRAVERSE_ORDER = [
    ("B", 1), ("B", 2), ("B", 3), ("B", 6), ("B", 5), ("B", 4),
    ("A", 1), ("A", 2), ("A", 3), ("A", 6), ("A", 5), ("A", 4),
]

# ---------------- 参考货号（方案基线 §1.1 离线回归样本） ----------------
# ★ 现场货号是随机的，这张表只是那组照片的离线解码结果：
#   - 界面拿它做"参考"对照；
#   - 模拟机载端拿它做默认布局（走真实链路时会被现场识别结果覆盖）。
#   正式识别逻辑绝不允许把这张表当答案，只能现场解码。
REFERENCE_NUMBERS = {
    "A": {1: 13, 2: 14, 3: 17, 4: 15, 5: 12, 6: 16},
    "B": {1: 8, 2: 11, 3: 9, 4: 7, 5: 5, 6: 6},
}


def reference_number(face: str, pos: int):
    """位置 -> 参考货号；表里没有返回 None。"""
    return REFERENCE_NUMBERS.get(face, {}).get(pos)


def reference_layout():
    """参考样本 -> {(face_idx, pos): num}，face_idx 按 protocol.FACE_NAMES（A=0, B=1）。"""
    return {(FACE_NAMES.index(f), p): n
            for f, cells in REFERENCE_NUMBERS.items()
            for p, n in cells.items()}


# 二维码面相对板面的水平伸出量 cm（板很薄，取 3cm 表示"贴在面上"）
_FACE_OFFSET = 3


def cell_coord(face: str, pos: int):
    """坐标名(面, 位) -> 货位水平坐标 (x, y) cm（场景坐标）。

    列沿板面长轴等距；**列的左右由观察者视角决定**：
      B 面观察者在 x<150 侧朝东看，观察者左侧 = 场景 y 较小的一侧；
      A 面观察者在 x>150 侧朝西看，观察者左侧 = 场景 y 较大的一侧（左右反转）。
    """
    shelf, side = FACE_SHELF[face]
    x0, x1, y0, y1 = SHELVES[shelf]
    cx = (x0 + x1) / 2
    col = (pos - 1) % COLS                     # 0..2，观察者视角的列（左起）
    idx = col if side == "west" else (COLS - 1 - col)
    x = cx + (_FACE_OFFSET if side == "east" else -_FACE_OFFSET)
    y = y0 + (idx + 0.5) * (y1 - y0) / COLS
    return (round(x, 1), round(y, 1))


def cell_xyz(face: str, pos: int):
    """货位 3D 坐标 (x, y, z)：x/y 同 cell_coord，z = 该排二维码中心离地高。"""
    x, y = cell_coord(face, pos)
    return (x, y, ROW_HEIGHTS[(pos - 1) // COLS])


# 观察/识别距离（相机到板面）：方案基线按"一帧只出现一个码"算出 0.35m
OBSERVE_DIST = 35
_CORRIDOR = {
    f: round((SHELVES[FACE_SHELF[f][0]][0] + SHELVES[FACE_SHELF[f][0]][1]) / 2
             + (OBSERVE_DIST if FACE_SHELF[f][1] == "east" else -OBSERVE_DIST))
    for f in FACE_NAMES
}
_SOUTH_Y = 330                  # 板南侧绕行通道 y（板面到 315，留 15cm）
_NORTH_Y = 100                  # 板北侧绕行通道 y（板面从 115，留 15cm）


def traversal_points():
    """遍历盘点航线（含起降）：起飞 → B 面走廊 → 绕板南端 → A 面走廊 → 降落。"""
    route = [TAKEOFF]
    last_face = None
    for face_char, pos in TRAVERSE_ORDER:
        col_x = _CORRIDOR[face_char]
        ty = cell_coord(face_char, pos)[1]
        if last_face is None:
            route.append((col_x, _SOUTH_Y))
        elif face_char != last_face:
            route.append((_CORRIDOR[last_face], _SOUTH_Y))
            route.append((col_x, _SOUTH_Y))
        route.append((col_x, ty))
        last_face = face_char
    route.append((_CORRIDOR[last_face], _SOUTH_Y))
    route.append(LANDING)
    return route


def targeted_route(face: str, pos: int):
    """定点盘点航线：起飞 → 板南侧 → 目标面走廊 → 目标列中心 → 撤回南侧 → 降落。"""
    col_x = _CORRIDOR[face]
    ty = cell_coord(face, pos)[1]
    return [
        TAKEOFF,
        (col_x, _SOUTH_Y),
        (col_x, ty),
        (col_x, _SOUTH_Y),
        LANDING,
    ]
