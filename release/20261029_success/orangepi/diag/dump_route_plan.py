"""离线打印 mission_node 的完整航线（不连 ROS、不连飞控）。

用法（在香橙派上跑）:
    python3 ~/diag/dump_route_plan.py                # 整块 12 格（A+B 两面）
    python3 ~/diag/dump_route_plan.py B1,B2,B3       # 只飞指定货位

除了打印顺序，还做**穿板检查**：相邻两航点连成的直线如果穿过板面所在的竖直
平面 x = PANEL_X，就报出穿越点的 y。y 落在 BOARD_Y 范围内 = 会撞板。
"""

import sys

sys.path.insert(0, '/home/orangepi/diag')

import mission_node as M  # noqa: E402

# 板子沿 y 的实际范围（现场量：含框 210cm，近端 y=90 → y=90~300）
BOARD_Y = (90.0, 300.0)


def crossing_y(p0, p1, panel_x):
    """线段 p0->p1 与平面 x=panel_x 的穿越点 y；不穿越返回 None。"""
    x0, y0 = p0
    x1, y1 = p1
    if (x0 - panel_x) * (x1 - panel_x) > 0:
        return None
    if x1 == x0:
        return y0
    t = (panel_x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)


def tail_detour(pts):
    """复刻 run_traverse 里"扫完还在 B 侧 → 绕板远端去降落"那段。"""
    if not pts or pts[-1][1] >= M.PANEL_X:
        return []
    z = M.ROW_CAM_Z[0] + M.CAM_DOWN + M.Z_TRIM
    return [('绕行-B侧到板远端', M.PANEL_X - M.OBS_DIST - M.CAM_FWD, M.DETOUR_Y, z),
            ('绕行-横穿到降落侧', M.LANDING_FIELD[0], M.DETOUR_Y, z)]


def main():
    subset = sys.argv[1] if len(sys.argv) > 1 else ''
    pts = M.plan_points(subset)
    print('subset=%r  →  %d 个航点（含过渡点）' % (subset, len(pts)))
    if not pts:
        return

    detour = tail_detour(pts)
    seq = ([('takeoff',) + M.TAKEOFF_FIELD + (pts[0][3],)]
           + list(pts) + list(detour)
           + [('land',) + M.LANDING_FIELD + (120.0,)])

    print('起飞点 场地(%.0f,%.0f) | 降落点 场地(%.0f,%.0f) | 板子 y 范围假设 (%.0f,%.0f)'
          % (M.TAKEOFF_FIELD + M.LANDING_FIELD + BOARD_Y))
    print()
    print('%-18s %8s %8s %6s %6s %6s %9s'
          % ('航点', '场地x', '场地y', '机体x', '机体y', '高度', 'yaw'))
    for name, fx, fy, fz in seq:
        bx, by = M.to_flight(fx, fy, 0.0)
        if name == 'takeoff':
            yaw = 'psi0'
        elif name == 'land':
            yaw = '(同上一个)'
        else:
            # 与 mission_node 的 yaw_for() 同口径：按点名判
            import re as _re
            m = _re.search(r'掉头(\d+)$', name)
            if m:
                yaw = 'psi0+%s' % m.group(1)
            elif name[:1] == 'B' or '稳定' in name:
                yaw = 'psi0+180'
            else:
                yaw = 'psi0'
        print('%-18s %8.1f %8.1f %6.1f %6.1f %6.0f %9s'
              % (name, fx, fy, bx, by, fz, yaw))

    print()
    print('--- 穿板检查（板面 x=%.0f，板子 y %.0f~%.0f）---' % ((M.PANEL_X,) + BOARD_Y))
    bad = 0
    for (na, xa, ya, _), (nb, xb, yb, _) in zip(seq, seq[1:]):
        cy = crossing_y((xa, ya), (xb, yb), M.PANEL_X)
        if cy is None:
            continue
        inside = BOARD_Y[0] <= cy <= BOARD_Y[1]
        if inside:
            bad += 1
        print('  %s(%.0f,%.0f) → %s(%.0f,%.0f)   穿越 @ y=%.0f  %s'
              % (na, xa, ya, nb, xb, yb, cy,
                 '⚠ 板子范围内 → 撞板' if inside else '✓ 板子外（安全）'))
    print('  危险穿越段数: %d' % bad)


if __name__ == '__main__':
    main()
