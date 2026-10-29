"""画 mission_node 的完整航线俯视图（本地跑，不需要连板子）。

用法:
    python tools/plot_route_plan.py [输出png] [subset]
例:
    python tools/plot_route_plan.py simple_map/route_12.png
    python tools/plot_route_plan.py simple_map/route_B.png B1,B2,B3,B4,B5,B6
"""

import importlib.util
import os
import sys
import types

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, 'board_diag', 'mission_node.py')


def load_mission():
    """mission_node 依赖 ROS，本地用空壳顶掉，只为拿到纯几何函数。"""
    for name in ('rclpy', 'rclpy.node', 'tf2_ros', 'std_msgs', 'std_msgs.msg'):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
    sys.modules['rclpy'].init = sys.modules['rclpy'].ok = lambda: True
    sys.modules['rclpy'].shutdown = sys.modules['rclpy'].spin = lambda *a, **k: None
    sys.modules['rclpy.node'].Node = type('Node', (object,), {})
    sys.modules['tf2_ros'].Buffer = sys.modules['tf2_ros'].TransformListener = object
    for attr in ('Empty', 'Int16', 'Int16MultiArray', 'String', 'UInt8',
                 'UInt8MultiArray'):
        setattr(sys.modules['std_msgs.msg'], attr, type(attr, (object,), {}))
    spec = importlib.util.spec_from_file_location('mission_node', SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


M = load_mission()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'simple_map/route_plan.png'
    subset = sys.argv[2] if len(sys.argv) > 2 else ''
    pts = M.plan_points(subset)

    detour = []
    if pts and pts[-1][1] < M.PANEL_X:
        z = M.ROW_CAM_Z[0] + M.CAM_DOWN + M.Z_TRIM
        detour = [('绕行-B侧到板远端', M.PANEL_X - M.OBS_DIST - M.CAM_FWD, M.DETOUR_Y, z),
                  ('绕行-横穿到降落侧', M.LANDING_FIELD[0], M.DETOUR_Y, z)]

    seq = ([('takeoff',) + M.TAKEOFF_FIELD] + [(p[0], p[1], p[2]) for p in pts]
           + [(d[0], d[1], d[2]) for d in detour]
           + [('land',) + M.LANDING_FIELD])

    fig, ax = plt.subplots(figsize=(8.4, 9.6))
    ax.set_xlim(-20, 450)
    ax.set_ylim(-20, 450)
    ax.set_aspect('equal')
    ax.grid(alpha=0.25, lw=0.5)
    ax.set_xlabel('场地 x (cm) →')
    ax.set_ylabel('场地 y (cm) →')
    ax.set_title('航线俯视图  subset=%r  (%d 个航点)' % (subset or '整块12格', len(pts)),
                 fontsize=11)
    ax.set_facecolor('#fafafa')

    by0, by1 = 90.0, 300.0           # 白板含框 210cm
    ax.add_patch(Rectangle((M.PANEL_X - 3, by0), 6, by1 - by0,
                           color='#333', zorder=3))
    ax.text(M.PANEL_X, by1 + 8, '白板（含框 210cm）', ha='center', fontsize=9,
            color='#333')
    ax.text(M.PANEL_X - 8, by1 - 16, 'A 面', ha='right', fontsize=10,
            color='#1a73e8', fontweight='bold')
    ax.text(M.PANEL_X + 8, by1 - 16, 'B 面', ha='left', fontsize=10,
            color='#188038', fontweight='bold')

    for row in (0, 1):
        for lab in (1, 2, 3):
            c = lab if row == 0 else lab        # 上下排同列
            cy = M.COL_Y[c - 1]
            for face, dx in (('A', -8), ('B', 8)):
                cl = lab if face == 'B' else 4 - lab
                ax.add_patch(Rectangle((M.PANEL_X + dx - 3, cy - 3), 6, 6,
                                       color='#c8a24a', zorder=4))
                ax.text(M.PANEL_X + dx + (-11 if face == 'A' else 11),
                        cy + (7 if row == 0 else -11),
                        '%s%d' % (face, cl + row * 3),
                        ha='right' if face == 'A' else 'left', va='center',
                        fontsize=7.5, color='#8a6d1f')

    xs = [p[1] for p in seq]
    ys = [p[2] for p in seq]
    ax.plot(xs, ys, '-', color='#1f77b4', lw=1.2, alpha=0.55, zorder=2)
    for (na, xa, ya), (nb, xb, yb) in zip(seq, seq[1:]):
        if (xa, ya) == (xb, yb):
            ax.plot([xa], [ya], marker='o', ms=9, mfc='none', mec='#d62728',
                    mew=1.6, zorder=6)
            ax.annotate('换排(换高度)', (xa, ya), textcoords='offset points',
                        xytext=(10, -4), fontsize=7.5, color='#d62728')
        else:
            ax.add_patch(FancyArrowPatch((xa, ya), (xb, yb), arrowstyle='-|>',
                                         mutation_scale=11, color='#1f77b4',
                                         lw=1.4, zorder=5))

    for i, (name, fx, fy) in enumerate(seq):
        ax.plot([fx], [fy], marker='o', ms=4.5, color='#1f77b4', zorder=7)
        row, dy = '', 5
        if len(name) == 2 and name[1].isdigit():
            # 上/下排投影到俯视图是同一个点，标签往两边岔开才不重叠
            if int(name[1]) <= 3:
                row, dy = '上', 6
            else:
                row, dy = '下', -14
        ax.annotate('%d.%s%s' % (i + 1, name, row), (fx, fy),
                    textcoords='offset points', xytext=(8, dy), fontsize=8,
                    zorder=8)

    ax.plot([M.TAKEOFF_FIELD[0]], [M.TAKEOFF_FIELD[1]], marker='^', ms=12,
            color='#2ca02c', zorder=8)
    ax.annotate('起飞点', M.TAKEOFF_FIELD, textcoords='offset points',
                xytext=(8, -12), fontsize=9, color='#2ca02c')
    ax.plot([M.LANDING_FIELD[0]], [M.LANDING_FIELD[1]], marker='v', ms=12,
            color='#d62728', zorder=8)
    ax.annotate('结束点', M.LANDING_FIELD, textcoords='offset points',
                xytext=(8, 8), fontsize=9, color='#d62728')

    # 绕行/过渡通道（画出来看离板子端头有多远）
    for name, fx, fy in seq:
        if name.startswith(('过渡', '绕行', '引导')):
            ax.axhline(fy, color='#999', lw=1, ls='--')
            ax.text(4, fy + 4, '%s 通道 y=%.0f' % (name.split('-')[0], fy),
                    fontsize=7.5, color='#777')

    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    fig.savefig(out, dpi=140, bbox_inches='tight')
    print('saved %s  (%d 个航点)' % (out, len(pts)))


if __name__ == '__main__':
    main()
