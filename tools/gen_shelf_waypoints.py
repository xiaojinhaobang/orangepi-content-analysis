"""按《D题货架盘点-香橙派方案基线.md》生成货位航点表。

坐标约定
--------
* 场地系（方案基线）：原点=场地图左下角，x 向右、y 向上，单位 m
* 飞控系（我们能下发的）：原点=起飞点，+x = 起飞时机头方向，+y = 机头左侧，单位 cm
* 两者关系：飞控系 = R(psi0) · (场地系 - 起飞点)，psi0 = 起飞时机头在雷达地图里的朝向
  ⇒ **每次摆放后 psi0 都会变，航点表必须重新生成**（和之前正方形航线一样的道理）

几何（来自方案基线 §1）
----------------------
场地 3.00 x 4.20 m；起飞点 (0.75,0.75)；降落点 (2.25,3.45)
白板面 x = 1.50 m，板面 y = 1.05 ~ 3.05 m
三列中心 y = 1.55 / 2.05 / 2.55 m，相邻 0.50 m
上排中心离地 1.50 m，下排 1.10 m，码 0.20 m
B 面朝 -x（起飞点一侧），A 面朝 +x（降落点一侧）

编号规则（方案基线）：面对某一面观察时，上排 1-3、下排 4-6，
所以从 A 面走到 B 面后，观察者左右与场地坐标的对应会反转。

用法
----
python gen_shelf_waypoints.py --psi 12.5 --mode both --out-dir .
  --psi   起飞时机头朝向（度，读 tf map->dr_base 的 yaw）
  --mode  one_shot | per_cell | both
"""
import argparse
import math
from pathlib import Path

# ---- 场地几何（方案基线，改动前先核对文档）----
TAKEOFF_FIELD = (0.75, 0.75)        # m
LANDING_FIELD = (2.25, 3.45)        # m
PANEL_X = 1.50                      # m，白板面所在 x
COL_Y = (1.55, 2.05, 2.55)          # m，三列中心
ROW_Z = (1.50, 1.10)                # m，上排 / 下排
FLY_Z_CM = 120                      # 默认飞行高度（0x66 起飞高度）

# ---- 相机安装偏移（2026-09-28 用户给定，飞测后再微调）----
CAM_FWD = 0.235                     # m，镜头在机体中心【前方】
CAM_LEFT = 0.0                      # m，镜头相对机体中心【向左】偏移（0 = 不偏）
CAM_DOWN = 0.065                    # m，镜头【低于】机体中心

# ---- 观察距离与两排目标 ----
OBS_DIST = 0.35                     # m，相机到板面。单码窗口最大处 0.34m，取 0.35
ROW_CAM_Z = (1.50, 1.10)            # m，要让【相机】对准的上下排中心高度


def body_for_camera(face, code_y, cam_z):
    """给定"相机要在哪"，反推"机体中心航点"（场地系，m）。

    B 面机头朝 +x（任务 0°），相机在机体前方 ⇒ 机体要比相机再退 CAM_FWD；
    A 面机头朝 -x（任务 180°），同理退到另一侧。
    """
    if face == "B":
        body_x = PANEL_X - OBS_DIST - CAM_FWD
    else:
        body_x = PANEL_X + OBS_DIST + CAM_FWD
    return (body_x, code_y, cam_z + CAM_DOWN)


def to_flight(field_x, field_y, psi_deg):
    """场地系(m) -> 飞控系(cm)，绕起飞点按机头朝向旋转。"""
    dx = field_x - TAKEOFF_FIELD[0]
    dy = field_y - TAKEOFF_FIELD[1]
    psi = math.radians(psi_deg)
    fx = dx * math.cos(psi) + dy * math.sin(psi)
    fy = -dx * math.sin(psi) + dy * math.cos(psi)
    return round(fx * 100, 1), round(fy * 100, 1)


def face_cells(face, start_from_left=True):
    """某一面的 6 个机体航点（场地系 m），蛇形顺序。

    观察者左起 1,2,3 在上排、右起 6,5,4 在下排（即 1,2,3,6,5,4 蛇形）。
    start_from_left=False 时从上排右端开始（1,2,3 变成 3,2,1），
    用于绕板后从远端切入、少飞一段。
    """
    # 观察者视角的列：1=最左、3=最右（编号绑在物理位置上，与遍历顺序无关）
    if face == "B":                       # 观察者在 x<PANEL_X，朝 +x，观察者左侧=+y
        left_to_right = [(1, COL_Y[2]), (2, COL_Y[1]), (3, COL_Y[0])]
    else:                                 # A 面：观察者在 x>PANEL_X，朝 -x，观察者左侧=-y
        left_to_right = [(1, COL_Y[0]), (2, COL_Y[1]), (3, COL_Y[2])]
    order = left_to_right if start_from_left else list(reversed(left_to_right))

    out = []
    for col, y in order:                    # 上排：编号 = 列号
        bx, by, bz = body_for_camera(face, y, ROW_Z[0])
        out.append((f"{face}{col}", bx, by, bz))
    for col, y in reversed(order):          # 下排：蛇形折返，编号 = 列号 + 3
        bx, by, bz = body_for_camera(face, y, ROW_Z[1])
        out.append((f"{face}{col + 3}", bx, by, bz))
    return out


def yaml_block(name, points, psi_deg, note):
    lines = [f"# {note}", "# 坐标单位 cm，原点=起飞点，+x=起飞时机头方向（自动生成，勿手改）",
             f"# 生成用 psi0 = {psi_deg:.2f} 度",
             "tolerances: {pos_cm: 25.0, z_cm: 10.0, yaw_deg: 180.0, hold_s: 2.0}",
             "loop: false", "field: {x_min: -400.0, x_max: 400.0, y_min: -400.0, y_max: 400.0}",
             "obstacles: []", "waypoints:"]
    # 起飞高度取第一个航点的高度，避免起飞后再改一次高度
    lines.append("  - {name: takeoff, x: 0.0, y: 0.0, z: %d, yaw: 0.0}"
                 % round(points[0][3] * 100))
    for label, fx, fy, fz in points:
        x, y = to_flight(fx, fy, psi_deg)
        lines.append(f"  - {{name: {label}, x: {x}, y: {y}, z: {round(fz * 100)}, yaw: 0.0}}")
    lx, ly = to_flight(LANDING_FIELD[0], LANDING_FIELD[1], psi_deg)
    lines.append(f"  - {{name: land, x: {lx}, y: {ly}, z: {FLY_Z_CM}.0, yaw: 0.0}}")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--psi", type=float, required=True, help="起飞时机头朝向(度)")
    ap.add_argument("--mode", default="both", choices=("one_shot", "per_cell", "both"))
    ap.add_argument("--out-dir", default=".")
    args = ap.parse_args()
    out = Path(args.out_dir)

    # 连续扫描航线：B 面从近端到远端，绕板后 A 面从远端切入（少飞一段）
    pts = face_cells("B", True) + face_cells("A", False)
    text = yaml_block("shelf", pts, args.psi,
                      "连续扫描航线：B1-B3 / B6-B4 / 绕板 / A3-A1 / A4-A6 / 降落"
                      "（一帧一个码，相机距板面 0.35m）")
    (out / "waypoints_shelf_scan.yaml").write_text(text, encoding="utf-8")
    print("已生成 waypoints_shelf_scan.yaml")

    print("\n机体航点（飞控坐标 cm）与对应相机高度：")
    print("  名字     场地坐标(m)        飞控坐标(cm)        机体高度  相机高度")
    for label, fx, fy, fz in pts:
        x, y = to_flight(fx, fy, args.psi)
        cam_z = fz - CAM_DOWN
        print(f"  {label:5s} ({fx:5.2f},{fy:5.2f})  ({x:8.1f},{y:8.1f})  "
              f"{fz * 100:6.1f}cm {cam_z * 100:6.1f}cm")


if __name__ == "__main__":
    main()
