#!/usr/bin/env python3
"""Offline checks for the planned-line and heading-gate fixes."""
import ast
import math
import os
import pathlib


HERE = pathlib.Path(__file__).resolve().parent
SOURCE = HERE / "body_vel_adapter.py"


def load_functions(names):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    keep = [node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name in names]
    if len(keep) != len(names):
        missing = set(names) - {node.name for node in keep}
        raise AssertionError("missing functions: %s" % sorted(missing))
    module = ast.Module(body=keep, type_ignores=[])
    namespace = {"math": math}
    exec(compile(module, str(SOURCE), "exec"), namespace)
    return namespace


def main():
    ns = load_functions(("planned_line", "heading_mark_ok",
                         "add_cross_track"))
    planned_line = ns["planned_line"]
    mark_ok = ns["heading_mark_ok"]
    cross_track = ns["add_cross_track"]

    # B lower row was accepted at (177.6, 72.5), although the planned row is
    # x=183.5. The corridor must stay on the route endpoint, not the live pose.
    start, direction = planned_line(
        (183.5, 69.9), (183.6, 199.9), (177.6, 72.5))
    assert start == (183.5, 69.9), start
    assert abs(direction[0]) < 0.001, direction
    assert direction[1] > 0.999, direction
    _, _, cross, corr = cross_track(
        0.0, 24.0, 177.6, 100.0, start, direction[0], direction[1], 0.2, 4.0)
    assert abs(cross) > 5.5, cross
    assert corr < 0.0, corr

    # First target and a position-invariant yaw-only target use live pose.
    assert planned_line(None, (10.0, 20.0), (3.0, 4.0)) == (
        (3.0, 4.0), None)
    assert planned_line((10.0, 20.0), (10.0, 20.0), (3.0, 4.0)) == (
        (3.0, 4.0), None)

    # A3 was blocked at err≈5.1 after the previous 5° threshold. The final
    # small margin accepts that sample while still rejecting unsettled motion.
    assert mark_ok(143.0, 40.0, -8.7, 0.0, 5.1, 18.0, 6.0)
    assert mark_ok(143.0, 40.0, 12.3, 0.0, 2.8, 18.0, 6.0)
    assert not mark_ok(30.0, 40.0, 0.0, 0.0, 0.0, 12.0, 4.0)
    assert not mark_ok(143.0, 40.0, 22.0, 0.0, 0.0, 18.0, 6.0)
    assert not mark_ok(143.0, 40.0, 0.0, 0.0, 7.0, 18.0, 6.0)
    source = SOURCE.read_text(encoding="utf-8")
    assert "BVA_ROTATE_XY_MAX', 12.0" in source
    assert "BVA_MARK_GYRO_MAX', 18.0" in source
    assert "BVA_MARK_HEADING_ERR_MAX', 6.0" in source

    mission_path = (HERE.parent.parent / "mission_candidates" /
                    "20261026_vision_rot_v3" / "mission_node.py")
    if not mission_path.exists():
        mission_path = HERE / "mission_node.py"
    mission = mission_path.read_text(encoding="utf-8")
    assert "MARK_WINDOW_TOL = 0.10" in mission
    assert "and self._aim_fresh()" in mission
    assert "if not self.running:" in mission
    assert "pts.append(('过渡-稳定'" not in mission
    assert "('引导-稳定'" not in mission
    assert "if '掉头' in name or '稳定' in name:" in mission
    assert "ROTATE_POS_TOL_CM = 14.0" in mission
    assert "A_LOWER_SCAN_DOWN_CM = 3.0" in mission
    assert "B_UPPER_SCAN_DOWN_CM = 5.0" in mission
    assert "ROTATE_YTOL_DEG = 5.0" in mission
    assert "-p confirm_frames:=1" in mission
    assert "-p publish_debug:=false" in mission
    assert "or self._label_by_pose() != label" not in mission
    assert "if name == 'B进-0':" in mission
    assert "ztol_cm: 4.0" in mission

    # Historical premature narrow-window triggers: B6 fired at pixel x=724
    # and B3 at x=492. Both were outside the new 0.10 window, so prediction
    # gets the next frame/time sample instead of firing off-centre.
    def norm_dx(px):
        return (px - 640.0) / 640.0
    assert abs(norm_dx(724.0) - (-0.06)) > 0.10
    assert abs(norm_dx(492.0) - (-0.06)) > 0.10

    tree = ast.parse(mission)
    plan_nodes = [
        node for node in tree.body
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.FunctionDef))
    ]
    plan_module = ast.Module(body=plan_nodes, type_ignores=[])
    plan_ns = {
        "math": math,
        "os": os,
        "re": __import__("re"),
        "shutil": __import__("shutil"),
        "subprocess": __import__("subprocess"),
        "threading": __import__("threading"),
        "time": __import__("time"),
        "json": __import__("json"),
    }
    exec(compile(plan_module, "mission_node.py", "exec"), plan_ns)
    pts = plan_ns["plan_points"]()
    names = [point[0] for point in pts]
    assert "过渡-掉头" in names
    assert "过渡-稳定" not in names
    assert names.count("过渡-掉头") == 1
    b_entry = names.index("B进-1")
    assert names[b_entry - 1] == "过渡-掉头"
    a_lower = {name: z for name, _, _, z in pts
               if name in ("A进-1", "A出-1")}
    assert a_lower == {"A进-1": 100, "A出-1": 100}, a_lower
    b_upper = {name: z for name, _, _, z in pts
               if name in ("B进-0", "B出-0")}
    assert b_upper == {"B进-0": 136.5, "B出-0": 136.5}, b_upper

    wp_source = mission_path.with_name("waypoint_nav.py").read_text(
        encoding="utf-8")
    wp_tree = ast.parse(wp_source)
    choose_node = next(
        node for node in wp_tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "choose_yaw_deg")
    wp_module = ast.Module(body=[choose_node], type_ignores=[])
    wp_ns = {"math": math}
    exec(compile(wp_module, "waypoint_nav.py", "exec"), wp_ns)
    choose_yaw = wp_ns["choose_yaw_deg"]
    assert choose_yaw(12.0, -179.0, 0.1) == (-179.0, "fused")
    assert choose_yaw(12.0, -179.0, 1.0) == (12.0, "tf")
    assert choose_yaw(12.0, float("nan"), 0.1) == (12.0, "tf")

    print("VISION_ROT_GEOMETRY_OK")


if __name__ == "__main__":
    main()
