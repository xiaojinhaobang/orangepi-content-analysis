#!/usr/bin/env python3
"""Offline checks for the v4 B-upper vertical-aim candidate (no ROS)."""
import importlib.util
import pathlib
import sys
import types


def _stub_imports():
    """Load pure helpers on Windows without ROS or Linux fcntl."""
    fcntl = types.ModuleType("fcntl")
    sys.modules["fcntl"] = fcntl

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy.node = rclpy_node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node

    tf2_ros = types.ModuleType("tf2_ros")
    tf2_ros.Buffer = type("Buffer", (), {})
    tf2_ros.TransformListener = type("TransformListener", (), {})
    sys.modules["tf2_ros"] = tf2_ros

    msg = types.ModuleType("std_msgs.msg")
    for name in (
            "Bool", "Empty", "Float32MultiArray", "Int16",
            "Int16MultiArray", "String", "UInt8", "UInt8MultiArray"):
        setattr(msg, name, type(name, (), {}))
    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = msg


def load_module():
    _stub_imports()
    path = pathlib.Path(__file__).with_name("mission_node.py")
    spec = importlib.util.spec_from_file_location("vision_rot_v4_mission", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main():
    m = load_module()

    # Geometry: only B upper moves down; the other five row targets stay put.
    assert abs(m.row_body_z("B", 0) - 133.5) < 1e-6, m.row_body_z("B", 0)
    assert abs(m.row_body_z("B", 1) - 98.0) < 1e-6
    assert abs(m.row_body_z("A", 0) - 141.5) < 1e-6
    assert abs(m.row_body_z("A", 1) - 100.0) < 1e-6
    b_upper = m.face_pass("B", 0, [1, 2, 3], reverse=True)
    assert b_upper[0][0] == "B进-0" and b_upper[-1][0] == "B出-0"
    assert all(abs(p[3] - 133.5) < 1e-6 for p in b_upper)

    # Cell names are unchanged: left/middle/right upper = B1/B2/B3.
    assert m.row_of_label("B1") == 0
    assert m.row_of_label("B3") == 0
    assert m.row_of_label("B4") == 1
    assert m.label_col("B", 1) == 1
    assert m.label_col("B", 2) == 2
    assert m.label_col("B", 3) == 3

    # Measured v3 pulse geometry drove the aim target.
    assert m.aim_target_dy_for("B1") == -0.10
    assert m.aim_target_dy_for("B2") == -0.13
    assert m.aim_target_dy_for("B3") == -0.28
    assert m.aim_target_dy_for("B4") == 0.0
    assert m.aim_target_dy_for("A1") == 0.0

    # v4 firing/old-entry points still pass/fail the per-cell v5 aim lines.
    assert m.mark_dy_ok("B3", -0.34)
    assert not m.mark_dy_ok("B2", 0.01)
    assert not m.mark_dy_ok("B1", 0.05)
    assert m.mark_dy_ok("B1", -0.10)
    assert m.mark_dy_ok("B2", -0.13)
    assert m.mark_dy_ok("B3", -0.28)

    # Lower/A rows keep the historical absolute window.
    assert m.mark_dy_ok("B4", 0.19)
    assert m.mark_dy_ok("A1", 0.21)
    assert not m.mark_dy_ok("A1", 0.50)

    obj = m.Mission.__new__(m.Mission)
    obj.aim_dy = -0.10
    assert obj._mark_dy_ok("B1")
    obj.aim_dy = 0.06
    assert not obj._mark_dy_ok("B1")

    # New 133.5cm row height must be acquired; v4's 128-130cm is rejected.
    obj.height = 128.0
    assert not obj._row_height_ready("B1")
    obj.height = 133.5
    assert obj._row_height_ready("B2")
    obj.height = 136.0
    assert not obj._row_height_ready("B1")
    obj.height = None
    assert not obj._row_height_ready("B3")
    assert obj._row_height_ready("B4")

    # Retry path evaluates dy relative to the per-row aim line.
    assert m.mark_after_cross_ok(
        0.20, -0.02, 0.0, 1, 0.8, 0.12, 0.35, 0.02)

    src = pathlib.Path(__file__).with_name("mission_node.py").read_text(
        encoding="utf-8")
    assert "if name == 'B进-1':" in src
    assert "tol_cm: 25.0" in src
    assert "ztol_cm: 2.0" in src

    print("VISION_AIM_UNIT_OK", {
        "B_upper_z": m.row_body_z("B", 0),
        "B_upper_dy": m.B_UPPER_AIM_DY,
        "B_upper_dy_tol": m.B_UPPER_AIM_DY_TOL,
    })


if __name__ == "__main__":
    main()
