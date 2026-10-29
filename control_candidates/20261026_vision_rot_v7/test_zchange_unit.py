#!/usr/bin/env python3
"""Offline checks for BVA v6 vertical row-change XY lock (no ROS)."""
import importlib.util
import pathlib
import sys
import types


def _stub_imports():
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

    geometry = types.ModuleType("geometry_msgs")
    geometry_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msg.Vector3 = type("Vector3", (), {})
    geometry.msg = geometry_msg
    sys.modules["geometry_msgs"] = geometry
    sys.modules["geometry_msgs.msg"] = geometry_msg

    msg = types.ModuleType("std_msgs.msg")
    for name in (
            "Bool", "Float32", "Float32MultiArray", "Int16",
            "UInt8", "UInt8MultiArray"):
        setattr(msg, name, type(name, (), {}))
    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = msg


def load_module():
    _stub_imports()
    path = pathlib.Path(__file__).with_name("body_vel_adapter.py")
    spec = importlib.util.spec_from_file_location("bva_v6", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main():
    m = load_module()
    prev = (183.7, 199.8, 98.0)
    up = (183.7, 199.8, 134.0)
    down = (183.7, 199.8, 100.0)

    assert m.is_upward_row_change(prev, up, 98.0, 60.0, 2)
    assert not m.is_upward_row_change(prev, down, 135.0, 60.0, 2)
    assert not m.is_upward_row_change(prev, up, 30.0, 60.0, 2)
    assert not m.is_upward_row_change(prev, up, 98.0, 60.0, 1)
    assert not m.is_upward_row_change(prev, (196.0, 199.8, 134.0),
                                      98.0, 60.0, 2)
    assert m.is_upward_row_change(
        prev, (183.7, 199.8, 98.0 + 20.0), 98.0, 60.0, 2)

    # One centimetre offset: command points back to the locked point, bounded.
    vx, vy = m.bounded_position_hold(-1.0, 0.0, 0.6, 8.0, 0.5)
    assert vx < 0.0 and abs(vx) <= 8.0 and abs(vy) < 1e-9
    assert m.bounded_position_hold(-0.2, 0.0, 0.6, 8.0, 0.5) == (0.0, 0.0)
    assert m.heading_mark_ok(142.0, 60.0, 1.0, 0.0, 6.0, 18.0, 7.0)
    assert not m.heading_mark_ok(142.0, 60.0, 1.0, 0.0, 7.1, 18.0, 7.0)

    src = pathlib.Path(__file__).with_name("body_vel_adapter.py").read_text(
        encoding="utf-8")
    assert "换高竖直锁" in src
    assert "BVA_ZCHANGE_XY_MAX" in src
    assert "abs(self.height - z_tgt) <= self.zchange_release_cm" in src
    assert "BVA_MARK_HEADING_ERR_MAX', 7.0" in src

    print("ZCHANGE_UNIT_OK", {
        "min_dz": 20.0,
        "max_xy": 12.0,
        "hold_max": 8.0,
        "release_cm": 2.0,
    })


if __name__ == "__main__":
    main()
