#!/usr/bin/env python3
"""Offline checks for sparse-frame mark prediction; no ROS node is created."""
import importlib.util
import pathlib
import sys


def load_module():
    path = pathlib.Path(__file__).with_name("mission_node.py")
    spec = importlib.util.spec_from_file_location("fast_mission", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def make_obj(module, label="B4", code=7):
    obj = module.Mission.__new__(module.Mission)
    obj._mark_track = None
    obj._mark_predict_timer = None
    obj._mark_predict_seq = 0
    obj.pending_laser = label
    obj.bound = {label: code}
    obj.scheduled = []

    def cancel():
        obj._mark_predict_seq += 1
        obj._mark_predict_timer = None

    obj._cancel_mark_prediction = cancel
    obj._schedule_mark_prediction = (
        lambda label, code, dx, dy, delay:
        obj.scheduled.append((label, code, round(dx, 3), round(dy, 3), round(delay, 3)))
    )
    return obj


def run_case(module, samples, label="B4", code=7):
    obj = make_obj(module, label, code)
    clock = iter(t for t, _, _ in samples)
    old_monotonic = module.time.monotonic
    module.time.monotonic = lambda: next(clock)
    try:
        for _, dx, dy in samples:
            obj._on_qr_center_cross(label, code, dx, dy)
    finally:
        module.time.monotonic = old_monotonic
    return obj.scheduled


def main():
    m = load_module()

    # Sparse B4-like pair: QR crosses the target between frames.
    scheduled = run_case(
        m,
        [(100.000, 0.52, 0.13), (100.730, 0.17, 0.20)],
    )
    assert len(scheduled) == 1, scheduled
    label, code, dx, dy, delay = scheduled[-1]
    assert (label, code) == ("B4", 7)
    assert 0.30 <= delay <= 0.60, scheduled

    # Faster pair: predict through a wider inter-frame step.
    scheduled = run_case(
        m,
        [(200.000, 0.18, 0.18), (200.167, -0.02, 0.18)],
    )
    assert len(scheduled) == 1, scheduled
    assert scheduled[-1][-1] <= 0.25, scheduled

    # Vertical error must disable tracking/prediction.
    scheduled = run_case(
        m,
        [(300.000, 0.20, 0.60), (300.200, -0.02, 0.60)],
    )
    assert scheduled == [], scheduled

    print("PREDICTION_UNIT_OK", scheduled)


if __name__ == "__main__":
    main()
