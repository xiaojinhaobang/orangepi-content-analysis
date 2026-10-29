#!/usr/bin/env python3
"""Static checks for the vertical-transition speed candidate."""
import pathlib


def main():
    source = pathlib.Path(__file__).with_name(
        "stage3_simple.launch.py").read_text(encoding="utf-8")
    assert "'kp_z': 1.3" in source
    assert "'kd_z': 0.05" in source
    assert "'max_vertical_velocity': 40.0" in source
    print("VERTICAL_PARAMS_OK")


if __name__ == "__main__":
    main()
