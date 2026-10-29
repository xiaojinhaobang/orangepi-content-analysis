#!/usr/bin/env python3
"""Summarize the 2026-10-24 fast-entry flight without changing any state."""

from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    pos = (len(ordered) - 1) * q
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def load_samples(path: Path) -> tuple[int, dict[str, list[tuple[float, object]]]]:
    samples: dict[str, list[tuple[float, object]]] = {}
    mission_cmd_ns: int | None = None
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            topic = record["topic"]
            value = record["value"]
            samples.setdefault(topic, []).append((record["wall_ns"] / 1e9, value))
            if topic == "/mission_cmd" and value == [1, 156]:
                mission_cmd_ns = record["wall_ns"]
    if mission_cmd_ns is None:
        raise RuntimeError("mission_cmd [1,156] not found")
    t0 = mission_cmd_ns / 1e9
    for topic in samples:
        samples[topic] = [(timestamp - t0, value) for timestamp, value in samples[topic]]
    return mission_cmd_ns, samples


def values_in(
    samples: dict[str, list[tuple[float, object]]],
    topic: str,
    lo: float,
    hi: float,
) -> list[tuple[float, object]]:
    return [item for item in samples.get(topic, []) if lo <= item[0] <= hi]


def height_stats(
    samples: dict[str, list[tuple[float, object]]],
    lo: float,
    hi: float,
) -> dict[str, float | int]:
    values = [float(value) for _, value in values_in(samples, "/height", lo, hi)]
    return {
        "n": len(values),
        "median": percentile(values, 0.5),
        "p10": percentile(values, 0.1),
        "p90": percentile(values, 0.9),
        "min": min(values),
        "max": max(values),
        "mean": statistics.fmean(values),
    }


def first_crossing(
    samples: dict[str, list[tuple[float, object]]],
    threshold: float,
    start: float,
    end: float,
) -> tuple[float, float] | None:
    for timestamp, value in values_in(samples, "/height", start, end):
        if float(value) >= threshold:
            return timestamp, float(value)
    return None


def first_descent_after_peak(
    samples: dict[str, list[tuple[float, object]]],
    start: float,
    end: float,
    below: float,
) -> tuple[float, float] | None:
    rows = values_in(samples, "/height", start, end)
    if not rows:
        return None
    peak_index = max(range(len(rows)), key=lambda index: float(rows[index][1]))
    peak_t, peak_h = rows[peak_index]
    for timestamp, value in rows[peak_index + 1 :]:
        if float(value) < below:
            return timestamp, peak_h
    return peak_t, peak_h


def status_transitions(
    samples: dict[str, list[tuple[float, object]]],
    lo: float,
    hi: float,
) -> list[dict[str, object]]:
    last = None
    transitions = []
    for timestamp, value in values_in(samples, "/fc_status", lo, hi):
        current = tuple(int(item) for item in value)
        if current != last:
            transitions.append({"t_s": timestamp, "status": list(current)})
            last = current
    return transitions


def controller_transitions(
    samples: dict[str, list[tuple[float, object]]],
    lo: float,
    hi: float,
) -> list[dict[str, object]]:
    last = None
    transitions = []
    for timestamp, value in values_in(samples, "/active_controller", lo, hi):
        current = int(value)
        if current != last:
            transitions.append({"t_s": timestamp, "active_controller": current})
            last = current
    return transitions


def target_z_stats(
    samples: dict[str, list[tuple[float, object]]],
    lo: float,
    hi: float,
) -> dict[str, float | int]:
    values = [
        float(value[2])
        for _, value in values_in(samples, "/target_velocity", lo, hi)
        if isinstance(value, list) and len(value) >= 3
    ]
    return {
        "n": len(values),
        "min": min(values),
        "max": max(values),
        "nonzero": sum(abs(item) > 1e-9 for item in values),
    }


def height_diag_rows(
    samples: dict[str, list[tuple[float, object]]],
    timestamps: list[float],
) -> list[dict[str, float | int]]:
    diag = samples.get("/fc_height_diag", [])
    rows = []
    for wanted in timestamps:
        if not diag:
            break
        timestamp, value = min(diag, key=lambda item: abs(item[0] - wanted))
        rows.append(
            {
                "requested_t_s": wanted,
                "diag_t_s": timestamp,
                "fus_cm": int(value[0]),
                "add_cm": int(value[1]),
                "state": int(value[2]),
            }
        )
    return rows


def main() -> None:
    path = Path(sys.argv[1])
    mission_cmd_ns, samples = load_samples(path)

    # The handoff was logged at +2.580 s; use the first sustained armed interval.
    handoff_t = 2.580
    status_rows = values_in(samples, "/fc_status", 0.0, 120.0)
    armed_rows = [item for item in status_rows if int(item[1][1]) == 1]
    if not armed_rows:
        raise RuntimeError("no armed interval after mission command")
    armed_start = armed_rows[0][0]
    armed_end = next(
        (
            timestamp
            for timestamp, value in status_rows
            if timestamp > armed_start and int(value[1]) != 1
        ),
        armed_rows[-1][0],
    )

    baseline_lo, baseline_hi = -2.0, 0.0
    laser_baseline = height_stats(samples, baseline_lo, baseline_hi)
    plateau_lo = handoff_t + 0.5
    plateau_hi = max(plateau_lo, armed_end - 0.5)
    descent = first_descent_after_peak(samples, 5.0, armed_end, 120)
    stable_hi = descent[0] if descent is not None else plateau_hi
    diag_baseline = [
        item for item in values_in(samples, "/fc_height_diag", baseline_lo, baseline_hi)
    ]
    diag_plateau = [
        item
        for item in values_in(samples, "/fc_height_diag", plateau_lo, plateau_hi)
    ]

    crossings = {
        str(threshold): first_crossing(samples, threshold, 0.0, armed_end)
        for threshold in (132, 136, 142, 152, 156)
    }

    print(
        json.dumps(
            {
                "mission_cmd_ns": mission_cmd_ns,
                "armed_start_t_s": armed_start,
                "armed_end_t_s": armed_end,
                "handoff_logged_t_s": handoff_t,
                "laser_baseline": laser_baseline,
                "laser_at_handoff_plus_0_5_to_landing": height_stats(
                    samples, plateau_lo, plateau_hi
                ),
                "laser_plateau_5s_to_descent": height_stats(
                    samples, 5.0, stable_hi
                ),
                "descent_below_120cm": descent,
                "height_crossings": crossings,
                "fusion_baseline": {
                    "n": len(diag_baseline),
                    "fus_median_cm": percentile(
                        [float(value[0]) for _, value in diag_baseline], 0.5
                    ),
                    "add_median_cm": percentile(
                        [float(value[1]) for _, value in diag_baseline], 0.5
                    ),
                },
                "fusion_plateau": {
                    "n": len(diag_plateau),
                    "fus_median_cm": percentile(
                        [float(value[0]) for _, value in diag_plateau], 0.5
                    ),
                    "add_median_cm": percentile(
                        [float(value[1]) for _, value in diag_plateau], 0.5
                    ),
                    "state_values": sorted({int(value[2]) for _, value in diag_plateau}),
                },
                "status_transitions": status_transitions(samples, -1.0, 120.0),
                "controller_transitions": controller_transitions(
                    samples, -1.0, 120.0
                ),
                "target_velocity_z": target_z_stats(
                    samples, handoff_t, armed_end
                ),
                "diag_checkpoints": height_diag_rows(
                    samples,
                    [0.0, 1.0, 2.0, 2.58, 3.67, 5.0, 8.0, 12.0, 16.0, armed_end],
                ),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
