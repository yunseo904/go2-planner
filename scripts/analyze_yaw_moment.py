#!/usr/bin/env python3
"""Read two run_planner_replay traces and ask whether the second one holds a LINE.

    python3 scripts/analyze_yaw_moment.py A=outputs/yawm/trace_base.npz \
                                          B=outputs/yawm/trace_h5.npz

Why this exists rather than reading the curvature already in the results row.

``curvature`` there is ``|mean yaw rate| / v_x``, and a heading loop drives the MEAN yaw
rate to zero by construction.  A robot that yaws +8 deg and then -8 deg back has a mean
of zero and a curvature of zero and has not held a line.  That is exactly the shape of
error CLAUDE.md 6.5 is about: the reported number is *almost* the quantity wanted.

So the same claim is read a second way, from the trajectory itself:

    heading excursion   max |yaw - yaw_0| over the run.  A bound, not a mean.
    total |yaw| swept   sum of |d yaw|, which a mean cannot cancel
    lateral drift       max |y - y_0| in the SPAWN's frame -- the quantity
                        trot_straight.md 5 says actually fails the benchmark's goals,
                        and the one a heading controller has no term for
    path curvature      net yaw over path length, the budget's own unit

Yaw comes from ``sim.replay.quat_to_rpy_deg``.  These traces are scalar-LAST and a
hand-written scalar-first conversion has already cost this project one wrong mechanism
(harness_findings.md 11).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.replay import quat_to_rpy_deg  # noqa: E402

#: The benchmark's straightness budget, deg/m -- outputs/analyze_heading_budget.py
BUDGET_DEG_PER_M = 0.565


def summarise(path: Path) -> dict:
    z = np.load(path, allow_pickle=False)
    pos, quat = np.asarray(z["root_pos_w"]), np.asarray(z["root_quat_w"])
    dt = float(z["dt"])
    T = pos.shape[0]
    yaw = np.unwrap(np.radians(np.array(
        [quat_to_rpy_deg(quat[t][None, :])[2][0] for t in range(T)])))
    yaw_deg = np.degrees(yaw - yaw[0])
    # Lateral offset in the SPAWN's frame: the world y axis is only the right one if the
    # robot happened to be launched along x, and it is launched along its settle heading.
    c, s = np.cos(yaw[0]), np.sin(yaw[0])
    d = pos[:, :2] - pos[0, :2]
    along = d[:, 0] * c + d[:, 1] * s
    lateral = -d[:, 0] * s + d[:, 1] * c
    step = np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1)
    path_len = float(step.sum())
    return {
        "T": T, "t_s": T * dt,
        "path_len_m": path_len,
        "along_m": float(along[-1]),
        "heading_excursion_deg": float(np.abs(yaw_deg).max()),
        "yaw_swept_deg": float(np.abs(np.diff(yaw_deg)).sum()),
        "net_yaw_deg": float(yaw_deg[-1]),
        "lateral_max_m": float(np.abs(lateral).max()),
        "lateral_end_m": float(lateral[-1]),
        "curv_net_deg_per_m": abs(yaw_deg[-1]) / path_len if path_len > 1e-6 else np.nan,
    }


def main() -> int:
    args = [a for a in sys.argv[1:] if "=" in a]
    if not args:
        print(__doc__)
        return 2
    res = [(a.split("=", 1)[0], summarise(Path(a.split("=", 1)[1]))) for a in args]
    keys = ["t_s", "path_len_m", "along_m", "net_yaw_deg", "curv_net_deg_per_m",
            "heading_excursion_deg", "yaw_swept_deg", "lateral_max_m", "lateral_end_m"]
    w = max(len(k) for k in keys) + 2
    print(f"{'':{w}}" + "".join(f"{lbl:>14s}" for lbl, _ in res))
    for k in keys:
        print(f"{k:{w}}" + "".join(f"{r[k]:14.3f}" for _, r in res))
    print(f"\nbenchmark straightness budget {BUDGET_DEG_PER_M} deg/m.")
    print("net_yaw / heading_excursion: a loop that oscillates has a small net and a "
          "large excursion.\nlateral_*: what trot_straight.md 5 says actually fails the "
          "goals; a heading\ncontroller has no term for it, so it is reported, not claimed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
