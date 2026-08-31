#!/usr/bin/env python3
"""Score TROT heading conditions on the same cell, over the window it is still upright.

    python3 scripts/analyze_trot_heading.py lateral=outputs/trace_ab1_A.npz ...

Everything is measured to **one second before the fall**.  Including the fall makes the
controller look far worse than it is: the terminal tumble adds 30-50 deg of yaw in half a
second, and read whole-run it turned a 3.4 deg/m controller into a 4.7 deg/m one and gave
two conditions opposite-signed drift that were in fact tracking together the whole way.
Same trap as CLAUDE.md 6.5 -- a number that is almost, but not, the quantity wanted.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.replay import quat_to_rpy_deg  # noqa: E402

FALL_HEIGHT_M = 0.15
#: Seconds trimmed off the end.  The fall is a rigid-body tumble, not a control response.
TRIM_S = 1.0
#: What the benchmark allows, deg/m.
BUDGET = 0.565


def one(path: Path, k: int = 0) -> dict:
    z = np.load(path, allow_pickle=False)
    dt = float(z["dt"])
    p, q = z["root_pos_w"][:, k, :], z["root_quat_w"][:, k, :]
    T = p.shape[0]
    yaw = np.unwrap(np.radians([quat_to_rpy_deg(q[t][None, :])[2][0] for t in range(T)]))
    yd = np.degrees(yaw - yaw[0])
    alive = p[:, 2] >= FALL_HEIGHT_M
    end = int(np.argmax(~alive)) if (~alive).any() else T
    c = max(end - int(TRIM_S / dt), 2)
    s = float(np.linalg.norm(np.diff(p[:c, :2], axis=0), axis=1).sum())
    out = {"alive_s": end * dt, "path_m": s, "yaw_max_deg": float(np.abs(yd[:c]).max()),
           "curv": float(np.abs(yd[:c]).max() / max(s, 1e-9)),
           "y_drift_m": float(abs(p[c - 1, 1] - p[0, 1])),
           "x_reached_m": float(p[c - 1, 0] - p[0, 0]),
           "vx": float(p[c - 1, 0] - p[0, 0]) / max(c * dt, 1e-9)}
    if "cap_hits" in z.files and z["cap_hits"].shape[-1] >= 4:
        ch = z["cap_hits"]
        sw = max(int(ch[:c, k, 1].sum()), 1)
        out["head_sat"] = float(ch[:c, k, 2].sum()) / sw * 100
        out["len_sat"] = float(ch[:c, k, 3].sum()) / sw * 100
    return out


def main() -> int:
    args = [a.partition("=") for a in sys.argv[1:]]
    if not args:
        raise SystemExit(__doc__)
    print(f"{'condition':28s} {'alive':>7s} {'x':>6s} {'vx':>6s} {'|yaw|max':>9s} "
          f"{'curv':>7s} {'y drift':>8s} {'head sat':>9s} {'len sat':>8s}")
    for lab, _, pth in args:
        d = one(Path(pth))
        print(f"{lab:28s} {d['alive_s']:6.2f}s {d['x_reached_m']:6.2f} {d['vx']:6.2f} "
              f"{d['yaw_max_deg']:8.1f} {d['curv']:7.2f} {d['y_drift_m']:8.2f} "
              f"{d.get('head_sat', float('nan')):8.1f}% {d.get('len_sat', float('nan')):7.1f}%")
    print(f"\nbudget {BUDGET} deg/m; heading_hold.md measured TROT at 3.20 deg/m on the "
          f"60-cycle flat rig with the lateral half alone")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
