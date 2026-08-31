#!/usr/bin/env python3
"""Read two grid traces and say whether the second one holds a line better.

    python3 scripts/analyze_heading_ab.py A=outputs/trace_trot_headA.npz \
                                          B=outputs/trace_trot_headB.npz

The question is the one outputs/lip_failure.md 3 left open: TROT arrives at the lip
having drifted to y = 3.4-4.0 in a 4 m lane, so `STEP_TROT_MAX` is scoring a lane
departure and not a step.  What is measured here is therefore the APPROACH -- from the
spawn to the obstacle line -- and not the obstacle:

    lateral drift   |y - y_spawn| at the moment the base first crosses the obstacle x
    curvature       total yaw over path length, deg/m, the benchmark's own budget unit
    yaw at the lip  heading error there, against heading_hold.md's 3.20 deg/m prediction

Yaw comes from ``sim.replay.quat_to_rpy_deg``.  The traces are scalar-LAST and a
hand-written scalar-first conversion has already cost this project one wrong mechanism
(CLAUDE.md 6.5, harness_findings.md 11).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.replay import quat_to_rpy_deg  # noqa: E402

#: A robot below this has fallen; the same value run_calibration scores with.
FALL_HEIGHT_M = 0.15


def summarise(path: Path) -> dict:
    z = np.load(path, allow_pickle=False)
    pos = z["root_pos_w"]                      # (T, n, 3)
    quat = z["root_quat_w"]                    # (T, n, 4), scalar-last
    spawn, off = z["spawn"], z["offsets"]
    obstacle_x = float(z["obstacle_x"])
    T, n, _ = pos.shape
    lip_x = off[:, 0] + obstacle_x
    out = []
    for k in range(n):
        p, q = pos[:, k, :], quat[:, k, :]
        yaw = np.unwrap(np.radians(np.array([quat_to_rpy_deg(q[t][None, :])[2][0]
                                             for t in range(T)])))
        yaw_deg = np.degrees(yaw - yaw[0])
        alive = p[:, 2] >= FALL_HEIGHT_M
        # last step before it fell; after that the pose is a corpse tumbling
        end = int(np.argmax(~alive)) if (~alive).any() else T
        end = max(end, 2)
        # first crossing of the obstacle line while still upright
        past = np.where((p[:end, 0] >= lip_x[k]))[0]
        i_lip = int(past[0]) if past.size else end - 1
        seg = slice(0, i_lip + 1)
        path_len = float(np.abs(np.diff(np.linalg.norm(
            p[seg, :2] - p[0, :2], axis=1))).sum()) or np.nan
        s = np.linalg.norm(np.diff(p[seg, :2], axis=0), axis=1).sum()
        out.append({
            "reached_lip": bool(past.size),
            "y_drift_m": float(abs(p[i_lip, 1] - p[0, 1])),
            "x_at_end_m": float(p[end - 1, 0] - off[k, 0]),
            "y_at_end_m": float(p[end - 1, 1] - off[k, 1]),
            "yaw_at_lip_deg": float(yaw_deg[i_lip]),
            "yaw_abs_max_deg": float(np.abs(yaw_deg[:end]).max()),
            "curv_deg_per_m": float(abs(yaw_deg[i_lip]) / s) if s > 1e-6 else np.nan,
            "path_m": float(s),
            "fell": bool((~alive).any()),
        })
    return {"probe": [str(x) for x in z["probe"]], "skill": [str(x) for x in z["skill"]],
            "level": z["level_m"], "rows": out}


def main() -> int:
    labels, paths = [], []
    for a in sys.argv[1:]:
        lab, _, pth = a.partition("=")
        labels.append(lab); paths.append(Path(pth))
    if not paths:
        raise SystemExit(__doc__)
    res = [summarise(p) for p in paths]
    print(f"{'probe':16s} {'lvl':>6s} " + " ".join(
        f"| {l}: {'y drift':>8s} {'curv':>7s} {'yaw@lip':>8s} {'end x,y':>13s}" for l in labels))
    for j in range(len(res[0]["rows"])):
        line = f"{res[0]['probe'][j]:16s} {res[0]['level'][j]:6.3f} "
        for r in res:
            d = r["rows"][j]
            line += (f"| {d['y_drift_m']:8.2f} {d['curv_deg_per_m']:7.2f} "
                     f"{d['yaw_at_lip_deg']:8.1f} {d['x_at_end_m']:6.2f},{d['y_at_end_m']:6.2f} ")
        print(line)
    print()
    for lab, r in zip(labels, res):
        ys = np.array([d["y_drift_m"] for d in r["rows"]])
        cs = np.array([d["curv_deg_per_m"] for d in r["rows"]])
        lip = sum(d["reached_lip"] for d in r["rows"])
        fell = sum(d["fell"] for d in r["rows"])
        print(f"{lab}: reached the lip {lip}/{len(ys)}, fell {fell}/{len(ys)}, "
              f"y drift at the lip median {np.nanmedian(ys):.2f} m max {np.nanmax(ys):.2f}, "
              f"curvature median {np.nanmedian(cs):.2f} deg/m")
    print("\nbenchmark curvature budget: 0.565 deg/m;  heading_hold.md measured TROT at "
          "3.20 deg/m on flat with the lateral half alone")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
