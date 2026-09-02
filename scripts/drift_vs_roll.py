#!/usr/bin/env python3
"""Is the cross-track drift walking, or is it the robot toppling over?

    python3 scripts/drift_vs_roll.py outputs/xt/trace_walk_s1.npz [more.npz ...]

`outputs/cross_track.md` measures 0.53 m of lateral offset per metre forward with 149/200
cells the same way, and concludes the robot cannot walk straight.  That number is read at
the END of the episode.  The `alive` mask admits |roll| and |pitch| up to 60 deg, and a
0.30 m base at 55 deg of roll has already carried its centre a quarter of a metre sideways
without a foot going anywhere.

So the offset is re-accumulated over steps that satisfy a roll cut, INCREMENT BY INCREMENT
rather than by truncating at the first excursion -- truncating would shorten the window
every time the gait leans transiently and would understate forward travel for the same
reason it understates lateral.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.replay import quat_to_rpy_deg  # noqa: E402

CUTS = (("alive only (|roll|<60)", 60.0), ("|roll| < 30", 30.0), ("|roll| < 20", 20.0),
        ("|roll| < 15", 15.0), ("|roll| < 10", 10.0))


def report(path: Path) -> None:
    z = np.load(path, allow_pickle=False)
    pos, quat, alive = z["root_pos_w"], z["root_quat_w"], z["alive"]
    T, n, _ = pos.shape
    yaw0 = np.radians(np.array([quat_to_rpy_deg(quat[0, k][None, :])[2][0]
                                for k in range(n)]))
    c0, s0 = np.cos(yaw0), np.sin(yaw0)
    rp = [quat_to_rpy_deg(quat[:, k, :]) for k in range(n)]
    M = np.median
    print(f"\n=== {path.name} ({n} cells) ===")
    print(f"{'cut':>24} {'fwd m':>8} {'lat m':>9} {'|lat| m':>8} {'lat/fwd':>9} {'+lat':>10}")
    for name, lim in CUTS:
        F, L, sgn, tot = [], [], 0, 0
        for k in range(n):
            r, p, _ = rp[k]
            ok = alive[:, k] & (np.abs(r) < lim) & (np.abs(p) < lim)
            d = np.diff(pos[:, k, :2], axis=0)
            m = ok[1:] & ok[:-1]
            if m.sum() < 20:
                continue
            f = float((d[m, 0] * c0[k] + d[m, 1] * s0[k]).sum())
            l = float((-d[m, 0] * s0[k] + d[m, 1] * c0[k]).sum())
            F.append(f); L.append(l)
            if abs(f) > 0.15:
                tot += 1; sgn += l > 0
        print(f"{name:>24} {M(F):8.3f} {M(L):+9.3f} {M(np.abs(L)):8.3f} "
              f"{M(np.abs(L))/M(F):9.3f} {sgn:>4}/{tot:<5}")
    # the topple signature
    ag, tot, rr = 0, 0, []
    for k in range(n):
        r, _p, _ = rp[k]
        a = alive[:, k]
        e = max(int(np.argmax(~a)) if (~a).any() else T, 3)
        d = pos[e - 1, k, :2] - pos[0, k, :2]
        lat = -d[0] * s0[k] + d[1] * c0[k]
        if abs(lat) > 0.05 and abs(float(r[e - 1])) > 5:
            tot += 1; ag += (lat > 0) == (float(r[e - 1]) > 0)
            rr.append((float(r[e - 1]), lat))
    if tot:
        rr = np.asarray(rr)
        print(f"  final lateral sign vs final roll sign: {ag}/{tot}, "
              f"corr {np.corrcoef(rr[:, 0], rr[:, 1])[0, 1]:+.3f}; "
              f"median |roll| at the last upright step {M(np.abs(rr[:, 0])):.1f} deg")


def main() -> int:
    args = sys.argv[1:] or ["outputs/xt/trace_walk_s1.npz"]
    print(__doc__.split("\n\n", 1)[1].strip())
    for a in args:
        report(Path(a))
    print("\nForward travel that does not move across the cuts, next to a lateral offset")
    print("that collapses, means the forward metre is walked and the sideways metre is the")
    print("fall.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
