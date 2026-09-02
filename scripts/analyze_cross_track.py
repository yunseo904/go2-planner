#!/usr/bin/env python3
"""Where the cross-track drift comes from: before the fall, and whether the feet slide.

    python3 scripts/analyze_cross_track.py outputs/xt/trace_walk_s1.npz

`outputs/cross_track.md` establishes that the drift is real (0.53 m sideways per metre
forward, 149/200 cells the same way) and that the lateral foot-placement law does not
touch it at any gain, either sign, or three times the authority.  This asks the next
question -- what DOES move the body sideways -- with three separated measurements.

1. BEFORE THE FALL.  `cross_track_m` is read at the end of the episode and includes
   post-mortem sliding; `cross_track_abs_max_m` stops at the last upright step.  They
   disagree (0.598 against 0.408) and the gap is the fall.  Everything here is trimmed to
   the last step the robot was alive, so "drift" means drift while walking.

2. DO THE STANCE FEET SLIDE?  A foot that is planted is STATIONARY IN THE WORLD -- that
   is what planted means, and it is the one thing about a gait that needs no model to
   check.  So: while the RECORDING calls a foot stance, how far does it travel in world
   xy?  Reported per stance bout and summed, split into the forward and lateral axes of
   the robot's own start line.

3. DOES THE SLIDE ACCOUNT FOR THE DRIFT?  If the summed lateral slip of the stance feet
   is the same size as the body's lateral displacement, the drift is the feet sliding and
   no foot-PLACEMENT law can fix it (which is what cross_track.md measured from the other
   side).  If it is much smaller, the body is moving over feet that are staying put, and
   the cause is somewhere else -- yaw-coupled sliding, or the swing legs' own momentum.

The stance mask is the CLIP's, not a contact sensor: this harness has none, and adding one
would change the scene the published numbers were measured in.  So a foot called stance
here may be airborne (WALK's rear feet scuff -- `lip_failure.md` §1), which makes this a
LOWER bound on how planted the feet are and an UPPER bound on measured slip.  Foot height
above its own stance-bout minimum is printed alongside so that can be judged.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.replay import quat_to_rpy_deg  # noqa: E402


def bouts(mask: np.ndarray):
    """(start, stop) index pairs of the True runs in a 1-D boolean array."""
    d = np.diff(np.concatenate([[False], mask, [False]]).astype(np.int8))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def main() -> int:
    path = Path(sys.argv[1] if len(sys.argv) > 1 else "outputs/xt/trace_walk_s1.npz")
    z = np.load(path, allow_pickle=False)
    pos, quat = z["root_pos_w"], z["root_quat_w"]
    feet, alive, stance = z["foot_pos_w"], z["alive"], z["stance_cmd"]
    dt = float(z["dt"])
    T, n, _ = pos.shape
    legs = [str(x) for x in z["leg_order"]]
    print(f"{path.name}: {T} steps x {n} cells, dt {dt:.3f} s, legs {legs}")

    # The robot's own start line: forward = its settle heading, lateral = perpendicular.
    # The same datum the harness's cross-track column uses, so the numbers are comparable.
    yaw0 = np.radians(np.array([quat_to_rpy_deg(quat[0, k][None, :])[2][0]
                                for k in range(n)]))
    c0, s0 = np.cos(yaw0), np.sin(yaw0)

    rows = []
    for k in range(n):
        a = alive[:, k]
        end = int(np.argmax(~a)) if (~a).any() else T          # first dead step
        end = max(end, 2)
        d = pos[:end, k, :2] - pos[0, k, :2]
        fwd = d[:, 0] * c0[k] + d[:, 1] * s0[k]
        lat = -d[:, 0] * s0[k] + d[:, 1] * c0[k]
        # foot slip, in the same frame, only while the clip calls the foot stance AND the
        # robot is still up
        slip_f = slip_l = slip_abs = 0.0
        nb = 0
        air = []
        for j in range(4):
            m = stance[:end, j] & a[:end]
            for i0, i1 in bouts(m):
                if i1 - i0 < 2:
                    continue
                p = feet[i0:i1, k, j, :]
                dd = p[-1, :2] - p[0, :2]
                slip_f += dd[0] * c0[k] + dd[1] * s0[k]
                slip_l += -dd[0] * s0[k] + dd[1] * c0[k]
                slip_abs += float(np.abs(np.diff(p[:, :2], axis=0)).sum())
                air.append(float(p[:, 2].max() - p[:, 2].min()))
                nb += 1
        rows.append(dict(k=k, end=end, t=end * dt, fwd=float(fwd[-1]), lat=float(lat[-1]),
                         lat_max=float(np.abs(lat).max()), nb=nb,
                         slip_f=slip_f, slip_l=slip_l, slip_abs=slip_abs,
                         air=float(np.median(air)) if air else np.nan,
                         alive_end=bool(a[-1])))

    def med(key, sub=None):
        v = [r[key] for r in (sub if sub is not None else rows) if np.isfinite(r[key])]
        return float(np.median(v)) if v else float("nan")

    print("\n=== 1. before the fall (trimmed to the last upright step) ===")
    print(f"  cells that were still up at 20 s : {sum(r['alive_end'] for r in rows)}/{n}")
    print(f"  upright time, median             : {med('t'):.2f} s")
    print(f"  forward travel, median           : {med('fwd'):+.3f} m")
    print(f"  lateral offset at that moment    : {med('lat'):+.3f} m   "
          f"(|.| median {np.median([abs(r['lat']) for r in rows]):.3f})")
    print(f"  peak |lateral| while upright     : {med('lat_max'):.3f} m")
    f_, l_ = med('fwd'), np.median([abs(r['lat']) for r in rows])
    print(f"  DRIFT PER METRE, upright only    : {l_/f_ if f_ else float('nan'):.3f} m/m")
    same = sum(1 for r in rows if r['lat'] > 0)
    print(f"  cells drifting +lateral          : {same}/{n}")

    print("\n=== 2. do the feet the clip calls STANCE stay put? ===")
    print(f"  stance bouts per cell, median    : {med('nb'):.0f}")
    print(f"  foot vertical range within a bout: {med('air')*1000:.1f} mm "
          f"(a planted foot should be ~0)")
    print(f"  net world slip per cell, forward : {med('slip_f'):+.3f} m")
    print(f"  net world slip per cell, lateral : {med('slip_l'):+.3f} m")
    print(f"  path length of stance feet       : {med('slip_abs'):.3f} m")

    print("\n=== 3. does the sliding account for the drift? ===")
    # per cell, so the ratio is not two medians divided
    r_l = [abs(r['slip_l']) / abs(r['lat']) for r in rows
           if abs(r['lat']) > 0.05 and np.isfinite(r['slip_l'])]
    r_f = [abs(r['slip_f']) / abs(r['fwd']) for r in rows
           if abs(r['fwd']) > 0.05 and np.isfinite(r['slip_f'])]
    print(f"  |lateral stance slip| / |lateral body move|, median : {np.median(r_l):.2f}")
    print(f"  |forward stance slip| / |forward body move|, median : {np.median(r_f):.2f}")
    print("\n  A ratio near 1 means the body went where the feet slid -- the drift IS the")
    print("  feet sliding, and no foot-PLACEMENT law can fix it.  A ratio near 0 means the")
    print("  feet stayed put and the body moved over them, which points elsewhere.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
