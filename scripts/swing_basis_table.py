#!/usr/bin/env python3
"""Swing height on four bases, from a floored replay trace.  Offline, no simulator.

    python3 scripts/swing_basis_table.py outputs/traces/swingbasis_TROT.npz

`check_swing_basis.py` measures the same thing on the RECORDING with the robot held in
the air.  That measurement is well defined only through the clip's contact channel: with
no floor the foot's z is one continuous descent-and-rise, so the threshold-free
"where does the foot start rising" reference collapses onto the cycle minimum, which on a
real gait is MID-STANCE and not lift-off.  This is the second reference frame CLAUDE.md
6.5 requires, taken where lift-off is a physical event: a replay with a floor and the
simulator's own contact sensor.

    takeoff/sim    apex z minus the foot's z at the last frame the SIM contact had it
                   loaded -- quadruped_pympc's basis, with a real lift-off
    takeoff/clip   the same, using the CLIP's contact channel instead
    stance-plane   apex z minus the mean z of the feet in stance at that instant
    ground         apex z minus the floor (z = 0)

Bouts are per swing, so a 60-cycle run gives a distribution rather than one number --
fixed-interval sampling of foot height has already produced two wrong results here
(CLAUDE.md 6.5).
"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def bouts(down: np.ndarray):
    """[(takeoff_frame, [swing frames])] for one leg's boolean stance channel."""
    sw = ~np.asarray(down, dtype=bool)
    out, cur, k0 = [], [], None
    for i, v in enumerate(sw):
        if v:
            if not cur:
                k0 = i - 1
            cur.append(i)
        elif cur:
            if k0 is not None and k0 >= 0:
                out.append((k0, cur))
            cur = []
    return out


def table(path: Path, thresh_n: float = 30.0) -> None:
    z = np.load(path, allow_pickle=True)
    fp = z["foot_pos_w"]                      # (T, 4, 3), articulation body order
    foot_names = [str(x) for x in z["foot_names"]]
    con_names = [str(x) for x in z["contact_names"]]
    legs = [str(x) for x in z["leg_order"]]
    cf = z["contact_f"]                       # (T, 4) sensor order, force magnitude
    # `contact` in the trace is the SIM sensor's boolean in SENSOR order, not the clip's
    # channel -- indexing it with a clip leg index silently compares a thing to itself.
    # The clip's own gate is `swing`, which is what the foot-placement law was handed,
    # and it is in CLIP leg order.
    clipc = ~z["swing"].astype(bool)          # (T, 4) stance per the CLIP, clip legs
    T = fp.shape[0]
    # The three orderings are different lists (harness_findings.md 5). By name or not at all.
    f_col = {n.split("_")[0]: i for i, n in enumerate(foot_names)}
    c_col = {n.split("_")[0]: i for i, n in enumerate(con_names)}
    term = float(z["terminated_s"]) if np.isfinite(z["terminated_s"]) else float("nan")
    print(f"\n{z['clip_name']}  {T} control steps, dt {float(z['dt']):.4f} s, "
          f"{'terminated %.2f s' % term if np.isfinite(term) else 'ran to the end'}")
    print(f"  {'leg':4s} {'bouts':>5s} {'takeoff/sim':>26s} {'takeoff/clip':>13s} "
          f"{'stance-plane':>13s} {'ground':>9s}   (mm)")
    for j, leg in enumerate(legs):
        fc, cc = f_col[leg], c_col[leg]
        sim_down = cf[:, cc] > thresh_n
        rows = {}
        for tag, down in (("sim", sim_down), ("clip", clipc[:, j])):
            h = []
            for k0, blk in bouts(down):
                apex = blk[int(np.argmax(fp[blk, fc, 2]))]
                h.append(fp[apex, fc, 2] - fp[k0, fc, 2])
            rows[tag] = np.array(h)
        # stance plane and ground, on the sim bouts
        hp, hg = [], []
        for k0, blk in bouts(sim_down):
            apex = blk[int(np.argmax(fp[blk, fc, 2]))]
            st = [f_col[l] for l in legs if cf[apex, c_col[l]] > thresh_n]
            if st:
                hp.append(fp[apex, fc, 2] - float(np.mean(fp[apex, st, 2])))
            hg.append(fp[apex, fc, 2])
        s, c = rows["sim"], rows["clip"]
        def f(a):
            return (f"{np.median(a)*1000:6.1f} [{np.percentile(a,10)*1000:5.1f},"
                    f"{np.percentile(a,90)*1000:5.1f}]") if len(a) else "     --      "
        print(f"  {leg:4s} {len(s):5d} {f(s):>26s} {np.median(c)*1000 if len(c) else float('nan'):13.1f} "
              f"{np.median(hp)*1000 if hp else float('nan'):13.1f} "
              f"{np.median(hg)*1000 if hg else float('nan'):9.1f}")
    # hip height, for the 0.2 x hip_height comparison
    hz = float(np.median(z["root_pos_w"][:, 2]))
    print(f"  base height (median) {hz*1000:.0f} mm; quadruped_pympc's rule "
          f"0.2 x hip_height would be {0.2*hz*1000:.0f} mm for this body")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        table(Path(p))
