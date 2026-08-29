#!/usr/bin/env python3
"""The swing-lift curve: what the edit buys, in two references, with vx beside it.

    python3 scripts/swing_lift_table.py

Clearance is measured from the replay's OWN foot positions (``foot_pos_w``), which is
the achieved geometry -- no forward kinematics and no commanded angles involved -- in the
two references that answer different questions:

* **stance plane** -- height above the plane the loaded feet define at that instant.  This
  is what a step's lip is measured against, and it absorbs whatever the trunk is doing.
* **own chord** -- height above the straight line from that foot's own liftoff to its own
  touchdown.  This is the arc the edit actually shapes, and it is independent of the
  trunk entirely.

Stance is taken from MEASURED contact force, resolved to legs by name.
"""
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def bouts(mask):
    d = np.diff(np.concatenate([[False], mask, [False]]).astype(np.int8))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def clearances(path, thresh=30.0):
    z = np.load(path, allow_pickle=False)
    fp, cf = z["foot_pos_w"], z["contact_f"]
    fn = [str(x).split("_")[0] for x in z["foot_names"]]
    cn = [str(x).split("_")[0] for x in z["contact_names"]]
    ccol = [cn.index(l) for l in fn]                 # by name, never by position
    stance = cf[:, ccol] > thresh
    n = len(fp)
    plane, chord = {l: [] for l in fn}, {l: [] for l in fn}
    for i in range(n):
        st = stance[i]
        if st.sum() < 3:
            continue
        P = fp[i][st]
        A = np.column_stack([np.ones(st.sum()), P[:, 0], P[:, 1]])
        coef, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
        for j, l in enumerate(fn):
            if not st[j]:
                f = fp[i][j]
                plane[l].append(f[2] - (coef[0] + coef[1] * f[0] + coef[2] * f[1]))
    for j, l in enumerate(fn):
        for a, b in bouts(~stance[:, j]):
            if b - a < 3:
                continue
            p0, p1 = fp[a, j], fp[b - 1, j]
            t = np.linspace(0, 1, b - a)
            h = fp[a:b, j, 2] - (p0[2] + (p1[2] - p0[2]) * t)
            chord[l].append(h.max())
    return ({l: float(np.median(v) * 1000) if v else float("nan") for l, v in plane.items()},
            {l: float(np.median(v) * 1000) if v else float("nan") for l, v in chord.items()})


def main() -> int:
    rows = {r["tag"]: r for r in csv.DictReader(open(ROOT / "outputs/swing_lift.csv"))
            if r.get("tag", "").startswith("sl_")}
    exp = {"WALK": (1.367, 0.1875), "TROT": (1.5554, 0.4437), "TURN": (1.1207, 0.0075)}
    print(f"{'clip':5s} {'lift':>5s} {'survived':>9s} {'stride':>14s} {'vx':>14s} "
          f"{'apex vs plane (mm)':>28s} {'vs own chord':>28s}")
    for clip in ("WALK", "TROT", "TURN"):
        for mm in (0, 20, 40, 60, 80):
            tag = f"sl_{clip}_{mm}"
            if tag not in rows:
                continue
            r = rows[tag]
            st, vx = float(r["stride_hz"]), float(r["vx_mean"])
            es, ev = exp[clip]
            surv = "60+ cyc" if r["terminated_s"] == "" else f"{float(r['terminated_s']):6.2f}s"
            tr = ROOT / f"outputs/traces/swinglift/{tag}.npz"
            if tr.is_file():
                pl, ch = clearances(tr)
                ps = " ".join(f"{k}{pl[k]:5.1f}" for k in pl)
                cs = " ".join(f"{k}{ch[k]:5.1f}" for k in ch)
            else:
                ps = cs = "(no trace)"
            print(f"{clip:5s} {mm:5.0f} {surv:>9s} {st:6.2f}/{es:5.2f}({(st-es)/es*100:+3.0f}%) "
                  f"{vx:6.3f}/{ev:5.3f} {ps:>28s} {cs:>28s}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
