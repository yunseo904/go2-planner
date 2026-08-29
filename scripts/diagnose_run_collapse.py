#!/usr/bin/env python3
"""Why does RUN collapse in 1.13 s, when foot placement does not move it at all?

    python3 scripts/diagnose_run_collapse.py [tag ...]

Reads the traces already on disk -- no simulator, no GPU.  Answers, per run:

  * which axis terminated it, and what roll/pitch were doing before that
  * which foot unloads first, and how far ahead of the collapse
  * whether the sim gait has the FLIGHT PHASE the clip does: how high the swing
    feet actually get off the floor, and how many feet are loaded at once
  * whether the clip's swing schedule and the sim's own contacts agree, which is
    what the foot-placement gate depends on

The last two are the point.  RUN is duty 0.31 with a flight phase; if the sim feet
never leave the floor the clip is not being reproduced at all, and a correction
applied to a "swing" foot that is actually bearing load is not foot placement.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.replay import quat_to_rpy_deg                                    # noqa: E402

TRACES = ROOT / "outputs/traces/footcomp"


def report(tag: str) -> None:
    z = np.load(TRACES / f"{tag}.npz", allow_pickle=False)
    dt = float(z["dt"])
    n = z["q"].shape[0]
    r, p, y = quat_to_rpy_deg(z["root_quat_w"])
    f = z["contact_f"]                       # (n, 4) contact-sensor order
    fp = z["foot_pos_w"]                     # (n, 4, 3) articulation body order
    legs = [str(x) for x in z["leg_order"]]
    fnames = [str(x).split("_")[0] for x in z["foot_names"]]
    cnames = [str(x).split("_")[0] for x in z["contact_names"]]
    swing = z["swing"].astype(bool)          # clip schedule, clip leg order
    t = np.arange(n) * dt

    print("=" * 78)
    print(f"{tag}  clip={str(z['clip_name'])}  {n} steps = {n*dt:.2f} s  "
          f"(dt {dt*1000:.1f} ms)")
    print(f"  attitude   |roll| max {np.abs(r).max():5.1f} deg at t={t[np.argmax(np.abs(r))]:.2f} s"
          f"   |pitch| max {np.abs(p).max():5.1f} deg at t={t[np.argmax(np.abs(p))]:.2f} s")
    print(f"             roll at the end {r[-1]:+6.1f}, pitch {p[-1]:+6.1f}, "
          f"base height {z['root_pos_w'][-1, 2]:.3f} m")
    # Which axis was diverging: growth of the per-cycle extreme.
    half = n // 2
    print(f"             |roll| mean first half {np.abs(r[:half]).mean():5.2f} -> "
          f"second half {np.abs(r[half:]).mean():5.2f} deg;   "
          f"|pitch| {np.abs(p[:half]).mean():5.2f} -> {np.abs(p[half:]).mean():5.2f} deg")

    # --- unloading order: first sustained loss of load per leg -----------------
    thr = 30.0
    print(f"  first unload (force < {thr:.0f} N for 3 consecutive steps, after t=0.2 s):")
    lo = int(0.2 / dt)
    for k, nm in enumerate(cnames):
        below = f[:, k] < thr
        run3 = below[lo:-2] & below[lo + 1:-1] & below[lo + 2:]
        i = np.flatnonzero(run3)
        when = (i[0] + lo) * dt if i.size else np.nan
        print(f"      {nm:3s} t={when:5.2f} s   peak load {f[:, k].max():6.1f} N   "
              f"mean {f[:, k].mean():5.1f} N   loaded {100*(f[:, k] > thr).mean():4.1f}% of the run")

    # --- flight phase: is the sim gait actually leaving the ground? -----------
    ground = np.percentile(fp[..., 2], 5)          # the floor, from the feet themselves
    clear = fp[..., 2] - ground
    loaded = (f > thr).sum(axis=1)
    print(f"  flight     feet loaded per step: mean {loaded.mean():.2f}, "
          f"zero-foot steps {100*(loaded == 0).mean():4.1f}%  "
          f"(a flight phase needs zero-foot steps)")
    print(f"             swing clearance (foot z above the 5th-percentile floor), mm:")
    for k, nm in enumerate(fnames):
        j = legs.index(nm) if nm in legs else k
        sw = swing[:, j]
        c = clear[:, k] * 1000
        print(f"      {nm:3s} max {c.max():6.1f}   mean-in-clip-swing {c[sw].mean():6.1f}   "
              f"max-in-clip-swing {c[sw].max() if sw.any() else float('nan'):6.1f}")

    # --- do the clip's labels and the sim's contacts agree? -------------------
    order = [cnames.index(l) for l in legs if l in cnames]
    sim_swing = f[:, order] < thr
    agree = (sim_swing == swing).mean()
    print(f"  gate       clip swing schedule vs the sim's own contacts: agree "
          f"{100*agree:.1f}% of leg-steps")
    print(f"             clip says swing {100*swing.mean():.1f}% of leg-steps, "
          f"sim says {100*sim_swing.mean():.1f}%")


if __name__ == "__main__":
    tags = sys.argv[1:] or ["run_off", "run_cap005", "trot_off", "trot_cap005"]
    for tg in tags:
        if (TRACES / f"{tg}.npz").is_file():
            report(tg)
        else:
            print(f"(no trace for {tg})")
