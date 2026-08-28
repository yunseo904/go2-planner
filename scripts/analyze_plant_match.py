#!/usr/bin/env python3
"""Does the sim robot hold the pose the recording holds, or does it sink into it?

    python scripts/analyze_plant_match.py                      # every trace + the logs
    python scripts/analyze_plant_match.py --traces outputs/traces/TROT_long.npz

The hypothesis this measures: the collapse starts as a plant mismatch rather than
as a gait instability.  The joint targets were produced for the real robot's mass
and its controller; if Isaac's Go2 sags under the same targets, the legs fold, the
swing foot scuffs, and the roll that follows is a consequence rather than a cause.

Four quantities, three of which exist on both sides:

1. base height -- sim ``root_pos_w[:,2]``, log ``body_height``.  NOTE the two are
   not guaranteed to share a datum: the sim's is the base link origin from the USD,
   the log's is the sport controller's own estimate.  Compared as a CHANGE from
   each side's own standing value, which is datum-free; the absolute pair is
   printed too, and they start within a centimetre of each other.
2. tracking error q - q_des, per joint type.  Same PD target on both sides, so
   this is the plant comparison with nothing in between.  Aggregated by joint TYPE
   (hip/thigh/calf), which is invariant to the leg-order permutation and so cannot
   be wrong for the reason harness_findings.md 5 was wrong.
3. leg fold, ``cos(thigh) + cos(thigh+calf)``.  This is the knee-plane extension of
   the leg divided by the link length, so it is dimensionless, needs no link
   lengths, and -- because it uses only the two sagittal joints -- is independent
   of the hip sign convention that is still unverified.  1.0 is a straight leg
   hanging down, smaller is more folded.
4. swing foot clearance -- SIM ONLY.  ``foot_pos_*`` in the logs is all zeros
   (firmware, CLAUDE.md 6), and reconstructing it by forward kinematics needs the
   joint convention that this project has not settled, so there is no honest
   real-robot number to compare against.  Reported as height above the same foot's
   own stance level, which cancels the collision-sphere radius.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motion_toolkit.contact import detect_contact
from motion_toolkit.session import JOINTS, iter_sessions
from motion_toolkit.window import detect_motion
from sim.replay import quat_to_rpy_deg
from terrain_toolkit.paths import DATA_DIR, OUTPUTS_DIR, require_curated

CANON = ("FL", "FR", "RL", "RR")
#: Which session each replayed clip came from, read from the archives rather than named.
ARCHIVES = (DATA_DIR / "skill_clips.npz", DATA_DIR / "full_sessions.npz")


def order(names):
    out = []
    for leg in CANON:
        hit = [i for i, nm in enumerate(names) if nm.upper().startswith(leg)]
        if len(hit) != 1:
            raise SystemExit(f"cannot map {names} onto {list(CANON)}")
        out.append(hit[0])
    return out


def leg_fold(q: np.ndarray) -> np.ndarray:
    """cos(thigh) + cos(thigh+calf) per leg, from a (n,12) leg-major matrix."""
    a = q.reshape(len(q), 4, 3)
    thigh, calf = a[:, :, 1], a[:, :, 2]
    return np.cos(thigh) + np.cos(thigh + calf)


#: Everything after the robot has clearly begun to topple is not a gait measurement:
#: a foot on the high side of a rolling base reads as a 40 cm "swing clearance".
#: 10 deg is a stated cut, not a fitted one -- it is well inside every run's
#: geometric tipping angle (26.9-30.9 deg) and well outside WALK's converged
#: +-4 deg oscillation, so it separates "walking" from "going over" without
#: touching either.
ROLL_CUT_DEG = 10.0


def per_swing_peak(feet, con, lo, hi):
    """Median over swing phases of that swing's peak height above stance level.

    Per SWING, not per sample: the maximum over all swing samples is whatever the
    worst moment of the run produced, which for a run that falls is the topple.
    """
    out = []
    for i in range(4):
        st = con[lo:hi, i]
        base = float(np.median(feet[lo:hi, i, 2][st])) if st.sum() >= 3 else np.nan
        if not np.isfinite(base):
            continue
        sw = ~st
        edges = np.flatnonzero(np.diff(sw.astype(int)))
        starts = edges[::2] + 1 if not sw[0] else np.r_[0, edges[1::2] + 1]
        stops = np.r_[edges[1::2] + 1, len(sw)] if not sw[0] else np.r_[edges[::2] + 1, len(sw)]
        for a, b in zip(starts, stops):
            if b - a >= 2:
                out.append(float(feet[lo + a: lo + b, i, 2].max() - base))
    return (float(np.median(out)), float(np.min(out)), len(out)) if out else (np.nan, np.nan, 0)


def sim_row(path: Path) -> dict | None:
    t = np.load(path, allow_pickle=False)
    if "q_cmd" not in t.files:
        return None
    dt = float(t["dt"])
    q, qc = t["q"], t["q_cmd"]
    n = min(len(q), len(qc))
    q, qc = q[:n], qc[:n]
    z = t["root_pos_w"][:n, 2]
    con = t["contact"][:n].astype(bool)[:, order([str(x) for x in t["contact_names"]])]
    feet = t["foot_pos_w"][:n][:, order([str(x) for x in t["foot_names"]])]

    roll, _, _ = quat_to_rpy_deg(t["root_quat_w"][:n])   # scalar-LAST; sim/replay.py owns this
    over = np.flatnonzero(np.abs(roll) > ROLL_CUT_DEG)
    # A run can be over the cut at its very first recorded step (RUN's handover is
    # already rolling).  Keep a floor of 5 steps so the row still reports something,
    # and let the printed clean-window length say that is what happened.
    hi = max(int(over[0]), 5) if over.size else n  # clean window = up to the topple
    clean = slice(0, hi)

    err = (q - qc)[clean]
    E = err.reshape(len(err), 4, 3)
    fold = leg_fold(q[clean])
    sw_med, sw_min, n_sw = per_swing_peak(feet, con, 0, hi)

    # settled height: the first 5 control steps, before the gait has done anything
    z0 = float(np.median(z[: max(5, int(0.1 / dt))]))
    # swing clearance, per foot, against that foot's own stance level
    clear, scuff = [], []
    for i in range(4):
        st, sw = con[:, i], ~con[:, i]
        if st.sum() < 3 or sw.sum() < 3:
            continue
        base = float(np.median(feet[st, i, 2]))
        clear.append(float(feet[sw, i, 2].max() - base))
        scuff.append(float(feet[sw, i, 2].min() - base))
    return {
        "name": str(t["clip_name"]), "file": path.name, "n": n, "dur": n * dt,
        "term": float(t["terminated_s"]),
        "z0": z0, "z_min": float(z.min()), "z_end": float(z[-1]),
        "z_drop_max": z0 - float(z.min()),
        "t_below_1cm": next((i * dt for i in range(n) if z[i] < z0 - 0.01), None),
        "t_below_3cm": next((i * dt for i in range(n) if z[i] < z0 - 0.03), None),
        "err_rms": [float(np.sqrt((E[:, :, j] ** 2).mean())) for j in range(3)],
        "err_mean": [float(E[:, :, j].mean()) for j in range(3)],
        "err_max": [float(np.abs(E[:, :, j]).max()) for j in range(3)],
        "fold_mean": float(fold.mean()), "fold_min": float(fold.min()),
        "clear_max": sw_med, "clear_min": sw_min, "scuff": min(scuff) if scuff else float("nan"),
        "clean_s": hi * dt, "n_sw": n_sw,
        "z_clean_min": float(z[clean].min()) if hi else float("nan"),
        "z_clean_drop": z0 - (float(z[clean].min()) if hi else np.nan),
    }


def log_row(sess, name: str) -> dict:
    win = detect_motion(sess)
    m = win.mask if win.ok else np.ones(sess.n, dtype=bool)
    q = sess.joint_matrix("q")[m]
    qd = sess.joint_matrix("q_des")[m]
    ok = np.isfinite(q) & np.isfinite(qd) & (np.abs(qd) < 1e9)
    err = np.where(ok, q - qd, np.nan)
    E = err.reshape(len(err), 4, 3)
    bh = sess.col("body_height")
    pre = bh[: int(sess.fs * 1.0)]                    # standing, before the motion window
    z0 = float(np.nanmedian(pre))
    bhm = bh[m]
    fold = leg_fold(np.where(np.isfinite(q), q, 0.0))
    return {
        "name": name, "file": sess.path.name, "n": int(m.sum()), "dur": float(m.sum() / sess.fs),
        "term": float("nan"),
        "z0": z0, "z_min": float(np.nanmin(bhm)), "z_end": float(np.nanmedian(bhm[-20:])),
        "z_drop_max": z0 - float(np.nanmin(bhm)),
        "t_below_1cm": None, "t_below_3cm": None,
        "err_rms": [float(np.sqrt(np.nanmean(E[:, :, j] ** 2))) for j in range(3)],
        "err_mean": [float(np.nanmean(E[:, :, j])) for j in range(3)],
        "err_max": [float(np.nanmax(np.abs(E[:, :, j]))) for j in range(3)],
        "fold_mean": float(np.nanmean(fold)), "fold_min": float(np.nanmin(fold)),
        "clear_max": float("nan"), "clear_min": float("nan"), "scuff": float("nan"),
        "clean_s": float(m.sum() / sess.fs), "n_sw": 0,
        "z_clean_min": float(np.nanmin(bhm)), "z_clean_drop": z0 - float(np.nanmin(bhm)),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--traces", nargs="*", type=Path, default=None)
    ap.add_argument("--no-logs", action="store_true", help="skip the real-robot side")
    args = ap.parse_args()

    traces = args.traces or sorted((OUTPUTS_DIR / "traces").glob("*.npz"))
    sim = [r for r in (sim_row(p) for p in traces) if r]

    logs = []
    if not args.no_logs:
        want = {}
        for arc in ARCHIVES:
            if not arc.is_file():
                continue
            meta = json.loads(arc.with_suffix("").with_suffix(".meta.json").read_text())
            for cname, c in meta["clips"].items():
                want[c["session"]] = cname
        by = {s.path.name: s for s in iter_sessions(require_curated())}
        for sname, cname in sorted(want.items(), key=lambda kv: kv[1]):
            if sname in by:
                logs.append(log_row(by[sname], cname))

    def table(rows, title, show_clear):
        print("=" * 108)
        print(title)
        print(f"{'run':26s} {'dur':>6s} {'fell':>6s} {'clean':>6s} | {'z0':>6s} "
              f"{'zmin*':>6s} {'drop*':>6s} {'-1cm':>6s} {'-3cm':>6s} | "
              f"{'q-qdes RMS mrad h/t/c':>22s} | {'fold':>5s}"
              + ("  | swing peak med/min mm (n)" if show_clear else ""))
        for r in rows:
            t1 = f"{r['t_below_1cm']:.2f}" if r["t_below_1cm"] is not None else "-"
            t3 = f"{r['t_below_3cm']:.2f}" if r["t_below_3cm"] is not None else "-"
            fell = f"{r['term']:.2f}" if np.isfinite(r["term"]) else "no"
            e = "/".join(f"{1000*v:.0f}" for v in r["err_rms"])
            line = (f"{r['name']+' '+r['file'][:-4]:26.26s} {r['dur']:6.2f} {fell:>6s} "
                    f"{r['clean_s']:6.2f} | "
                    f"{r['z0']:6.3f} {r['z_clean_min']:6.3f} {r['z_clean_drop']:6.3f} "
                    f"{t1:>6s} {t3:>6s} | {e:>22s} | {r['fold_mean']:5.3f}")
            if show_clear:
                line += (f"  | {1000*r['clear_max']:6.1f} / {1000*r['clear_min']:+6.1f} ({r['n_sw']})"
                         if np.isfinite(r["clear_max"]) else "  |      -")
            print(line)

    table(sim, "SIM  -- * = measured in the clean window only (up to |roll| > 10 deg)", True)
    if logs:
        table(logs, "REAL ROBOT  (body_height; z0 from the standing second before the motion window)", False)

    print("=" * 108)
    print("fold = cos(thigh)+cos(thigh+calf), leg extension / link length. Lower = more folded.")
    print("swing peak = per swing phase, that swing's highest point above the same foot's stance")
    print("level; reported as median / minimum over the swings in the clean window (n swings).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
