#!/usr/bin/env python3
"""The real robot does not roll and the sim does. Where do they part company?

    python3 scripts/real_vs_sim_roll.py

`why_it_rolls.md` §3: over the same 4-5 s the real robot's worst |roll| in any
`03_slow_walk` session is 3.39 deg, while the sim's flat-ground MEAN is 2.3-2.6 and it
passes 20 deg and does not come back.  The same joint angles are being played.  So either
the replay is not reproducing the recording (fixable) or open-loop replay of a recording
cannot hold this robot up (a result).  This separates the two.

Three comparisons, all on the clip's OWN source session:

  1. per-cycle peak |roll|, real and sim side by side -- is the sim wrong on cycle 1, or
     does it accumulate?
  2. per-leg foot load.  The real logs carry `foot_force_*`; the sim trace carries the
     contact sensor.  CLAUDE.md 6 forbids the log's fixed 20 N `contact_*` threshold, so
     stance is taken per leg per session at a fraction of that leg's own p95.
  3. joint tracking: |q - q_des| in the log against |q - q_cmd| in the sim, per joint.
     The log's own PD misses too; the question is by how much more the sim misses.
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.replay import quat_to_rpy_deg  # noqa: E402

CURATED = Path("/home/waffle/projects/curated")
LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("hip", "thigh", "calf")
#: stance if the leg carries more than this fraction of its own p95 force.  Per leg, per
#: session -- CLAUDE.md 6 measured the real per-leg thresholds at 24-51 N and forbids the
#: log's fixed 20 N column for exactly this reason.
STANCE_FRAC = 0.25


def find_session(name: str) -> Path:
    hits = list(CURATED.glob(f"*/{name}/data.csv"))
    if not hits:
        raise SystemExit(f"session {name} not found under {CURATED}")
    return hits[0]


def load_real(name: str) -> dict:
    f = find_session(name)
    rows = list(csv.DictReader(open(f)))
    g = lambda c: np.array([float(r[c]) if r[c] not in ("", "nan") else np.nan
                            for r in rows])
    d = {"t": g("t"), "roll": np.degrees(g("roll")), "vx": g("base_vx")}
    d["force"] = np.stack([g(f"foot_force_{l}") for l in LEGS], axis=1)
    d["q"] = np.stack([g(f"{l}_{j}_q") for l in LEGS for j in JOINTS], axis=1)
    d["q_des"] = np.stack([g(f"{l}_{j}_q_des") for l in LEGS for j in JOINTS], axis=1)
    return d


def stance_mask(force: np.ndarray) -> np.ndarray:
    thr = STANCE_FRAC * np.nanpercentile(force, 95, axis=0)
    return force > thr[None, :]


def cycles_from(mask: np.ndarray) -> list:
    """Touchdown-to-touchdown index pairs of leg 0, the cycle boundary."""
    m = mask[:, 0].astype(np.int8)
    td = np.flatnonzero(np.diff(m) == 1)
    return list(zip(td[:-1], td[1:]))


def per_cycle_peak(roll: np.ndarray, cyc: list) -> np.ndarray:
    return np.array([np.nanmax(np.abs(roll[a:b])) for a, b in cyc if b - a > 3])


def main() -> int:
    print(__doc__.split("\n\n", 1)[1].strip())
    real = load_real("gait_classic_walk_20260824_224559")
    mv = np.abs(real["vx"]) > 0.05
    i = np.flatnonzero(mv)
    lo, hi = int(i[0]), int(i[-1])
    rs = {k: v[lo:hi] for k, v in real.items()}
    r_mask = stance_mask(rs["force"])
    r_cyc = cycles_from(r_mask)
    r_peak = per_cycle_peak(rs["roll"], r_cyc)

    z = np.load(ROOT / "outputs/trace_ext_walk.npz", allow_pickle=False)
    s_roll = quat_to_rpy_deg(z["root_quat_w"])[0]
    s_mask = stance_mask(z["contact_f"])
    s_cyc = cycles_from(s_mask)
    s_peak = per_cycle_peak(s_roll, s_cyc)

    print("\n=== 1. peak |roll| per gait cycle: is the sim wrong at once, or does it grow? ===")
    print(f"{'cycle':>6} {'real deg':>10} {'sim deg':>10}")
    for k in range(max(len(r_peak), len(s_peak))):
        a = f"{r_peak[k]:10.2f}" if k < len(r_peak) else " " * 10
        b = f"{s_peak[k]:10.2f}" if k < len(s_peak) else " " * 10
        print(f"{k + 1:6d} {a} {b}")
        if k >= 11:
            print(f"{'...':>6} {len(r_peak)} real cycles, {len(s_peak)} sim cycles total")
            break
    print(f"\n  real : first cycle {r_peak[0]:.2f}, median {np.median(r_peak):.2f}, "
          f"max {r_peak.max():.2f} deg over {len(r_peak)} cycles")
    print(f"  sim  : first cycle {s_peak[0]:.2f}, median {np.median(s_peak):.2f}, "
          f"max {s_peak.max():.2f} deg over {len(s_peak)} cycles")
    if len(s_peak) > 3:
        k = np.polyfit(np.arange(len(s_peak)), s_peak, 1)[0]
        kr = np.polyfit(np.arange(len(r_peak)), r_peak, 1)[0]
        print(f"  trend per cycle: real {kr:+.3f} deg, sim {k:+.3f} deg")

    print("\n=== 2. per-leg load ===")
    print(f"{'leg':>4} {'real duty':>10} {'real p95 N':>11} {'sim duty':>9} {'sim p95 N':>10}")
    for j, l in enumerate(LEGS):
        print(f"{l:>4} {r_mask[:, j].mean():10.3f} {np.nanpercentile(rs['force'][:, j], 95):11.1f} "
              f"{s_mask[:, j].mean():9.3f} {np.nanpercentile(z['contact_f'][:, j], 95):10.1f}")
    print(f"  real  front/rear duty ratio {r_mask[:, :2].mean() / r_mask[:, 2:].mean():.3f}")
    print(f"  sim   front/rear duty ratio {s_mask[:, :2].mean() / s_mask[:, 2:].mean():.3f}")

    print("\n=== 3. joint tracking, |achieved - commanded| in radians ===")
    print(f"{'joint':>6} {'real mean':>10} {'real p99':>9} {'sim mean':>9} {'sim p99':>9} {'ratio':>7}")
    rq = np.abs(rs["q"] - rs["q_des"])
    sq = np.abs(z["q"] - z["q_cmd"])
    for k, j in enumerate(JOINTS):
        a, b = rq[:, k::3], sq[:, k::3]
        print(f"{j:>6} {np.nanmean(a):10.4f} {np.nanpercentile(a, 99):9.4f} "
              f"{np.nanmean(b):9.4f} {np.nanpercentile(b, 99):9.4f} "
              f"{np.nanmean(b) / np.nanmean(a):7.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
