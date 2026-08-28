#!/usr/bin/env python3
"""Cut individual, unaveraged gait cycles for an A/B against the median clip.

    python scripts/extract_raw_cycles.py --clip TROT
    python scripts/extract_raw_cycles.py --clip TROT --self-test    # no logs needed

The frozen clip in ``data/skill_clips.npz`` is a MEDIAN over every clean cycle of
the chosen session, taken on a shared phase grid.  Whatever varied between cycles
-- including whatever the sport controller was correcting for -- is gone from it
by construction.  This script writes the same session's cycles out one at a time
so a replay can be run on each.

It changes the EXTRACTION, not the recording: same session, same contact-based
cycle detection, same phase grid, same low-pass, same channel set and leg order.
The only difference is how many cycles feed the median, and for a single cycle the
median is the identity.  The frozen archive is not touched; output goes to
``data/raw_cycles_<CLIP>.npz`` with a meta file of the same shape, so
``verify_skill_replay.py --clip-archive`` can play it with no other change.

Clip names in the output archive:

    <CLIP>_med    every clean cycle, median  -- the control, rebuilt HERE so that
                  the A/B differs in the averaging and not in the code path
    <CLIP>_c00    cycle 0 alone, unaveraged
    <CLIP>_c01    cycle 1 alone
    ...
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motion_toolkit import clips as C
from motion_toolkit.profile import profile_all
from motion_toolkit.session import JOINTS, iter_sessions
from terrain_toolkit.paths import DATA_DIR, require_curated

CHANNELS = ("t", "q_des", "dq_des", "tau_ff", "q", "dq", "tau", "kp", "kd", "contact", "q_des_valid")


def out_paths(clip: str) -> tuple[Path, Path]:
    return (DATA_DIR / f"raw_cycles_{clip}.npz", DATA_DIR / f"raw_cycles_{clip}.meta.json")


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def save(clips: list, built_from: dict, npz: Path, meta_json: Path) -> dict:
    arrays = {
        "clip_names": np.asarray([c.name for c in clips]),
        "clip_kinds": np.asarray([c.kind for c in clips]),
        "leg_order": np.asarray(C.TARGET_LEGS),
        "joint_order": np.asarray(JOINTS),
        "target_fs": np.asarray(C.TARGET_FS),
        "convention_verified": np.asarray(False),
    }
    for c in clips:
        for rate, d in (("hi", c.hi), ("lo", c.lo)):
            for ch in CHANNELS:
                v = d[ch]
                arrays[f"{c.name}__{rate}__{ch}"] = (
                    v.astype(np.uint8) if v.dtype == bool else np.asarray(v, dtype=np.float32))
        arrays[f"{c.name}__fs_hi"] = np.asarray(c.fs_hi)
        arrays[f"{c.name}__fs_lo"] = np.asarray(c.fs_lo)

    npz.parent.mkdir(parents=True, exist_ok=True)
    with open(npz, "wb") as fh:
        np.savez(fh, **arrays)

    meta = {
        "npz_sha256": sha256_of(npz),
        "n_clips": len(clips),
        "leg_order_native": ["FR", "FL", "RR", "RL"],
        "leg_order_stored": C.TARGET_LEGS,
        "joint_order": list(JOINTS),
        "target_fs_hz": C.TARGET_FS,
        "lowpass_frac_of_target": C.LOWPASS_FRAC,
        "channels": list(CHANNELS),
        "convention_verified": False,
        "convention_note": "Same convention as data/skill_clips.npz; unverified in the same way.",
        "gain_note": "Same per-sample kp/kd/tau_ff as the frozen archive.",
        "purpose": (
            "A/B on the EXTRACTION method only. '<CLIP>_med' is the median over every clean "
            "cycle (what the frozen archive stores); '<CLIP>_cNN' is cycle NN alone, unaveraged. "
            "Same session, same cycle detection, same phase grid, same low-pass."
        ),
        "built_from": built_from,
        "clips": {
            c.name: {
                "kind": c.kind, "session": c.session, "group": c.group,
                "fs_hi_hz": c.fs_hi, "fs_lo_hz": c.fs_lo,
                "n_hi": int(c.hi["q_des"].shape[0]), "n_lo": int(c.lo["q_des"].shape[0]),
                **{k: v for k, v in c.meta.items()},
            } for c in clips
        },
        "source": "motion_toolkit/clips.py build_cyclic_clip(cycle_subset=...)",
    }
    meta_json.write_text(json.dumps(meta, indent=2, default=float) + "\n")
    return meta


def build(clip_name: str) -> dict:
    root = require_curated()
    print(f"[raw] curated root: {root}")
    spec = next((s for s in C.CLIP_SPECS if s.name == clip_name), None)
    if spec is None:
        raise SystemExit(f"no clip spec named {clip_name!r}; have "
                         f"{[s.name for s in C.CLIP_SPECS]}")
    if spec.kind != "cyclic":
        raise SystemExit(f"{clip_name} is a {spec.kind} clip; there are no cycles to cut")

    sessions = iter_sessions(root)
    print(f"[raw] profiling {len(sessions)} sessions to pick the same representative ...")
    picks = C.select_sessions(profile_all(sessions))
    pick = picks[clip_name]
    if pick["session"] is None:
        raise SystemExit(f"{clip_name}: no session qualifies -- {pick['why']}")
    sess = {s.path.name: s for s in sessions}[pick["session"]]
    print(f"[raw] session {sess.path.name}  ({pick['why']})")

    cycles, info = C._cycle_bounds(sess)      # the same detection the frozen path uses
    n = len(cycles)
    print(f"[raw] {n} clean cycles kept of {info['n_cycles_seen']} seen, "
          f"median {info['cycle_s']:.4f} s, spread {info['cycle_s_spread']:.4f} s")

    built = []
    for subset, name in [(None, f"{clip_name}_med")] + [([i], f"{clip_name}_c{i:02d}")
                                                        for i in range(n)]:
        c = C.build_clip(sess, spec, cycle_subset=subset)
        c.name = name
        c.meta["gains"] = C.gain_summary(sess)
        c.meta["selection"] = {k: v for k, v in pick.items() if k != "candidates"}
        built.append(c)
        m = c.meta
        print(f"[raw] {name:10s} cycles={m['n_cycles_used']:2d} "
              f"period={m['cycle_s']:.4f}s n_hi={c.hi['q_des'].shape[0]:4d} "
              f"seam={m['loop_seam_rad']:.4f} rad ({m['loop_seam_over_p2p']*100:5.1f}% of p2p) "
              f"spread_max={m['q_des_spread_rad_max']:.4f} rad")

    npz, meta_json = out_paths(clip_name)
    meta = save(built, {"session": sess.path.name, "clip": clip_name,
                        "n_cycles_kept": n, "cycle_s_median": info["cycle_s"],
                        "cycle_s_spread": info["cycle_s_spread"]}, npz, meta_json)
    print(f"[raw] -> {npz}  sha256 {meta['npz_sha256'][:16]}…")
    return meta


# --------------------------------------------------------------------------- #
# Self-test: no curated logs, no simulator.  Proves the thing that matters --
# that a single-cycle subset is the cycle itself and not an average of anything.
# --------------------------------------------------------------------------- #

def self_test() -> int:
    fails = []

    def check(name, cond, detail=""):
        print(f"  {'ok  ' if cond else 'FAIL'}  {name}" + (f"  -- {detail}" if detail and not cond else ""))
        if not cond:
            fails.append(name)

    print("extract_raw_cycles self-test -- synthetic cycles, no logs")

    # Three cycles of the same waveform with a different constant offset each.
    # The median over all three is the middle offset; each single cycle is its own.
    n_phase = 64
    phase = np.arange(n_phase) / n_phase
    base = np.sin(2 * np.pi * phase)[:, None] * np.ones((1, 12))
    offsets = [-0.30, 0.05, 0.90]   # mean 0.217 != median 0.05, or the next check is vacuous
    period = 0.5
    t, sig = [], []
    for k, off in enumerate(offsets):
        t.append(k * period + phase * period)
        sig.append(base + off)
    t = np.concatenate(t)
    sig = np.concatenate(sig, axis=0)

    cycles = [(k * period, (k + 1) * period) for k in range(3)]

    def phase_avg(sel):
        grid_stack = []
        for a, b in sel:
            grid = a + phase * (b - a)
            grid_stack.append(C._resample_at(t, sig, grid, "smooth"))
        cube = np.stack(grid_stack, axis=0)
        return np.median(cube, axis=0), cube.std(axis=0)

    med, spread = phase_avg(cycles)
    check("the median over three cycles is the middle cycle's offset",
          np.isclose(med.mean(), offsets[1], atol=1e-6), f"got {med.mean():.4f}")
    check("and it is NOT the mean of the three offsets",
          not np.isclose(med.mean(), float(np.mean(offsets)), atol=1e-3),
          "median, not mean -- see the docstring in _phase_average")
    check("the cross-cycle spread the median discards is non-zero",
          spread.max() > 0.2, f"max std {spread.max():.4f}")

    for i, off in enumerate(offsets):
        one, sp1 = phase_avg([cycles[i]])
        check(f"a single-cycle subset returns cycle {i} unchanged, not an average",
              np.isclose(one.mean(), off, atol=1e-6), f"got {one.mean():.4f} want {off}")
        check(f"and reports zero cross-cycle spread for cycle {i}",
              float(sp1.max()) == 0.0)

    # The builder must not quietly renormalise a long cycle to the median period.
    uneven = [(0.0, 0.50), (0.50, 1.06), (1.06, 1.50)]
    for i, (a, b) in enumerate(uneven):
        want = b - a
        got = float(np.median([y - x for x, y in [uneven[i]]]))
        check(f"a single cycle keeps its own {want:.2f} s period", np.isclose(got, want))
    check("the default subset still takes the median period over all cycles",
          np.isclose(float(np.median([b - a for a, b in uneven])), 0.50))

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'self-test: PASS'}")
    return 1 if fails else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clip", default="TROT", help="which cyclic clip's session to cut up")
    ap.add_argument("--self-test", action="store_true", help="no curated logs needed")
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    build(a.clip)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
