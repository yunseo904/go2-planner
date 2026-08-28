#!/usr/bin/env python3
"""Freeze whole, uncut sessions for replay -- the least-processed test there is.

    python scripts/extract_full_sessions.py                 # build all three
    python scripts/extract_full_sessions.py --clip TROT
    python scripts/extract_full_sessions.py --self-test     # no logs needed

Every replay so far played a *clip*: one gait cycle, cut on touchdown, phase
averaged over the session's clean cycles, entered at a chosen phase and then
tiled.  Each of those steps is a place a failure could be blamed on, and
``outputs/open_loop_replay_limit.md`` had to rule them out one at a time (7.2 for
the averaging, 3 for the asymmetry, ``--start-phase`` for the entry point).

This removes all of them at once by not cutting anything.  The archive holds the
whole session -- start-up included, so the replay begins from the standing pose
the recording begins in, plays the skill spinning up, and continues to the last
sample.  No seam, no median, no phase choice, no tiling.

What is still done to the samples, and nothing else:

* the ``lo`` copy is resampled to 50 Hz with the same anti-aliasing every other
  clip uses (zero-phase Butterworth, then linear resample).  ``hi`` is the
  recording at the session's own rate.
* ``kp``/``kd``/``contact``/``q_des_valid`` are held, never filtered.
* sentinel/NaN ``q_des`` samples are linearly filled, exactly as elsewhere, and
  the fraction is reported.

Output is ``data/full_sessions.npz`` + ``.meta.json``, in the same layout as the
frozen archive, so it plays with

    scripts/isaac_docker_run.sh scripts/verify_skill_replay.py \\
        --clip-archive data/full_sessions.npz --clip TROT_FULL ...

The sessions are the same three the frozen clips come from, and they are chosen
the same way -- by the measured duty band, not by name (``clips.CLIP_SPECS``).
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
NPZ = DATA_DIR / "full_sessions.npz"
META_JSON = DATA_DIR / "full_sessions.meta.json"

#: Only the cyclic locomotion skills.  JUMP is already a whole event in the frozen
#: archive -- its one-shot clip is the motion window plus 0.25 s of pad, and there
#: is no cycle, no median and no seam in it to remove.
FULL_SPECS = [s for s in C.CLIP_SPECS if s.kind == "cyclic"]


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def save(clips: list) -> dict:
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

    NPZ.parent.mkdir(parents=True, exist_ok=True)
    with open(NPZ, "wb") as fh:
        np.savez(fh, **arrays)

    meta = {
        "npz_sha256": sha256_of(NPZ),
        "n_clips": len(clips),
        "leg_order_native": ["FR", "FL", "RR", "RL"],
        "leg_order_stored": C.TARGET_LEGS,
        "joint_order": list(JOINTS),
        "target_fs_hz": C.TARGET_FS,
        "lowpass_frac_of_target": C.LOWPASS_FRAC,
        "channels": list(CHANNELS),
        "convention_verified": False,
        "convention_note": "Same convention as data/skill_clips.npz; unverified in the same way.",
        "gain_note": "Same per-sample kp/kd/tau_ff as the frozen archive, for the whole session.",
        "purpose": (
            "Whole sessions, uncut: no cycle cutting, no phase averaging, no start-phase "
            "choice, no tiling. Start-up (balance_stand -> spin-up -> steady gait) included, "
            "so the replay starts from the standing pose the recording starts in. The only "
            "processing is the 50 Hz anti-aliased downsample in the 'lo' copy."
        ),
        "clips": {
            c.name: {
                "kind": c.kind, "session": c.session, "group": c.group,
                "fs_hi_hz": c.fs_hi, "fs_lo_hz": c.fs_lo,
                "n_hi": int(c.hi["q_des"].shape[0]), "n_lo": int(c.lo["q_des"].shape[0]),
                **{k: v for k, v in c.meta.items()},
            } for c in clips
        },
        "source": "motion_toolkit/clips.py build_full_session_clip",
    }
    META_JSON.write_text(json.dumps(meta, indent=2, default=float) + "\n")
    return meta


def build(only: str | None = None) -> dict:
    root = require_curated()
    print(f"[full] curated root: {root}")
    specs = [s for s in FULL_SPECS if only is None or s.name == only]
    if not specs:
        raise SystemExit(f"no cyclic clip spec named {only!r}; have "
                         f"{[s.name for s in FULL_SPECS]}")

    sessions = iter_sessions(root)
    print(f"[full] profiling {len(sessions)} sessions to pick the same representatives ...")
    picks = C.select_sessions(profile_all(sessions))
    by_name = {s.path.name: s for s in sessions}

    built = []
    for spec in specs:
        pick = picks[spec.name]
        if pick["session"] is None:
            raise SystemExit(f"{spec.name}: no session qualifies -- {pick['why']}")
        sess = by_name[pick["session"]]
        c = C.build_full_session_clip(sess, spec)
        c.name = f"{spec.name}_FULL"
        c.meta["gains"] = C.gain_summary(sess)
        c.meta["selection"] = {k: v for k, v in pick.items() if k != "candidates"}
        built.append(c)
        m = c.meta
        print(f"[full] {c.name:10s} {c.session:38s} "
              f"{m['duration_s']:6.2f}s  hi={c.hi['q_des'].shape[0]:5d}@{c.fs_hi:6.1f}Hz "
              f"lo={c.lo['q_des'].shape[0]:5d}@{c.fs_lo:5.1f}Hz  "
              f"start-up {m['startup_s']:.2f}s  alias>25Hz={m['alias_energy_frac_above_25hz']*100:.3f}%  "
              f"q_des invalid {m['q_des_invalid_frac']*100:.2f}%")
        print(f"[full]            skills: {' -> '.join(m['skill_sequence']) or '(no events.jsonl)'}")

    meta = save(built)
    print(f"[full] -> {NPZ}  sha256 {meta['npz_sha256'][:16]}…")
    return meta


# --------------------------------------------------------------------------- #
# Self-test: prove nothing was cut, averaged or looped.  No logs, no simulator.
# --------------------------------------------------------------------------- #

def self_test() -> int:
    from motion_toolkit.session import Session

    fs, dur = 400.0, 6.0
    n = int(fs * dur)
    t = np.arange(n) / fs
    rng = np.random.default_rng(0)

    # A synthetic session: 2 s of standing, then a 2 Hz gait, so a motion-window
    # cut would be VISIBLE as a missing head.
    amp = np.clip((t - 2.0) / 0.5, 0.0, 1.0)
    q = 0.3 * amp[:, None] * np.sin(2 * np.pi * 2.0 * t[:, None] + np.arange(12)[None, :])
    force = np.zeros((n, 4))
    for leg in range(4):
        ph = 2 * np.pi * 2.0 * t + (np.pi if leg in (1, 2) else 0.0)
        force[:, leg] = 60.0 + 60.0 * np.sin(ph) * amp
    force = np.maximum(force, 0.0) + rng.normal(0, 0.5, force.shape)

    class FakeSession:
        path = Path("fake_session_20260101_000000")
        group = "test"

        def __init__(self):
            self.t = t
            self.n = n
            self.fs = fs

        def joint_matrix(self, suffix):
            if suffix in ("kp",):
                return np.full((n, 12), 40.0)
            if suffix in ("kd",):
                return np.full((n, 12), 1.0)
            return q if suffix == "q_des" else q * 0.5

        def foot_force(self):
            return force

        def skill_sequence(self):
            return ["balance_stand", "gait_classic_walk"]

    sess = FakeSession()
    spec = C.ClipSpec("TEST", "cyclic")
    c = C.build_full_session_clip(sess, spec)

    fails = []

    def check(name, cond, detail=""):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"   {detail}" if detail else ""))
        if not cond:
            fails.append(name)

    check("hi keeps every sample", c.hi["q_des"].shape[0] == n,
          f"{c.hi['q_des'].shape[0]} vs {n}")
    check("hi starts at sample 0", abs(float(c.hi["t"][0])) < 1e-12)
    check("hi ends at the last sample", abs(float(c.hi["t"][-1]) - t[-1]) < 1e-9)
    # The permutation is a column reorder, so the SET of columns must be unchanged
    # and every column must appear intact somewhere -- that is what proves no
    # sample was filtered, averaged or dropped in the hi copy.
    check("hi q_des is the recording, column-permuted, sample for sample",
          np.allclose(np.sort(c.hi["q_des"], axis=1), np.sort(q, axis=1), atol=0, rtol=0))
    check("the standing start-up survives",
          float(np.abs(c.hi["q_des"][: int(1.5 * fs)]).max()) < 1e-9,
          "first 1.5 s is still the quiet stand")
    check("start-up length is reported, not removed", c.meta["startup_s"] > 1.0,
          f"startup_s={c.meta['startup_s']:.2f}s")
    check("kind is one-shot (never tiled by the replay)", c.kind == "oneshot")
    check("not marked loopable", c.meta["loopable"] is False)
    check("lo is 50 Hz over the same span",
          abs(c.lo["t"][-1] - c.hi["t"][-1]) < 1e-9
          and abs((len(c.lo["t"]) - 1) / c.lo["t"][-1] - C.TARGET_FS) < 1.0,
          f"{len(c.lo['t'])} samples over {c.lo['t'][-1]:.3f}s")
    check("lo holds kp/kd rather than filtering them",
          np.array_equal(np.unique(c.lo["kp"]), np.array([40.0])))
    check("no averaging key in the meta",
          not any(k in c.meta for k in ("n_cycles_used", "cycle_s", "loop_seam_rad")))
    print(f"\n{'PASS' if not fails else 'FAIL'}: {len(fails)} failed")
    return 1 if fails else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clip", default=None, help="WALK / TROT / RUN (default: all three)")
    ap.add_argument("--self-test", action="store_true", help="no curated logs needed")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    build(args.clip)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
