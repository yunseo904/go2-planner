#!/usr/bin/env python3
"""Clips for testing a SKILL TRANSITION, not a skill.

    python scripts/extract_transition_clips.py

Writes ``data/transition_clips.npz``: the four gait clips exactly as
``data/skill_clips.npz`` holds them (rebuilt by the same code, and checked
byte-for-byte against it), plus ``BALANCE`` -- a stretch of the quiet
``balance_stand`` posture from ``stand_up_down_20260824_222143``.

Why a separate archive: ``skill_clips.npz`` is shared and someone else is working
against it. Nothing here touches it; it is only read, to prove the gait clips in
this archive are the same ones.

Why BALANCE at all: the real robot put ``balance_stand`` in front of
``front_jump`` in 8 of 8 recordings (CLAUDE.md 3), so routing a skill change
through it is the machine's own procedure. The window is taken AFTER the
session's final ``balance_stand`` command, where kp is 40, there is no PosStopF
sample, and q_des moves 0.048 rad total -- i.e. genuinely standing still.
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
from terrain_toolkit.paths import DATA_DIR, SKILL_CLIPS_NPZ, require_curated

sys.path.insert(0, str(Path(__file__).resolve().parent))
from extract_full_sessions import CHANNELS, sha256_of   # noqa: E402  (same layout)

NPZ = DATA_DIR / "transition_clips.npz"
META_JSON = DATA_DIR / "transition_clips.meta.json"
#: seconds after the session's last balance_stand command, and how long to keep
BALANCE_AFTER_S, BALANCE_LEN_S = 2.0, 3.0


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
        "npz_sha256": sha256_of(NPZ), "n_clips": len(clips),
        "leg_order_native": ["FR", "FL", "RR", "RL"], "leg_order_stored": C.TARGET_LEGS,
        "joint_order": list(JOINTS), "target_fs_hz": C.TARGET_FS,
        "lowpass_frac_of_target": C.LOWPASS_FRAC, "channels": list(CHANNELS),
        "convention_verified": False,
        "convention_note": "Same convention as data/skill_clips.npz; unverified in the same way.",
        "gain_note": "Same per-sample kp/kd/tau_ff as the frozen archive.",
        "purpose": "Gait clips identical to skill_clips.npz, plus BALANCE for transition tests.",
        "clips": {c.name: {"kind": c.kind, "session": c.session, "group": c.group,
                           "fs_hi_hz": c.fs_hi, "fs_lo_hz": c.fs_lo,
                           "n_hi": int(c.hi["q_des"].shape[0]),
                           "n_lo": int(c.lo["q_des"].shape[0]),
                           **{k: v for k, v in c.meta.items()}} for c in clips},
        "source": "motion_toolkit/clips.py build_clip + build_hold_clip",
    }
    META_JSON.write_text(json.dumps(meta, indent=2, default=float) + "\n")
    return meta


def main() -> int:
    argparse.ArgumentParser(description=__doc__).parse_args()
    root = require_curated()
    sessions = iter_sessions(root)
    print(f"[trans] profiling {len(sessions)} sessions ...")
    picks = C.select_sessions(profile_all(sessions))
    by = {s.path.name: s for s in sessions}

    built = []
    for spec in C.CLIP_SPECS:
        pick = picks[spec.name]
        if pick["session"] is None:
            continue
        c = C.build_clip(by[pick["session"]], spec)
        c.meta["gains"] = C.gain_summary(by[pick["session"]])
        c.meta["selection"] = {k: v for k, v in pick.items() if k != "candidates"}
        built.append(c)

    stand = next(s for s in sessions if s.path.name.startswith("stand_up_down"))
    sends = [e for e in stand.skill_sends()]
    t_last = stand.event_time(sends[-1]) - stand.t[0]
    spec = C.ClipSpec("BALANCE", "oneshot", motion_type=None,
                      note="balance_stand: the posture the robot holds between skills")
    b = C.build_hold_clip(stand, spec, t_last + BALANCE_AFTER_S,
                          t_last + BALANCE_AFTER_S + BALANCE_LEN_S)
    b.meta["gains"] = C.gain_summary(stand)
    b.meta["after_skill_send_s"] = t_last
    built.append(b)
    print(f"[trans] BALANCE  {b.session}  {b.meta['duration_s']:.2f}s from t={t_last+BALANCE_AFTER_S:.2f}s, "
          f"q_des range {b.meta['q_des_range_rad']:.4f} rad, mean|dq| {b.meta['mean_abs_dq']:.4f} rad/s")

    meta = save(built)
    print(f"[trans] -> {NPZ}  sha256 {meta['npz_sha256'][:16]}…")

    # The gait clips must be the ones already frozen, or a transition result would
    # be about a different clip than every result it is compared with.
    a, c = np.load(SKILL_CLIPS_NPZ), np.load(NPZ)
    shared = [k for k in a.files if k in c.files and k not in ("clip_names", "clip_kinds")]
    bad = [k for k in shared if not np.array_equal(a[k], c[k])]
    print(f"[trans] {len(shared)-len(bad)}/{len(shared)} shared arrays byte-identical to "
          f"skill_clips.npz" + (f"; DIFFER: {bad}" if bad else ""))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
