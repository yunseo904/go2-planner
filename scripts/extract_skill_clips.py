#!/usr/bin/env python3
"""Freeze replayable skill clips to ``data/skill_clips.npz``.

    python scripts/extract_skill_clips.py            # build + verify + report
    python scripts/extract_skill_clips.py --verify   # re-check the frozen file only

One clip per skill (WALK / TROT / RUN / JUMP), each at the session's own rate and
at 50 Hz, leg order FL,FR,RL,RR.  See ``motion_toolkit/clips.py`` for what a clip
is and for the two things a consumer must not assume: the gain schedule is not
constant, and the sign/zero convention is unverified.

Archive layout
--------------
``<CLIP>__<rate>__<channel>``  with rate in ``hi`` / ``lo`` and channel in
``t, q_des, dq_des, tau_ff, q, dq, tau, kp, kd, contact, q_des_valid``.
Plus ``clip_names``, ``leg_order``, ``joint_order``, ``target_fs``.
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
from terrain_toolkit.paths import (
    CURATED_ROOT,
    SKILL_CLIPS_META_JSON,
    SKILL_CLIPS_MD,
    SKILL_CLIPS_NPZ,
    SKILL_CLIPS_SHA256,
    require_curated,
)

CHANNELS = ("t", "q_des", "dq_des", "tau_ff", "q", "dq", "tau", "kp", "kd", "contact", "q_des_valid")


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def build() -> tuple:
    root = require_curated()
    print(f"[clips] curated root: {root}")
    sessions = iter_sessions(root)
    print(f"[clips] profiling {len(sessions)} sessions to pick representatives ...")
    df = profile_all(sessions)
    picks = C.select_sessions(df)

    by_name = {s.path.name: s for s in sessions}
    built, meta_clips = [], {}
    for spec in C.CLIP_SPECS:
        pick = picks[spec.name]
        if pick["session"] is None:
            print(f"[clips] {spec.name}: SKIPPED -- {pick['why']}")
            continue
        sess = by_name[pick["session"]]
        clip = C.build_clip(sess, spec)
        clip.meta["selection"] = {k: v for k, v in pick.items() if k != "candidates"}
        clip.meta["n_candidates"] = len(pick["candidates"])
        clip.meta["gains"] = C.gain_summary(sess)
        built.append(clip)
        m = clip.meta
        print(
            f"[clips] {clip.name:5s} {clip.kind:8s} {clip.session:38s} "
            f"hi={clip.hi['q_des'].shape[0]:5d}@{clip.fs_hi:6.1f}Hz "
            f"lo={clip.lo['q_des'].shape[0]:4d}@{clip.fs_lo:5.1f}Hz "
            f"alias>{25:.0f}Hz={m['alias_energy_frac_above_25hz']*100:6.3f}%"
        )
    return built, picks


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
                arrays[f"{c.name}__{rate}__{ch}"] = v.astype(np.uint8) if v.dtype == bool else np.asarray(v, dtype=np.float32)
        arrays[f"{c.name}__fs_hi"] = np.asarray(c.fs_hi)
        arrays[f"{c.name}__fs_lo"] = np.asarray(c.fs_lo)

    SKILL_CLIPS_NPZ.parent.mkdir(parents=True, exist_ok=True)
    with open(SKILL_CLIPS_NPZ, "wb") as fh:
        np.savez(fh, **arrays)
    digest = sha256_of(SKILL_CLIPS_NPZ)
    SKILL_CLIPS_SHA256.write_text(f"{digest}  {SKILL_CLIPS_NPZ.name}\n")

    meta = {
        "npz_sha256": digest,
        "n_clips": len(clips),
        "leg_order_native": ["FR", "FL", "RR", "RL"],
        "leg_order_stored": C.TARGET_LEGS,
        "joint_order": list(JOINTS),
        "target_fs_hz": C.TARGET_FS,
        "lowpass_frac_of_target": C.LOWPASS_FRAC,
        "channels": list(CHANNELS),
        "convention_verified": False,
        "convention_note": (
            "Joint sign convention, zero offsets and hip abduction direction are taken from the "
            "log as-is and have NOT been checked against the Isaac Lab Go2 asset. Run "
            "scripts/verify_skill_replay.py on a machine with Isaac Lab before trusting a replay."
        ),
        "gain_note": (
            "kp/kd are scheduled per skill and within a skill (walk 40/40/40, running trot "
            "13/3/2 with ~11 Nm RMS calf tau_ff). They are stored as per-sample time series and "
            "must be applied; a position-only replay under the fork's configured gains (kp 40 / kd 1) will not "
            "reproduce the running trot."
        ),
        "clips": {
            c.name: {
                "kind": c.kind,
                "session": c.session,
                "group": c.group,
                "fs_hi_hz": c.fs_hi,
                "fs_lo_hz": c.fs_lo,
                "n_hi": int(c.hi["q_des"].shape[0]),
                "n_lo": int(c.lo["q_des"].shape[0]),
                **{k: v for k, v in c.meta.items()},
            }
            for c in clips
        },
        "source": "motion_toolkit/clips.py from the read-only curated log set",
    }
    SKILL_CLIPS_META_JSON.write_text(json.dumps(meta, indent=2, default=float) + "\n")
    return meta


def verify() -> int:
    if not SKILL_CLIPS_NPZ.is_file():
        print("[clips] no archive to verify")
        return 1
    want = SKILL_CLIPS_SHA256.read_text().split()[0]
    got = sha256_of(SKILL_CLIPS_NPZ)
    z = np.load(SKILL_CLIPS_NPZ, allow_pickle=False)
    names = [str(x) for x in z["clip_names"]]
    ok = want == got
    print(f"[clips] sha256 {'OK' if ok else 'MISMATCH'}  {got}")
    for n in names:
        for rate in ("hi", "lo"):
            q = z[f"{n}__{rate}__q_des"]
            t = z[f"{n}__{rate}__t"]
            assert q.shape[1] == 12 and len(t) == len(q), f"{n}/{rate} shape"
            assert np.isfinite(q).all(), f"{n}/{rate} non-finite q_des"
            assert np.abs(q).max() < 1e3, f"{n}/{rate} sentinel leaked into q_des"
        print(f"[clips]   {n}: hi={z[f'{n}__hi__q_des'].shape} lo={z[f'{n}__lo__q_des'].shape} OK")
    return 0 if ok else 1


def report(meta: dict) -> None:
    L = ["# Skill clips", "",
         f"`{SKILL_CLIPS_NPZ.name}` — sha256 `{meta['npz_sha256'][:16]}…`, {meta['n_clips']} clips.",
         "",
         "Built by `scripts/extract_skill_clips.py` from the read-only curated logs.",
         "Leg order stored is **" + ",".join(meta["leg_order_stored"]) + "** "
         "(native log order is " + ",".join(meta["leg_order_native"]) + ").",
         "", "## Clips", "",
         "| clip | kind | source session | hi rate | n hi | lo rate | n lo | cycle / duration | duty | q_des update | loop seam | alias >25 Hz |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for name, c in meta["clips"].items():
        span = f"{c['cycle_s']*1000:.0f} ms" if "cycle_s" in c else f"{c['duration_s']:.2f} s"
        duty = f"{c['duty_clip']:.3f}" if "duty_clip" in c else "—"
        seam = (f"{c['loop_seam_rad']:.3f} rad ({c['loop_seam_over_max_step']:.2f}× max step)"
                if c["kind"] == "cyclic" else "n/a")
        L.append(f"| `{name}` | {c['kind']} | `{c['session']}` | {c['fs_hi_hz']:.1f} Hz | {c['n_hi']} | "
                 f"{c['fs_lo_hz']:.1f} Hz | {c['n_lo']} | {span} | {duty} | {c['q_des_update_hz']:.0f} Hz | {seam} | "
                 f"{c['alias_energy_frac_above_25hz']*100:.3f}% |")
    L += ["", "`loop seam` is the wrap-around jump in `q_des`, in radians and as a multiple of the "
              "largest sample-to-sample step the clip already contains. At or below 1× the seam is "
              "no faster than a transition inside the cycle. One-shot clips are not looped.", "",
          "`q_des update` is how often the sport controller actually wrote a new command. The log "
          "samples at ~419 Hz but holds the value in between, so a clip whose command rate is already "
          "near 50 Hz has little left to decimate and one in the hundreds has a lot.", "",
          "`alias >25 Hz` is the share of `q_des` variance above the 50 Hz Nyquist — what plain "
          "decimation would have folded into the gait band. It was removed by the filter, not by "
          "the decimation.", "",
          "## Selection", "",
          "| clip | picked from | why | duty | stride Hz | vx steady |", "|---|---|---|---|---|---|"]
    for name, c in meta["clips"].items():
        s = c.get("selection", {})
        L.append(f"| `{name}` | {c['n_candidates']} candidates | {s.get('why','—')} | "
                 f"{s.get('duty_mean',float('nan')):.3f} | {s.get('stride_hz',float('nan')):.2f} | "
                 f"{s.get('vx_steady_mean',float('nan')):.3f} m/s |")
    L += ["", "## Commanded gains — not constant, and not what a replay can ignore", "",
          "Medians inside the motion window, per joint type (hip / thigh / calf):", "",
          "| clip | kp | kd | tau_ff RMS (Nm) | position-controlled? |", "|---|---|---|---|---|"]
    for name, c in meta["clips"].items():
        g = c["gains"]
        f3 = lambda v: " / ".join(f"{x:g}" for x in v)
        L.append(f"| `{name}` | {f3(g['kp'])} | {f3(g['kd'])} | {f3(g['tau_ff_rms'])} | "
                 f"{'yes' if g['position_controlled'] else '**no**'} |")
    L += ["", meta["gain_note"], "", "## Caveats", ""]

    cyc = [(n, c) for n, c in meta["clips"].items() if c["kind"] == "cyclic"]
    fast = [(n, c) for n, c in cyc if c["loop_seam_over_max_step"] > 1.0]
    if fast:
        L += ["**Loop seam.** " + ", ".join(
            f"`{n}` closes with a {c['loop_seam_rad']:.3f} rad jump "
            f"({c['loop_seam_over_max_step']:.2f}\u00d7 its own largest step)" for n, c in fast) +
            ". A ratio above 1 means the wrap is faster than anything inside the cycle, so a looped "
            "replay commands a step the real robot never commanded. The cut is on the reference "
            "foot's touchdown, which is a genuine discontinuity in the command stream — this is the "
            "cost of cutting there rather than an extraction bug. Blend the seam or play a "
            "single cycle if it shows up in the replay.", ""]

    run = meta["clips"].get("RUN")
    if run:
        L += [f"**Cycle rate.** The working figure carried into this task was 3.25 Hz / 308 ms. "
              f"Measured on the selected session the running-trot cycle is "
              f"**{run['cycle_hz']:.2f} Hz / {run['cycle_s']*1000:.0f} ms** "
              f"(spread {run['cycle_s_spread']*1000:.0f} ms over {run['n_cycles_kept']} cycles), "
              f"and across all 11 running-trot sessions `skill_profile.csv` gives 2.79-3.09 Hz. "
              f"Nothing in the clip is snapped to 308 ms; the cut is on contact events and the "
              f"clip carries whatever period that produced.", ""]

    slow = [n for n, c in cyc if c["q_des_update_hz"] < 60]
    if slow:
        L += ["**Downsampling is nearly free for " + ", ".join(f"`{n}`" for n in slow) +
              ".** The sport controller wrote new commands at ~43 Hz on those sessions, below the "
              "50 Hz target, so the ~419 Hz copy is a zero-order-held staircase of the same "
              "information. `RUN` and `JUMP` update in the hundreds of Hz and do lose content.", ""]

    L += ["**Mapping discrimination.** `sim/diagnose.py`'s leg-order/sign search separates the "
          "candidates far better on `RUN` and `JUMP` (margin ~0.18) than on `WALK` and `TROT` "
          "(~0.04, under its own 0.05 warning threshold): the slow gaits are too symmetric to tell "
          "a mirrored mapping from the right one. Verify the Isaac convention on `RUN` or `JUMP` "
          "first.", ""]

    L += ["## Unverified", "", meta["convention_note"], ""]
    SKILL_CLIPS_MD.parent.mkdir(parents=True, exist_ok=True)
    SKILL_CLIPS_MD.write_text("\n".join(L))
    print(f"[clips] wrote {SKILL_CLIPS_MD}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--verify", action="store_true", help="verify the frozen archive and exit")
    args = ap.parse_args()
    if args.verify:
        return verify()
    clips, _ = build()
    if not clips:
        print("[clips] nothing built", file=sys.stderr)
        return 1
    meta = save(clips)
    report(meta)
    print(f"[clips] wrote {SKILL_CLIPS_NPZ}  sha256 {meta['npz_sha256']}")
    return verify()


if __name__ == "__main__":
    raise SystemExit(main())
