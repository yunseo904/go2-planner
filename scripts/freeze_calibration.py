#!/usr/bin/env python
"""Freeze the synthetic calibration probes to data/calibration_probes.npz.

The probes settle the ``CALIBRATION_NEEDED`` placeholders in ``planner.config``
without touching the benchmark, which `CLAUDE.md` §2 forbids tuning against.
Nothing upstream is imported and no RNG is used, so the archive is a pure
function of ``terrain_toolkit/calibrate.py``.

Writes:
    data/calibration_probes.npz        height fields, goals, per-probe metadata
    data/calibration_probes.sha256     sha256sum -c format (committed)
    data/calibration_probes.meta.json  human-readable index
    outputs/calibration_plan.md        which probe settles which parameter + protocol

Usage:
    python scripts/freeze_calibration.py [--verify] [--no-plan]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from planner.config import DEFAULT, Provenance  # noqa: E402
from terrain_toolkit import paths  # noqa: E402
from terrain_toolkit.calibrate import (  # noqa: E402
    CALIBRATION_MAP, GAP_WIDTHS_M, HORIZONTAL_SCALE, OBSTACLE_X, PIT_DEPTH_M,
    PROBE_LENGTH_M, PROBE_WIDTH_M, SPAWN_X, SPAWN_Y, STEP_DOWN_NOTE,
    STEP_HEIGHTS_M, VERTICAL_SCALE, build_probes,
)


def content_sha256(arrays: dict) -> str:
    h = hashlib.sha256()
    for key in sorted(arrays):
        a = np.asarray(arrays[key])
        h.update(key.encode())
        h.update(str(a.dtype).encode())
        h.update(str(a.shape).encode())
        h.update(np.ascontiguousarray(a).tobytes())
    return h.hexdigest()


def build_archive() -> dict:
    probes = build_probes()
    arrays = {
        "height_fields": np.stack([p.height_field for p in probes]),
        "goals": np.stack([p.goals_m for p in probes]),
        "families": np.array([p.family for p in probes]),
        "names": np.array([p.name for p in probes]),
        "levels": np.array([p.level for p in probes], dtype=np.int64),
        "params_m": np.array([p.param_m for p in probes], dtype=np.float64),
        "descriptions": np.array([p.description for p in probes]),
        "step_heights_m": np.asarray(STEP_HEIGHTS_M, dtype=np.float64),
        "gap_widths_m": np.asarray(GAP_WIDTHS_M, dtype=np.float64),
        "horizontal_scale": np.array(HORIZONTAL_SCALE),
        "vertical_scale": np.array(VERTICAL_SCALE),
        "terrain_length_m": np.array(PROBE_LENGTH_M),
        "terrain_width_m": np.array(PROBE_WIDTH_M),
        "spawn_x": np.array(SPAWN_X),
        "spawn_y": np.array(SPAWN_Y),
        "obstacle_x": np.array(OBSTACLE_X),
        "pit_depth_m": np.array(PIT_DEPTH_M),
        "num_goals": np.array(3),
    }
    arrays["content_sha256"] = np.array(content_sha256(
        {k: v for k, v in arrays.items() if k != "content_sha256"}))
    return arrays


def save(arrays: dict) -> str:
    paths.DATA_DIR.mkdir(parents=True, exist_ok=True)
    # OUT_NPZ lets a new probe set be frozen beside the existing archive instead of
    # over it.  data/calibration_probes.npz is hash-pinned and 42 runs of results already
    # reference it; adding the slope and roughness families writes a v2 rather than
    # invalidating that.
    out = OUT_NPZ or paths.CALIBRATION_NPZ
    sha = out.with_suffix("").with_suffix(".sha256") if OUT_NPZ else paths.CALIBRATION_SHA256
    np.savez_compressed(out, **arrays)
    digest = hashlib.sha256(out.read_bytes()).hexdigest()
    sha.write_text(f"{digest}  {out.name}\n")
    meta = {
        "content_sha256": str(arrays["content_sha256"]),
        "npz_sha256": digest,
        "n_probes": int(arrays["height_fields"].shape[0]),
        "shape": list(arrays["height_fields"].shape),
        "horizontal_scale": float(arrays["horizontal_scale"]),
        "vertical_scale": float(arrays["vertical_scale"]),
        "terrain_length_m": float(arrays["terrain_length_m"]),
        "terrain_width_m": float(arrays["terrain_width_m"]),
        "spawn": [float(arrays["spawn_x"]), float(arrays["spawn_y"])],
        "obstacle_x": float(arrays["obstacle_x"]),
        "pit_depth_m": float(arrays["pit_depth_m"]),
        "families": {f: int((arrays["families"] == f).sum())
                     for f in sorted(set(arrays["families"].tolist()))},
        "step_heights_m": arrays["step_heights_m"].tolist(),
        "gap_widths_m": arrays["gap_widths_m"].tolist(),
        "names": arrays["names"].tolist(),
        "source": "terrain_toolkit/calibrate.py (synthetic; no upstream code, no RNG)",
    }
    (paths.CALIBRATION_META_JSON if OUT_NPZ is None else OUT_NPZ.with_suffix('').with_suffix('.meta.json')).write_text(json.dumps(meta, indent=2) + "\n")
    return digest


def load_probes(verify: bool = True) -> dict:
    if verify and paths.CALIBRATION_SHA256.is_file():
        expected = paths.CALIBRATION_SHA256.read_text().split()[0]
        actual = hashlib.sha256(paths.CALIBRATION_NPZ.read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError(
                f"{paths.CALIBRATION_NPZ.name} sha256 mismatch: {actual} != {expected}")
    with np.load(paths.CALIBRATION_NPZ, allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def _plan_md(arrays: dict) -> str:
    covered, uncovered = [], []
    for key, (family, what, how) in CALIBRATION_MAP.items():
        prov = DEFAULT.items()
        current = next(v for k, v, *_ in prov if k == key)
        if family is None:
            uncovered.append([f"`{key}`", current, what, how])
        else:
            n = int((arrays["families"] == family).sum())
            covered.append([f"`{key}`", current, f"`{family}`", n, what, how])

    def table(header, rows):
        out = ["| " + " | ".join(map(str, header)) + " |",
               "| " + " | ".join(["---"] * len(header)) + " |"]
        out += ["| " + " | ".join(map(str, r)) + " |" for r in rows]
        return "\n".join(out)

    fams = {f: int((arrays["families"] == f).sum())
            for f in sorted(set(arrays["families"].tolist()))}
    n_cal = len(DEFAULT.needs_calibration())
    return f"""# Calibration plan — {int(arrays['height_fields'].shape[0])} probe terrains

Generated by `scripts/freeze_calibration.py` from `terrain_toolkit/calibrate.py`.
Frozen to `data/calibration_probes.npz`
(`content_sha256` `{str(arrays['content_sha256'])[:16]}…`).

## Why a separate terrain

`CLAUDE.md` §2: thresholds are derived from terrain geometry and measured skill
capability, never from performance on the benchmark. Calibrating on the 20
benchmark tasks would fit the measuring instrument to the thing being measured
and quietly hand the rule planner an advantage the E2E policy does not get.

These probes share **no code** with the benchmark: nothing imports
`set_terrain_benchmark`, and there is no RNG call in `calibrate.py`, so there is
no seed that could collide. The one shared value is the pit depth
({float(arrays['pit_depth_m']):.1f} m), so that "gap" means the same thing to
`planner.features` on both.

## Probe geometry

Each probe is a flat {float(arrays['terrain_length_m']):.0f} m x {float(arrays['terrain_width_m']):.0f} m lane
with **one** obstacle at x = {float(arrays['obstacle_x']):.1f} m, spawn at
({float(arrays['spawn_x']):.1f}, {float(arrays['spawn_y']):.1f}), and three goals:
approach, just-past-the-obstacle, far end. Reaching goal 2 is what "cleared it"
means. One obstacle per terrain rather than a staircase, so a failure at 0.14 m
cannot contaminate the reading at 0.16 m.

| family | n | parameter sweep |
| --- | --- | --- |
| `step_up` | {fams.get('step_up', 0)} | {float(arrays['step_heights_m'][0]):.2f} → {float(arrays['step_heights_m'][-1]):.2f} m in 0.02 m |
| `step_down` | {fams.get('step_down', 0)} | same heights, descending |
| `gap` | {fams.get('gap', 0)} | {float(arrays['gap_widths_m'][0]):.2f} → {float(arrays['gap_widths_m'][-1]):.2f} m in 0.05 m, pit at {float(arrays['pit_depth_m']):.1f} m |

## What each probe settles

{table(["parameter", "current placeholder", "probe", "levels", "measures", "how to read it off"], covered)}

**{len(uncovered)} of the {n_cal} `CALIBRATION_NEEDED` parameters are not covered by this set:**

{table(["parameter", "current placeholder", "measures", "what it would need"], uncovered)}

A ramp probe (sustained incline, 0-40° in 2.5° steps) and a roughness probe
(band-limited noise, RMS 0-0.06 m in 0.005 m steps) would close those five. They
are deliberately not in this archive — the request was for step and gap probes,
and adding untested families to a frozen artefact is worse than recording the
gap.

## `step_down`

{STEP_DOWN_NOTE}

## Protocol

1. Freeze once, commit the hash: `python scripts/freeze_calibration.py --verify`.
   `data/*.sha256` is committed (`.gitignore` has `!data/*.sha256`); the `.npz`
   is not.
2. For each gait in WALK / TROT / RUN and each probe level, run **n ≥ 5 repeats**
   from a settled stand. Record cleared / failed / fell.
3. The limit is the largest level cleared on **every** repeat, minus one level of
   margin. Report the raw pass/fail matrix alongside the chosen number — a single
   threshold hides how sharp the transition was.
4. Write the results back into `planner/config.py` and flip the parameter's
   `Provenance` from `CALIBRATION_NEEDED` to `MEASURED`, with the probe archive's
   `content_sha256` as the `source`.
5. Re-run `scripts/simulate_planner_offline.py`; §5 of its report exists to show
   whether a conclusion moved because of the new numbers.

Do not calibrate against the benchmark, and do not re-freeze the probes with
different levels after collecting data — that silently changes what the numbers
mean.
"""


#: Set by --out; None means the canonical archive.
OUT_NPZ = None


def _sha_path_for(out):
    """Where the digest for ``out`` goes.  None -> the canonical sha256 file."""
    from terrain_toolkit import paths as _p
    return _p.CALIBRATION_SHA256 if out is None else out.with_suffix("").with_suffix(".sha256")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None,
                    help="write to this .npz instead of the canonical archive, so a new "
                         "probe family can be frozen without invalidating the pinned one")
    ap.add_argument("--verify", action="store_true", help="rebuild once more and check determinism")
    ap.add_argument("--no-plan", action="store_true", help="skip outputs/calibration_plan.md")
    args = ap.parse_args()
    global OUT_NPZ
    OUT_NPZ = Path(args.out) if args.out else None

    arrays = build_archive()
    if args.verify:
        again = build_archive()
        assert str(again["content_sha256"]) == str(arrays["content_sha256"]), \
            "probe generation is not deterministic!"
        print("[calib] determinism check passed")

    digest = save(arrays)
    n = int(arrays["height_fields"].shape[0])
    fams = {f: int((arrays["families"] == f).sum())
            for f in sorted(set(arrays["families"].tolist()))}
    print(f"[calib] wrote {OUT_NPZ or paths.CALIBRATION_NPZ} ({n} probes: "
          f"{', '.join(f'{k}x{v}' for k, v in fams.items())})")
    print(f"[calib] sha256 {digest}")
    print(f"[calib] wrote {_sha_path_for(OUT_NPZ)}")
    print(f"[calib] wrote {(paths.CALIBRATION_META_JSON if OUT_NPZ is None else OUT_NPZ.with_suffix('').with_suffix('.meta.json'))}")

    if not args.no_plan:
        paths.OUTPUTS_DIR.mkdir(parents=True, exist_ok=True)
        paths.CALIBRATION_PLAN_MD.write_text(_plan_md(arrays), encoding="utf-8")
        print(f"[calib] wrote {paths.CALIBRATION_PLAN_MD}")

    missing = [k for k, v in CALIBRATION_MAP.items() if v[0] is None]
    print(f"[calib] covers {len(CALIBRATION_MAP) - len(missing)}/{len(CALIBRATION_MAP)} "
          f"CALIBRATION_NEEDED parameters; not covered: {', '.join(missing)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
