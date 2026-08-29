#!/usr/bin/env python3
"""Read outputs/foot_comp.csv into the one table the stage-2 question is asked in.

    python3 scripts/foot_comp_table.py [csv]

Columns are chosen to make the failure mode named in advance -- roll suppressed,
travel destroyed -- impossible to miss: vx is next to the survival time, always,
and the expected stride sits next to the measured one.
"""
import csv, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def expectations():
    """(stride Hz, vx m/s) per clip, from the archive meta -- never hardcoded.

    They were hardcoded once and RUN's stride was copied from a *measured* row
    instead of the clip's own cycle_hz: 2.47 against the real 3.09, which made a
    replay that was 20% slow read as exact. The harness's own verdict was never
    wrong -- it reads expected_from_meta -- but this table was.
    """
    import json
    m = json.loads((ROOT / "data/skill_clips.meta.json").read_text())
    out = {}
    for name, c in m["clips"].items():
        out[name] = (c.get("cycle_hz", float("nan")),
                     c.get("selection", {}).get("vx_steady_mean", float("nan")))
    return out


EXP = expectations()


def f(x, d=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return d


def main(path=None):
    rows = list(csv.DictReader(open(path or ROOT / "outputs/foot_comp.csv")))
    print(f"{'tag':14s} {'clip':5s} {'cap':>6s} {'k':>5s} {'sign':>5s} "
          f"{'survived':>9s} {'cycles':>7s} {'stride':>12s} {'vx':>13s} {'vy':>7s} "
          f"{'ovr%':>6s} {'hipRMS':>7s} {'maxdev':>7s} {'cap%':>5s}  verdict")
    for r in rows:
        clip = r["clip"]
        e_st, e_vx = EXP.get(clip, (float("nan"), float("nan")))
        term = f(r["terminated_s"])
        st = f(r["stride_hz"])
        n = int(f(r["n_steps"], 0))
        dt = f(r["dt"], 1.0)
        cyc = n * dt * e_st
        surv = "60+ cyc" if r["terminated_s"] in ("", None) else f"{term:6.2f} s"
        rel = (st - e_st) / e_st * 100 if st == st else float("nan")
        print(f"{r.get('tag',''):14s} {clip:5s} {f(r['foot_clip_rad']):6.3f} "
              f"{f(r['foot_k']):5.2f} {f(r['foot_sign']):5.0f} "
              f"{surv:>9s} {cyc:7.1f} {st:6.2f}({rel:+5.0f}%) "
              f"{f(r['vx_mean']):6.3f}/{e_vx:5.3f} {f(r['vy_mean']):7.3f} "
              f"{f(r['overwrite_frac_time'])*100:6.1f} "
              f"{f(r['dev_cmd_rms_hip_swing']):7.4f} {f(r['dev_cmd_max_rad']):7.4f} "
              f"{f(r['foot_cap_hit_frac'])*100:5.1f}  {r['verdict']}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)


def dev_table(tag: str, path=None):
    """The per-leg/per-joint/per-phase departure table for one run."""
    rows = [r for r in csv.DictReader(open(path or ROOT / "outputs/foot_comp_dev.csv"))
            if r.get("tag") == tag]
    if not rows:
        print(f"no rows tagged {tag!r}")
        return
    print(f"departure from the recording -- {tag} ({rows[0]['clip']}, "
          f"cap {rows[0]['foot_clip_rad']} rad)")
    print(f"  {'leg':4s} {'joint':6s} {'phase':7s} {'steps':>6s} "
          f"{'cmd RMS':>9s} {'cmd max':>9s} {'meas RMS':>9s} {'meas max':>9s}   (rad)")
    for r in rows:
        print(f"  {r['leg']:4s} {r['joint']:6s} {r['phase']:7s} {int(f(r['n_steps'])):6d} "
              f"{f(r['cmd_rms_rad']):9.4f} {f(r['cmd_max_rad']):9.4f} "
              f"{f(r['meas_rms_rad']):9.4f} {f(r['meas_max_rad']):9.4f}")
