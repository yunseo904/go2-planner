#!/usr/bin/env python3
"""Read a whole-session replay trace: what survived, for how long, how far.

    python scripts/analyze_full_session.py outputs/traces/WALK_FULL.npz ...

A full-session trace cannot be read with ``analyze_drift.py --trace``: that one
slices the run into blocks of ``clip_n`` control steps, which for a clip is one
gait cycle and for a whole session is the whole session.  Here the cycles come
from the RECORDING's own reference-foot touchdowns (the same contact channel the
extraction cuts on), read out of the archive that was played, and everything
before the motion window is reported separately -- a session begins standing, and
averaging that stretch into the gait numbers is how a replay gets credited with
survival time it did not earn.

The tipping angle is the same geometric criterion as
``outputs/open_loop_replay_limit.md`` 2: ``atan(half stance width / base
height)``, both measured from the robot's own kinematics in the first gait cycle
of THIS run.  Nothing here is fitted.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.replay import quat_rotate_inv, quat_to_rpy_deg
from terrain_toolkit.paths import DATA_DIR

CANON = ("FL", "FR", "RL", "RR")


def order(names):
    out = []
    for leg in CANON:
        hit = [i for i, nm in enumerate(names) if nm.upper().startswith(leg)]
        if len(hit) != 1:
            raise SystemExit(f"cannot map {names} onto {list(CANON)}")
        out.append(hit[0])
    return out


def touchdowns(contact_ref: np.ndarray) -> np.ndarray:
    """Indices where the reference foot goes down."""
    c = contact_ref.astype(bool).astype(int)
    return np.flatnonzero(np.diff(c) == 1) + 1


def report(trace: Path, archive: Path, meta: dict) -> None:
    t = np.load(trace, allow_pickle=False)
    name = str(t["clip_name"])
    dt = float(t["dt"])
    pos, quat = t["root_pos_w"], t["root_quat_w"]
    vb, wb = t["root_lin_vel_b"], t["root_ang_vel_b"]
    conf = t["contact_f"]
    feet = t["foot_pos_w"]
    steps = len(pos)
    term = float(t["terminated_s"])

    fo = order([str(x) for x in t["foot_names"]])
    co = order([str(x) for x in t["contact_names"]])
    conf, feet = conf[:, co], feet[:, fo]
    roll, pitch, yaw = quat_to_rpy_deg(quat)

    cm = meta["clips"][name]
    onset, offset = (float(v) for v in cm["motion_window_s"])
    dur_rec = float(cm["duration_s"])
    i_on, i_off = int(round(onset / dt)), int(round(offset / dt))

    z = np.load(archive, allow_pickle=False)
    con_clip = z[f"{name}__lo__contact"].astype(bool)
    td = touchdowns(con_clip[:, 0])
    td = td[td >= i_on]

    print("=" * 78)
    print(f"{name}   session {cm['session']}   recording {dur_rec:.2f} s; the recording "
          f"moves from {onset:.2f} s to {offset:.2f} s ({offset-onset:.2f} s of gait), "
          f"standing either side")
    survived = steps * dt
    if np.isfinite(term):
        print(f"  FELL at {term:.2f} s of the recording "
              f"({term - onset:+.2f} s relative to motion onset)")
    else:
        print(f"  NO FALL: played all {steps} control steps = {survived:.2f} s "
              f"(the whole recording)")

    if steps <= i_on:
        print(f"  the run ended {onset - steps*dt:.2f} s BEFORE the gait was commanded; "
              f"nothing below is about the gait")
        gait = slice(0, steps)
    else:
        gait = slice(i_on, min(i_off, steps))
        played = (gait.stop - gait.start) * dt
        print(f"  gait played: {played:.2f} s of the {offset - onset:.2f} s "
              f"the recording contains ({100*played/(offset-onset):.0f} %)")

    # --- where it went, over the moving part only -------------------------
    p0, p1 = pos[gait][0], pos[gait.stop - 1]
    d = p1 - p0
    dist = float(np.hypot(d[0], d[1]))
    span = max((gait.stop - gait.start) * dt, 1e-9)
    print(f"  travelled  x {d[0]:+.3f} m  y {d[1]:+.3f} m  |xy| {dist:.3f} m "
          f"in {span:.2f} s  -> {dist/span:.3f} m/s")
    print(f"  body vel   vx {vb[gait,0].mean():+.3f}  vy {vb[gait,1].mean():+.3f} m/s   "
          f"yaw rate {np.degrees(wb[gait,2]).mean():+.1f} deg/s   "
          f"yaw total {yaw[gait.stop-1]-yaw[gait][0]:+.1f} deg")
    if abs(d[0]) > 1e-3:
        print(f"  net turn   {(yaw[gait.stop-1]-yaw[gait][0])/max(dist,1e-9):+.2f} deg/m "
              f"(net yaw over net distance -- NOT a steady curvature over this "
              f"short a run; the yaw wanders)")

    # --- standing stretch --------------------------------------------------
    if i_on > 0 and steps > 1:
        s = slice(0, min(i_on, steps))
        print(f"  before motion ({s.stop*dt:.2f} s standing): base z "
              f"{pos[s,2].mean():.3f} m (start {pos[0,2]:.3f} -> end {pos[s.stop-1,2]:.3f}), "
              f"|roll| max {np.abs(roll[s]).max():.2f} deg, "
              f"drift {np.hypot(*(pos[s.stop-1,:2]-pos[0,:2])):.3f} m")

    # --- the state the gait was handed, which is the clip runs' confound ---
    # A clip replay starts its gait ~0.5 s after a settle and inherits whatever
    # that settle left behind (0.068 m/s for the frozen TROT).  A full session
    # stands for seconds first, so this number says how much of that confound is
    # left by the time the recording actually asks for the gait.
    if 0 < i_on < steps:
        fl = int((conf[i_on] > 30.0).sum())
        print(f"  state at motion onset: |v| {np.linalg.norm(vb[i_on]):.3f} m/s, "
              f"|w| {np.degrees(np.linalg.norm(wb[i_on])):.1f} deg/s, "
              f"roll {roll[i_on]:+.2f} deg, base z {pos[i_on,2]:.3f} m, {fl}/4 feet loaded")

    # --- tipping angle from this run's own first gait cycle ----------------
    if len(td) >= 2 and td[1] < steps:
        c0 = slice(int(td[0]), int(min(td[1], steps)))
        fb = quat_rotate_inv(quat[c0][:, None, :], feet[c0] - pos[c0][:, None, :])
        half_w = float(np.abs(fb[:, :, 1]).mean())
        h = float(pos[c0, 2].mean())
        tip = float(np.degrees(np.arctan2(half_w, h)))
        over = np.flatnonzero(np.abs(roll) > tip)
        over = over[over >= i_on]
        print(f"  tipping angle atan({half_w*1000:.1f} mm / {h*1000:.1f} mm) = {tip:.1f} deg"
              + (f"  -> crossed at {over[0]*dt:.2f} s "
                 f"({(over[0]-i_on)*dt:.2f} s into the gait, "
                 f"{np.hypot(*(pos[over[0],:2]-p0[:2])):.3f} m travelled)"
                 if over.size else "  -> never crossed"))

    # --- per gait cycle, cut on the RECORDING's reference-foot touchdowns ---
    td = td[(td < steps) & (td <= i_off)]
    if len(td) >= 2:
        print(f"\n  per gait cycle ({len(td)-1} complete, cut on the recording's "
              f"FL touchdowns):")
        print("   cyc | t_start |    vx     vy  yawrate | roll_mean roll_max | "
              "stance FL   FR   RL   RR")
        peaks = []
        for k in range(len(td) - 1):
            s = slice(int(td[k]), int(td[k + 1]))
            if s.stop - s.start < 2:
                continue
            st = (conf[s] > 30.0).mean(axis=0)
            rmax = roll[s][np.abs(roll[s]).argmax()]
            peaks.append(abs(float(rmax)))
            print(f"   {k:3d} | {td[k]*dt:7.2f} | {vb[s,0].mean():+.3f} {vb[s,1].mean():+.3f} "
                  f"{np.degrees(wb[s,2]).mean():+7.1f} |   {roll[s].mean():+6.2f}  "
                  f"{rmax:+7.2f} |        "
                  f"{st[0]:.2f} {st[1]:.2f} {st[2]:.2f} {st[3]:.2f}")
        if len(peaks) >= 2:
            g = [peaks[i + 1] / peaks[i] for i in range(len(peaks) - 1) if peaks[i] > 1e-6]
            print("   peak |roll| per cycle: " + "  ".join(f"{p:.2f}" for p in peaks))
            print("   growth ratio        : " + "  ".join(f"x{r:.1f}" for r in g))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("traces", nargs="+", type=Path)
    ap.add_argument("--archive", type=Path, default=DATA_DIR / "full_sessions.npz")
    args = ap.parse_args()
    meta = json.loads(args.archive.with_suffix("").with_suffix(".meta.json").read_text())
    for p in args.traces:
        report(p, args.archive, meta)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
