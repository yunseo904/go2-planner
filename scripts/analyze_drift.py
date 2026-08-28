#!/usr/bin/env python3
"""Why the open-loop replay drifts sideways.

Two questions, deliberately kept apart:

  --clips   Is the RECORDING asymmetric?  Reads data/skill_clips.npz only; no
            simulator.  A cyclic quadruped gait is left-right symmetric when the
            left legs at phase p do what the right legs do at phase p + T/2,
            mirrored.  Anything left over after that map is asymmetry the clip
            itself carries, and it is compared against what the real robot's
            own odometry did with those same commands.

  --trace   Does the SIM drift, and how?  Reads a --trace-npz from
            verify_skill_replay.py: per-cycle lateral velocity, yaw, per-leg
            stance time, per-leg vertical impulse, foot placement in the base
            frame, and which foot unloads first.

Nothing here modifies a clip or a replay.  It only measures.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim.replay import quat_rotate_inv, quat_to_rpy_deg          # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
NPZ = ROOT / "data" / "skill_clips.npz"
META = ROOT / "data" / "skill_clips.meta.json"
PROFILE = ROOT / "outputs" / "skill_profile.csv"

CANON = ("FL", "FR", "RL", "RR")
# Mirroring a quadruped about its own sagittal plane swaps left for right and
# reverses the abduction axis.  Thigh and calf rotate about the pitch axis, which
# the mirror leaves alone.
MIRROR_LEG = {"FL": "FR", "FR": "FL", "RL": "RR", "RR": "RL"}


def _leg_slice(leg_order, leg):
    return slice(3 * list(leg_order).index(leg), 3 * list(leg_order).index(leg) + 3)


def mirror(q, leg_order, hip_sign=-1.0):
    """q[:, 12] -> the same motion performed by a left-right mirrored robot."""
    out = np.empty_like(q)
    for leg in leg_order:
        out[:, _leg_slice(leg_order, MIRROR_LEG[leg])] = q[:, _leg_slice(leg_order, leg)]
    for leg in leg_order:                      # hip column of every leg
        out[:, _leg_slice(leg_order, leg).start] *= hip_sign
    return out


def best_shift(a, b):
    """Circular shift k of b minimising RMS(a - roll(b, k)); returns (k, rms)."""
    n = len(a)
    errs = np.array([np.sqrt(((a - np.roll(b, k, axis=0)) ** 2).mean()) for k in range(n)])
    k = int(errs.argmin())
    return k, float(errs[k]), errs


def real_robot_row(session):
    for r in csv.DictReader(PROFILE.open()):
        if r["session"] == session:
            return r
    return None


def clip_symmetry(rate="hi") -> int:
    meta = json.loads(META.read_text())
    z = np.load(NPZ)
    legs = meta["leg_order_stored"]
    print(f"Clip symmetry  [rate={rate}, leg order {legs}, joints {meta['joint_order']}]")
    print("A left-right symmetric gait maps onto itself under mirror + half-cycle shift.\n")

    for name, c in meta["clips"].items():
        if c["kind"] != "cyclic":
            continue
        q = z[f"{name}__{rate}__q_des"].astype(float)
        con = z[f"{name}__{rate}__contact"].astype(float)
        n = len(q)
        p2p = float(np.ptp(q, axis=0).max())

        print("=" * 78)
        print(f"{name}   {n} frames = one cycle ({c['cycle_s']:.3f} s), "
              f"q_des peak-to-peak {p2p:.3f} rad")

        # --- 1. does mirror + shift reproduce the clip? ------------------------
        for hs, label in ((-1.0, "hip negated"), (+1.0, "hip kept")):
            k, rms, errs = best_shift(q, mirror(q, legs, hs))
            print(f"  mirror ({label:11s}): best shift {k}/{n} = {k/n:.3f} cycle, "
                  f"residual {rms:.4f} rad = {100*rms/p2p:.1f}% of p2p "
                  f"(at exactly half a cycle: {errs[n//2]:.4f} rad = "
                  f"{100*errs[n//2]/p2p:.1f}%)")

        # --- 2. per-leg stance fraction from the clip's own contact labels -----
        duty = {leg: float(con[:, i].mean()) for i, leg in enumerate(legs)}
        L = 0.5 * (duty["FL"] + duty["RL"])
        R = 0.5 * (duty["FR"] + duty["RR"])
        print(f"  clip duty  " + "  ".join(f"{k} {v:.3f}" for k, v in duty.items())
              + f"   | left {L:.3f} right {R:.3f}  diff {L-R:+.3f}")

        # --- 3. commanded hip bias --------------------------------------------
        hip = {leg: float(q[:, _leg_slice(legs, leg).start].mean()) for leg in legs}
        print(f"  mean hip   " + "  ".join(f"{k} {v:+.4f}" for k, v in hip.items())
              + f"   | FL+FR {hip['FL']+hip['FR']:+.4f}  RL+RR {hip['RL']+hip['RR']:+.4f} rad "
                f"(0 if mirror-symmetric)")

        # --- 4. what the real robot did with these commands -------------------
        r = real_robot_row(c["session"])
        if r:
            vx, vy = float(r["vx_steady_mean"]), float(r["vy_steady_mean"])
            print(f"  REAL ROBOT {c['session']}: vx {vx:+.3f}  vy {vy:+.4f} m/s "
                  f"({100*abs(vy)/max(abs(vx),1e-9):.1f}% of forward), "
                  f"yaw rate {float(r['yaw_rate_steady_mean']):+.4f} rad/s, "
                  f"odom yaw {float(r['odom_yaw_deg']):+.2f} deg over "
                  f"{float(r['odom_dist_m']):.2f} m")
    return 0


# --------------------------------------------------------------------------- #

def trace_report(path: Path, cycle_table: bool = True) -> int:
    t = np.load(path, allow_pickle=False)
    name = str(t["clip_name"]) if "clip_name" in t else path.stem
    dt = float(t["dt"])
    n_cycle = int(t["clip_n"]) if "clip_n" in t else 0
    pos, quat = t["root_pos_w"], t["root_quat_w"]
    vb, wb = t["root_lin_vel_b"], t["root_ang_vel_b"]
    con, conf = t["contact"].astype(bool), t["contact_f"]
    feet = t["foot_pos_w"]
    steps = len(pos)
    term = float(t["terminated_s"])

    # The two orderings are different lists; map both onto FL/FR/RL/RR by name.
    fnames = [str(x) for x in t["foot_names"]] if "foot_names" in t else list(CANON)
    cnames = [str(x) for x in t["contact_names"]] if "contact_names" in t else list(CANON)

    def order(names):
        out = []
        for leg in CANON:
            hit = [i for i, nm in enumerate(names) if nm.upper().startswith(leg)]
            if len(hit) != 1:
                raise SystemExit(f"cannot map {names} onto {CANON}")
            out.append(hit[0])
        return out

    fo, co = order(fnames), order(cnames)
    con, conf, feet = con[:, co], conf[:, co], feet[:, fo]

    roll, pitch, yaw = quat_to_rpy_deg(quat)
    foot_b = quat_rotate_inv(quat[:, None, :], feet - pos[:, None, :])   # base frame

    print("=" * 78)
    print(f"{name}  {steps} control steps = {steps*dt:.2f} s at {1/dt:.1f} Hz, "
          f"cycle = {n_cycle} steps ({n_cycle*dt:.3f} s)"
          + (f",  TERMINATED at {term:.2f} s" if np.isfinite(term) else ",  no fall"))
    print(f"  body order feet={fnames} contact={cnames} -> reported as {list(CANON)}")

    # --- where did it go --------------------------------------------------
    d = pos[-1] - pos[0]
    print(f"  travelled  x {d[0]:+.3f} m   y {d[1]:+.3f} m   "
          f"|lateral|/|forward| {abs(d[1])/max(abs(d[0]),1e-9):.2f}   "
          f"yaw {yaw[-1]-yaw[0]:+.1f} deg")
    print(f"  body vel   vx {vb[:,0].mean():+.3f}  vy {vb[:,1].mean():+.3f} m/s   "
          f"yaw rate {np.degrees(wb[:,2]).mean():+.1f} deg/s   "
          f"roll {roll.mean():+.1f}+-{roll.std():.1f}  pitch {pitch.mean():+.1f} deg")

    # --- centre of pressure, in the base frame ----------------------------
    # The quantity that decides whether the robot rolls: where the ground pushes
    # relative to where the weight is.  cop_y > 0 means the support is LEFT of the
    # base origin, which rolls the base to the right.  Reported in mm.  A frame
    # with no foot loaded has no CoP; those frames are excluded, not zero-filled.
    tot = conf.sum(axis=1)
    loaded = tot > 1e-6
    cop_y = np.full(steps, np.nan)
    cop_y[loaded] = (conf[loaded] * foot_b[loaded, :, 1]).sum(axis=1) / tot[loaded]
    cop_x = np.full(steps, np.nan)
    cop_x[loaded] = (conf[loaded] * foot_b[loaded, :, 0]).sum(axis=1) / tot[loaded]
    print(f"  CoP (base) y {1000*np.nanmean(cop_y):+.1f} mm   x {1000*np.nanmean(cop_x):+.1f} mm"
          f"   ({100*loaded.mean():.0f}% of steps have a foot loaded)")

    # --- per cycle --------------------------------------------------------
    if cycle_table and n_cycle:
        nc = steps // n_cycle
        print(f"\n  per cycle ({nc} complete):")
        print("   cyc |    vx     vy   yawrate | roll_mean roll_max | cop_y  | "
              "stance frac  FL   FR   RL   RR | impulse L/R")
        for c in range(nc):
            s = slice(c * n_cycle, (c + 1) * n_cycle)
            st = con[s].mean(axis=0)
            imp = (conf[s] * dt).sum(axis=0)
            LR = (imp[0] + imp[2]), (imp[1] + imp[3])
            rmax = roll[s][np.abs(roll[s]).argmax()]
            print(f"   {c:3d} | {vb[s,0].mean():+.3f} {vb[s,1].mean():+.3f} "
                  f"{np.degrees(wb[s,2]).mean():+7.1f} |   {roll[s].mean():+6.2f}  "
                  f"{rmax:+7.2f} | {1000*np.nanmean(cop_y[s]):+5.1f} | "
                  f"            {st[0]:.2f} {st[1]:.2f} {st[2]:.2f} {st[3]:.2f} | "
                  f"{LR[0]:6.1f}/{LR[1]:6.1f} {100*(LR[0]-LR[1])/max(LR[0]+LR[1],1e-9):+5.1f}%")

    # --- left/right over the whole run -----------------------------------
    stance = con.mean(axis=0)
    imp = (conf * dt).sum(axis=0)
    peak = conf.max(axis=0)
    print(f"\n  whole run   stance frac " + " ".join(f"{l} {v:.3f}" for l, v in zip(CANON, stance)))
    print(f"              impulse Ns  " + " ".join(f"{l} {v:6.1f}" for l, v in zip(CANON, imp)))
    print(f"              peak N      " + " ".join(f"{l} {v:6.1f}" for l, v in zip(CANON, peak)))
    Ls, Rs = stance[0] + stance[2], stance[1] + stance[3]
    Li, Ri = imp[0] + imp[2], imp[1] + imp[3]
    print(f"              left-right  stance {Ls-Rs:+.3f}   impulse "
          f"{100*(Li-Ri)/max(Li+Ri,1e-9):+.1f}%  (left {Li:.1f} vs right {Ri:.1f} Ns)")

    # mean foot placement in the base frame: is one side planted further out?
    print("              mean foot position in base frame (x fwd, y left, z up):")
    for i, l in enumerate(CANON):
        m = foot_b[:, i].mean(axis=0)
        print(f"                {l}  x {m[0]:+.3f}  y {m[1]:+.3f}  z {m[2]:+.3f} m")
    yb = foot_b[:, :, 1].mean(axis=0)
    print(f"                lateral stance width: front {abs(yb[0]-yb[1]):.3f} m, "
          f"rear {abs(yb[2]-yb[3]):.3f} m; midline offset front {0.5*(yb[0]+yb[1]):+.4f} m, "
          f"rear {0.5*(yb[2]+yb[3]):+.4f} m")

    # --- the collapse ------------------------------------------------------
    if np.isfinite(term):
        k = steps - 1
        w = slice(max(0, k - int(0.4 / dt)), steps)
        print(f"\n  last {0.4:.1f} s before the fall:")
        print("     t      z   roll  pitch    yaw |    vx     vy | FL_N  FR_N  RL_N  RR_N")
        for i in range(w.start, steps, max(1, (steps - w.start) // 12)):
            print(f"   {i*dt:5.2f} {pos[i,2]:6.3f} {roll[i]:+6.1f} {pitch[i]:+6.1f} "
                  f"{yaw[i]:+6.1f} | {vb[i,0]:+.3f} {vb[i,1]:+.3f} | "
                  + " ".join(f"{conf[i,j]:5.0f}" for j in range(4)))
        # which foot stopped carrying load first, counted over the final second
        last = slice(max(0, steps - int(1.0 / dt)), steps)
        print("   load in the final second (Ns): "
              + " ".join(f"{l} {v:.1f}" for l, v in zip(CANON, (conf[last] * dt).sum(axis=0))))
    return 0


def ab_report(d: Path) -> int:
    """Survival-time distribution over the per-cycle runs in ``d``."""
    rows = []
    for f in sorted(d.glob("*.csv")):
        for r in csv.DictReader(f.open()):
            rows.append(r)
    if not rows:
        raise SystemExit(f"no result CSVs in {d}")

    def num(r, k):
        v = r.get(k, "")
        try:
            return float(v)
        except (TypeError, ValueError):
            return float("nan")

    med = [r for r in rows if r["clip"].endswith("_med")]
    raw = [r for r in rows if not r["clip"].endswith("_med")]

    print(f"Extraction A/B over {len(rows)} runs in {d}")
    print("A run with no terminated_s did not fall within the requested cycles.\n")
    print(f"  {'clip':12s} {'survived s':>10s} {'stride Hz':>10s} {'vx m/s':>8s} "
          f"{'handover':>9s} {'start':>6s} {'verdict':>8s}")
    for r in med + raw:
        t = num(r, "terminated_s")
        print(f"  {r['clip']:12s} {('%10.2f' % t) if np.isfinite(t) else '   no fall'} "
              f"{num(r,'stride_hz'):10.2f} {num(r,'vx_mean'):8.3f} "
              f"{num(r,'handover_speed_mps'):9.3f} {int(num(r,'start_frame')):6d} "
              f"{r.get('verdict',''):>8s}")

    surv = np.array([num(r, "terminated_s") for r in raw])
    fell = surv[np.isfinite(surv)]
    print(f"\n  raw cycles: {len(fell)}/{len(raw)} fell")
    if len(fell):
        print(f"    survival  min {fell.min():.2f}  median {np.median(fell):.2f}  "
              f"max {fell.max():.2f}  mean {fell.mean():.2f} +- {fell.std():.2f} s")
    if med:
        m = num(med[0], "terminated_s")
        ms = f"{m:.2f} s" if np.isfinite(m) else "no fall"
        print(f"    median clip (the control): {ms}")
        if len(fell) and np.isfinite(m):
            better = int((fell > m).sum())
            print(f"    raw cycles that outlasted the median clip: {better}/{len(fell)}")
            print("    -> the averaging is not what is killing the trot"
                  if better <= len(fell) / 2 else
                  "    -> raw cycles do better; the averaging removed something that mattered")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clips", action="store_true", help="clip symmetry, no simulator")
    ap.add_argument("--rate", choices=("hi", "lo"), default="hi")
    ap.add_argument("--trace", nargs="*", default=None, help="trace npz files to report")
    ap.add_argument("--ab", default=None,
                    help="directory of per-cycle result CSVs from run_raw_cycle_ab.sh")
    a = ap.parse_args()
    rc = 0
    if a.ab:
        return ab_report(Path(a.ab))
    if a.clips or a.trace is None:
        rc |= clip_symmetry(a.rate)
    for p in a.trace or []:
        rc |= trace_report(Path(p))
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
