#!/usr/bin/env python3
"""Rear-foot clearance from ACHIEVED angles, real robot and sim, one pipeline.

    scripts/isaac_docker_run.sh scripts/check_achieved_clearance.py --headless --device cpu

`q_des` could not settle it: the commanded trajectory does not describe a rigid trunk on
flat ground (see outputs/commanded_angles.md), so a clearance computed from it is a
clearance the robot was never in.  The archive also carries `q` -- what the real robot's
joints actually did -- and the replay traces carry the sim's `q`.  Both go through the
same path here: hold the articulation in the air, drive it to those angles, read
`body_pos_w`.  No hand-written kinematics, one reference, two robots.

Clearance is measured against the plane the STANCE feet define at that frame, not a level
body, because legs of different extension tilt the trunk and a level reference credits the
rear legs with clearance the tilt has already spent.

The consistency of that fit is itself the check: if the angles describe a real stance, the
implied trunk tilt is steady and no swing foot comes out below the plane.  On `q_des` it
was not and they did.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

_SIM_APP = None


def stance_plane_clearance(fpos, stance):
    """(clearance per leg per frame, implied tilt deg) from body-frame foot xyz."""
    n = len(fpos)
    clear = np.full((n, 4), np.nan)
    tilt = np.full(n, np.nan)
    for i in range(n):
        st = stance[i]
        if st.sum() < 3:
            continue
        P = fpos[i][st]
        A = np.column_stack([np.ones(st.sum()), P[:, 0], P[:, 1]])
        coef, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
        tilt[i] = np.degrees(np.arctan(-coef[1]))
        for j in range(4):
            if not st[j]:
                f = fpos[i][j]
                clear[i, j] = f[2] - (coef[0] + coef[1] * f[0] + coef[2] * f[1])
    return clear, tilt


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clip", default="WALK")
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo")
    ap.add_argument("--trace", default="outputs/traces/height/h_walk_a0.npz")
    ap.add_argument("--contact-threshold-n", type=float, default=30.0)
    ap.add_argument("--max-sim-frames", type=int, default=1200)
    ap.add_argument("--out", default="outputs/achieved_clearance.json")
    try:
        from isaaclab.app import AppLauncher
        AppLauncher.add_app_launcher_args(ap)
        ap.set_defaults(device="cpu")
    except Exception:
        ap.add_argument("--headless", action="store_true")
        ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    from isaaclab.app import AppLauncher
    global _SIM_APP
    _SIM_APP = AppLauncher(args).app

    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sim import SimulationCfg, SimulationContext

    from sim import isaac_cfg as IC
    from sim.replay import quat_rotate_inv, snap
    from verify_skill_replay import load_clip

    ucfg = IC.load()
    sys.path.insert(0, str(Path(ucfg.source).parents[3]))
    from legged_gym.envs.base.legged_robot_config import UNITREE_GO2_CFG

    sim = SimulationContext(SimulationCfg(dt=float(ucfg.sim_dt), device=args.device))
    sim_utils.DomeLightCfg(intensity=1000.0).func("/World/light", sim_utils.DomeLightCfg(intensity=1000.0))
    robot = Articulation(UNITREE_GO2_CFG.replace(prim_path="/World/Robot"))
    sim.reset()
    foot_ids, foot_names = robot.find_bodies(".*_foot")
    fcol = {n.split("_")[0]: i for i, n in enumerate(foot_names)}
    jname = {n: i for i, n in enumerate(robot.joint_names)}

    clip = load_clip(args.clip, args.rate)
    legs, joints = clip["leg_order"], clip["joint_order"]
    idx_t = torch.as_tensor([jname[f"{l}_{j}_joint"] for l in legs for j in joints],
                            device=sim.device, dtype=torch.long)
    air = robot.data.default_root_state.clone(); air[:, 2] = 1.5

    def fk(Q):
        out = np.zeros((len(Q), 4, 3))
        for i, qi in enumerate(Q):
            qj = robot.data.default_joint_pos.clone()
            qj[:, idx_t] = torch.as_tensor(np.asarray(qi, np.float32), device=sim.device,
                                           dtype=torch.float32)
            robot.write_root_state_to_sim(air)
            robot.write_joint_state_to_sim(qj, torch.zeros_like(qj))
            robot.write_data_to_sim(); sim.step(); robot.update(float(ucfg.sim_dt))
            bq = snap(robot.data.root_quat_w[0]); bpos = snap(robot.data.root_pos_w[0])
            bp = snap(robot.data.body_pos_w[0])
            out[i] = [quat_rotate_inv(bq[None, :], (bp[foot_ids[fcol[l]]] - bpos)[None, :])[0]
                      for l in legs]
        return out

    results = {}
    contact = np.asarray(clip["contact"], dtype=bool)

    sources = [("real robot, q_des (for contrast)", np.asarray(clip["q_des"]), contact),
               ("real robot, q ACHIEVED", np.asarray(clip["q"]), contact)]
    z = np.load(args.trace, allow_pickle=False)
    qs = np.asarray(z["q"])[: args.max_sim_frames]
    cf = np.asarray(z["contact_f"])[: args.max_sim_frames]
    cn = [str(x).split("_")[0] for x in z["contact_names"]]
    ccol = [cn.index(l) for l in legs]
    st_sim = cf[:, ccol] > args.contact_threshold_n           # MEASURED contact, by name
    sources.append((f"sim replay, q ACHIEVED ({Path(args.trace).name})", qs, st_sim))

    for label, Q, stance in sources:
        fpos = fk(Q)
        clear, tilt = stance_plane_clearance(fpos, stance)
        tt = tilt[np.isfinite(tilt)]
        print("=" * 78)
        print(f"{label}   [{len(Q)} frames, {stance.mean()*100:.0f}% of leg-frames in stance]")
        print(f"   {'leg':4s} {'median clearance':>17s} {'p90':>8s} {'% below the plane':>19s}")
        med = {}
        for j, l in enumerate(legs):
            c = clear[:, j][np.isfinite(clear[:, j])]
            if not c.size:
                print(f"   {l:4s} {'n/a':>17s}"); continue
            med[l] = float(np.median(c) * 1000)
            print(f"   {l:4s} {np.median(c)*1000:17.1f} {np.percentile(c,90)*1000:8.1f} "
                  f"{100*(c<0).mean():18.1f}%")
        spread = float(tt.max() - tt.min())
        verdict = ("CONSISTENT" if spread < 4 else
                   "INCONSISTENT -- these angles do not describe one rigid trunk on flat ground")
        print(f"   implied trunk tilt: median {np.median(tt):+.2f} deg, "
              f"range {tt.min():+.2f} to {tt.max():+.2f} (spread {spread:.2f})  {verdict}")
        results[label] = {"median_mm": med, "tilt_median": float(np.median(tt)),
                          "tilt_range": [float(tt.min()), float(tt.max())]}
    Path(args.out).write_text(json.dumps(results, indent=2))
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        if _SIM_APP is not None:
            exc = sys.exc_info()[1]
            if exc is not None and not isinstance(exc, SystemExit):
                import traceback
                traceback.print_exc(); sys.stderr.flush(); sys.stdout.flush()
            _SIM_APP.close()
