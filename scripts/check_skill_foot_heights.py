#!/usr/bin/env python3
"""Per-skill swing-foot height of the REAL robot, from achieved angles.

    NAME=go2_fh scripts/isaac_docker_run.sh scripts/check_skill_foot_heights.py \
        --headless --device cpu --clips WALK TROT RUN TURN JUMP

Answers one question: in the recording, do the gaits lift their feet in the order a
gait library would expect (slow walk lowest, faster gaits higher)?  Any target we pick
for --swing-lift has to keep whatever order the real robot had, or the library stops
describing one robot.

Two references, per CLAUDE.md 6.5, and they must agree on the ORDER:

  chord   apex above the straight line joining that bout's liftoff and touchdown feet.
          Trunk attitude cannot inflate it.  This is the quantity --swing-lift targets.
  plane   height above the plane the stance feet define at the same frame, median over
          the swing.  Independent of the bout endpoints.

Both from `q`, the achieved angles -- `q_des` is not a pose the robot holds
(outputs/commanded_angles.md).  No hand-written kinematics: the articulation is held in
the air, driven to the angles, and `body_pos_w` is read.
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


def bouts(mask):
    d = np.diff(np.concatenate([[False], mask, [False]]).astype(np.int8))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clips", nargs="+", default=["WALK", "TROT", "RUN", "TURN", "JUMP"])
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo")
    ap.add_argument("--out", default="outputs/skill_foot_heights.json")
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
    from check_achieved_clearance import stance_plane_clearance

    ucfg = IC.load()
    sys.path.insert(0, str(Path(ucfg.source).parents[3]))
    from legged_gym.envs.base.legged_robot_config import UNITREE_GO2_CFG

    sim = SimulationContext(SimulationCfg(dt=float(ucfg.sim_dt), device=args.device))
    sim_utils.DomeLightCfg(intensity=1000.0).func("/World/light",
                                                  sim_utils.DomeLightCfg(intensity=1000.0))
    robot = Articulation(UNITREE_GO2_CFG.replace(prim_path="/World/Robot"))
    sim.reset()
    foot_ids, foot_names = robot.find_bodies(".*_foot")
    fcol = {n.split("_")[0]: i for i, n in enumerate(foot_names)}
    jname = {n: i for i, n in enumerate(robot.joint_names)}
    air = robot.data.default_root_state.clone(); air[:, 2] = 1.5

    out = {}
    for name in args.clips:
        try:
            clip = load_clip(name, args.rate)
        except Exception as e:                       # a clip the archive does not carry
            print(f"[fh] {name}: skipped ({e})")
            continue
        legs, joints = clip["leg_order"], clip["joint_order"]
        idx_t = torch.as_tensor([jname[f"{l}_{j}_joint"] for l in legs for j in joints],
                                device=sim.device, dtype=torch.long)
        Q = np.asarray(clip["q"], dtype=np.float32)
        contact = np.asarray(clip["contact"], dtype=bool)
        fpos = np.zeros((len(Q), 4, 3))
        for i, qi in enumerate(Q):
            qj = robot.data.default_joint_pos.clone()
            qj[:, idx_t] = torch.as_tensor(qi, device=sim.device, dtype=torch.float32)
            robot.write_root_state_to_sim(air)
            robot.write_joint_state_to_sim(qj, torch.zeros_like(qj))
            robot.write_data_to_sim(); sim.step(); robot.update(float(ucfg.sim_dt))
            bq = snap(robot.data.root_quat_w[0]); bpos = snap(robot.data.root_pos_w[0])
            bp = snap(robot.data.body_pos_w[0])
            fpos[i] = [quat_rotate_inv(bq[None, :], (bp[foot_ids[fcol[l]]] - bpos)[None, :])[0]
                       for l in legs]

        chord, plane = {}, {}
        clear, _tilt = stance_plane_clearance(fpos, contact)
        for j, leg in enumerate(legs):
            ap_ = []
            for a, b in bouts(~contact[:, j]):
                if b - a < 3:
                    continue
                p0, p1 = fpos[a, j], fpos[b - 1, j]
                t = np.linspace(0.0, 1.0, b - a)
                cz = p0[2] + (p1[2] - p0[2]) * t
                ap_.append(float((fpos[a:b, j, 2] - cz).max()))
            chord[leg] = round(float(np.median(ap_)) * 1000, 1) if ap_ else None
            c = clear[:, j][np.isfinite(clear[:, j])]
            plane[leg] = round(float(np.percentile(c, 90)) * 1000, 1) if c.size else None
        vals = [v for v in chord.values() if v is not None]
        pv = [v for v in plane.values() if v is not None]
        out[name] = {"chord_apex_mm": chord, "plane_p90_mm": plane,
                     "chord_median_mm": round(float(np.median(vals)), 1) if vals else None,
                     "chord_max_mm": round(float(np.max(vals)), 1) if vals else None,
                     "plane_median_mm": round(float(np.median(pv)), 1) if pv else None,
                     "n_frames": int(len(Q)), "swing_frac": round(float((~contact).mean()), 3)}
        print(f"[fh] {name:5s} chord apex {chord}  median {out[name]['chord_median_mm']} mm | "
              f"plane p90 median {out[name]['plane_median_mm']} mm")

    Path(args.out).write_text(json.dumps(out, indent=2))
    print(f"\nwrote {args.out}")
    print("order by chord median: " + " < ".join(
        k for k, _ in sorted(((k, v["chord_median_mm"]) for k, v in out.items()
                              if v["chord_median_mm"] is not None), key=lambda x: x[1])))
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
