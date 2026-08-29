#!/usr/bin/env python3
"""Is WALK's rear swing really 4 mm?  Three checks, one script.

    scripts/isaac_docker_run.sh scripts/check_foot_kinematics.py --headless --device cpu

1. **Leg order.**  ``robot.find_bodies(".*_foot")`` returns ARTICULATION order, which is
   not the clip's FL/FR/RL/RR and not the log's FR/FL/RR/RL.  Every column label in the
   step-probe analysis was an assumption; here each foot's body-frame x says outright
   which are front and which are rear.

2. **The recording, not the replay.**  The robot is held off the ground and driven
   through the clip's own ``q_des`` frame by frame, and the foot heights are read from
   Isaac's own kinematics (``body_pos_w``) rather than from a hand-written forward
   model.  That answers "does the RECORDING command a 4 mm rear lift?" without a floor,
   without contact, and without the replay's dynamics in the way.  No hand-rolled FK is
   involved, so no front/rear formula asymmetry can hide in it.

3. **Swing, defined two ways.**  The clip's own contact channel, and a purely kinematic
   one (foot above its own per-leg minimum by more than a threshold).  If the two
   disagree, the contact channel's leg columns are suspect -- measuring an apex over a
   window when the foot is actually in stance is exactly how you get 4 mm.

Nothing here touches the archive, and the robot never contacts the ground.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

_SIM_APP = None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clips", nargs="*", default=["WALK", "TROT"])
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo")
    ap.add_argument("--lift-thresh-mm", type=float, default=3.0)
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

    # ---------------------------------------------------------------- 1. leg order
    foot_ids, foot_names = robot.find_bodies(".*_foot")
    hip_ids, hip_names = robot.find_bodies(".*_hip")
    root = robot.data.default_root_state.clone()
    root[:, 2] = 1.5                      # hold it in the air; no ground exists anyway
    robot.write_root_state_to_sim(root)
    q0 = robot.data.default_joint_pos.clone()
    robot.write_joint_state_to_sim(q0, torch.zeros_like(q0))
    robot.write_data_to_sim(); sim.step(); robot.update(float(ucfg.sim_dt))

    bp = snap(robot.data.body_pos_w[0])
    bq = snap(robot.data.root_quat_w[0])
    bpos = snap(robot.data.root_pos_w[0])
    print("=" * 78)
    print("1. LEG ORDER -- what find_bodies actually returns, and where those feet are")
    print(f"   {'col':>3s} {'body name':12s} {'body-frame x':>13s} {'y':>8s}   front/rear, left/right")
    order_ok = True
    for c, (bid, nm) in enumerate(zip(foot_ids, foot_names)):
        r = quat_rotate_inv(bq[None, :], (bp[bid] - bpos)[None, :])[0]
        fr = "FRONT" if r[0] > 0 else "REAR "
        lr = "LEFT " if r[1] > 0 else "RIGHT"
        label = ("FL" if r[0] > 0 and r[1] > 0 else "FR" if r[0] > 0 else
                 "RL" if r[1] > 0 else "RR")
        assumed = ["FL", "FR", "RL", "RR"][c]
        if label != assumed:
            order_ok = False
        print(f"   {c:3d} {nm:12s} {r[0]:+13.3f} {r[1]:+8.3f}   {fr} {lr}  -> {label}"
              + ("" if label == assumed else f"   [column {c} was assumed {assumed}]"))
    print(f"\n   positional labelling FL,FR,RL,RR is {'CORRECT' if order_ok else 'WRONG'} "
          f"for this articulation")

    # ---------------------------------------------------------------- 2. the recording
    idx_by_name = {n: i for i, n in enumerate(robot.joint_names)}
    print("\n" + "=" * 78)
    print("2. THE RECORDING ITSELF -- clip q_des driven through Isaac's own kinematics,")
    print("   robot held in the air, no floor, no contact, no replay dynamics")
    for cname in args.clips:
        clip = load_clip(cname, args.rate)
        legs, joints = clip["leg_order"], clip["joint_order"]
        want = [f"{l}_{j}_joint" for l in legs for j in joints]
        idx_t = torch.as_tensor([idx_by_name[w] for w in want], device=sim.device,
                                dtype=torch.long)
        q = np.asarray(clip["q_des"], dtype=np.float32)
        n = len(q)
        fz = np.zeros((n, 4))
        hz = np.zeros((n, 4))
        for i in range(n):
            qj = robot.data.default_joint_pos.clone()
            qj[:, idx_t] = torch.as_tensor(q[i], device=sim.device, dtype=torch.float32)
            robot.write_joint_state_to_sim(qj, torch.zeros_like(qj))
            robot.write_root_state_to_sim(root)
            robot.write_data_to_sim(); sim.step(); robot.update(float(ucfg.sim_dt))
            b = snap(robot.data.body_pos_w[0])
            fz[i] = b[foot_ids, 2]
            hz[i] = b[hip_ids, 2]
        # foot height BELOW the hip: the only frame-independent thing while airborne
        drop = hz - fz                      # (n, 4), per articulation column
        lift = drop.max(axis=0) - drop      # how far the foot is retracted from its lowest
        contact = np.asarray(clip["contact"], dtype=bool)      # clip leg order
        col_for_leg = {}
        for c, nm in enumerate(foot_names):
            col_for_leg[nm.split("_")[0]] = c
        print(f"\n   {cname}: {n} frames")
        print(f"   {'leg':4s} {'max lift':>9s} {'hip->foot drop':>15s} {'at apex':>9s}"
              f"   (mm)  -- lift is retraction from this leg's own lowest")
        for j, leg in enumerate(legs):
            c = col_for_leg[leg]
            print(f"   {leg:4s} {lift[:, c].max()*1000:9.1f} {drop[:, c].max()*1000:15.1f} "
                  f"{drop[:, c].min()*1000:9.1f}")
        # If the body were level at the nominal stand height, what ground clearance would
        # each foot have at its apex?  drop is hip-to-foot, and the hips are fixed to the
        # base, so this isolates the LEG's contribution from the body's attitude.
        print(f"   deepest drop overall {drop.max()*1000:.1f} mm; if the base held every hip "
              f"at that height, apex ground clearance would be:")
        print("        " + "  ".join(
            f"{leg} {(drop.max() - drop[:, col_for_leg[leg]].min())*1000:5.1f}" for leg in legs))
        # Kinematic swing, done properly: PER FRAME, across legs.  A foot is in stance
        # when it is among the lowest -- i.e. its hip-to-foot drop is within a threshold
        # of the largest drop at that instant.  The earlier version compared each leg
        # against ITS OWN lowest over the whole clip, which flags most of stance as
        # swing (the hip-to-foot distance changes as the body passes over a planted
        # foot) and produced a spurious 51% disagreement.  Same failure shape as the
        # clearance error: almost the quantity being reasoned about.
        low = drop.max(axis=1, keepdims=True)          # the lowest foot this frame
        sw_kin_all = (low - drop) > args.lift_thresh_mm / 1000.0
        agree = np.mean([(~contact[:, j]) == sw_kin_all[:, col_for_leg[l]]
                         for j, l in enumerate(legs)])
        print(f"   contact channel vs per-frame kinematic swing: agree on "
              f"{100*agree:.1f}% of leg-frames")
        print(f"   clip says swing {100*(~contact).mean():.1f}% of leg-frames, "
              f"kinematics say {100*sw_kin_all.mean():.1f}%")
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
