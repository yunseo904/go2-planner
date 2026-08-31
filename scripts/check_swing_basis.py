#!/usr/bin/env python3
"""Two questions the external comparison raised, measured on our own clips.

    scripts/isaac_docker_run.sh scripts/check_swing_basis.py --headless --device cpu

1. **Entry phase.**  Two different rules pick where in the cycle a replay starts.
   ``verify_skill_replay.level_start`` picks the most coplanar pose (kinematic);
   ``run_calibration_grid.py`` does not pick at all -- rep *r* enters at frame
   ``round(r * n / reps)``, a uniform sample.  TURN's flat control passed 2 of its
   5 uniform phases and failed 3.  The question is whether the coplanarity rule,
   which the planner and the single-clip harness both use, lands on a phase that
   works -- and whether the criterion means anything on a turn in place, whose
   contact pattern is not a translating gait's.

2. **Swing height, three bases.**  ``quadruped_pympc`` defines step height as
   ``0.2 x hip_height = 0.056 m`` measured **against that leg's own lift-off z**.
   We measured against the **stance plane**.  Those are different quantities and
   the ``footRaiseHeight`` 0.08 m argument may be nothing but the difference.  All
   three are reported side by side, per leg, from Isaac's own kinematics with the
   robot held in the air -- no floor, no contact, no replay dynamics, so the number
   is the RECORDING's and not a body attitude's (``foot_clearance_check.md`` 3).

   * ``takeoff``      apex z minus the foot's z at the frame it last left stance
   * ``stance-plane`` apex z minus the mean z of the feet in stance at the apex
   * ``own-min``      apex z minus that leg's own lowest z in the cycle

Nothing here touches the archive and nothing is written to it.
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

#: The uniform entry frames run_calibration_grid.py uses at --reps 5, and the two
#: that TURN's flat control passed (outputs/turn_probes.md 3).
TURN_PASSING_REPS = (1, 4)


def swing_blocks(contact_leg: np.ndarray) -> list:
    """``[(takeoff_frame, [swing frames...])]`` for one leg's contact channel.

    Cyclic: a swing that straddles the wrap is one block.  ``takeoff_frame`` is the
    last stance frame before the block, which is what ``quadruped_pympc`` measures
    its step height against.
    """
    n = len(contact_leg)
    sw = ~np.asarray(contact_leg, dtype=bool)
    if sw.all() or not sw.any():
        return []
    start = int(np.flatnonzero(sw & ~np.roll(sw, 1))[0])
    order = [(start + k) % n for k in range(n)]
    blocks, cur = [], []
    for i in order:
        if sw[i]:
            cur.append(i)
        elif cur:
            blocks.append(cur)
            cur = []
    if cur:
        blocks.append(cur)
    return [(int((b[0] - 1) % n), b) for b in blocks]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clips", nargs="*", default=["WALK", "TROT", "TURN", "RUN"])
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo")
    ap.add_argument("--lift-thresh-mm", type=float, default=3.0,
                    help="kinematic stance band, the same one check_foot_kinematics.py uses")
    ap.add_argument("--reps", type=int, default=5,
                    help="the grid's rep count, to reproduce its uniform entry frames")
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
    from sim.replay import foot_body_ids, ground_material_cfg, snap
    from verify_skill_replay import load_clip, level_start, quiescent_start

    ucfg = IC.load()
    sys.path.insert(0, str(Path(ucfg.source).parents[3]))
    from legged_gym.envs.base.legged_robot_config import UNITREE_GO2_CFG

    phys_dt = float(ucfg.sim_dt)
    sim = SimulationContext(SimulationCfg(dt=phys_dt, device=args.device))
    ground = sim_utils.GroundPlaneCfg(physics_material=ground_material_cfg(sim_utils))
    ground.func("/World/ground", ground)
    robot = Articulation(UNITREE_GO2_CFG.replace(prim_path="/World/Robot"))
    sim.reset()
    foot_ids, foot_names = foot_body_ids(robot)
    hip_ids, hip_names = robot.find_bodies(".*_hip")
    col_for_leg = {nm.split("_")[0]: c for c, nm in enumerate(foot_names)}
    idx_by_name = {n: i for i, n in enumerate(robot.joint_names)}

    print("=" * 78)
    print("1. ENTRY PHASE -- which frame each rule starts the cycle at")
    print("=" * 78)
    for cname in args.clips:
        clip = load_clip(cname, args.rate)
        n = len(clip["q_des"])
        want = [f"{l}_{j}_joint" for l in clip["leg_order"] for j in clip["joint_order"]]
        idx_t = torch.as_tensor([idx_by_name[w] for w in want], device=sim.device,
                                dtype=torch.long)
        k_lv, lv = level_start(clip, robot, sim, idx_t, phys_dt)
        k_qs, qs = quiescent_start(clip)
        uni = [int(round(r * n / max(args.reps, 1))) % n for r in range(args.reps)]
        print(f"\n   {cname}: {n} frames/cycle")
        print(f"     level_start (coplanarity, used by the planner and verify_skill_replay)"
              f"  -> frame {k_lv}, foot spread {lv['spread_at_start']*1000:.1f} mm "
              f"(worst in cycle {lv['spread_worst']*1000:.1f} mm, "
              f"{lv['n_level_frames']} frames under 10 mm)")
        print(f"     quiescent_start (contact channel)                                   "
              f"  -> frame {k_qs}, {qs['feet_down']}/4 down, all_stance={qs['all_stance']}")
        print(f"     run_calibration_grid --reps {args.reps} (uniform, no criterion)      "
              f"  -> frames {uni}")
        if cname == "TURN":
            passing = [uni[r] for r in TURN_PASSING_REPS]
            hit = k_lv in passing
            print(f"     TURN's flat control passed reps {list(TURN_PASSING_REPS)} "
                  f"= frames {passing}, failed the other three (turn_probes.md 3)")
            print(f"     >>> level_start picks frame {k_lv}: "
                  f"{'INSIDE' if hit else 'NOT ONE OF'} the passing set")

    print("\n" + "=" * 78)
    print("2. SWING HEIGHT -- three bases, robot held in the air, clip q_des only")
    print("=" * 78)
    root = robot.data.default_root_state.clone()
    root[:, 2] = 1.5
    for cname in args.clips:
        clip = load_clip(cname, args.rate)
        legs = clip["leg_order"]
        want = [f"{l}_{j}_joint" for l in legs for j in clip["joint_order"]]
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
            robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)
            b = snap(robot.data.body_pos_w[0])
            fz[i] = b[foot_ids, 2]
            hz[i] = b[hip_ids, 2]
        contact = np.asarray(clip["contact"], dtype=bool)      # clip leg order
        # SECOND FRAME OF REFERENCE, per CLAUDE.md 6.5.  The takeoff basis is only as
        # good as the frame it calls lift-off, and that frame comes from the contact
        # CHANNEL.  The kinematic gate is check_foot_kinematics.py's corrected one --
        # a foot is in stance while its hip-to-foot drop is within a threshold of the
        # lowest foot THAT FRAME -- and the two are required to agree before the
        # number is reported.
        drop = hz - fz
        low = drop.max(axis=1, keepdims=True)
        sw_kin = (low - drop) > args.lift_thresh_mm / 1000.0     # articulation columns
        kin_by_leg = np.stack([~sw_kin[:, col_for_leg[l]] for l in legs], axis=1)
        agree = float(np.mean(kin_by_leg == contact))
        hip_h = float(np.mean(hz)) - float(np.min(fz))
        print(f"\n   {cname}: {n} frames, hip height (mean hip z - lowest foot z) "
              f"{hip_h*1000:.0f} mm; 0.2 x that = {0.2*hip_h*1000:.0f} mm")
        print(f"   contact channel vs kinematic stance: agree on {100*agree:.1f}% "
              f"of leg-frames (clip swing {100*(~contact).mean():.1f}%, "
              f"kinematic swing {100*(~kin_by_leg).mean():.1f}%)")
        print(f"   {'leg':4s} {'blk':>3s} {'takeoff':>9s} {'takeoff/kin':>12s} "
              f"{'stance-plane':>13s} {'own-min':>8s}   (mm; takeoff = quadruped_pympc's basis)")
        for j, leg in enumerate(legs):
            c = col_for_leg[leg]
            rows = []
            for gate, tag in ((contact[:, j], "chan"), (kin_by_leg[:, j], "kin")):
                blocks = swing_blocks(gate)
                if not blocks:
                    rows.append((None, None, None))
                    continue
                h_take, h_plane = [], []
                for k0, blk in blocks:
                    apex = blk[int(np.argmax(fz[blk, c]))]
                    h_take.append(fz[apex, c] - fz[k0, c])
                    st = contact[apex]
                    st_cols = [col_for_leg[legs[m]] for m in range(4) if st[m]]
                    h_plane.append(fz[apex, c] - float(np.mean(fz[apex, st_cols]))
                                   if st_cols else np.nan)
                rows.append((len(blocks), float(np.mean(h_take)), float(np.nanmean(h_plane))))
            own = fz[:, c].max() - fz[:, c].min()
            nb, take, plane = rows[0]
            _nb2, take_k, _p2 = rows[1]
            f = lambda v: "  --   " if v is None else f"{v*1000:7.1f}"
            print(f"   {leg:4s} {(nb if nb else 0):3d} {f(take):>9s} {f(take_k):>12s} "
                  f"{f(plane):>13s} {own*1000:8.1f}")
        # THRESHOLD-FREE ALIGNMENT.  The takeoff basis stands or falls on which frame is
        # called lift-off.  If the contact channel's lift-off frame is not the frame the
        # foot is actually lowest at, the basis is measuring the channel's phase error
        # and not the gait.  Both frames are reported so the reader can see the offset
        # rather than take the height on trust.
        # SECOND REFERENCE, threshold-free.  The whole takeoff basis rests on which frame
        # is called lift-off, and the cycle's z-MINIMUM is not it: on a recorded gait the
        # foot is lowest at MID-STANCE, when the leg is most extended, not when it leaves
        # the ground.  The geometric lift-off is the local minimum immediately BEFORE the
        # apex -- where the foot stops descending and starts to rise -- and that needs no
        # threshold and no contact channel.  If it agrees with the channel's takeoff the
        # height is a property of the recording; if it does not, the height is a property
        # of the channel's phase and must not be quoted.
        print(f"   {'leg':4s} {'apex':>5s} {'chan takeoff':>12s} {'rise start':>11s} "
              f"{'offset':>7s} {'h chan':>8s} {'h rise':>8s} {'cycle z-min':>12s}")
        for j, leg in enumerate(legs):
            c = col_for_leg[leg]
            blocks = swing_blocks(contact[:, j])
            kmin = int(np.argmin(fz[:, c]))
            if not blocks:
                print(f"   {leg:4s}  -- no swing block in the contact channel")
                continue
            k0, blk = blocks[0]
            apex = blk[int(np.argmax(fz[blk, c]))]
            # walk backwards from the apex while the foot is still descending
            r = apex
            while fz[(r - 1) % n, c] < fz[r, c]:
                r = (r - 1) % n
                if r == apex:
                    break
            off = (k0 - r) % n
            off = off if off <= n // 2 else off - n
            print(f"   {leg:4s} {apex:5d} {k0:12d} {r:11d} {off:+7d} "
                  f"{(fz[apex, c]-fz[k0, c])*1000:7.1f}  {(fz[apex, c]-fz[r, c])*1000:7.1f}  "
                  f"{kmin:12d}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        if _SIM_APP is not None:
            _SIM_APP.close()
