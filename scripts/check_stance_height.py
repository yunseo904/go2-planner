#!/usr/bin/env python3
"""How far the replayed body sits below the clip's own stance geometry, and what
constant joint offset would close it.

    scripts/isaac_docker_run.sh scripts/check_stance_height.py --headless --device cpu

Two measurements, both in the same reference so they can be subtracted:

1. **Commanded extension.**  The robot is held in the air and driven through the clip's
   ``q_des``; hip-to-foot drop per leg per frame is read from ``body_pos_w``.  This is
   what the recording asks the leg to be.

2. **Achieved extension.**  The robot is settled on the ground exactly as the replay
   settles it and then plays the same frames; the same drop is read again.  The
   difference is what gravity and the PD take out.

Then the correction.  Lengthening a leg by moving one joint also swings the foot
fore-aft, and moving the stance fore-aft is how the previous plant compensation killed a
third of the forward speed.  So the offset is solved as a 2x2: find the (thigh, calf)
pair that moves the foot straight DOWN by the deficit and leaves its x alone, using a
numerical Jacobian taken at the clip's own mean stance pose.  Nothing is assumed about
which joint does what.
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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clips", nargs="*", default=["WALK", "TROT", "TURN"])
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo")
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--cycles", type=int, default=6)
    ap.add_argument("--eps", type=float, default=0.02, help="rad, finite-difference step")
    ap.add_argument("--out", default="outputs/stance_height.json")
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
    from sim.replay import ground_material_cfg, quat_rotate_inv, set_robot_friction, snap
    from verify_skill_replay import load_clip

    ucfg = IC.load()
    sys.path.insert(0, str(Path(ucfg.source).parents[3]))
    from legged_gym.envs.base.legged_robot_config import UNITREE_GO2_CFG

    phys_dt = float(ucfg.sim_dt)
    decim = int(ucfg.decimation or 4)
    dt = decim * phys_dt
    sim = SimulationContext(SimulationCfg(dt=phys_dt, device=args.device))
    ground = sim_utils.GroundPlaneCfg(physics_material=ground_material_cfg(sim_utils))
    ground.func("/World/ground", ground)
    sim_utils.DomeLightCfg(intensity=2000.0).func("/World/light", sim_utils.DomeLightCfg(intensity=2000.0))
    robot = Articulation(UNITREE_GO2_CFG.replace(prim_path="/World/Robot"))
    sim.reset()
    if ucfg.ground_friction is not None:
        set_robot_friction(robot, ucfg.ground_friction)

    foot_ids, foot_names = robot.find_bodies(".*_foot")
    hip_ids, hip_names = robot.find_bodies(".*_hip")
    fcol = {n.split("_")[0]: i for i, n in enumerate(foot_names)}
    hcol = {n.split("_")[0]: i for i, n in enumerate(hip_names)}
    jname = {n: i for i, n in enumerate(robot.joint_names)}
    out = {}

    def foot_body_xyz(legs):
        bq = snap(robot.data.root_quat_w[0]); bpos = snap(robot.data.root_pos_w[0])
        bp = snap(robot.data.body_pos_w[0])
        return np.array([quat_rotate_inv(bq[None, :], (bp[foot_ids[fcol[l]]] - bpos)[None, :])[0]
                         for l in legs])

    def drops_now(legs):
        """hip-to-foot drop and foot body-frame x, per leg, right now."""
        bq = snap(robot.data.root_quat_w[0]); bpos = snap(robot.data.root_pos_w[0])
        bp = snap(robot.data.body_pos_w[0])
        d, fx = [], []
        for l in legs:
            fr = quat_rotate_inv(bq[None, :], (bp[foot_ids[fcol[l]]] - bpos)[None, :])[0]
            hr = quat_rotate_inv(bq[None, :], (bp[hip_ids[hcol[l]]] - bpos)[None, :])[0]
            d.append(hr[2] - fr[2]); fx.append(fr[0])
        return np.array(d), np.array(fx)

    for cname in args.clips:
        clip = load_clip(cname, args.rate)
        legs, joints = clip["leg_order"], clip["joint_order"]
        idx_t = torch.as_tensor([jname[f"{l}_{j}_joint"] for l in legs for j in joints],
                                device=sim.device, dtype=torch.long)
        q = np.asarray(clip["q_des"], dtype=np.float32)
        contact = np.asarray(clip["contact"], dtype=bool)
        n = len(q)

        # ---- 1. commanded, in the air -------------------------------------------
        air = robot.data.default_root_state.clone(); air[:, 2] = 1.5
        cmd = np.zeros((n, 4))
        fpos = np.zeros((n, 4, 3))
        for i in range(n):
            qj = robot.data.default_joint_pos.clone()
            qj[:, idx_t] = torch.as_tensor(q[i], device=sim.device, dtype=torch.float32)
            robot.write_root_state_to_sim(air)
            robot.write_joint_state_to_sim(qj, torch.zeros_like(qj))
            robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)
            cmd[i], _ = drops_now(legs)
            fpos[i] = foot_body_xyz(legs)

        # ---- 2. achieved, on the ground, settled the way the replay settles ------
        root = robot.data.default_root_state.clone()
        robot.write_root_state_to_sim(root)
        q_stand = robot.data.default_joint_pos.clone()
        q0 = q_stand.clone(); q0[:, idx_t] = torch.as_tensor(q[0], device=sim.device, dtype=torch.float32)

        def hold(target, k):
            for _ in range(k):
                robot.set_joint_position_target(target)
                for _ in range(decim):
                    robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)

        ns = max(int(args.settle_s / dt), 1)
        robot.write_joint_state_to_sim(q_stand, torch.zeros_like(q_stand))
        hold(q_stand, ns)
        for k in range(1, ns + 1):
            hold(q_stand * (1 - k / ns) + q0 * (k / ns), 1)
        base_settle = float(snap(robot.data.root_pos_w[0])[2])

        ach = []
        base_h = []
        for i in range(args.cycles * n):
            tgt = robot.data.default_joint_pos.clone()
            tgt[:, idx_t] = torch.as_tensor(q[i % n], device=sim.device, dtype=torch.float32)
            robot.set_joint_position_target(tgt)
            for _ in range(decim):
                robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)
            d, _ = drops_now(legs)
            ach.append(d)
            base_h.append(float(snap(robot.data.root_pos_w[0])[2]))
        ach = np.asarray(ach)
        frames = np.arange(len(ach)) % n

        # compare like with like: only the frames the clip has that leg in STANCE
        print("=" * 78)
        print(f"{cname}: base at handover {base_settle*1000:.1f} mm, mean over "
              f"{args.cycles} cycles {np.mean(base_h)*1000:.1f} mm")
        print(f"   {'leg':4s} {'commanded stance drop':>22s} {'achieved':>10s} {'deficit':>9s}   (mm)")
        deficit = np.zeros(4)
        for j, l in enumerate(legs):
            st = contact[frames, j]
            c = cmd[frames[st], j].mean() * 1000
            a = ach[st, j].mean() * 1000
            deficit[j] = c - a
            print(f"   {l:4s} {c:22.1f} {a:10.1f} {deficit[j]:9.1f}")
        print(f"   mean deficit {deficit.mean():.1f} mm")

        # ---- 2b. what clearance the RECORDING itself gives, per frame ------------
        # The earlier version took (deepest drop over all legs and all frames) minus
        # (this leg's drop at its apex), which assumes the body stands at the height set
        # by the most-extended leg at a moment when it may not even be in stance.  Per
        # frame the body height is set by the STANCE legs of THAT frame, so that is the
        # reference a swing foot clears against.
        clear = np.full((n, 4), np.nan)
        for i in range(n):
            st = contact[i]
            if not st.any():
                continue
            ground = cmd[i, st].max()          # deepest stance leg holds the body up
            for j in range(4):
                if not st[j]:
                    clear[i, j] = ground - cmd[i, j]
        # Against the plane the STANCE FEET actually define, not a level body.  The real
        # trunk pitches to accommodate legs of different extension -- WALK's rear legs sit
        # 17 mm shorter than its front ones -- so a level-body reference credits the rear
        # legs with clearance the tilt has already spent.  Least squares z = a + b x + c y
        # through the stance feet; the swing foot's height above that plane is what it
        # actually clears.
        clear_p = np.full((n, 4), np.nan)
        tilt = np.full(n, np.nan)
        for i in range(n):
            st = contact[i]
            if st.sum() < 3:
                continue
            P = fpos[i][st]
            A = np.column_stack([np.ones(st.sum()), P[:, 0], P[:, 1]])
            coef, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
            tilt[i] = np.degrees(np.arctan(-coef[1]))
            for j in range(4):
                if not st[j]:
                    f = fpos[i][j]
                    clear_p[i, j] = f[2] - (coef[0] + coef[1] * f[0] + coef[2] * f[1])
        print(f"   {'leg':4s} {'level-body ref':>15s} {'stance-plane ref':>17s}   (mm, median "
              f"over swing frames)")
        for j, l in enumerate(legs):
            c = clear[:, j][np.isfinite(clear[:, j])]
            cp = clear_p[:, j][np.isfinite(clear_p[:, j])]
            print(f"   {l:4s} {np.median(c)*1000 if c.size else float('nan'):15.1f} "
                  f"{np.median(cp)*1000 if cp.size else float('nan'):17.1f}")
        tt = tilt[np.isfinite(tilt)]
        print(f"   implied trunk pitch to keep the stance feet coplanar: "
              f"{np.median(tt):+.2f} deg (min {tt.min():+.2f}, max {tt.max():+.2f})")

        # ---- 3. the 2x2: which (thigh, calf) offset lowers the foot without moving x
        mean_stance = np.zeros(12, dtype=np.float32)
        for j in range(4):
            st = contact[:, j]
            mean_stance[3*j:3*j+3] = q[st, 3*j:3*j+3].mean(axis=0) if st.any() else q[:, 3*j:3*j+3].mean(axis=0)
        def probe(delta):
            qj = robot.data.default_joint_pos.clone()
            qj[:, idx_t] = torch.as_tensor(mean_stance + delta, device=sim.device, dtype=torch.float32)
            robot.write_root_state_to_sim(air)
            robot.write_joint_state_to_sim(qj, torch.zeros_like(qj))
            robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)
            return drops_now(legs)
        d0, x0 = probe(np.zeros(12, dtype=np.float32))
        J = {}
        for jn, off in (("thigh", 1), ("calf", 2)):
            dv = np.zeros(12, dtype=np.float32); dv[off::3] = args.eps
            d1, x1 = probe(dv)
            J[jn] = ((d1 - d0) / args.eps, (x1 - x0) / args.eps)
        sol = {}
        print(f"   {'leg':4s} {'d drop/d thigh':>15s} {'d x/d thigh':>12s} {'d drop/d calf':>14s} "
              f"{'d x/d calf':>11s} {'-> d thigh':>11s} {'d calf':>9s}  (m/rad, rad)")
        for j, l in enumerate(legs):
            A = np.array([[J["thigh"][0][j], J["calf"][0][j]],
                          [J["thigh"][1][j], J["calf"][1][j]]])
            b = np.array([deficit[j] / 1000.0, 0.0])       # drop MORE by the deficit, x unchanged
            try:
                s = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                s = np.array([np.nan, np.nan])
            sol[l] = [float(s[0]), float(s[1])]
            print(f"   {l:4s} {A[0,0]:15.4f} {A[1,0]:12.4f} {A[0,1]:14.4f} {A[1,1]:11.4f} "
                  f"{s[0]:+11.4f} {s[1]:+9.4f}")
        out[cname] = {"base_settle_mm": base_settle * 1000,
                      "deficit_mm": deficit.tolist(), "legs": legs,
                      "offset_rad": sol}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
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
