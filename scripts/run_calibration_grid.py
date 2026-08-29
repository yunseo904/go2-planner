#!/usr/bin/env python3
"""Calibration probes, all of them at once, on one terrain grid.

    # smoke first: three probes, one skill
    scripts/isaac_docker_run.sh scripts/run_calibration_grid.py --headless --device cpu \
        --max-probes 3 --params skill.STEP_WALK_MAX

    # the full sweep
    scripts/isaac_docker_run.sh scripts/run_calibration_grid.py --headless --device cpu

    # anywhere, no Isaac Lab
    python3 scripts/run_calibration_grid.py --plan
    python3 scripts/run_calibration_grid.py --self-test

Why this replaces the per-episode loop
--------------------------------------
``scripts/run_calibration.py`` rebuilds the terrain and the robot at fixed prim
paths once per probe, and Isaac Lab 3.0 does not support that teardown inside one
``SimulationContext``: the previous ``Articulation`` stays registered after its
prim is removed and the next ``sim.reset()`` dies.  Episode 1 ran; episode 2 did
not.  The two ways out its comment named were (a) build once and swap, or (b) one
OS process per probe.

This is (a), and it is also how Isaac Lab is meant to be driven: **every probe is
a cell of one terrain, and every probe gets its own robot, and they all step
together.**  N probes cost one scene build and N-times-nothing per step, instead
of N process starts at ~25 s each.  The same grid is what a benchmark run needs
-- 20 tasks x 10 levels is the same problem shape -- so this code is not
calibration-only.

Layout
------
Each probe's height field becomes a mesh, translated to its cell of a
``rows x cols`` grid with a one-cell gutter, and all of them are concatenated
into a single mesh handed to ``TerrainImporter.import_mesh``.  One mesh, one
collider: the importer's own ``configure_env_origins`` then places one robot per
cell at that probe's spawn.  Probes cannot interact -- a cell is 8 m long and the
gutter is a full cell wide -- and a robot that falls off its own cell is scored a
failure by its own probe's goal test, not by wandering into its neighbour.

What is scored
--------------
Unchanged from the per-episode harness, deliberately: a repeat succeeds if the
robot reaches the probe's second goal upright inside the time budget.  This file
changes how the runs are *executed*, not what counts as passing, so a limit
measured here is comparable with one measured there.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from sim import heightfield as HF
from sim import isaac_cfg as IC
from terrain_toolkit.calibrate import CALIBRATION_MAP
from terrain_toolkit.paths import CALIBRATION_NPZ, SKILL_CLIPS_META_JSON

from run_calibration import (FALL_HEIGHT_M, GOAL_RADIUS_M, PARAM_SKILL, TIME_BUDGET_S,
                             load_probes, planned_runs)

#: Measured steady speeds, m/s -- the same numbers planner/config.py carries.
SKILL_SPEED = {"WALK": 0.187, "TROT": 0.444, "TURN": 0.008, "RUN": 0.514, "JUMP": 0.006}
#: Per-skill heading cap, from the open-loop steering probe.
HEADING_CAP = {"WALK": 0.04, "TROT": 0.02}

_SIM_APP = None
_RUN_UTC = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def grid_shape(n: int, cols: int | None = None) -> tuple:
    """``(rows, cols)`` for ``n`` cells, squarish unless told otherwise."""
    if cols:
        return int(np.ceil(n / cols)), int(cols)
    c = int(np.ceil(np.sqrt(n)))
    return int(np.ceil(n / c)), c


def build_grid(probes: dict, idx: list, gutter_cells: float = 1.0, cols: int | None = None):
    """One mesh holding every selected probe, plus each probe's world spawn.

    Returns ``(vertices, faces, spawns_xy, cell)``.  Cells are laid out in row
    major order with a gutter, so cell ``k`` sits at
    ``(col * (Lx + gap), row * (Ly + gap))`` and the probe's own spawn/goal
    coordinates are simply offset by that.
    """
    hs, vs = probes["horizontal_scale"], probes["vertical_scale"]
    hf0 = probes["hf"][idx[0]]
    Lx, Ly = (hf0.shape[0] - 1) * hs, (hf0.shape[1] - 1) * hs
    gap_x, gap_y = Lx * gutter_cells, Ly * gutter_cells
    rows, ncols = grid_shape(len(idx), cols)

    V, F, spawns, offsets = [], [], [], []
    n_v = 0
    for k, i in enumerate(idx):
        r, c = divmod(k, ncols)
        ox, oy = c * (Lx + gap_x), r * (Ly + gap_y)
        v, f = HF.to_trimesh(probes["hf"][i], hs, vs)
        v = v + np.array([ox, oy, 0.0])
        V.append(v)
        F.append(f + n_v)
        n_v += len(v)
        offsets.append((ox, oy))
        spawns.append((probes["spawn"][0] + ox, probes["spawn"][1] + oy))
    return (np.concatenate(V), np.concatenate(F), np.asarray(spawns, dtype=float),
            np.asarray(offsets, dtype=float), (rows, ncols), (Lx, Ly))


def plan_report(args) -> str:
    probes = load_probes()
    params = args.params or sorted(PARAM_SKILL)
    runs = planned_runs(probes, params, 1, args.max_probes)
    idx = [r[3] for r in runs]
    if not idx:
        return "no probes selected"
    _, _, spawns, offsets, (rows, cols), (Lx, Ly) = build_grid(probes, idx, args.gutter, args.cols)
    L = [f"probes in the frozen archive: {len(probes['hf'])}",
         f"selected: {len(idx)} ({', '.join(sorted({r[2] for r in runs}))})",
         f"grid {rows} x {cols}, cell {Lx:.1f} x {Ly:.1f} m, gutter {args.gutter:g} cell(s)",
         f"footprint {cols*Lx*(1+args.gutter):.0f} x {rows*Ly*(1+args.gutter):.0f} m",
         f"reps {args.reps}, time budget {TIME_BUDGET_S:.0f} s each",
         "",
         f"  one scene build, {len(idx)} robots stepping together.",
         f"  the per-episode harness would need {len(idx)*args.reps} process starts.",
         ""]
    L.append(f"  {'k':>3s} {'probe':16s} {'family':10s} {'param m':>8s} {'skill':6s} {'spawn x,y':>16s}")
    for k, (p, skill, family, i, level) in enumerate(runs):
        L.append(f"  {k:3d} {probes['names'][i]:16s} {family:10s} {level:8.3f} {skill:6s} "
                 f"{spawns[k][0]:7.2f},{spawns[k][1]:7.2f}")
    return "\n".join(L)


def self_test() -> int:
    """The grid arithmetic, with no simulator."""
    probes = load_probes()
    fails = 0

    def ok(label, cond):
        nonlocal fails
        fails += 0 if cond else 1
        print(f"  {'ok  ' if cond else 'FAIL'} {label}")

    idx = list(range(min(7, len(probes["hf"]))))
    V, F, spawns, offsets, (rows, cols), (Lx, Ly) = build_grid(probes, idx, 1.0)
    ok(f"grid {rows}x{cols} holds {len(idx)} probes", rows * cols >= len(idx))
    ok("one mesh, one vertex block per probe",
       len(V) == sum(probes["hf"][i].size for i in idx))
    ok("faces index inside the mesh", int(F.max()) < len(V))
    # every probe's own patch must keep its own heights: the mesh is a translation
    per = probes["hf"][idx[0]].size
    z0 = V[:per, 2]
    ok("heights survive the translation",
       np.allclose(z0, probes["hf"][idx[0]].ravel() * probes["vertical_scale"]))
    # Cells overlap only if they are within one cell on BOTH axes -- the cell is
    # 7.95 x 3.95 m, not square, and comparing the larger separation against the
    # longer side is the wrong test (it failed on a grid that was in fact disjoint).
    dx = np.abs(offsets[:, None, 0] - offsets[None, :, 0])
    dy = np.abs(offsets[:, None, 1] - offsets[None, :, 1])
    overlap = (dx < Lx - 1e-9) & (dy < Ly - 1e-9)
    np.fill_diagonal(overlap, False)
    ok(f"cells are disjoint (x step {np.unique(np.round(dx[dx>0],2))[:1]} m vs cell {Lx:.2f}, "
       f"y step {np.unique(np.round(dy[dy>0],2))[:1]} m vs cell {Ly:.2f})", not overlap.any())
    ok("one spawn per probe, offset with its cell", len(spawns) == len(idx))

    print(f"\ngrid self-test: {'PASS' if fails == 0 else f'{fails} FAILURES'}")
    return 0 if fails == 0 else 1


def run_isaac(args) -> list:
    from isaaclab.app import AppLauncher

    global _SIM_APP
    _SIM_APP = AppLauncher(args).app

    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sim import SimulationCfg, SimulationContext
    from isaaclab.terrains import TerrainImporter, TerrainImporterCfg
    # This image has neither isaacsim.core.utils.prims nor omni.isaac.core.utils.prims
    # (checked, both ModuleNotFoundError), so the env Xforms are defined through plain
    # USD, which is always there.  They have to exist before the Articulation is built:
    # a regex prim path resolves against prims that are already on the stage.
    import omni.usd
    from pxr import Gf, UsdGeom

    from verify_skill_replay import load_clip
    from sim.replay import ground_material_cfg, quat_to_rpy_deg, set_robot_friction, snap
    from sim.footcomp import FootPlacement, stance_time_s
    from planner.skills import SkillId, SUPPORTED  # noqa: F401

    probes = load_probes()
    params = args.params or sorted(PARAM_SKILL)
    runs = planned_runs(probes, params, 1, args.max_probes)
    runs = [r for r in runs if r[1] in {s.value for s in SUPPORTED}] if args.skip_unsupported else runs
    if not runs:
        raise SystemExit("no probes selected")
    idx = [r[3] for r in runs]
    n = len(idx)
    print(plan_report(args))

    V, F, spawns, offsets, (rows, cols), (Lx, Ly) = build_grid(probes, idx, args.gutter, args.cols)
    ucfg = IC.load()
    sys.path.insert(0, str(Path(ucfg.source).parents[3]))
    from legged_gym.envs.base.legged_robot_config import UNITREE_GO2_CFG

    phys_dt = float(ucfg.sim_dt)
    decim = int(ucfg.decimation or 4)
    dt = decim * phys_dt
    sim = SimulationContext(SimulationCfg(dt=phys_dt, device=args.device))

    # One mesh, one collider, one importer.  `import_mesh` is the supported way to
    # hand Isaac Lab a terrain it did not generate itself; the alternative -- a
    # ground plane plus per-probe rigid prims -- would give every probe its own
    # collision body and no shared origin bookkeeping.
    import trimesh
    mesh = trimesh.Trimesh(vertices=V, faces=F)
    ti_cfg = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane",
                                physics_material=ground_material_cfg(sim_utils),
                                num_envs=n, env_spacing=0.0)
    importer = TerrainImporter(ti_cfg)
    importer.import_mesh("probes", mesh)
    origins = np.concatenate([spawns, np.full((n, 1), 0.0)], axis=1)
    # NOT importer.configure_env_origins(origins): in Isaac Lab 3.0 that call expects a
    # generator-shaped (rows, cols, 3) grid and, given a flat (N, 3) list, takes the
    # curriculum path and indexes it with terrain_levels/terrain_types.  With exactly
    # three probes the (3, 3) array made that fancy indexing succeed and return nonsense,
    # so the smoke passed and the 72-probe run raised IndexError -- a shape coincidence
    # standing in for a check.  Every robot's pose is written explicitly below, which is
    # what actually places them, so the importer does not need to own the origins.
    print(f"[grid] terrain: {len(V)} vertices, {len(F)} faces, {n} cells in a {rows}x{cols} grid")

    # One robot per cell.  The prims are created first and then a SINGLE Articulation
    # picks them all up by regex, so there is one view, one write, one step for the
    # whole grid -- which is the entire point of doing it this way.
    stage = omni.usd.get_context().get_stage()
    for k in range(n):
        xf = UsdGeom.Xform.Define(stage, f"/World/envs/env_{k}")
        xf.AddTranslateOp().Set(Gf.Vec3d(float(spawns[k][0]), float(spawns[k][1]), 0.0))
    robot_cfg = UNITREE_GO2_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    robot = Articulation(robot_cfg)
    sim.reset()
    print(f"[grid] articulation instances: {robot.num_instances} (want {n})")
    if robot.num_instances != n:
        raise SystemExit(f"[grid] {robot.num_instances} robots for {n} probes -- the regex "
                         f"prim path did not pick up one per cell; refusing to score a grid "
                         f"whose robots are not one-to-one with its probes")
    mu = ucfg.ground_friction
    if mu is not None:
        set_robot_friction(robot, mu)

    clips = {s: load_clip(s, args.rate) for s in sorted({r[1] for r in runs})}
    per_robot_clip = [clips[r[1]] for r in runs]
    q_seq = [np.asarray(c["q_des"], dtype=np.float32) for c in per_robot_clip]
    want = [f"{leg}_{j}_joint" for leg in clips[runs[0][1]]["leg_order"]
            for j in clips[runs[0][1]]["joint_order"]]
    idx_t = torch.as_tensor([robot.joint_names.index(nm) for nm in want],
                            device=sim.device, dtype=torch.long)

    # Put every robot at its own cell's spawn, at the env's spawn height.
    root = robot.data.default_root_state.clone()
    root[:, :3] = torch.as_tensor(origins, device=sim.device, dtype=root.dtype) + \
        torch.as_tensor([0.0, 0.0, float(args.spawn_z)], device=sim.device, dtype=root.dtype)
    robot.write_root_state_to_sim(root)

    # Settle exactly the way the single-robot path does, because the initial condition
    # decides the verdict: dropping into a mid-gait pose lands the robot on three legs
    # and it starts tipping, and a probe scored from there measures the drop.  Stand on
    # the default pose first, then drive to each robot's own clip frame 0 with the PD.
    q_stand = robot.data.default_joint_pos.clone()
    q_first = q_stand.clone()
    q_first[:, idx_t] = torch.as_tensor(
        np.stack([np.asarray(c["q_des"][0], dtype=np.float32) for c in per_robot_clip]),
        device=sim.device, dtype=torch.float32)
    robot.write_joint_state_to_sim(q_stand, torch.zeros_like(q_stand))

    def _hold(target, n_ctrl):
        for _ in range(n_ctrl):
            robot.set_joint_position_target(target)
            for _ in range(decim):
                robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)

    n_settle = max(int(args.settle_s / dt), 1)
    _hold(q_stand, n_settle)
    for k in range(1, n_settle + 1):
        _hold(q_stand * (1 - k / n_settle) + q_first * (k / n_settle), 1)
    v0 = float(torch.linalg.norm(robot.data.root_lin_vel_b, dim=1).max().item())
    print(f"[grid] settled: worst |v| across the grid {v0:.3f} m/s, "
          f"base height {snap(robot.data.root_pos_w)[:,2].min():.3f}-"
          f"{snap(robot.data.root_pos_w)[:,2].max():.3f} m")

    # Foot placement, per robot, with the geometry measured from THIS robot standing on
    # ITS clip pose -- the lever differs between clips and between cells, and the
    # single-robot runs showed a 7% spread in it.
    foots = None
    if args.foot_comp != "off":
        from verify_skill_replay import _log_motion_for
        from sim.replay import quat_rotate_inv
        meta = json.loads(SKILL_CLIPS_META_JSON.read_text())
        hip_ids, hip_names = robot.find_bodies(".*_hip")
        foot_ids, foot_names = robot.find_bodies(".*_foot")
        h_by = {nm.split("_")[0]: j for j, nm in zip(hip_ids, hip_names)}
        f_by = {nm.split("_")[0]: j for j, nm in zip(foot_ids, foot_names)}
        legs = per_robot_clip[0]["leg_order"]
        bp = snap(robot.data.body_pos_w)
        bq = snap(robot.data.root_quat_w)
        bpos = snap(robot.data.root_pos_w)
        foots = []
        for k, c in enumerate(per_robot_clip):
            lever = np.array([bp[k, h_by[l], 2] - bp[k, f_by[l], 2] for l in legs])
            hxy = np.array([quat_rotate_inv(bq[k][None, :], (bp[k, h_by[l]] - bpos[k])[None, :])[0, :2]
                            for l in legs])
            vy_log, wz_log = _log_motion_for(meta, c["name"])
            yaw_mode = "log-cycle" if c["name"] == "TURN" else "off"
            hcap = 0.0
            if args.heading != "off" and c["name"] != "TURN":
                yaw_mode = args.heading
                hcap = HEADING_CAP.get(c["name"], 0.0)
            foots.append(FootPlacement(
                t_stance_s=stance_time_s(c["contact"], c["fs"]), lever_m=lever,
                hip_x_m=hxy[:, 0], hip_y_m=hxy[:, 1], vy_log=vy_log, wz_log=wz_log,
                cap_rad=args.foot_clip_rad, yaw_mode=yaw_mode, heading_cap_rad=hcap,
                cycle_len=len(c["q_des"])))
        print(f"[grid] *** FOOT PLACEMENT ON for all {n} robots (cap {args.foot_clip_rad:g} rad). "
              f"This CLOSES A LOOP on base velocity and OVERWRITES the recording. ***")
    swing = [~np.asarray(c["contact"], dtype=bool) for c in per_robot_clip]

    # Time budget from the SKILL's own measured speed, not one number shared across
    # skills.  The probes put goal 2 about 3.95 m from the spawn; at WALK's 0.233 m/s
    # that is 17 s, and the old shared 20 s left 15% margin for a gait that curves --
    # so a probe could fail for running out of clock rather than for the obstacle.
    # The budget is now distance / speed x a stated slack factor, per robot.
    goal2_pre = np.array([probes["goals"][i][1] for i in idx]) + offsets
    spawn_d = np.linalg.norm(goal2_pre - spawns, axis=1)
    speed = np.array([SKILL_SPEED[r[1]] for r in runs])
    budget = spawn_d / speed * args.budget_slack
    n_steps = int(np.ceil(budget.max() / dt))
    print(f"[grid] time budget from speed: distance {spawn_d.min():.2f}-{spawn_d.max():.2f} m, "
          f"slack x{args.budget_slack:g} -> {budget.min():.1f}-{budget.max():.1f} s "
          f"({n_steps} steps; each robot is scored against its own)")
    budget_steps = np.ceil(budget / dt).astype(int)

    # Repeats have to vary something, and physics here does not vary: the friction is
    # fixed at the env's midpoint rather than sampled per episode, so five identical
    # repeats of a deterministic sim give five identical rows.  The original protocol's
    # reps assumed the env's per-episode randomisation, which this harness deliberately
    # does not do.
    #
    # What IS arbitrary is where in its cycle a clip starts -- whoever cut the clip chose
    # frame 0 -- so rep r enters every clip at frame round(r * n / reps).  A step limit
    # that only holds at one entry phase is not a limit, which is exactly what a repeat
    # is supposed to find out.
    def entry_frames(rep: int) -> list:
        return [int(round(rep * len(q) / max(args.reps, 1))) % len(q) for q in q_seq]
    goal2 = np.array([probes["goals"][i][1] for i in idx]) + offsets     # world frame
    rows_out = []

    for rep in range(max(args.reps, 1)):
        ent = entry_frames(rep)
        # Re-place and re-settle on the SAME scene.  No prim is created or destroyed
        # between repeats, which is the whole reason this harness exists.
        robot.write_root_state_to_sim(root)
        q_ent = q_stand.clone()
        q_ent[:, idx_t] = torch.as_tensor(
            np.stack([q_seq[k][ent[k]] for k in range(n)]),
            device=sim.device, dtype=torch.float32)
        robot.write_joint_state_to_sim(q_stand, torch.zeros_like(q_stand))
        _hold(q_stand, n_settle)
        for j in range(1, n_settle + 1):
            _hold(q_stand * (1 - j / n_settle) + q_ent * (j / n_settle), 1)
        if foots is not None:
            for f in foots:
                f.reset()

        reached = np.zeros(n, dtype=bool)
        fell = np.zeros(n, dtype=bool)
        t_reached = np.full(n, np.nan)
        print(f"[grid] rep {rep+1}/{args.reps}: entry frames {ent[:6]}"
              f"{'...' if n > 6 else ''}, settled |v| max "
              f"{float(torch.linalg.norm(robot.data.root_lin_vel_b, dim=1).max().item()):.3f} m/s")

        for step in range(n_steps):
            tgt = robot.data.default_joint_pos.clone()
            frames = [(ent[k] + step) % len(q_seq[k]) for k in range(n)]
            cmd = np.stack([q_seq[k][frames[k]] for k in range(n)])
            if foots is not None:
                vb = snap(robot.data.root_lin_vel_b)
                wb = snap(robot.data.root_ang_vel_b)
                for k in range(n):
                    cmd[k] = cmd[k] + foots[k].step(float(vb[k, 1]), float(wb[k, 2]),
                                                    swing[k][frames[k]], vx=float(vb[k, 0]))
            tgt[:, idx_t] = torch.as_tensor(cmd, device=sim.device, dtype=torch.float32)
            robot.set_joint_position_target(tgt)
            for _ in range(decim):
                robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)

            pos = snap(robot.data.root_pos_w)
            d = np.linalg.norm(pos[:, :2] - goal2, axis=1)
            newly = (~reached) & (d < GOAL_RADIUS_M) & (step <= budget_steps)
            t_reached[newly] = step * dt
            reached |= newly
            fell |= pos[:, 2] < FALL_HEIGHT_M
            if reached.all() or fell.all():
                break

        dist = np.linalg.norm(snap(robot.data.root_pos_w)[:, :2] - goal2, axis=1)
        for k, (param, skill, family, i, level) in enumerate(runs):
            rows_out.append({"param": param, "skill": skill, "family": family,
                             "probe": probes["names"][i], "level_m": level, "rep": rep,
                             "entry_frame": ent[k],
                             "reached": int(reached[k]), "t_reached_s": t_reached[k],
                             "fell": int(fell[k]), "final_dist_m": float(dist[k]),
                             "grid_rows": rows, "grid_cols": cols, "n_probes": n,
                             "foot_comp": args.foot_comp, "steps": step + 1,
                             "run_utc": _RUN_UTC, "argv": " ".join(sys.argv[1:])})
        print(f"[grid]   rep {rep+1}: reached {int(reached.sum())}/{n}, "
              f"fell {int(fell.sum())}/{n}, {step+1} steps")

    # -- the protocol's own reduction: a level passes only if EVERY repeat passed ----
    print(f"\n  {'probe':16s} {'skill':6s} {'param m':>8s} {'passed':>10s}  reps")
    for k, (param, skill, family, i, level) in enumerate(runs):
        mine = [r for r in rows_out if r["probe"] == probes["names"][i] and r["param"] == param]
        got = sum(r["reached"] for r in mine)
        print(f"  {probes['names'][i]:16s} {skill:6s} {level:8.3f} {got:5d}/{len(mine):<4d}  "
              + "".join("R" if r["reached"] else ("F" if r["fell"] else ".") for r in mine))
    print("\n  R = reached goal 2, F = fell, . = neither inside the time budget")
    print(config_patch(rows_out))
    return rows_out


def config_patch(rows: list) -> str:
    """Turn the per-repeat rows into the config values, using the existing protocol.

    Reuses ``run_calibration.thresholds_from_rows`` rather than re-deriving the rule,
    so the grid harness and the per-episode one reduce identical data identically:
    the reported limit is the top of the unbroken run of levels that passed on EVERY
    repeat, minus one level of margin.  A limit that works four repeats in five is a
    limit the planner will walk off.
    """
    from run_calibration import thresholds_from_rows
    adapted = [{"parameter": r["param"], "level_m": r["level_m"], "rep": r["rep"],
                "passed": r["reached"], "family": r["family"], "skill": r["skill"]}
               for r in rows]
    try:
        ths = thresholds_from_rows(adapted)
    except Exception as exc:                                    # pragma: no cover
        return f"(threshold reduction failed: {type(exc).__name__}: {exc})"
    L = ["", "=" * 78, "CONFIG PATCH -- proposed values, not applied", "=" * 78, "",
         f"  {'parameter':26s} {'skill':6s} {'value':>8s} {'top all-pass':>13s} "
         f"{'first fail':>11s}  monotone"]
    fmt = lambda x: f"{x:8.3f}" if isinstance(x, (int, float)) else f"{'none':>8s}"
    for t in ths:
        L.append(f"  {t.parameter:26s} {t.skill:6s} {fmt(t.value_m)} "
                 f"{fmt(t.highest_all_pass_m):>13s} {fmt(t.first_failure_m):>11s}  {t.monotone}")
    L += ["", "  apply by editing planner/config.py:", ""]
    any_value = False
    for t in ths:
        field = t.parameter.split(".", 1)[1]
        if isinstance(t.value_m, (int, float)):
            any_value = True
            L.append(f"    {field}: float = {t.value_m:.3f}")
        else:
            L.append(f"    # {field}: NO VALUE -- no level passed every repeat "
                     f"({t.note or 'the lowest probe already failed'})")
    if not any_value:
        L.append("")
        L.append("    Nothing to apply: not one level of any family passed all repeats, so")
        L.append("    the limits are below the smallest probe or the protocol did not")
        L.append("    measure what it meant to. Read the per-probe table above before")
        L.append("    touching config.py.")
    L += ["", "  and move each one's _p() provenance from CALIBRATION_NEEDED to MEASURED,",
          "  citing outputs/calibration_grid.csv and this run's run_utc.",
          "",
          "  NOT applied automatically: a threshold becoming a planner guarantee is a",
          "  decision, and a non-monotone column means the probe family did not behave",
          "  like a ladder and the number should not be read as a limit at all."]
    return "\n".join(L)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", nargs="*", default=None)
    ap.add_argument("--max-probes", type=int, default=None, help="cap levels per family (smoke)")
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--cols", type=int, default=None, help="grid columns (default: squarish)")
    ap.add_argument("--gutter", type=float, default=1.0, help="empty cells between probes")
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo")
    ap.add_argument("--spawn-z", type=float, default=0.42)
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--foot-comp", choices=("off", "on"), default="on")
    ap.add_argument("--foot-clip-rad", type=float, default=0.05)
    ap.add_argument("--time-budget-s", type=float, default=TIME_BUDGET_S,
                    help="ignored: the budget is derived per robot from its skill's speed")
    ap.add_argument("--budget-slack", type=float, default=2.0,
                    help="budget = distance / skill speed x this. 2.0 gives a gait twice "
                         "the time a straight-line traverse needs.")
    ap.add_argument("--heading", choices=("off", "heading", "heading-only"), default="off",
                    help="hold the spawn heading with differential lateral foot placement")
    ap.add_argument("--skip-unsupported", action="store_true",
                    help="drop probes whose skill the low level cannot execute (RUN/JUMP)")
    ap.add_argument("--results-csv", default=None)
    ap.add_argument("--plan", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    try:
        from isaaclab.app import AppLauncher
        AppLauncher.add_app_launcher_args(ap)
        ap.set_defaults(device="cpu")
    except Exception:
        ap.add_argument("--headless", action="store_true")
        ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    if args.plan:
        print(plan_report(args)); return 0
    if args.self_test:
        return self_test()
    rows = []
    try:
        rows = run_isaac(args)
    finally:
        if _SIM_APP is not None:
            exc = sys.exc_info()[1]
            if exc is not None:
                import traceback
                print("[grid] EXCEPTION -- printed before the app teardown takes it:",
                      file=sys.stderr)
                traceback.print_exc(); sys.stderr.flush(); sys.stdout.flush()
            if rows and args.results_csv:
                from verify_skill_replay import report
                report(rows, args.results_csv)
            _SIM_APP.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
