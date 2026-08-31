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
from terrain_toolkit.calibrate import CALIBRATION_MAP, RAMP_RUN_M, ROUGH_RUN_M
from terrain_toolkit.paths import CALIBRATION_NPZ, SKILL_CLIPS_META_JSON

from run_calibration import (FALL_HEIGHT_M, GOAL_RADIUS_M, PARAM_SKILL, TIME_BUDGET_S,
                             load_probes, planned_runs)

#: Measured steady speeds, m/s -- the same numbers planner/config.py carries.
SKILL_SPEED = {"WALK": 0.187, "TROT": 0.444, "TURN": 0.008, "RUN": 0.514, "JUMP": 0.006}
#: Per-skill heading cap, from the open-loop steering probe.
HEADING_CAP = {"WALK": 0.04, "TROT": 0.02}
#: Measured yaw rate of the TURN clip, rad/s (planner.config skill.YAW_RATE_TURN, from
#: session turn_right_20260824_223951).  Used only to size the in-place time budget.
YAW_RATE = {"TURN": -0.3954}
#: How much yaw an in-place repeat has to complete to count.  90 deg is a quarter turn:
#: large enough that a robot cannot pass by wobbling (the TURN clip's own cycle is 20.2
#: deg, so this is ~4.5 cycles of sustained turning) and small enough to fit a budget.
#: The full yaw-vs-time curve is written to the CSV as well, so a different threshold
#: can be read off these runs without repeating them.
INPLACE_YAW_DEG = 90.0
#: How far the base may wander from its spawn and still count as having turned IN PLACE.
#: Deliberately GOAL_RADIUS_M, the same 0.35 m the traversal families score "arrived"
#: with, rather than a new number.

_SIM_APP = None
_RUN_UTC = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def grid_shape(n: int, cols: int | None = None) -> tuple:
    """``(rows, cols)`` for ``n`` cells, squarish unless told otherwise."""
    if cols:
        return int(np.ceil(n / cols)), int(cols)
    c = int(np.ceil(np.sqrt(n)))
    return int(np.ceil(n / c)), c


#: Half-length and half-width, m, of the box the local ground height is taken over when a
#: robot is spawned on top of an obstacle instead of in front of one.  It is the Go2's own
#: footprint: hips sit at x = +-0.193, y = +-0.11 (measured from the articulation in
#: outputs/heading_candidates.md 1), plus the foot radius of 0.023 measured at the lip.
FOOTPRINT_HALF_XY = (0.216, 0.133)


def inplace_spawn_x(family: str, level: float, obstacle_x: float) -> float:
    """Where to stand a turn-in-place skill so the obstacle is UNDER it.

    The traversal families spawn 3 m short of the obstacle and are scored on reaching a
    goal past it.  A skill that does not translate can never be scored that way -- TURN's
    measured speed is 0.0075 m/s, so the 3.95 m to goal 2 is 527 s against a 20 s budget,
    and every level would read 0/N for the same reason regardless of the terrain.  The
    obstacle has to be brought to the robot instead, and then the question the sweep
    answers is a different one and has to be named as such: not *what can TURN cross*
    but **what can TURN turn on**.

    Placement puts the discontinuity under the middle of the footprint, so the feet
    sweeping through a turn cross it:

    ``step_up``/``step_down``  the edge itself, so two feet are up and two down
    ``gap``                    the pit's centre, so the hole is inside the footprint
    ``slope``/``roughness``    the middle of the patch, so the whole robot is on it
    """
    if family in ("step_up", "step_down"):
        return obstacle_x
    if family == "gap":
        return obstacle_x + 0.5 * level
    if family == "slope":
        return obstacle_x + 0.5 * RAMP_RUN_M
    if family == "roughness":
        return obstacle_x + 0.5 * ROUGH_RUN_M
    raise ValueError(f"no in-place spawn defined for family {family!r}")


def ground_z_at(hf: np.ndarray, hs: float, vs: float, x: float, y: float) -> float:
    """Highest terrain height under a footprint-sized box at ``(x, y)``, metres.

    Needed because ``--spawn-z`` is measured from z = 0 and the in-place spawns stand on
    top of the obstacle: dropping a robot from 0.42 m onto a 0.30 m step puts it inside
    the mesh.  MAX rather than mean or nearest -- a robot half on a step has to clear the
    high side, and starting it interpenetrating the terrain is not a measurement of
    anything.
    """
    dx, dy = FOOTPRINT_HALF_XY
    i0 = max(0, int(np.floor((x - dx) / hs)))
    i1 = min(hf.shape[0] - 1, int(np.ceil((x + dx) / hs)))
    j0 = max(0, int(np.floor((y - dy) / hs)))
    j1 = min(hf.shape[1] - 1, int(np.ceil((y + dy) / hs)))
    # Floored at 0, the lane level every family shares.  Without the floor a robot
    # standing astride a gap wider than its own footprint reads the PIT BOTTOM as its
    # ground -- the box is entirely inside the hole -- and gets spawned 1 m down it.
    # Measured: the 42-probe run settled at base z -0.524 m with a hip-to-foot lever of
    # 0.034 m and FootPlacement refused to build.  A robot standing at the bottom of a
    # 1 m pit is not a measurement of turning over a gap; the lane surface is.
    return max(float(hf[i0:i1 + 1, j0:j1 + 1].max()) * vs, 0.0)


def build_grid(probes: dict, idx: list, gutter_cells: float = 1.0, cols: int | None = None,
               inplace: bool = False):
    """One mesh holding every selected probe, plus each probe's world spawn.

    Returns ``(vertices, faces, spawns_xy, offsets, (rows, cols), (Lx, Ly), spawn_z0)``.
    Cells are laid out in row major order with a gutter, so cell ``k`` sits at
    ``(col * (Lx + gap), row * (Ly + gap))`` and the probe's own spawn/goal
    coordinates are simply offset by that.

    ``inplace`` moves each spawn onto its own obstacle (see ``inplace_spawn_x``) and
    returns the ground height there, which the caller adds to ``--spawn-z``.
    """
    hs, vs = probes["horizontal_scale"], probes["vertical_scale"]
    hf0 = probes["hf"][idx[0]]
    Lx, Ly = (hf0.shape[0] - 1) * hs, (hf0.shape[1] - 1) * hs
    gap_x, gap_y = Lx * gutter_cells, Ly * gutter_cells
    rows, ncols = grid_shape(len(idx), cols)
    obstacle_x = float(probes.get("obstacle_x", 4.0))

    V, F, spawns, offsets, z0 = [], [], [], [], []
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
        sx, sy = probes["spawn"][0], probes["spawn"][1]
        if inplace:
            sx = inplace_spawn_x(str(probes["families"][i]), float(probes["params_m"][i]),
                                 obstacle_x)
        z0.append(ground_z_at(probes["hf"][i], hs, vs, sx, sy) if inplace else 0.0)
        spawns.append((sx + ox, sy + oy))
    return (np.concatenate(V), np.concatenate(F), np.asarray(spawns, dtype=float),
            np.asarray(offsets, dtype=float), (rows, ncols), (Lx, Ly),
            np.asarray(z0, dtype=float))


def plan_report(args) -> str:
    probes = load_probes()
    params = args.params or sorted(PARAM_SKILL)
    runs = planned_runs(probes, params, 1, args.max_probes,
                        getattr(args, 'families', None), getattr(args, 'skill', None))
    idx = [r[3] for r in runs]
    if not idx:
        return "no probes selected"
    _, _, spawns, offsets, (rows, cols), (Lx, Ly), _z0 = build_grid(
        probes, idx, args.gutter, args.cols,
        args.score == 'inplace' and args.inplace_at == 'obstacle')
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
    V, F, spawns, offsets, (rows, cols), (Lx, Ly), _z0 = build_grid(probes, idx, 1.0)
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
    runs = planned_runs(probes, params, 1, args.max_probes, args.families, args.skill)
    runs = [r for r in runs if r[1] in {s.value for s in SUPPORTED}] if args.skip_unsupported else runs
    if not runs:
        raise SystemExit("no probes selected")
    idx = [r[3] for r in runs]
    n = len(idx)
    print(plan_report(args))

    V, F, spawns, offsets, (rows, cols), (Lx, Ly), spawn_z0 = build_grid(
        probes, idx, args.gutter, args.cols,
        args.score == 'inplace' and args.inplace_at == 'obstacle')
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
    # DELETE the ground plane the importer just made.  terrain_type="plane" is the only
    # option that needs no extra config, and it calls import_ground_plane(), which lays a
    # 2000 km x 2000 km plane at z = 0 UNDER the probe mesh.  That floors every pit and
    # makes every gutter solid: the gap family measured a robot walking over its own
    # 1 m deep, 0.60 m wide trench on an invisible floor, reported 60/60 reached and 0%
    # fallen, and produced a FOOT_SPAN_X of 0.550 m that no walking Go2 could earn.
    # Removing it is the whole fix; a probe that walks off its cell now falls, which is
    # the correct verdict for a probe that walks off its cell.
    plane_path = ti_cfg.prim_path + "/terrain"
    _stage = omni.usd.get_context().get_stage()
    if _stage.GetPrimAtPath(plane_path).IsValid():
        _stage.RemovePrim(plane_path)
    if _stage.GetPrimAtPath(plane_path).IsValid():
        raise SystemExit(f"[grid] refusing to run: the ground plane at {plane_path} is "
                         f"still on the stage, so every pit in the archive is floored")
    print(f"[grid] removed the importer's infinite ground plane at {plane_path}; "
          f"pits are open and the gutters are void")
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
    # Contact reporting covers EVERY link, not just the feet.  The question a step
    # failure asks is which part of the robot is touching the obstacle -- if only the
    # feet were instrumented, a shin hitting the lip would read as no contact at all,
    # which is the answer that would be most wrong.
    contacts = None
    if args.trace_full:
        from isaaclab.sensors import ContactSensor, ContactSensorCfg
        contacts = ContactSensor(ContactSensorCfg(
            prim_path="/World/envs/env_.*/Robot/.*", history_length=0,
            track_air_time=False, update_period=0.0))
    # Side-view recording of ONE cell.  The camera is created before sim.reset() like
    # every other sensor, renders ON DEMAND (update_period 0.0) and never schedules
    # itself, and grab_frame() calls sim.render() and nothing else -- the physics substep
    # loop below is untouched: same phys_dt, same decimation, same order of writes.  The
    # check that this held is the recorded run's own termination against the unrecorded
    # one, which is what CLAUDE.md 8 asks a render run to demonstrate.
    video = None
    if args.video:
        from isaaclab.sensors import Camera, CameraCfg
        if not (0 <= args.video_cell < n):
            raise SystemExit(f"--video-cell {args.video_cell} is outside 0..{n-1}")
        video = {"cam": Camera(CameraCfg(
            prim_path="/World/side_cam", update_period=0.0,
            height=args.video_height, width=args.video_width, data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=24.0,
                                             clipping_range=(0.05, 60.0)))),
            "frames": 0, "k": int(args.video_cell)}
        print(f"[grid] video: side view of cell {args.video_cell} "
              f"({probes['names'][idx[args.video_cell]]}, {runs[args.video_cell][1]}), "
              f"{args.video_width}x{args.video_height} -> {args.video}")
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

    # ---- swing lift, the same edit the flat sweep uses, resolved once per CLIP ------
    if args.swing_lift > 0:
        from verify_skill_replay import swing_lift_offsets
        lift_by_clip = {}
        for name, c in clips.items():
            off, rep_, _ = swing_lift_offsets(robot, sim, idx_t, c,
                                              args.swing_lift / 1000.0, phys_dt,
                                              symmetric=not args.swing_lift_asym)
            lift_by_clip[name] = off
            print(f"[grid] swing lift {args.swing_lift:g} mm on {name}: "
                  + ("PER-LEG " if args.swing_lift_asym else "SYMMETRIC ")
                  + ", ".join(f"{l} +{r['added_mm']}"
                              for l, r in rep_.items() if not l.startswith("_")))
        q_seq = [q + lift_by_clip[r[1]] for q, r in zip(q_seq, runs)]
        print(f"[grid] *** SWING LIFT ON: the recording's swing arcs are raised. Stance and "
              f"the hip are untouched; the archive on disk is unchanged. ***")

    # Put every robot at its own cell's spawn, at the env's spawn height.
    root = robot.data.default_root_state.clone()
    # --spawn-z is measured from z = 0, which is the ground for every traversal probe
    # (they all spawn on the flat run-up).  The in-place spawns stand ON the obstacle, so
    # the drop height has to be taken from the terrain under the footprint instead --
    # otherwise a 0.30 m step spawns the robot 0.30 m inside the mesh and the probe
    # measures the extraction, not the turn.
    root[:, :3] = torch.as_tensor(origins, device=sim.device, dtype=root.dtype) + \
        torch.as_tensor(np.stack([np.zeros(n), np.zeros(n), spawn_z0 + float(args.spawn_z)],
                                 axis=1), device=sim.device, dtype=root.dtype)
    if args.score == "inplace":
        print(f"[grid] in-place spawns stand on the obstacle: ground under the footprint "
              f"{spawn_z0.min():+.3f} to {spawn_z0.max():+.3f} m, drop {args.spawn_z:g} m "
              f"above that")
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
    settle_ok = np.ones(n, dtype=bool)
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
        # A robot whose settle did not produce a stance has no lever to measure.  It
        # happens on the in-place spawns and only there: standing astride a 0.60 m pit
        # there is nothing under the feet, and the robot is already falling into the hole
        # when the levers are read (measured: base z 0.020 m, levers 0.034-0.114 m
        # against a nominal 0.31).  FootPlacement then refuses to build -- correctly, its
        # law is linearised about a standing pose -- and took the whole 42-robot run down
        # with it, including the 40 robots that had settled fine.
        #
        # Such a robot is scored a FAILURE, which is the true answer: a gap it falls into
        # during the settle is a gap it cannot turn over.  The median lever is substituted
        # only so the object can be constructed and the grid can step; the substitution
        # is stamped on every one of that robot's rows so a reader can never mistake one
        # for a measurement.
        all_lever = np.array([[bp[k, h_by[l], 2] - bp[k, f_by[l], 2] for l in legs]
                              for k in range(n)])
        settle_ok = np.all(all_lever > 0.05, axis=1)
        med_lever = np.median(all_lever[settle_ok], axis=0) if settle_ok.any() \
            else np.full(len(legs), 0.31)
        if not settle_ok.all():
            bad = np.where(~settle_ok)[0]
            print(f"[grid] {len(bad)} of {n} robots did not settle into a stance and are "
                  f"scored FAILED at settle: "
                  + ", ".join(f"{probes['names'][idx[k]]} (worst lever "
                              f"{all_lever[k].min():.3f} m)" for k in bad[:8])
                  + (" ..." if len(bad) > 8 else ""))
        # COM height for the capture-point gain, measured per robot from the settle --
        # sqrt(h/g) is a property of this body at this stance, so a nominal number would
        # be exactly the kind of "almost the right quantity" CLAUDE.md 6.5 is about.
        # A robot that failed its settle gets the median, stamped like its levers are.
        com_h = np.where(settle_ok, bpos[:, 2] - np.array(
            [np.mean([bp[j, f_by[l], 2] for l in legs]) for j in range(n)]),
            np.nan)
        if np.isfinite(com_h).any():
            com_h = np.where(np.isfinite(com_h), com_h, np.nanmedian(com_h))
        else:
            com_h = np.full(n, 0.31)
        foots = []
        for k, c in enumerate(per_robot_clip):
            lever = all_lever[k] if settle_ok[k] else med_lever
            hxy = np.array([quat_rotate_inv(bq[k][None, :], (bp[k, h_by[l]] - bpos[k])[None, :])[0, :2]
                            for l in legs])
            vy_log, wz_log = _log_motion_for(meta, c["name"])
            yaw_mode = "log-cycle" if c["name"] == "TURN" else "off"
            hcap = 0.0
            if args.heading != "off" and c["name"] != "TURN":
                yaw_mode = args.heading
                hcap = args.heading_cap or HEADING_CAP.get(c["name"], 0.0)
            foots.append(FootPlacement(
                t_stance_s=stance_time_s(c["contact"], c["fs"]), lever_m=lever,
                hip_x_m=hxy[:, 0], hip_y_m=hxy[:, 1], vy_log=vy_log, wz_log=wz_log,
                cap_rad=args.foot_clip_rad, yaw_mode=yaw_mode, heading_cap_rad=hcap,
                heading_len=bool(args.heading_len) and hcap > 0.0,
                heading_len_cap_rad=args.heading_len_cap,
                yaw_bias=args.foot_yaw_bias, len_bias=args.foot_len_bias,
                cycle_len=len(c["q_des"]),
                gain_mode=args.foot_gain, com_height_m=com_h[k],
                vy_avg_n=args.foot_vy_avg_n,
                offset_clip_m=args.foot_offset_clip_m))
        print(f"[grid] *** FOOT PLACEMENT ON for all {n} robots (cap {args.foot_clip_rad:g} rad). "
              f"This CLOSES A LOOP on base velocity and OVERWRITES the recording. ***")
        if args.foot_gain != "half-stance" or args.foot_vy_avg_n or args.foot_offset_clip_m:
            print(f"[grid] *** CAPTURE-POINT VARIANT: gain={args.foot_gain}"
                  + (f" (sqrt(h/g), h measured {com_h.min():.3f}-{com_h.max():.3f} m -> "
                     f"{float(np.sqrt(com_h.mean()/9.81)):.4f} s against half-stance "
                     f"{0.5*float(np.mean([stance_time_s(c['contact'], c['fs']) for c in per_robot_clip])):.4f} s)"
                     if args.foot_gain == "capture" else "")
                  + (f", v_y averaged over {args.foot_vy_avg_n} control steps "
                     f"({args.foot_vy_avg_n*dt:.3f} s)" if args.foot_vy_avg_n else
                     ", v_y unaveraged")
                  + (f", foot offset clipped to +-{args.foot_offset_clip_m:g} m "
                     f"(the radian cap would be {args.foot_clip_rad*float(np.mean(med_lever)):.4f} m)"
                     if args.foot_offset_clip_m else "")
                  + ". Default is OFF and reproduces every earlier run. ***")
    if video is not None:
        import imageio.v2 as imageio
        fps = args.video_fps if args.video_fps else max(1.0, 1.0 / dt)
        Path(args.video).parent.mkdir(parents=True, exist_ok=True)
        video["writer"] = imageio.get_writer(args.video, fps=fps, macro_block_size=None,
                                             codec="libx264", quality=8)
        video["fps"] = fps
        video["y0"] = float(spawns[video["k"]][1])
        print(f"[grid] video: writing {fps:.1f} fps "
              f"({'real time' if not args.video_fps else 'forced -- slow motion'})")

    def grab_frame():
        """Pull one RGB frame of the tracked cell. Renders only; never steps physics."""
        k = video["k"]
        p_ = snap(robot.data.root_pos_w)[k]
        eye = torch.tensor([[float(p_[0]), video["y0"] + args.video_side_m,
                             max(0.35, float(p_[2]) + 0.12)]],
                           device=sim.device, dtype=torch.float32)
        tgt_ = torch.tensor([[float(p_[0]), video["y0"], 0.28]],
                            device=sim.device, dtype=torch.float32)
        video["cam"].set_world_poses_from_view(eye, tgt_)
        sim.render()
        video["cam"].update(dt, force_recompute=True)
        rgb = video["cam"].data.output["rgb"][0].detach().cpu().numpy()
        video["writer"].append_data(np.ascontiguousarray(rgb[..., :3]).astype(np.uint8))
        video["frames"] += 1

    swing = [~np.asarray(c["contact"], dtype=bool) for c in per_robot_clip]

    # Time budget from the SKILL's own measured speed, not one number shared across
    # skills.  The probes put goal 2 about 3.95 m from the spawn; at WALK's 0.233 m/s
    # that is 17 s, and the old shared 20 s left 15% margin for a gait that curves --
    # so a probe could fail for running out of clock rather than for the obstacle.
    # The budget is now distance / speed x a stated slack factor, per robot.
    goal2_pre = np.array([probes["goals"][i][1] for i in idx]) + offsets
    spawn_d = np.linalg.norm(goal2_pre - spawns, axis=1)
    if args.score == "inplace":
        # The budget is the angle over the clip's own measured yaw rate, for the same
        # reason the traversal budget is the distance over the clip's own speed: a probe
        # must not be able to fail for running out of clock.  TURN at 22.66 deg/s needs
        # 4.0 s for the 90 deg; the slack factor is the same one.
        rate = np.array([abs(YAW_RATE[r[1]]) for r in runs])
        budget = np.radians(args.inplace_yaw_deg) / rate * args.budget_slack
    else:
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
    foot_ids_t, _foot_names_t = robot.find_bodies(".*_foot")
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

        # The heading each robot is asked to hold: the one it settled at, per cell.
        yaw0_deg = quat_to_rpy_deg(snap(robot.data.root_quat_w))[2]
        pos0 = snap(robot.data.root_pos_w)[:, :2].copy()
        reached = np.zeros(n, dtype=bool)
        fell = np.zeros(n, dtype=bool)
        t_reached = np.full(n, np.nan)
        # In-place bookkeeping.  Yaw is UNWRAPPED against the previous sample so a turn
        # past 180 deg keeps counting instead of folding back; the traversal path never
        # needed this because nothing there was supposed to rotate at all.
        yaw_prev = yaw0_deg.copy()
        yaw_cum = np.zeros(n)                 # signed, degrees, since the settle
        yaw_ok = np.zeros(n)                  # the most turned WHILE still legal
        drift_max = np.zeros(n)               # furthest the base got from its spawn
        legal = np.ones(n, dtype=bool)        # upright and still in place, so far
        tr = {"root_pos_w": [], "root_quat_w": [], "foot_z": []} if args.trace_npz else None
        if tr is not None and args.trace_full:
            tr.update({k: [] for k in ("body_pos_w", "contact_f_w", "torque", "q",
                                       "q_cmd", "foot_u", "swing", "raw_hip",
                                       "cap_hits", "lin_vel_b")})
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
                # Heading error per robot against ITS OWN spawn heading, wrapped to
                # (-180, 180].  Omitting this is not a small mistake: with psi_err left
                # at its default the heading term is identically zero and --heading
                # silently does nothing, which is how the first re-run came back
                # unimproved and looked like the controller had failed.
                yaw_now = quat_to_rpy_deg(snap(robot.data.root_quat_w))[2]
                psi = np.radians((yaw_now - yaw0_deg + 180.0) % 360.0 - 180.0)
                for k in range(n):
                    cmd[k] = cmd[k] + foots[k].step(float(vb[k, 1]), float(wb[k, 2]),
                                                    swing[k][frames[k]], vx=float(vb[k, 0]),
                                                    psi_err_rad=float(psi[k]))
            tgt[:, idx_t] = torch.as_tensor(cmd, device=sim.device, dtype=torch.float32)
            robot.set_joint_position_target(tgt)
            for _ in range(decim):
                robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)

            pos = snap(robot.data.root_pos_w)
            if args.score == "inplace":
                yaw_now = quat_to_rpy_deg(snap(robot.data.root_quat_w))[2]
                yaw_cum += (yaw_now - yaw_prev + 180.0) % 360.0 - 180.0
                yaw_prev = yaw_now
                drift = np.linalg.norm(pos[:, :2] - pos0, axis=1)
                drift_max = np.maximum(drift_max, drift)
                # "legal" latches off: once a robot has fallen or walked away, the yaw it
                # racks up afterwards is not a turn it performed, and crediting it would
                # let a robot that rolls onto its back score a pass.
                legal &= (pos[:, 2] >= FALL_HEIGHT_M) & (drift <= GOAL_RADIUS_M)
                yaw_ok = np.where(legal, np.maximum(yaw_ok, np.abs(yaw_cum)), yaw_ok)
            if video is not None and rep == args.video_rep and step % args.video_stride == 0:
                grab_frame()
            if tr is not None:
                tr["root_pos_w"].append(pos.copy())
                tr["root_quat_w"].append(snap(robot.data.root_quat_w))
                tr["foot_z"].append(snap(robot.data.body_pos_w[:, foot_ids_t, 2]))
            if tr is not None and args.trace_full:
                contacts.update(dt)
                tr["body_pos_w"].append(snap(robot.data.body_pos_w))
                tr["contact_f_w"].append(snap(contacts.data.net_forces_w))
                tr["torque"].append(snap(robot.data.applied_torque[:, idx_t]))
                tr["q"].append(snap(robot.data.joint_pos[:, idx_t]))
                tr["q_cmd"].append(cmd.copy())
                tr["lin_vel_b"].append(snap(robot.data.root_lin_vel_b))
                tr["swing"].append(np.stack([swing[k][frames[k]] for k in range(n)]))
                if foots is not None:
                    tr["foot_u"].append(np.stack([f.last_u for f in foots]))
                    tr["raw_hip"].append(np.stack([f.last_raw_hip for f in foots]))
                    # [lateral hits, swing legs, heading-half at its cap, step-length
                    # half at its cap].  The heading cap is the one lip_failure.md 3
                    # measured at 84-87% and the one the question "can the saturation be
                    # reduced without raising the cap" is about, and it was not in the
                    # trace at all -- only the LATERAL cap was, which is a different cap
                    # on a different term answering a different question.
                    tr["cap_hits"].append(np.array(
                        [[f.last_cap_hits, f.last_swing_n,
                          int(np.sum(np.abs(f.last_u_head) >= (f.heading_cap_rad or f.cap_rad)
                                     - 1e-12)),
                          int(np.sum(np.abs(f.last_u_len) >= f.heading_len_cap_rad - 1e-12))]
                         for f in foots]))
                else:
                    tr["foot_u"].append(np.zeros((n, 12)))
                    tr["raw_hip"].append(np.zeros((n, 4)))
                    tr["cap_hits"].append(np.zeros((n, 4), dtype=int))
            if args.score == "inplace":
                newly = (~reached) & legal & (yaw_ok >= args.inplace_yaw_deg) \
                    & (step <= budget_steps)
            else:
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
                             "reached": int(reached[k] and settle_ok[k]),
                             "settle_ok": int(settle_ok[k]),
                             "t_reached_s": t_reached[k],
                             "fell": int(fell[k]), "final_dist_m": float(dist[k]),
                             "grid_rows": rows, "grid_cols": cols, "n_probes": n,
                             "foot_comp": args.foot_comp, "steps": step + 1,
                             "swing_lift_mm": args.swing_lift,
                             "swing_lift_sym": int(not args.swing_lift_asym),
                             # Stamped because a sweep was once read back without knowing
                             # whether heading hold had been on, and `heading` (the full
                             # formula) and `heading-only` (the heading half) are not the
                             # same controller -- outputs/heading_hold.md measures the full
                             # one as WORSE in both gaits.  argv carried it; no column did.
                             "heading": args.heading,
                             "heading_cap_rad": args.heading_cap,
                             "heading_len": int(args.heading_len),
                             "foot_yaw_bias": args.foot_yaw_bias,
                             "foot_len_bias": args.foot_len_bias,
                             # Same reason as `heading` above: a capture-point row and a
                             # half-stance row are different controllers and the CSV has
                             # to say which one produced the number.
                             "foot_gain": args.foot_gain,
                             "foot_vy_avg_n": args.foot_vy_avg_n,
                             "foot_offset_clip_m": args.foot_offset_clip_m,
                             "foot_com_height_m": (float(com_h[k]) if args.foot_comp != "off"
                                                   and args.foot_gain == "capture" else ""),
                             "heading_len_cap_rad": args.heading_len_cap,
                             # The in-place raw measurements, written on every row so the
                             # 90 deg threshold can be moved afterwards without repeating
                             # the sweep -- and so a 0/5 column can be told apart from a
                             # 0/5 that turned 85 deg and ran out of angle.
                             "score": args.score,
                             "inplace_at": args.inplace_at,
                             "yaw_ok_deg": float(yaw_ok[k]) if args.score == "inplace" else "",
                             "yaw_cum_deg": float(yaw_cum[k]) if args.score == "inplace" else "",
                             "drift_max_m": float(drift_max[k]) if args.score == "inplace" else "",
                             "spawn_x": float(spawns[k][0] - offsets[k][0]),
                             "spawn_z0_m": float(spawn_z0[k]),
                             "run_utc": _RUN_UTC, "argv": " ".join(sys.argv[1:])})
        print(f"[grid]   rep {rep+1}: reached {int(reached.sum())}/{n}, "
              f"fell {int(fell.sum())}/{n}, {step+1} steps")
        if tr is not None and rep == 0:
            out = Path(args.trace_npz)
            out.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(
                out, dt=dt, obstacle_x=float(probes.get("obstacle_x", 4.0)),
                probe=np.array([probes["names"][i] for i in idx]),
                skill=np.array([r[1] for r in runs]),
                level_m=np.array([r[4] for r in runs]),
                spawn=spawns, goal2=goal2, offsets=offsets,
                body_names=np.array(robot.body_names),
                contact_body_names=np.array(contacts.body_names if contacts is not None else []),
                joint_names=np.array([robot.joint_names[j] for j in idx_t.tolist()]),
                foot_body_idx=np.array(foot_ids_t),
                **{k: np.asarray(v) for k, v in tr.items()})
            print(f"[grid] trace -> {out}  ({len(tr['root_pos_w'])} steps x {n} robots)")

    # -- the protocol's own reduction: a level passes only if EVERY repeat passed ----
    if video is not None:
        video["writer"].close()
        print(f"[grid] video: wrote {video['frames']} frames "
              f"({video['frames'] / video['fps']:.2f} s at {video['fps']:.1f} fps) "
              f"-> {args.video}")
    print(f"\n  {'probe':16s} {'skill':6s} {'param m':>8s} {'passed':>10s}  reps")
    for k, (param, skill, family, i, level) in enumerate(runs):
        mine = [r for r in rows_out if r["probe"] == probes["names"][i] and r["param"] == param]
        got = sum(r["reached"] for r in mine)
        print(f"  {probes['names'][i]:16s} {skill:6s} {level:8.3f} {got:5d}/{len(mine):<4d}  "
              + "".join("R" if r["reached"] else ("F" if r["fell"] else ".") for r in mine))
    if args.score == "inplace":
        print(f"\n  R = turned {args.inplace_yaw_deg:g} deg in place, F = fell, "
              f". = neither inside the time budget")
        print(f"  {'probe':16s} {'param m':>8s} {'yaw ok deg':>11s} {'drift m':>8s}   "
              f"(worst repeat)")
        for k, (param, skill, family, i, level) in enumerate(runs):
            mine = [r for r in rows_out if r["probe"] == probes["names"][i]
                    and r["param"] == param]
            print(f"  {probes['names'][i]:16s} {level:8.3f} "
                  f"{min(float(r['yaw_ok_deg']) for r in mine):11.1f} "
                  f"{max(float(r['drift_max_m']) for r in mine):8.3f}")
    else:
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
    ap.add_argument("--swing-lift", type=float, default=0.0,
                    help="raise every swing foot's arc to this apex in mm, the same edit "
                         "scripts/verify_skill_replay.py --swing-lift applies")
    ap.add_argument("--swing-lift-asym", action="store_true",
                    help="per-leg lift amplitude (the original, asymmetric choice) instead "
                         "of one amplitude per mirror pair. For the A/B only.")
    ap.add_argument("--probes", default=None,
                    help="probe archive to run (default: the pinned 42-probe one). The v2 "
                         "archive appends the slope and roughness families AFTER those 42, "
                         "so probe indices 0-41 are the same terrain in both")
    ap.add_argument("--families", nargs="*", default=None,
                    help="run these probe families instead of resolving them from --params; "
                         "requires --skill. The off-map path, for a skill that no "
                         "CALIBRATION_MAP entry pairs with a family (TURN has none)")
    ap.add_argument("--skill", default=None,
                    help="hold this clip on every selected probe; used with --families")
    ap.add_argument("--score", choices=("goal", "inplace"), default="goal",
                    help="'goal' = reach goal 2 past the obstacle (the traversal families' "
                         "own test, unchanged). 'inplace' = spawn ON the obstacle and score "
                         "completed yaw without falling or leaving a 0.35 m circle -- the "
                         "only question a skill that does not translate can be asked")
    ap.add_argument("--inplace-yaw-deg", type=float, default=INPLACE_YAW_DEG,
                    help="yaw an --score inplace repeat must complete to pass")
    ap.add_argument("--inplace-at", choices=("obstacle", "spawn"), default="obstacle",
                    help="'obstacle' stands the robot on the obstacle. 'spawn' leaves it at "
                         "the probe's own flat run-up 3 m short of it -- the FLAT CONTROL, "
                         "identical in every other respect, which is what says whether a "
                         "failed in-place score is the terrain or the skill")
    ap.add_argument("--foot-gain", choices=("half-stance", "capture"), default="half-stance",
                    help="the Raibert leading term. half-stance (default, every earlier run): "
                         "T_stance/2, measured off the clip's own contact channel. capture: "
                         "sqrt(com_height/g), the LIP time constant quadruped_pympc uses -- a "
                         "property of the body rather than of the gait. On TROT the two are "
                         "0.186 s and 0.178 s, so this flag is NOT where the two laws differ; "
                         "it is separated from the other two so that can be shown rather than "
                         "assumed")
    ap.add_argument("--foot-vy-avg-n", type=int, default=0,
                    help="moving average on the lateral velocity the law reads, in control "
                         "steps (0 = off, the default). quadruped_pympc averages v over 20 "
                         "samples before differencing against v_ref; the yaw RATE has been "
                         "averaged over a cycle here since stage 2 and the velocity never was")
    ap.add_argument("--foot-offset-clip-m", type=float, default=0.0,
                    help="clip the foot OFFSET in metres instead of the hip angle in radians "
                         "(0 = off, the default). quadruped_pympc bounds the offset at "
                         "+-0.05 m; --foot-clip-rad 0.05 through a 0.31 m lever is +-0.0155 m, "
                         "a third of the authority for the same-looking number")
    ap.add_argument("--foot-yaw-bias", type=float, default=0.0,
                    help="CONSTANT differential lateral placement, +front/-rear on the hips "
                         "(rad). Open loop, added after the feedback term and not clipped by "
                         "--foot-clip-rad. heading_candidates.md 2 measured -0.02 as the "
                         "largest TROT survives; WALK takes +-0.04")
    ap.add_argument("--foot-len-bias", type=float, default=0.0,
                    help="CONSTANT differential step length, +left/-right on the thighs (rad). "
                         "Same measurement: -0.04 removes 4.71 of TROT's 5.32 deg/s for 1.2% "
                         "of speed; -0.06 falls at 2.77 s and +0.02 destroys the gait")
    ap.add_argument("--heading-len", action="store_true",
                    help="add the step-length (thigh) half of the heading law alongside the "
                         "lateral (hip) one. One-sided by construction -- see sim/footcomp.py")
    ap.add_argument("--heading-len-cap", type=float, default=0.04,
                    help="magnitude bound on that half (rad). 0.04 is the largest amplitude "
                         "the open-loop probe survived on TROT; -0.06 fell at 2.77 s")
    ap.add_argument("--heading", choices=("off", "heading", "heading-only"), default="off",
                    help="hold the spawn heading with differential lateral foot placement")
    ap.add_argument("--skip-unsupported", action="store_true",
                    help="drop probes whose skill the low level cannot execute (RUN/JUMP)")
    ap.add_argument("--results-csv", default=None)
    ap.add_argument("--heading-cap", type=float, default=0.0,
                    help="override the per-skill heading cap (rad) for every skill. 0 = use "
                         "the HEADING_CAP table. For measuring whether a cap is what binds.")
    ap.add_argument("--video", default=None,
                    help="write an mp4 side view of ONE cell. NEEDS A GPU (this renders); "
                         "with GPU=none the run hangs in carb.cudainterop rather than "
                         "failing. Physics and control rate are unchanged -- the check is "
                         "that the recorded run terminates where the unrecorded one did.")
    ap.add_argument("--video-cell", type=int, default=0,
                    help="which cell of the grid to follow (0-based, plan order)")
    ap.add_argument("--video-rep", type=int, default=0, help="which repeat to record")
    ap.add_argument("--video-width", type=int, default=960)
    ap.add_argument("--video-height", type=int, default=540)
    ap.add_argument("--video-stride", type=int, default=1)
    ap.add_argument("--video-fps", type=float, default=None,
                    help="force the output frame rate. Half of 1/dt gives half speed.")
    ap.add_argument("--video-side-m", type=float, default=2.4,
                    help="camera offset from the cell's own centre line, metres")
    ap.add_argument("--trace-full", action="store_true",
                    help="record contact-force VECTORS on every link, joint torques, per-link "
                         "positions and the per-step foot-placement correction, not just the "
                         "root pose. For diagnosing a failure, not for scoring a grid.")
    ap.add_argument("--trace-npz", default=None,
                    help="dump rep 0's per-step base pose and foot heights for every "
                         "robot, so a failure can be located on the probe rather than "
                         "inferred from the verdict")
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
    if args.probes:
        import run_calibration
        run_calibration.PROBES_NPZ = Path(args.probes)
        print(f"[grid] probe archive: {args.probes}")
    if bool(args.families) != bool(args.skill):
        raise SystemExit("[grid] --families and --skill are used together")
    if args.score == "inplace" and not args.skill:
        raise SystemExit("[grid] --score inplace needs --skill (and --families): it is the "
                         "off-map path, and no CALIBRATION_MAP entry describes it")

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
