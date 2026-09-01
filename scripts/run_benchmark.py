#!/usr/bin/env python3
"""Score a skill on the frozen benchmark, with upstream's own goal rule.

    # the single-skill lower bound CLAUDE.md 2 requires, all 200 cells
    scripts/isaac_docker_run.sh scripts/run_benchmark.py --headless --device cpu --skill TROT

    # one task, for wiring only -- the score it prints is NOT comparable
    scripts/isaac_docker_run.sh scripts/run_benchmark.py --headless --device cpu \
        --skill TROT --tasks 0 --allow-partial

    # anywhere, no Isaac Lab
    python3 scripts/run_benchmark.py --plan
    python3 scripts/run_benchmark.py --self-test

What this is
------------
``data/benchmark_frozen.npz`` has never been simulated.  Every script that opens it
reads it with numpy: the offline planner sweep, the planner's feature extractor, the
freezer itself.  This is the path from the archive into Isaac Lab, and it scores the
result the way ``eurekaverse``'s ``evaluate.py`` does so that a number here and the
4.48 in ``docs/RUN_RESULTS.md`` are the same quantity.

The scoring rule, read off `extreme-parkour/legged_gym`, and why each part matters
-------------------------------------------------------------------------------
* A goal is REACHED when the base's xy distance to it is under ``next_goal_threshold``
  = 0.20 m and has been for ``reach_goal_delay`` = 0.1 s.  Goals are consumed IN ORDER,
  0 through 7.  **Forward progress in x is not the rule and must not be substituted for
  it**: the benchmark smoke measured a robot 9.13 m outside a 3.95 m lane as having
  "passed" 5 of 8 goals on an x-progress proxy.
* An episode ends on timeout (``episode_length_s`` = 20 s), on \|roll\| or \|pitch\|
  over 1.5 rad, on base z below -0.25 m, or on all 8 goals reached.  The score of an
  episode is the goal index it ended on, 0 to 8.
* A CELL's score is the mean over its episodes.  The RUN's score is the mean over
  cells, **each cell weighted equally** -- 20 tasks x 10 levels, not per robot.

Two guards, both from things that have already gone wrong
---------------------------------------------------------
* ``TerrainImporter(terrain_type="plane")`` lays a 2000 km ground plane at z = 0 UNDER
  the imported mesh.  On the calibration probes that floored every pit and produced a
  60/60 pass rate that was a robot walking on an invisible floor.  On the BENCHMARK it
  is worse: 12 of the 20 tasks carry -1.0 m pits over 16-48% of their cell area.  The
  plane is deleted and its absence is *verified*; the run refuses to start otherwise.
* The score is only comparable if every cell exists.  ``evaluate.py`` clamps its env
  count up to the grid size and then ``helpers.py`` overwrites it from ``--num_envs``,
  so asking for fewer envs than cells silently shrinks the denominator.  Here a run
  with fewer than ``tasks x levels`` cells is REFUSED unless ``--allow-partial``, and a
  partial run stamps every row so its mean can never be read as a benchmark score.

What this does NOT do
---------------------
It drives ONE clip, chosen on the command line: the single-skill lower bound.  The rule
planner's own choice is not wired in here -- ``run_planner_replay.py`` owns that -- and
neither is depth.  The three arms of the experiment need the same scorer and this is it.
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
from terrain_toolkit.paths import FROZEN_NPZ, SKILL_CLIPS_META_JSON

_SIM_APP = None

#: Upstream constants, from extreme-parkour/legged_gym/envs/base/legged_robot_config.py
#: and legged_robot.py.  Named here so a divergence is visible rather than implicit.
NEXT_GOAL_THRESHOLD_M = 0.20      # LeggedRobotCfg.env.next_goal_threshold
REACH_GOAL_DELAY_S = 0.1          # LeggedRobotCfg.env.reach_goal_delay
EPISODE_LENGTH_S = 20.0           # LeggedRobotCfg.env.episode_length_s
ROLL_PITCH_CUTOFF_RAD = 1.5       # legged_robot.check_termination
HEIGHT_CUTOFF_M = -0.25           # legged_robot.check_termination
NUM_GOALS = 8


class GoalTracker:
    """Upstream's goal rule, vectorised over cells and testable without a simulator.

    One instance per run.  ``update(pos_xy, dt)`` is called once per control step and
    returns the current goal index per cell, 0..NUM_GOALS.
    """

    def __init__(self, goals: np.ndarray, dt: float,
                 threshold_m: float = NEXT_GOAL_THRESHOLD_M,
                 delay_s: float = REACH_GOAL_DELAY_S):
        self.goals = np.asarray(goals, dtype=float)     # (n, NUM_GOALS, 2) world xy
        self.n = self.goals.shape[0]
        self.dt = float(dt)
        self.threshold = float(threshold_m)
        # Upstream counts control steps: inc when timer > delay/dt.  Same integer
        # comparison here rather than a float time, so the boundary lands identically.
        self.delay_steps = delay_s / self.dt
        self.reset()

    def reset(self) -> None:
        self.idx = np.zeros(self.n, dtype=int)
        self.timer = np.zeros(self.n, dtype=float)

    def update(self, pos_xy: np.ndarray) -> np.ndarray:
        # Order matches _update_goals: the increment is decided on the PREVIOUS step's
        # timer, then the timer is advanced against the (possibly new) current goal.
        inc = self.timer > self.delay_steps
        self.idx = np.where(inc & (self.idx < NUM_GOALS), self.idx + 1, self.idx)
        self.timer = np.where(inc, 0.0, self.timer)
        cur = self.goals[np.arange(self.n), np.minimum(self.idx, NUM_GOALS - 1)]
        near = np.linalg.norm(np.asarray(pos_xy, dtype=float) - cur, axis=1) < self.threshold
        # A cell that has consumed all eight goals is finished; it must not keep
        # ticking on goal 7 and roll the index past NUM_GOALS.
        near &= self.idx < NUM_GOALS
        self.timer = np.where(near, self.timer + 1.0, self.timer)
        return self.idx


def cell_list(z, tasks, levels):
    names = [str(x) for x in z["task_names"]]
    ts = list(range(len(names))) if tasks is None else list(tasks)
    ls = list(range(int(z["num_rows"]))) if levels is None else list(levels)
    return [(t, l) for t in ts for l in ls], names, ts, ls


def build_grid(z, cells, gutter: float = 1.0):
    """One mesh holding every cell, plus each cell's world spawn and goals."""
    hf_all = z["height_fields_before_fix"]
    goals_all = z["goals_before_fix"]
    hs, vs = float(z["horizontal_scale"]), float(z["vertical_scale"])
    nx, ny = hf_all.shape[2], hf_all.shape[3]
    Lx, Ly = (nx - 1) * hs, (ny - 1) * hs
    gap_x, gap_y = Lx * gutter, Ly * gutter
    ncols = int(z["num_rows"])                 # one column per difficulty level
    V, F, spawns, goals, offs = [], [], [], [], []
    groups, nf = [], 0                          # (vertex_start, face_start) per task
    last_task = None
    nv = 0
    sx, sy = float(z["spawn_x"]), float(z["spawn_y"])
    for k, (t, l) in enumerate(cells):
        if t != last_task:
            groups.append((nv, nf))
            last_task = t
        r, c = divmod(k, ncols)
        ox, oy = c * (Lx + gap_x), r * (Ly + gap_y)
        v, f = HF.to_trimesh(hf_all[t, l], hs, vs)
        V.append(v + np.array([ox, oy, 0.0]))
        F.append(f + nv)
        nv += len(v)
        nf += len(f)
        offs.append((ox, oy))
        spawns.append((sx + ox, sy + oy))
        goals.append(goals_all[t, l, :, :2] + np.array([ox, oy]))
    rows = (len(cells) + ncols - 1) // ncols
    bounds = [(groups[i], groups[i + 1] if i + 1 < len(groups) else (nv, nf))
              for i in range(len(groups))]
    return (np.concatenate(V), np.concatenate(F), np.asarray(spawns, float),
            np.asarray(goals, float), np.asarray(offs, float), (rows, ncols), (Lx, Ly),
            bounds)


def aggregate(rows: list) -> dict:
    """Per-cell episode mean, then the equal-weight mean over cells."""
    per_cell = {}
    for r in rows:
        per_cell.setdefault((r["task"], r["level"]), []).append(r["goals"])
    cell_means = {k: float(np.mean(v)) for k, v in per_cell.items()}
    by_task, by_level = {}, {}
    for (t, l), v in cell_means.items():
        by_task.setdefault(t, []).append(v)
        by_level.setdefault(l, []).append(v)
    return {
        "score": float(np.mean(list(cell_means.values()))) if cell_means else float("nan"),
        "n_cells": len(cell_means),
        "n_episodes": len(rows),
        "cell_means": cell_means,
        "per_task": {t: float(np.mean(v)) for t, v in by_task.items()},
        "per_level": {l: float(np.mean(v)) for l, v in by_level.items()},
    }


def plan_report(args) -> str:
    z = np.load(FROZEN_NPZ, allow_pickle=False)
    cells, names, ts, ls = cell_list(z, args.tasks, args.levels)
    full = len(z["task_names"]) * int(z["num_rows"])
    hs = float(z["horizontal_scale"])
    Lx = (z["height_fields_before_fix"].shape[2] - 1) * hs
    Ly = (z["height_fields_before_fix"].shape[3] - 1) * hs
    ncols = int(z["num_rows"])
    rows = (len(cells) + ncols - 1) // ncols
    L = [f"frozen archive: {FROZEN_NPZ.name}, {len(names)} tasks x {int(z['num_rows'])} levels "
         f"= {full} cells",
         f"selected: {len(cells)} cells ({len(ts)} tasks x {len(ls)} levels)",
         f"grid {rows} x {ncols}, cell {Lx:.1f} x {Ly:.1f} m, gutter {args.gutter:g} cell(s)",
         f"footprint {ncols*Lx*(1+args.gutter):.0f} x {rows*Ly*(1+args.gutter):.0f} m",
         f"episodes {args.episodes} x {EPISODE_LENGTH_S:.0f} s, skill {args.skill}",
         f"goal rule: within {NEXT_GOAL_THRESHOLD_M:.2f} m for {REACH_GOAL_DELAY_S:.2f} s, "
         f"{NUM_GOALS} goals in order",
         ""]
    if len(cells) < full:
        L += [f"  *** PARTIAL: {len(cells)} of {full} cells. The mean over a subset is NOT",
              f"      the benchmark score and every row will be stamped partial=1. ***", ""]
    return "\n".join(L)


def self_test() -> int:
    """The goal rule and the aggregation, with no simulator."""
    fails = 0

    def ok(label, cond):
        nonlocal fails
        fails += 0 if cond else 1
        print(f"  {'ok  ' if cond else 'FAIL'} {label}")

    dt = 0.02
    g = np.zeros((1, NUM_GOALS, 2))
    g[0, :, 0] = np.arange(1, NUM_GOALS + 1) * 1.0       # goals at x = 1..8, y = 0
    gt = GoalTracker(g, dt)

    # 1. sitting exactly on goal 0 for less than the delay does NOT consume it
    for _ in range(int(REACH_GOAL_DELAY_S / dt) - 1):
        gt.update(np.array([[1.0, 0.0]]))
    ok("under the dwell time, no goal is consumed", gt.idx[0] == 0)
    # 2. holding past it does
    for _ in range(3):
        gt.update(np.array([[1.0, 0.0]]))
    ok("past the dwell time, goal 0 is consumed", gt.idx[0] == 1)

    # 3. THE PROXY THE SMOKE FAILED ON: far off the lane but past every goal in x
    gt2 = GoalTracker(g, dt)
    for _ in range(2000):
        gt2.update(np.array([[9.0, 9.13]]))              # x past all 8, y 9.13 m aside
    ok("x-progress with 9.13 m of lateral offset scores 0", gt2.idx[0] == 0)

    # 4. goals are consumed in order and the index stops at NUM_GOALS
    gt3 = GoalTracker(g, dt)
    hold = int(REACH_GOAL_DELAY_S / dt) + 2
    for j in range(NUM_GOALS):
        for _ in range(hold):
            gt3.update(np.array([[float(j + 1), 0.0]]))
    ok(f"eight goals in order -> index {NUM_GOALS}", gt3.idx[0] == NUM_GOALS)
    for _ in range(hold):
        gt3.update(np.array([[8.0, 0.0]]))
    ok("the index does not run past NUM_GOALS", gt3.idx[0] == NUM_GOALS)

    # 5. skipping goal 0 and standing on goal 3 consumes NOTHING
    gt4 = GoalTracker(g, dt)
    for _ in range(500):
        gt4.update(np.array([[4.0, 0.0]]))
    ok("standing on goal 3 without goal 0 scores 0", gt4.idx[0] == 0)

    # 6. aggregation: cells weighted equally, not episodes
    rows = ([{"task": 0, "level": 0, "goals": 8}] * 10
            + [{"task": 0, "level": 1, "goals": 0}]
            + [{"task": 1, "level": 0, "goals": 4}])
    a = aggregate(rows)
    ok("three cells, equal weight -> (8 + 0 + 4)/3 = 4.0",
       abs(a["score"] - 4.0) < 1e-12 and a["n_cells"] == 3)
    ok("ten episodes in one cell do not outvote the other two", a["n_episodes"] == 12)

    # 7. the grid places every cell disjointly and keeps its own heights
    z = np.load(FROZEN_NPZ, allow_pickle=False)
    cells = [(t, l) for t in range(3) for l in range(10)]
    V, F, spawns, goals, offs, (rows_, cols_), (Lx, Ly), grp = build_grid(z, cells)
    ok(f"grid {rows_}x{cols_} holds {len(cells)} cells", rows_ * cols_ >= len(cells))
    ok("faces index inside the mesh", int(F.max()) < len(V))
    per = z["height_fields_before_fix"][0, 0].size
    ok("heights survive the translation",
       np.allclose(V[:per, 2],
                   z["height_fields_before_fix"][0, 0].ravel() * float(z["vertical_scale"])))
    dx = np.abs(offs[:, None, 0] - offs[None, :, 0])
    dy = np.abs(offs[:, None, 1] - offs[None, :, 1])
    overlap = (dx < Lx - 1e-9) & (dy < Ly - 1e-9)
    np.fill_diagonal(overlap, False)
    ok("cells are disjoint", not overlap.any())
    ok("every cell has 8 goals, offset with it", goals.shape == (len(cells), NUM_GOALS, 2))
    # the collider split must cover the mesh exactly once, or a task silently has no floor
    ok(f"{len(grp)} colliders, one per task, tiling the mesh with no gap or overlap",
       len(grp) == 3 and grp[0][0][0] == 0 and grp[-1][1] == (len(V), len(F))
       and all(grp[i][1] == grp[i + 1][0] for i in range(len(grp) - 1)))
    ok("the largest collider is under 1 M triangles",
       max(hi[1] - lo[1] for lo, hi in grp) < 1_000_000)
    ok("goal 0 of cell 0 is the archive's, untranslated",
       np.allclose(goals[0, 0], z["goals_before_fix"][0, 0, 0, :2]))

    print(f"\nbenchmark self-test: {'PASS' if fails == 0 else f'{fails} FAILURES'}")
    return 0 if fails == 0 else 1


#: Per-skill heading cap in radians, from the open-loop steering probe
#: (outputs/heading_candidates.md 2): WALK took +-0.04 rad with no measurable cost, TROT
#: falls in BOTH directions at +-0.04 and is safe at +-0.02.  The same table
#: run_calibration_grid.py and run_planner_replay.py carry, and it has to stay the same
#: table -- a benchmark run and a calibration run at different caps are different
#: controllers being compared as though they were one.
HEADING_CAP = {"WALK": 0.04, "TROT": 0.02}


def run_isaac(args) -> tuple:
    from isaaclab.app import AppLauncher

    global _SIM_APP
    _SIM_APP = AppLauncher(args).app

    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sim import SimulationCfg, SimulationContext
    from isaaclab.terrains import TerrainImporter, TerrainImporterCfg
    import omni.usd
    from pxr import Gf, UsdGeom
    import trimesh

    from sim import isaac_cfg as IC
    from sim.replay import (foot_body_ids, ground_material_cfg, quat_to_rpy_deg,
                            set_robot_friction, snap)
    from sim.footcomp import FootPlacement, stance_time_s
    from verify_skill_replay import (_log_motion_for, load_clip, level_start,
                                     quiescent_start, rotate_clip)

    z = np.load(FROZEN_NPZ, allow_pickle=False)
    cells, names, ts, ls = cell_list(z, args.tasks, args.levels)
    n = len(cells)
    full = len(names) * int(z["num_rows"])
    if n < full and not args.allow_partial:
        raise SystemExit(
            f"[bench] refusing to run: {n} of {full} cells selected. A mean over a subset "
            f"of the grid is not the benchmark score -- upstream weights all {full} cells "
            f"equally, so a missing cell silently changes the denominator (this is the same "
            f"trap evaluate.py's num_envs clamp has, defeated by helpers.py). Pass "
            f"--allow-partial to run anyway; every row will be stamped partial=1 and the "
            f"printed mean is a wiring check, not a score.")
    print(plan_report(args))

    ucfg = IC.load()
    sys.path.insert(0, str(Path(ucfg.source).parents[3]))
    from legged_gym.envs.base.legged_robot_config import UNITREE_GO2_CFG

    phys_dt = float(ucfg.sim_dt)
    decim = int(ucfg.decimation or 4)
    dt = decim * phys_dt
    sim = SimulationContext(SimulationCfg(dt=phys_dt, device=args.device))

    V, F, spawns, goals, offs, (rows, cols), (Lx, Ly), mesh_groups = build_grid(
        z, cells, args.gutter)
    ti = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane",
                            physics_material=ground_material_cfg(sim_utils),
                            num_envs=n, env_spacing=0.0)
    importer = TerrainImporter(ti)
    # ONE MESH PER TASK, not one for the whole grid.  All 200 cells concatenated is
    # 5.76 M vertices / 11.34 M triangles, and PhysX accepted that mesh, cooked it, and
    # then collided with nothing: every robot fell through during the settle and the
    # episode ended on the first control step with 0 goals on all 200 cells.  Nothing
    # errored.  Split by task it is 20 colliders of ~567 k triangles and the same cells
    # collide normally.  The split changes no geometry -- each cell's vertices are
    # identical either way, only which collider owns them differs.
    for gi, (lo, hi) in enumerate(mesh_groups):
        sub_v, sub_f = V[lo[0]:hi[0]], F[lo[1]:hi[1]] - lo[0]
        importer.import_mesh(f"benchmark_{gi}", trimesh.Trimesh(vertices=sub_v, faces=sub_f))
    # THE GUARD.  See the module docstring: terrain_type="plane" lays a 2000 km plane at
    # z = 0 under the mesh, and 12 of the 20 benchmark tasks are 16-48% pit.  Deleting it
    # is not enough -- the deletion is verified, because a silently floored benchmark
    # produces plausible high scores and nothing about them looks wrong.
    plane_path = ti.prim_path + "/terrain"
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(plane_path).IsValid():
        stage.RemovePrim(plane_path)
    if stage.GetPrimAtPath(plane_path).IsValid():
        raise SystemExit(f"[bench] refusing to run: the ground plane at {plane_path} is "
                         f"still on the stage. 12 of the 20 tasks carry -1.0 m pits over "
                         f"16-48% of their area and every one of them would be floored.")
    pit = int((z["height_fields_before_fix"][ts] < 0).any(axis=(1, 2, 3)).sum())
    print(f"[bench] removed the importer's infinite ground plane at {plane_path}; the "
          f"{pit} of {len(ts)} selected tasks that contain pits are open")
    print(f"[bench] terrain: {len(V)} vertices, {len(F)} faces, {n} cells in {rows}x{cols}")

    sim_utils.DomeLightCfg(intensity=2000.0).func("/World/light",
                                                  sim_utils.DomeLightCfg(intensity=2000.0))
    for k in range(n):
        xf = UsdGeom.Xform.Define(stage, f"/World/envs/env_{k}")
        xf.AddTranslateOp().Set(Gf.Vec3d(float(spawns[k][0]), float(spawns[k][1]), 0.0))
    robot = Articulation(UNITREE_GO2_CFG.replace(prim_path="/World/envs/env_.*/Robot"))
    sim.reset()
    if robot.num_instances != n:
        raise SystemExit(f"[bench] {robot.num_instances} robots for {n} cells")
    if ucfg.ground_friction is not None:
        set_robot_friction(robot, ucfg.ground_friction)

    clip = load_clip(args.skill, args.rate)
    # --heading, resolved once.  The cap is the skill's own, from the open-loop steering
    # probe (outputs/heading_candidates.md 2): WALK took +-0.04 rad with no measurable
    # cost, TROT falls in BOTH directions at +-0.04 and is safe at +-0.02.  TURN is not
    # trying to hold a heading -- it is trying to change one -- so it is 0, and asking
    # for it there is refused rather than silently ignored.
    yaw_mode, hcap, vy_log, wz_log = "off", 0.0, 0.0, 0.0
    if args.heading != "off":
        hcap = (args.heading_cap if args.heading_cap is not None
                else HEADING_CAP.get(args.skill, 0.0))
        if hcap <= 0.0:
            raise SystemExit(
                f"[bench] --heading was asked for on {args.skill}, whose measured heading "
                f"cap is {HEADING_CAP.get(args.skill)}. Running with a cap of 0 would make "
                f"the term identically zero while stamping every row heading={args.heading} "
                f"-- a whole benchmark that says it held a heading and did not. Pass "
                f"--heading-cap explicitly if that is really what is wanted.")
        yaw_mode = args.heading
        vy_log, wz_log = _log_motion_for(
            json.loads(SKILL_CLIPS_META_JSON.read_text()), clip["name"])
    idx_by_name = {nm: i for i, nm in enumerate(robot.joint_names)}
    want = [f"{l}_{j}_joint" for l in clip["leg_order"] for j in clip["joint_order"]]
    idx_t = torch.as_tensor([idx_by_name[w] for w in want], device=sim.device,
                            dtype=torch.long)
    # Entry phase, by the same rules the replay harness offers, so a benchmark row and a
    # flat row are the same configuration.
    if args.start_phase == "measured":
        from planner.config import DEFAULT as _CFG
        kf = int(getattr(_CFG.skill, f"ENTRY_FRAME_{clip['name']}", -1))
        if kf < 0:
            kf, _ = level_start(clip, robot, sim, idx_t, phys_dt)
            print(f"[bench] no measured entry frame for {clip['name']}; used the "
                  f"coplanarity rule -> frame {kf}")
        else:
            print(f"[bench] *** MEASURED ENTRY PHASE frame {kf} for {clip['name']} "
                  f"(planner.config.skill.ENTRY_FRAME_{clip['name']}) ***")
    elif args.start_phase == "level":
        kf, _ = level_start(clip, robot, sim, idx_t, phys_dt)
    elif args.start_phase == "stance":
        kf, _ = quiescent_start(clip)
    else:
        kf = 0
    clip = rotate_clip(clip, kf)
    q_seq = np.asarray(clip["q_des"], dtype=np.float32)
    swing_seq = ~np.asarray(clip["contact"], dtype=bool)
    ncyc = len(q_seq)

    ground_z = np.array([HF.height_at(z["height_fields_before_fix"][t, l],
                                      float(z["horizontal_scale"]),
                                      float(z["vertical_scale"]),
                                      float(z["spawn_x"]), float(z["spawn_y"]))
                         for (t, l) in cells])
    root0 = robot.data.default_root_state.clone()
    root0[:, 0] = torch.as_tensor(spawns[:, 0], device=sim.device, dtype=root0.dtype)
    root0[:, 1] = torch.as_tensor(spawns[:, 1], device=sim.device, dtype=root0.dtype)
    root0[:, 2] = torch.as_tensor(ground_z + args.spawn_z, device=sim.device,
                                  dtype=root0.dtype)
    q_stand = robot.data.default_joint_pos.clone()
    q_first = q_stand.clone()
    q_first[:, idx_t] = torch.as_tensor(q_seq[0], device=sim.device, dtype=torch.float32)

    def hold(target, m):
        for _ in range(m):
            robot.set_joint_position_target(target)
            for _ in range(decim):
                robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)

    n_settle = max(int(args.settle_s / dt), 1)
    ep_steps = int(EPISODE_LENGTH_S / dt)
    tracker = GoalTracker(goals, dt)
    rows_out = []
    t0 = time.time()
    for ep in range(args.episodes):
        robot.write_root_state_to_sim(root0)
        robot.write_joint_state_to_sim(q_stand, torch.zeros_like(q_stand))
        hold(q_stand, n_settle)
        for j in range(1, n_settle + 1):
            hold(q_stand * (1 - j / n_settle) + q_first * (j / n_settle), 1)
        tracker.reset()

        bp = snap(robot.data.body_pos_w)
        foot_ids, foot_names = foot_body_ids(robot)
        hip_ids, hip_names = robot.find_bodies(".*_hip")
        f_by = {nm.split("_")[0]: j for j, nm in zip(foot_ids, foot_names)}
        h_by = {nm.split("_")[0]: j for j, nm in zip(hip_ids, hip_names)}
        legs = clip["leg_order"]
        lever = np.array([[bp[k, h_by[l], 2] - bp[k, f_by[l], 2] for l in legs]
                          for k in range(n)])
        settle_ok = np.all(lever > 0.05, axis=1)
        # THE SECOND GUARD.  The hip-to-foot lever stays perfectly valid while a robot is
        # in free fall -- the legs keep their geometry -- so `settle_ok` alone cannot tell
        # a robot standing on the terrain from one that went through it.  The 200-cell
        # mesh did exactly that: 11.34 M triangles cooked without error, collided with
        # nothing, and every cell scored 0 on the first control step.  Base height above
        # the cell's own ground is the quantity that separates the two.
        base_h = snap(robot.data.root_pos_w)[:, 2] - ground_z
        through = base_h < 0.15
        if through.any():
            bad = np.where(through)[0]
            raise SystemExit(
                f"[bench] refusing to score: {len(bad)} of {n} robots are below 0.15 m over "
                f"their own cell's ground after the settle (lowest {base_h.min():+.3f} m, "
                f"e.g. cell {cells[bad[0]]}). They are inside or under the terrain, not on "
                f"it, and every one of them would score 0 for a reason that is not the "
                f"skill's. Check the collider: one mesh per task is what makes 200 cells "
                f"collide (see build_grid).")
        med = np.median(lever[settle_ok], axis=0) if settle_ok.any() else np.full(4, 0.31)
        # The heading each robot is asked to hold: the one IT settled at, per cell.  Not
        # a shared constant and not the cell's axis -- a robot that settles pointing 3 deg
        # off would otherwise be handed a 3 deg error at t=0 and asked to correct terrain
        # it cannot see.  The reference is proprioceptive and per robot, the same rule
        # run_calibration_grid.py already uses.
        _, _, yaw0 = quat_to_rpy_deg(snap(robot.data.root_quat_w))
        yaw_ref = np.asarray(yaw0, dtype=float)
        foots = None
        if args.foot_comp != "off":
            from sim.replay import quat_rotate_inv
            bq = snap(robot.data.root_quat_w)
            bpos = snap(robot.data.root_pos_w)
            foots = []
            for k in range(n):
                hxy = np.array([quat_rotate_inv(bq[k][None, :],
                                                (bp[k, h_by[l]] - bpos[k])[None, :])[0, :2]
                                for l in legs])
                foots.append(FootPlacement(
                    t_stance_s=stance_time_s(clip["contact"], clip["fs"]),
                    lever_m=lever[k] if settle_ok[k] else med,
                    hip_x_m=hxy[:, 0], hip_y_m=hxy[:, 1],
                    cap_rad=args.foot_clip_rad, cycle_len=ncyc,
                    yaw_mode=yaw_mode, heading_cap_rad=hcap,
                    vy_log=vy_log, wz_log=wz_log))
            print(f"[bench] *** FOOT PLACEMENT ON (cap {args.foot_clip_rad:g} rad): closes a "
                  f"loop on base lateral velocity and overwrites the recording. ***")
            if yaw_mode != "off":
                print(f"[bench] *** HEADING HOLD --heading {args.heading} on "
                      f"{args.skill}, cap {hcap:g} rad: each robot holds the heading it "
                      f"settled at. omega_target = omega_log - psi_err/T_stance; T_stance "
                      f"cancels, so the term carries no constant. The 0.98/1.11 scores in "
                      f"outputs/benchmark_harness.md were measured WITHOUT this and are "
                      f"not comparable to a run with it. ***")

        alive = np.ones(n, dtype=bool)
        ended = np.zeros(n, dtype=int)          # goal index at the moment it ended
        for step in range(ep_steps):
            frame = step % ncyc
            cmd = np.tile(q_seq[frame], (n, 1))
            if foots is not None:
                vb = snap(robot.data.root_lin_vel_b)
                wb = snap(robot.data.root_ang_vel_b)
                # Heading error per robot against ITS OWN settle heading, wrapped to
                # (-180, 180].  Omitting this is not a small mistake: psi_err defaults to
                # 0, which makes the heading term identically zero and --heading a flag
                # that changes a CSV column and nothing else.  run_calibration_grid.py
                # carries the same comment for the same reason.
                if yaw_mode != "off":
                    _, _, yaw_now = quat_to_rpy_deg(snap(robot.data.root_quat_w))
                    psi = np.radians((np.asarray(yaw_now, float) - yaw_ref + 180.0)
                                     % 360.0 - 180.0)
                else:
                    psi = np.zeros(n)
                for k in range(n):
                    if alive[k]:
                        cmd[k] = cmd[k] + foots[k].step(float(vb[k, 1]), float(wb[k, 2]),
                                                        swing_seq[frame], vx=float(vb[k, 0]),
                                                        psi_err_rad=float(psi[k]))
            tgt = robot.data.default_joint_pos.clone()
            tgt[:, idx_t] = torch.as_tensor(cmd, device=sim.device, dtype=torch.float32)
            robot.set_joint_position_target(tgt)
            for _ in range(decim):
                robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)

            pos = snap(robot.data.root_pos_w)
            idx = tracker.update(pos[:, :2])
            roll, pitch, _ = quat_to_rpy_deg(snap(robot.data.root_quat_w))
            dead = ((np.abs(np.radians(roll)) > ROLL_PITCH_CUTOFF_RAD)
                    | (np.abs(np.radians(pitch)) > ROLL_PITCH_CUTOFF_RAD)
                    | (pos[:, 2] - ground_z < HEIGHT_CUTOFF_M)
                    | (idx >= NUM_GOALS))
            newly = alive & dead
            ended[newly] = idx[newly]
            alive &= ~newly
            if not alive.any():
                break
        ended[alive] = tracker.idx[alive]        # the rest time out, upstream's way
        for k, (t, l) in enumerate(cells):
            rows_out.append({"task": t, "task_name": names[t], "level": l, "episode": ep,
                             "goals": int(ended[k]), "settle_ok": int(settle_ok[k]),
                             # Which arm this row is.  A results file whose rows do not
                             # say whether heading hold was on is one that gets read wrong
                             # once: `heading` and `heading-only` are not the same
                             # controller, and neither is `off`.
                             "heading": args.heading, "heading_cap_rad": hcap,
                             "partial": int(n < full), "skill": args.skill,
                             "start_phase": args.start_phase, "foot_comp": args.foot_comp,
                             "steps": step + 1})
        print(f"[bench] episode {ep+1}/{args.episodes}: goals reached "
              f"min {ended.min()} median {np.median(ended):.1f} max {ended.max()}, "
              f"{int((ended >= NUM_GOALS).sum())}/{n} cells completed the course "
              f"({time.time()-t0:.0f}s)")
    return rows_out, aggregate(rows_out), n < full


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skill", default="TROT", help="clip to hold on every cell")
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo")
    ap.add_argument("--tasks", type=int, nargs="*", default=None)
    ap.add_argument("--levels", type=int, nargs="*", default=None)
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--gutter", type=float, default=1.0)
    ap.add_argument("--spawn-z", type=float, default=0.42)
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--start-phase", choices=("first", "stance", "level", "measured"),
                    default="first")
    ap.add_argument("--foot-comp", choices=("off", "on"), default="on")
    ap.add_argument("--heading", choices=("off", "heading", "heading-only"), default="off",
                    help="off (default, and what the 0.98 / 1.11 scores in "
                         "outputs/benchmark_harness.md were measured with): the bare "
                         "Raibert lateral law. heading-only: add the heading half, which "
                         "outputs/heading_hold.md measures at WALK 13.26 -> 0.24 deg/m "
                         "(inside the 0.565 budget) and TROT 7.79 -> 3.20. heading: the "
                         "full substitution, which keeps the yaw-RATE term and is worse in "
                         "both gaits. The calibration sweeps run WITH heading hold and "
                         "these scores ran WITHOUT it, so the two were never comparable -- "
                         "that is what this flag is for. Report paired with --heading off.")
    ap.add_argument("--heading-cap", type=float, default=None,
                    help="override the per-skill heading cap in rad. The default is the "
                         "HEADING_CAP table, measured by the open-loop steering probe.")
    ap.add_argument("--foot-clip-rad", type=float, default=0.05)
    ap.add_argument("--allow-partial", action="store_true",
                    help="run fewer than all 200 cells. The printed mean is then a wiring "
                         "check and NOT a benchmark score; every row is stamped partial=1")
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

    if args.self_test:
        return self_test()
    if args.plan:
        print(plan_report(args))
        return 0

    rows, agg, partial = run_isaac(args)
    print("\n" + "=" * 70)
    if partial:
        print("PARTIAL RUN -- this mean is NOT the benchmark score")
    print(f"skill {args.skill}   cells {agg['n_cells']}   episodes {agg['n_episodes']}")
    print(f"goals reached, mean over cells (equal weight): {agg['score']:.2f} / {NUM_GOALS}")
    print("\nper task:")
    z = np.load(FROZEN_NPZ, allow_pickle=False)
    names = [str(x) for x in z["task_names"]]
    for t in sorted(agg["per_task"]):
        print(f"  {names[t]:36s} {agg['per_task'][t]:5.2f}")
    print("\nper level: " + "  ".join(f"{l}:{v:.2f}" for l, v in sorted(agg["per_level"].items())))
    if args.results_csv:
        import csv as _csv
        p = Path(args.results_csv)
        new = not p.exists()
        with p.open("a", newline="") as fh:
            w = _csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            if new:
                w.writeheader()
            w.writerows(rows)
        print(f"\n[bench] {len(rows)} rows -> {p}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        if _SIM_APP is not None:
            _SIM_APP.close()
