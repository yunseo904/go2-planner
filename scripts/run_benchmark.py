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
from sim import legged_eval_terrain as LET
from terrain_toolkit.paths import FROZEN_NPZ, SKILL_CLIPS_META_JSON
# Imported HERE, at module scope, and not lazily inside run_isaac().  Importing
# planner.features into an already-running Kit process exits the process with status 0,
# no traceback and no output -- see harness_findings.md.  run_planner_replay.py imports
# them at module top and has never had the problem.
from planner.config import DEFAULT as PLANNER_CFG   # NOT _CFG: run_isaac
                                                    # binds that name locally
from planner.features import maps_from_archive, extract, lookahead_distance
from planner.rules import RulePlanner
from planner.skills import SkillId as _SID
from sim.yawmoment import YawMoment
from sim.attitude import RollCouple

_SIM_APP = None

#: Upstream constants, from extreme-parkour/legged_gym/envs/base/legged_robot_config.py
#: and legged_robot.py.  Named here so a divergence is visible rather than implicit.
NEXT_GOAL_THRESHOLD_M = 0.20      # LeggedRobotCfg.env.next_goal_threshold
REACH_GOAL_DELAY_S = 0.1          # LeggedRobotCfg.env.reach_goal_delay
EPISODE_LENGTH_S = 20.0           # LeggedRobotCfg.env.episode_length_s
ROLL_PITCH_CUTOFF_RAD = 1.5       # legged_robot.check_termination
HEIGHT_CUTOFF_M = -0.25           # legged_robot.check_termination
NUM_GOALS = 8


# --------------------------------------------------------------------------- #
# Where the terrain comes from
# --------------------------------------------------------------------------- #
#: ``legged_eval`` is the default and ``frozen`` is kept only so the numbers measured
#: before 2026-09-01 stay reproducible.  They are DIFFERENT TERRAIN, not two readings of
#: one: the archive has no roughness and no rim, and it draws its courses from upstream's
#: own seed rather than the run seed.  Measured over all 200 cells, only 20 of them come
#: out identical -- the other 180 are a different draw of the same generator, and eight
#: courses differ by more than a metre in peak height.  So a frozen score and a
#: legged_eval score may never be averaged or compared, and every row says which it is.
TERRAIN_SOURCES = ("legged_eval", "frozen")


def load_terrain(args):
    """The archive-shaped dict the rest of this file reads, from either source."""
    if getattr(args, "terrain", "legged_eval") == "frozen":
        return np.load(FROZEN_NPZ, allow_pickle=False), None
    grid = LET.build(seed=args.terrain_seed,
                     roughness=not getattr(args, "no_roughness", False),
                     border_walls=not getattr(args, "no_border_walls", False))
    return LET.as_archive(grid), grid


def terrain_stamp(args, grid) -> str:
    """One string naming the terrain, for the CSV and the banner."""
    if grid is None:
        return "frozen-no-roughness-no-walls"
    bits = ["legged_eval", f"seed{grid.seed}"]
    if not grid.roughness:
        bits.append("no-roughness")
    if not grid.border_walls:
        bits.append("no-walls")
    return "-".join(bits)


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


def _frac(held: dict, name: str, k: int) -> float:
    """Share of the steps robot ``k`` spent upright that it spent in skill ``name``."""
    tot = sum(held[s][k] for s in held)
    return (held[name][k] / tot) if tot else 0.0


def support_stats(contact) -> tuple:
    """(mean feet down, fraction of the cycle below 3 feet) for a clip's contact channel.

    The CLAUDE.md 2.5 gate is about the COMMANDED support pattern, which is a property of
    the clip -- these arms replay it open loop, so nothing downstream chooses it.  See
    scripts/support_polygon.py for the standalone reading and what the numbers mean.
    """
    c = np.asarray(contact, dtype=bool)
    n = c.sum(axis=1)
    return float(n.mean()), float((n < 3).mean())


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


def plan_report(args, z=None, grid=None) -> str:
    if z is None:
        z, grid = load_terrain(args)
    cells, names, ts, ls = cell_list(z, args.tasks, args.levels)
    full = len(z["task_names"]) * int(z["num_rows"])
    hs = float(z["horizontal_scale"])
    Lx = (z["height_fields_before_fix"].shape[2] - 1) * hs
    Ly = (z["height_fields_before_fix"].shape[3] - 1) * hs
    ncols = int(z["num_rows"])
    rows = (len(cells) + ncols - 1) // ncols
    src = (f"legged_eval, terrain seed {grid.seed}, roughness "
           f"{'on' if grid.roughness else 'OFF'}, border walls "
           f"{'on' if grid.border_walls else 'OFF'}" if grid is not None
           else f"FROZEN ARCHIVE {FROZEN_NPZ.name} -- no roughness, no walls, upstream seed")
    L = [f"terrain: {src}",
         f"{len(names)} tasks x {int(z['num_rows'])} levels = {full} cells",
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

    # 7. the protocol constants are legged_eval's, read off legged_eval and not restated
    d = LET.protocol_defaults()
    ok("goal threshold is legged_eval's", d["next_goal_threshold"] == NEXT_GOAL_THRESHOLD_M)
    ok("dwell time is legged_eval's", d["reach_goal_delay"] == REACH_GOAL_DELAY_S)
    ok("episode length is legged_eval's", d["episode_length_s"] == EPISODE_LENGTH_S)
    ok("tilt cutoff is legged_eval's", d["roll_pitch_cutoff"] == ROLL_PITCH_CUTOFF_RAD)
    ok("height cutoff is legged_eval's", d["height_cutoff"] == HEIGHT_CUTOFF_M)
    ok("goal count is legged_eval's", d["num_goals"] == NUM_GOALS)

    # 8. the generated grid carries the two things the frozen archive does not
    g = LET.build(seed=1)
    ok("20 courses, in legged_eval's dispatch order", len(g.names) == 20)
    c = g.cells[(0, 0)]
    pad_w = int(0.1 // g.horizontal_scale)
    rim = int(0.5 // g.vertical_scale)
    ok(f"a {pad_w}-cell rim is raised to {rim*g.vertical_scale:.3f} m on all four sides",
       (c.hf_raw[:pad_w, :] == rim).all() and (c.hf_raw[-pad_w:, :] == rim).all()
       and (c.hf_raw[:, :pad_w] == rim).all() and (c.hf_raw[:, -pad_w:] == rim).all())
    ok("the rasterised field is not the clean course (roughness is on)",
       not np.allclose(c.hf_raw * g.vertical_scale, c.hf_clean_m))
    ok("the spawn is eurekaverse's own x=1.0, y=W/2, z=0",
       c.spawn == (1.0, g.terrain_width / 2.0, 0.0))
    g2 = LET.build(seed=2)
    ok("a different run seed draws a different terrain",
       not np.array_equal(g.cells[(0, 0)].hf_raw, g2.cells[(0, 0)].hf_raw))
    ok("the same run seed draws the same terrain",
       np.array_equal(g.cells[(0, 0)].hf_raw, LET.build(seed=1).cells[(0, 0)].hf_raw))

    z = LET.as_archive(g)

    # 8b. THE PRIVILEGED MAP IS THE TERRAIN THE ROBOT IS STANDING ON.
    #
    # legged_eval's rule_walker adapter has the bug this guards against: its RULE_ORACLE
    # mode loads go2-planner/data/benchmark_frozen.npz as its privileged height map while
    # the simulator runs legged_eval's terrain underneath -- a different draw, no roughness,
    # no rim.  It marks such runs unscorable, so nothing published is wrong, but the
    # comparison it exists to support does not hold.  Here the optimistic arm reads the SAME
    # dict the mesh was built from, and this asserts it cell for cell.
    from planner.features import maps_from_archive as _mfa
    _maps = _mfa(z)
    _nlev = z["height_fields_before_fix"].shape[1]
    ok("the optimistic arm's privileged map is the terrain under the robot, all 200 cells",
       all(np.allclose(_maps[t * _nlev + l].height_m,
                       g.cells[(t, l)].hf_raw * g.vertical_scale)
           and _maps[t * _nlev + l].task == g.names[t]
           for t in range(20) for l in range(10)))
    ok("that map carries the roughness and the rim, not the clean course",
       not np.allclose(_maps[0].height_m, g.cells[(0, 0)].hf_clean_m))

    # 9. the grid places every cell disjointly and keeps its own heights
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
    ok("goal 0 of cell 0 is legged_eval's, untranslated",
       np.allclose(goals[0, 0], z["goals_before_fix"][0, 0, 0, :2]))
    # The goals are eurekaverse's own coordinates and are NOT moved -- legged_eval
    # 2.5.  If this ever fails, someone has started nudging goals off pits again.
    ok("no goal has been moved from the course function's output",
       all(np.array_equal(g.cells[(t, l)].goals,
                          z["goals_before_fix"][t, l]) for t in range(3) for l in range(10)))

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

    # Isaac Lab's AppLauncher leaves the process in a state where an uncaught Python
    # exception exits with status 0, no message and no traceback.  A NameError in the
    # results-row builder cost several runs to localise by bisection, and the log looked
    # exactly like a clean early exit.  Reinstall a hook that says what happened.
    def _excepthook(et, ev, tb):
        import traceback
        sys.stderr.write("\n[bench] UNCAUGHT EXCEPTION "
                         "(Isaac's default hook would print nothing):\n")
        traceback.print_exception(et, ev, tb, file=sys.stderr)
        sys.stderr.flush(); sys.stdout.flush()
    sys.excepthook = _excepthook
    # sys.excepthook is NOT called for SystemExit, and every "refusing to run" guard in
    # this file raises one.  Under Kit those messages are lost, so a deliberate refusal
    # and a hard crash look identical from the log.  Print it ourselves.
    def _bye(msg="", code=None):
        if msg:
            sys.stderr.write(f"\n{msg}\n"); sys.stderr.flush()
        raise SystemExit(code if code is not None else 1)
    globals()["_bye"] = _bye

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

    z, grid = load_terrain(args)
    terr = terrain_stamp(args, grid)

    # Resolved HERE and not further down.  It only depends on args, and the depth camera
    # has to be constructed before sim.reset() while the old site was after it -- reading it
    # early cost a run to a NameError that Kit reports as exit status 0 with no traceback,
    # which is harness_findings.md 17 exactly.
    PLANNER_ARM = args.skill.upper() == "PLANNER"

    # THE RULE-PLANNER IS A DEPTH ARM.  The experiment's premise is a robot that reads the
    # terrain from its own camera, and since 2026-09-02 that is what the default does.
    # --perception optimistic is still there and is still useful, but it is now a CONTROL --
    # the perception upper bound, ground truth through a sensor model -- and has to be asked
    # for by name.  It was the default for as long as there was no depth path, and leaving
    # it as the default after there was one would have gone on publishing the control as
    # the result.
    #
    # The single-skill arms read no terrain at all, so perception does not apply to them
    # and is recorded as `n/a` rather than as a setting that happened not to matter.
    if not PLANNER_ARM:
        args.perception = "n/a"
    elif args.perception is None:
        args.perception = "depth"

    # The depth noise is drawn here, not by the fork, so it needs a stated seed.  It follows
    # the terrain seed for the same reason the terrain does: seed 1 must be seed 1 for every
    # model, and 1/2/3 must be three draws whose spread is the error bar.
    _rng = np.random.default_rng(args.terrain_seed)
    # Half-width of legged_eval's robot-local map.  Read off that module rather than written
    # as 2.0, because it is also where the planner is told it is standing.
    _DTM_Y_HALF = 2.0
    if args.perception == "depth":
        LET._import_legged_eval()
        from legged_eval.adapters import depth_terrain as _dtm
        _DTM_Y_HALF = float(_dtm.Y_HALF)

    # --planner-set: the planner's own thresholds, overridden FOR THIS RUN ONLY.
    #
    # planner/config.py keeps STEP_TROT_MAX at 0.08 m marked CALIBRATION_NEEDED while
    # SESSION_STATE 7 measured TROT's step bracket at 0.02-0.04 m, so the planner has been
    # choosing TROT under a belief about itself that is 2-4x too generous.  Overriding it
    # here rather than in the config is deliberate: a placeholder that has been quietly
    # replaced by a run argument is still a placeholder, and the bracket came from three
    # entry phases, which is a direction and not a calibration.  The override is stamped on
    # every row so a run under it can never be read as a run under the config.
    pcfg = PLANNER_CFG
    pset = {}
    for item in (args.planner_set or []):
        key, _, val = item.partition("=")
        if not val:
            raise SystemExit(f"[bench] --planner-set wants group.FIELD=value, got {item!r}")
        pset[key.strip()] = float(val)
    if pset:
        pcfg = PLANNER_CFG.replace(**pset)
        print("[bench] *** PLANNER THRESHOLDS OVERRIDDEN FOR THIS RUN ***")
        for k, v in sorted(pset.items()):
            grp, _, nm = k.partition(".")
            print(f"[bench]     {k} = {v:g}  (config: {getattr(getattr(PLANNER_CFG, grp), nm):g})")
    planner_stamp = ";".join(f"{k}={v:g}" for k, v in sorted(pset.items()))
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
    print(plan_report(args, z, grid))

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
    # legged_eval.terrain.PIT_DEPTH, not "< 0".  Once roughness is on, every cell dips a
    # few millimetres below datum somewhere, so a "< 0" test calls all 20 tasks pitted and
    # stops distinguishing anything.  -0.25 m is legged_eval's own threshold and it sits in
    # an empty gap: these courses' holes are all exactly -1.0 m.
    pit_depth = getattr(LET._import_legged_eval(), "PIT_DEPTH", -0.25)
    pit = int((z["height_fields_before_fix"][ts] * float(z["vertical_scale"])
               < pit_depth).any(axis=(1, 2, 3)).sum())
    print(f"[bench] removed the importer's infinite ground plane at {plane_path}; the "
          f"{pit} of {len(ts)} selected tasks that contain pits are open")
    print(f"[bench] terrain: {len(V)} vertices, {len(F)} faces, {n} cells in {rows}x{cols}")

    sim_utils.DomeLightCfg(intensity=2000.0).func("/World/light",
                                                  sim_utils.DomeLightCfg(intensity=2000.0))
    for k in range(n):
        xf = UsdGeom.Xform.Define(stage, f"/World/envs/env_{k}")
        xf.AddTranslateOp().Set(Gf.Vec3d(float(spawns[k][0]), float(spawns[k][1]), 0.0))
    robot = Articulation(UNITREE_GO2_CFG.replace(prim_path="/World/envs/env_.*/Robot"))

    # ---------------------------------------------------------------- side-view video
    # One camera per recorded cell, created HERE -- before sim.reset(), like every other
    # sensor, because a camera added after the scene is built is never rendered in this
    # stack.  They render ON DEMAND (update_period 0.0) and never schedule themselves, and
    # grab_frame() below calls sim.render() and nothing else: the physics substep loop is
    # untouched -- same phys_dt, same decimation, same order of writes, same control rate.
    # That is an argument, not a proof, so --video also prints the regression check the
    # user asked for: run the same cells with and without it and compare goals and steps.
    # ---------------------------------------------------------------- depth camera
    # The REAL perception arm.  --perception optimistic (the default) hands the planner
    # the terrain's own height field through planner.features' sensor model: ground truth,
    # range- and resolution-limited but never occluded, never noisy, never shaken by the
    # body's own pitch.  --perception depth renders what the robot can actually see and
    # inverts it, so the planner's input is an observation.
    #
    # The camera is eurekaverse's own, field for field, read off CustomDepthCfg rather than
    # restated: same mount, same 87 deg, same 106x60, same update rate, same one-step delay.
    # The inversion is legged_eval's (adapters/depth_terrain.py) -- the second copy of a
    # depth-to-heightmap that this project must not grow.
    depth = None
    if args.perception == "depth":
        if not PLANNER_ARM:
            raise SystemExit("[bench] --perception depth is only meaningful for "
                             "--skill PLANNER; a fixed clip does not look at anything.")
        from isaaclab.sensors import TiledCamera, TiledCameraCfg
        from scipy.spatial.transform import Rotation as _R
        DC = IC.depth_cfg()
        aperture = 20.955
        focal = aperture / (2 * np.tan(np.radians(DC["horizontal_fov"]) / 2))
        quat_xyzw = tuple(_R.from_euler("y", DC["pitch_rad"]).as_quat())
        w_, h_ = DC["resolution"]
        cam_cfg = TiledCameraCfg(
            prim_path="/World/envs/env_.*/Robot/base/front_cam",
            offset=TiledCameraCfg.OffsetCfg(pos=DC["pos"], rot=quat_xyzw,
                                            convention="world"),
            data_types=["distance_to_image_plane"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=focal,
                                             horizontal_aperture=aperture,
                                             clipping_range=(0.05, 20.0)),
            width=w_, height=h_,
            # Posed on demand from the loop, at the same cadence the fork uses.
            update_period=0.0)
        try:
            depth = {"cam": TiledCamera(cam_cfg), "cfg": DC, "buf": None, "frames": 0,
                     "reported": False}
        except BaseException:
            # Kit swallows this otherwise: harness_findings.md 17.
            import traceback
            sys.stderr.write("\n[bench] depth camera construction FAILED:\n")
            traceback.print_exc(file=sys.stderr); sys.stderr.flush()
            raise
        print(f"[bench] depth camera: {w_}x{h_} -> {DC['processed']}, "
              f"{DC['horizontal_fov']} deg, mount {DC['pos']} pitched "
              f"{np.degrees(DC['pitch_rad']):.1f} deg down, "
              f"every {DC['update_interval']} control steps, "
              f"{DC['delay_steps']} step of latency, clip {DC['near_clip']}-{DC['far_clip']} m")

    video = None
    if args.video:
        from isaaclab.sensors import Camera, CameraCfg
        vc = list(args.video_cells) if args.video_cells else [0]
        bad = [k for k in vc if not (0 <= k < n)]
        if bad:
            raise SystemExit(f"[bench] --video-cells {bad} outside 0..{n-1}")
        cams = [Camera(CameraCfg(
            prim_path=f"/World/side_cam_{i}", update_period=0.0,
            height=args.video_height, width=args.video_width, data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=24.0,
                                             clipping_range=(0.05, 60.0))))
            for i in range(len(vc))]
        video = {"cams": cams, "cells": vc, "frames": 0}
        print("[bench] video: side view of cells " +
              ", ".join(f"{k} ({names[cells[k][0]]}, level {cells[k][1]})" for k in vc) +
              f", {args.video_width}x{args.video_height} -> {args.video}")

    sim.reset()
    if robot.num_instances != n:
        raise SystemExit(f"[bench] {robot.num_instances} robots for {n} cells")
    if ucfg.ground_friction is not None:
        set_robot_friction(robot, ucfg.ground_friction)

    # ---------------------------------------------------------------- planner arm
    # --skill PLANNER is the Rule-Planner experimental group: every robot carries its own
    # RulePlanner, its own clip, its own phase and its own corrections, and the planner
    # picks the skill.  It is a SEPARATE PATH from the single-skill one below rather than
    # a generalisation of it, so a single-skill baseline run stays byte-for-byte the run
    # it always was -- the baselines are what the planner is scored against and they must
    # not move underneath it.
    #
    # What the planner is allowed to see: the frozen archive's own height field, through
    # planner.features.extract, which applies the SENSOR MODEL (near/far clip, blur by
    # ground resolution, confidence).  That is terrain information, and CLAUDE.md 2 puts
    # terrain at the planner and forbids it at the low level -- which is where it stays:
    # the clip replay, the foot placement and the yaw couple below see base velocity,
    # heading error and the clip's own contact channel, and nothing else.
    #
    # It is still an OPTIMISTIC perception arm and has to be reported as one: the geometry
    # is ground truth put through a sensor model, not a rendered depth image.
    clip = load_clip("WALK" if PLANNER_ARM else args.skill, args.rate)
    # --heading, resolved once.  The cap is the skill's own, from the open-loop steering
    # probe (outputs/heading_candidates.md 2): WALK took +-0.04 rad with no measurable
    # cost, TROT falls in BOTH directions at +-0.04 and is safe at +-0.02.  TURN is not
    # trying to hold a heading -- it is trying to change one -- so it is 0, and asking
    # for it there is refused rather than silently ignored.
    yaw_mode, hcap, vy_log, wz_log = "off", 0.0, 0.0, 0.0
    # The PLANNER arm resolves the cap PER CLIP further down (each skill carries its own
    # measured cap and TURN carries none), so this single-skill guard does not apply to
    # it.  Applying it anyway is what the first version did: HEADING_CAP.get("PLANNER")
    # is 0.0, the guard fired, and because SystemExit does not go through sys.excepthook
    # the process exited with status 0, no message and no CSV -- indistinguishable from a
    # crash, and it cost most of an hour to localise.
    if args.heading != "off" and not PLANNER_ARM:
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
    # AN IN-PLACE TURN NEEDS ITS OWN LOGGED YAW RATE, whatever --heading says.
    #
    # outputs/turn_target.md: the placement law drives every foot toward v_y = 0, and for a
    # turn a lateral foot velocity is not an error, it is the motion.  The fix was to give
    # the law the log's own omega.  The PLANNER arm has had it since (ymode = "log-cycle"
    # if nm == "TURN"); the SINGLE-SKILL arm never did, because the only way to run TURN is
    # --heading off (HEADING_CAP has no TURN entry and a cap of 0 is refused), and that
    # skips the block above -- leaving wz_log at 0.0.  So the TURN baseline has been
    # measured with its placement law told the robot is not turning.
    #
    # Measured: flat rig with the yaw term  -16.3 to -18.9 deg/s (72-83% of the log)
    #           benchmark without it         -7.8 to  -8.7 deg/s (34-38%)
    #
    # This is not a new intervention -- it is the law turn_target.md already established,
    # applied to the arm that was denied it.  --foot-yaw-turn off reproduces the old rows.
    if clip["name"] == "TURN" and args.foot_yaw_turn == "auto" and args.foot_comp != "off":
        yaw_mode = "log-cycle"
        hcap = 0.0
        vy_log, wz_log = _log_motion_for(
            json.loads(SKILL_CLIPS_META_JSON.read_text()), clip["name"])
        print(f"[bench] *** TURN gets its LOGGED yaw rate ({wz_log:+.4f} rad/s) in the "
              f"placement law. Without it the law targets zero rotation and fights the "
              f"turn -- that is outputs/turn_target.md, and this arm had never had the "
              f"fix. --foot-yaw-turn off reproduces the earlier TURN rows. ***")
    idx_by_name = {nm: i for i, nm in enumerate(robot.joint_names)}
    want = [f"{l}_{j}_joint" for l in clip["leg_order"] for j in clip["joint_order"]]
    jidx = [idx_by_name[w] for w in want]
    idx_t = torch.as_tensor(jidx, device=sim.device, dtype=torch.long)

    # The hip's effort limit, READ FROM THE ACTUATOR -- not from robot.data, which reports
    # PhysX's 1e9.  The actuator is an explicit IdealPDActuator, so the clip that binds is
    # applied in Python against actuator.effort_limit (harness_findings.md, and
    # isaac_actuator_probe.json measured 23.70 N.m on this config).
    hip_effort_limit_nm = float("nan")
    _wh = robot.joint_names[jidx[0]]
    for _act in robot.actuators.values():
        _nm = list(getattr(_act, "joint_names", []) or [])
        _e = getattr(_act, "effort_limit", None)
        if _e is None or _wh not in _nm:
            continue
        _a = np.asarray(snap(_e)).reshape(-1)
        _v = float(_a[_nm.index(_wh)] if _a.size == len(_nm) else _a.flat[0])
        if np.isfinite(_v) and 0.0 < _v < 1e4:
            hip_effort_limit_nm = _v
            break
    if args.yaw_moment != "off" and not np.isfinite(hip_effort_limit_nm):
        raise SystemExit("[bench] REFUSING --yaw-moment: could not read the hip's effort "
                         "limit off the articulation, so the headroom this term needs "
                         "cannot be checked.")
    if args.roll_couple != "off" and not np.isfinite(hip_effort_limit_nm):
        raise SystemExit("[bench] REFUSING --roll-couple: could not read the hip's effort "
                         "limit off the articulation. The cap check is the only thing "
                         "stopping this term from eating the PD that plays the clip, and "
                         "a guard that cannot fire is not a guard (SESSION_STATE 6).")
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

    # Clip library and per-cell terrain maps for the planner arm.  Entry frames: WALK and
    # TROT take the rule (level_start), which for TROT happens to pick frame 16 -- one of
    # the four phases the flat sweep found TROT survives at (trot_yaw_moment.md 5), so no
    # override is needed and none is applied.  TURN takes its MEASURED frame 6, because
    # the rule picks 24 and turn_entry_phase.md measured that phase as never turning.
    pl_clips, pl_tmaps, pl_meta = {}, None, None
    if PLANNER_ARM:
        pl_meta = json.loads(SKILL_CLIPS_META_JSON.read_text())
        # Entry frames, WITHOUT level_start.  That rule re-poses the articulation and
        # steps the sim to measure coplanarity, which is fine on the single-robot replay
        # rig and kills this one (200 robots already placed on their own cells); it also
        # is not the rule the single-skill baselines use, and the arms have to match.
        #
        #   WALK, TROT  --start-phase, exactly as the baselines resolve it. Frame 0 (the
        #               default) is one of the four phases TROT survives at on flat
        #               (trot_yaw_moment.md 5), so no override is needed.
        #   TURN        its MEASURED frame 6: the coplanarity rule picks 24 and
        #               turn_entry_phase.md measured that phase as never turning.
        for nm in ("WALK", "TROT", "TURN"):
            c = load_clip(nm, args.rate)
            kf_, note_ = 0, f"--start-phase {args.start_phase}"
            m_ = int(getattr(pcfg.skill, f"ENTRY_FRAME_{nm}", -1))
            if nm == "TURN" and m_ >= 0:
                kf_, note_ = m_ % len(c["q_des"]), "MEASURED, turn_entry_phase.md"
            elif args.start_phase == "measured" and m_ >= 0:
                kf_, note_ = m_ % len(c["q_des"]), "MEASURED"
            elif args.start_phase == "stance":
                kf_, _u = quiescent_start(c)
                note_ = "quiescent"
            pl_clips[nm] = rotate_clip(c, kf_)
            print(f"[bench]   {nm:5s} entry frame {kf_} ({note_}), "
                  f"{len(c['q_des'])} frames/cycle")
        allmaps = maps_from_archive(z)
        nlev = int(z["height_fields_before_fix"].shape[1])
        pl_tmaps = [allmaps[t * nlev + l] for (t, l) in cells]

    # ---- swing lift ------------------------------------------------------------------
    # The hypothesis this exists to test: the earlier ladder found the untouched recording
    # was the best row at 0/40/60/80 mm, but that was OPEN LOOP -- raising the swing arc
    # cost balance faster than it bought clearance.  With the roll couple holding the roll,
    # the trade may land somewhere else.  So the flag is here to be run PAIRED with
    # --roll-couple, not on its own.
    #
    # The edit is verify_skill_replay.swing_lift_offsets, unchanged: A*sin^2(pi*phase) over
    # each swing bout, zero value AND zero slope at liftoff and touchdown (so stride length
    # and forward speed, which live at the endpoints, are untouched), per leg per bout,
    # target minus the apex that leg already has.  Thigh and calf only -- the hip belongs to
    # foot placement and the two must not compete for it.  The archive is not written.
    lift_offsets = {}
    if args.swing_lift > 0:
        from verify_skill_replay import swing_lift_offsets
        # Computed from the ROTATED clips, which is what actually plays; offsets from the
        # unrotated ones would be a phase error, silent and plausible.
        _names = list(pl_clips) if PLANNER_ARM else [clip["name"]]
        _srcs = pl_clips if PLANNER_ARM else {clip["name"]: clip}
        for _nm in _names:
            # Above the tallest cell in the grid, so the measurement is of the clip and
            # not of a collision.  z is patch-local here because every cell's datum is 0.
            _air = float(z["height_fields_before_fix"].max()) * float(z["vertical_scale"]) + 2.0
            _off, _rep, _ = swing_lift_offsets(robot, sim, idx_t, _srcs[_nm],
                                               args.swing_lift / 1000.0, phys_dt,
                                               symmetric=not args.swing_lift_asym,
                                               air_z=_air, spread=True)
            lift_offsets[_nm] = _off
            print(f"[bench] swing lift {args.swing_lift:g} mm on {_nm}: "
                  + ("PER-LEG " if args.swing_lift_asym else "SYMMETRIC ")
                  + ", ".join(f"{l} +{r['added_mm']}"
                              for l, r in _rep.items() if not l.startswith("_")))
        print(f"[bench] swing lift measured with the robot held at z = {_air:.2f} m, "
              f"clear of the grid's tallest cell")
        print("[bench] *** SWING LIFT ON: the recording's swing arcs are raised. Stance and "
              "the hip are untouched; the archive on disk is unchanged. Report PAIRED with "
              "--swing-lift 0. ***")

        if not PLANNER_ARM:
            q_seq = (q_seq + lift_offsets[clip["name"]]).astype(np.float32)


    # Isaac Lab's AppLauncher installs an excepthook that SWALLOWS a Python traceback:
    # a NameError in the results-row builder above exited the process with status 0, no
    # message, and no CSV, and cost several runs to localise by bisection.  Anything that
    # raises inside the episode loop from here on is caught, printed, and re-raised.
    def _loud(exc: BaseException) -> None:
        import traceback
        print("\n[bench] EXCEPTION (Isaac's excepthook would otherwise eat this):",
              flush=True)
        traceback.print_exception(type(exc), exc, exc.__traceback__)
        sys.stdout.flush(); sys.stderr.flush()
    # THE SPAWN, and the datum the pit rule is measured against, are legged_eval's.
    #
    # eurekaverse resets to `base_init_state + env_origin` with `origin_zero_z = True`, so
    # the robot appears at a FIXED height above the patch datum z = 0 -- not above the
    # ground under its feet.  This harness used to add args.spawn_z to the sampled terrain
    # height, which was the same thing while the archive's spawn strip was exactly flat
    # and is not any more: roughness moves the ground under the spawn by +-0.025 m, so
    # sampling it would give every cell its own drop height and no two models the same one.
    #
    # The pit cutoff follows the same datum.  legged_eval terminates on
    # `root_states[:, 2] < -0.25` in world coordinates, and this grid puts every cell's
    # datum at z = 0, so patch-local and world z are the same number here.  Subtracting a
    # per-cell ground sample (what this did before) would let a robot walk along the floor
    # of a 1 m pit without ever tripping the rule.
    spawn_datum_z = float(z.get("spawn_z", 0.0)) if isinstance(z, dict) else 0.0
    ground_z = np.full(n, spawn_datum_z, dtype=float)
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

    if args.turn_target == "goal" and PLANNER_ARM:
        print("[bench] *** TURN TARGET = GOAL. The planner's heading question is now the "
              "bearing to the live waypoint, not the drift from the start heading, which "
              "is what RulePlanner._wants_turn documents. Heading hold is re-baselined "
              "when a TURN ends so the low level does not undo it. Report PAIRED with "
              "--turn-target settle. ***")
    if args.roll_couple != "off":
        print(f"[bench] *** LEVEL 2 -- ROLL COUPLE ON. Uniform feed-forward hip torque on "
              f"the stance legs: gain {args.roll_gain:g} N.m/rad, damp {args.roll_damp:g} "
              f"N.m/(rad/s), cap {args.roll_cap_nm:g} N.m, sign {args.roll_sign:+g}, bias "
              f"{args.roll_couple_nm:g} N.m. Hip effort limit read off the ACTUATOR: "
              f"{hip_effort_limit_nm:.2f} N.m. Every fall on this grid is a roll fall "
              f"(200/200 in both roughness arms) and this is the term aimed at that. ***")
        print(f"[bench]     Report this PAIRED with --roll-couple off. 'It did not fall' "
              f"is not a result on its own: stride and v_x must not move.")

    n_settle = max(int(args.settle_s / dt), 1)
    ep_steps = int(EPISODE_LENGTH_S / dt)
    tracker = GoalTracker(goals, dt)

    def depth_normalised():
        """One depth frame per env, in the fork's own [-0.5, +0.5] convention.

        legged_robot.update_depth_buffer + process_depth_image, in order and with the same
        constants: negate to the Isaac Gym sign and back, crop the noisy edges, clip the
        infinities a miss returns, add the granular and blackout noise, clip to
        near/far, normalise.  blur and erase are 0.0 in the config and are not applied.
        """
        DC = depth["cfg"]
        d = depth["cam"].data.output["distance_to_image_plane"]
        d = d.squeeze(-1).detach().cpu().numpy().astype(np.float32)   # (n, h, w), metres
        cl, cr = DC["crop_left"], DC["crop_right"]
        ct, cb = DC["crop_top"], DC["crop_bottom"]
        d = d[:, ct:d.shape[1] - cb if cb else None, cl:d.shape[2] - cr]
        d = np.clip(np.nan_to_num(d, nan=1e6, posinf=1e6, neginf=-1e6), -1e6, 1e6)
        if DC["granular_noise"]:
            d = d + DC["granular_noise"] * _rng.standard_normal(d.shape).astype(np.float32)
        if DC["blackout_noise"]:
            d[_rng.random(d.shape) < DC["blackout_noise"]] = 0.0
        d = np.clip(d, DC["near_clip"], DC["far_clip"])
        return (d - DC["near_clip"]) / (DC["far_clip"] - DC["near_clip"]) - 0.5

    def depth_maps():
        """The delayed depth frame, inverted to one robot-local TerrainMap per env.

        Unseen cells come back NaN and are filled with 0 -- the ground continues.  That is
        legged_eval's own choice in depth_terrain.local_map() and its reasons are recorded
        there: pulling the nearest observed value across invents obstacles that were never
        seen, and leaving the NaN makes planner.features count every unobserved cell as a
        pit, which is most of the frame.
        """
        from legged_eval.adapters import depth_terrain as DTM
        from planner.features import TerrainMap
        g = snap(robot.data.projected_gravity_b)
        H, cov, _z0 = DTM.local_maps_batch(depth["buf"], g)
        if not depth["reported"]:
            depth["reported"] = True
            d = depth["buf"]
            near = float((d < 0.49).mean())     # anything the far clip did not swallow
            print(f"[bench] depth check: {depth['frames']} frame(s), buffer "
                  f"{d.shape} in [{d.min():+.3f}, {d.max():+.3f}], "
                  f"{100*near:.1f}% of pixels returned inside 2 m, "
                  f"map coverage {100*float(np.mean(cov)):.1f}% of "
                  f"{H.shape[1]}x{H.shape[2]} cells, "
                  f"observed height {np.nanmin(H):+.2f}..{np.nanmax(H):+.2f} m", flush=True)
            if near < 0.01:
                raise SystemExit(
                    "[bench] refusing to score: the depth buffer is empty -- fewer than 1% "
                    "of pixels came back inside the far clip. A camera that renders nothing "
                    "does not fail here, it hands the planner a uniform pit and the run "
                    "looks like a bad policy instead of a broken sensor.")
        H = np.where(np.isfinite(H), H, 0.0)
        maps = [TerrainMap(height_m=H[k], horizontal_scale=DTM.HS,
                           task=names[cells[k][0]], level=cells[k][1],
                           spawn_m=(0.0, DTM.Y_HALF)) for k in range(n)]
        return maps, float(np.mean(cov))

    def grab_frame():
        """One RGB frame per recorded cell.  RENDERS ONLY -- never steps physics.

        All poses are set first, then ONE sim.render(), then the read-backs, because
        render() draws every sensor in the scene: doing it per camera would be one full
        render per recorded cell per frame.
        """
        p_ = snap(robot.data.root_pos_w)
        for cam, k in zip(video["cams"], video["cells"]):
            y0 = float(spawns[k][1])
            eye = torch.tensor([[float(p_[k, 0]), y0 + args.video_side_m,
                                 max(args.video_eye_m, float(p_[k, 2]) + 0.6)]],
                               device=sim.device, dtype=torch.float32)
            tgt_ = torch.tensor([[float(p_[k, 0]), y0, 0.28]],
                                device=sim.device, dtype=torch.float32)
            cam.set_world_poses_from_view(eye, tgt_)
        sim.render()
        for cam, k in zip(video["cams"], video["cells"]):
            cam.update(dt, force_recompute=True)
            rgb = cam.data.output["rgb"][0].detach().cpu().numpy()
            video["writers"][k].append_data(
                np.ascontiguousarray(rgb[..., :3]).astype(np.uint8))
        video["frames"] += 1

    if video is not None:
        import imageio.v2 as imageio
        out_dir = Path(args.video)
        out_dir.mkdir(parents=True, exist_ok=True)
        fps = max(1.0, 1.0 / (dt * args.video_stride))
        video["writers"] = {}
        for k in video["cells"]:
            t_, l_ = cells[k]
            fn = out_dir / f"cell{k:03d}_{names[t_]}_lvl{l_}.mp4"
            video["writers"][k] = imageio.get_writer(
                str(fn), fps=fps, macro_block_size=None, codec="libx264", quality=8)
        print(f"[bench] video: {len(video['cells'])} clip(s) at {fps:.1f} fps -> {out_dir}")

    rows_out = []
    t0 = time.time()
    try:
      for ep in range(args.episodes):
          peak_roll = np.zeros(n); peak_pitch = np.zeros(n)
          end_roll = np.zeros(n); end_pitch = np.zeros(n)
          end_cause = np.full(n, "none", dtype=object)
          # Steps each robot was upright.  pl_held only exists on the planner arm, and a
          # speed read off travelled_m without it is a distance divided by 20 s whether the
          # robot lived 20 s or 2.
          upright = np.zeros(n, dtype=int)
          # Cumulative yaw, unwrapped, while upright: the direct test of whether the sim
          # reproduces a clip's LOGGED yaw rate.  TURN's log says -22.66 deg/s and nothing
          # in this harness has ever read the rate back.
          yaw_cum = np.zeros(n); yaw_prev = None
          # Peak base height while upright: the direct test of whether a term that pushes
          # on the stance legs can put the body in the air (the RUN question).  A gait with
          # a flight phase raises the base above its standing height; one without does not.
          base_h_max = np.zeros(n)
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
          # CROSS-TRACK datum, by exactly the same rule and for the same reason: the y each
          # robot is asked to hold is the one IT settled at, read after the settle and per
          # robot.  Proprioceptive -- it is the robot's own start line, not the lane's
          # centre and not a goal, so this term has the same standing as heading hold and
          # the planner never touches it.
          y_ref = snap(robot.data.root_pos_w)[:, 1].astype(float).copy()
          x_ref = snap(robot.data.root_pos_w)[:, 0].astype(float).copy()
          ct_err = np.zeros(n)
          ct_abs_max = np.zeros(n)
          vy_b = np.zeros(n)
          foots = None
          rolls = None
          if args.foot_comp != "off":
              from sim.replay import quat_rotate_inv
              bq = snap(robot.data.root_quat_w)
              bpos = snap(robot.data.root_pos_w)
              foots = []
              rolls = []
              for k in range(n):
                  hxy = np.array([quat_rotate_inv(bq[k][None, :],
                                                  (bp[k, h_by[l]] - bpos[k])[None, :])[0, :2]
                                  for l in legs])
                  # The roll couple, one per robot, for the SINGLE-SKILL arms only -- the
                  # planner arm carries its own per-clip couples in pl_roll and never reads
                  # these.  Built as None here rather than as unused objects, because an
                  # unused object beside a used one is what made the first report wrong.
                  # WALK is
                  # the arm this term is aimed at (every fall on the grid is a roll fall,
                  # sim/attitude.py) and TROT/TURN are the controls the user requires: a
                  # term that helps one gait and is silent or harmful in the others is a
                  # different finding from one that helps the robot.
                  rolls.append(None if PLANNER_ARM else RollCouple(
                      lever_m=lever[k] if settle_ok[k] else med,
                      hip_x_m=hxy[:, 0], hip_y_m=hxy[:, 1],
                      bias_nm=args.roll_couple_nm,
                      kp_nm_per_rad=args.roll_gain, kd_nm_per_rad_s=args.roll_damp,
                      cap_nm=(args.roll_cap_nm if args.roll_couple != "off" else 0.0),
                      sign=args.roll_sign, effort_limit_nm=hip_effort_limit_nm))
                  foots.append(FootPlacement(
                      t_stance_s=stance_time_s(clip["contact"], clip["fs"]),
                      lever_m=lever[k] if settle_ok[k] else med,
                      hip_x_m=hxy[:, 0], hip_y_m=hxy[:, 1],
                      cap_rad=args.foot_clip_rad, cycle_len=ncyc,
                      yaw_mode=yaw_mode, heading_cap_rad=hcap,
                      vy_log=vy_log, wz_log=wz_log))
              print(f"[bench] *** FOOT PLACEMENT ON (cap {args.foot_clip_rad:g} rad): closes a "
                    f"loop on base lateral velocity and overwrites the recording. ***")
              if args.cross_track != "off":
                  print(f"[bench] *** CROSS-TRACK HOLD ON: gain {args.cross_track_gain:g} /s, "
                        f"cap {args.cross_track_cap:g} m/s, datum = the y each robot SETTLED "
                        f"at. Heading hold holds the ANGLE and nothing held the POSITION; "
                        f"WALK drifts 0.53 m sideways per metre forward, 149/200 cells the "
                        f"same way. This asks the lateral law for v_y = -gain * cross-track "
                        f"error. Report PAIRED with --cross-track off, and check v_x and "
                        f"the support count did not move. ***")
              if yaw_mode != "off":
                  print(f"[bench] *** HEADING HOLD --heading {args.heading} on "
                        f"{args.skill}, cap {hcap:g} rad: each robot holds the heading it "
                        f"settled at. omega_target = omega_log - psi_err/T_stance; T_stance "
                        f"cancels, so the term carries no constant. The 0.98/1.11 scores in "
                        f"outputs/benchmark_harness.md were measured WITHOUT this and are "
                        f"not comparable to a run with it. ***")

          alive = np.ones(n, dtype=bool)
          ended = np.zeros(n, dtype=int)          # goal index at the moment it ended

          # ---- planner arm: one planner, one clip, one phase, one couple per robot ----
          if PLANNER_ARM:
              from sim.replay import quat_rotate_inv as _qri
              bq_ = snap(robot.data.root_quat_w)
              bpos_ = snap(robot.data.root_pos_w)
              bp_ = snap(robot.data.body_pos_w)
              pl = [RulePlanner(cfg=pcfg, initial=_SID.WALK) for _ in range(n)]
              pl_sk = [_SID.WALK] * n
              pl_ph = np.zeros(n, dtype=int)
              pl_q = {nm: (np.asarray(c["q_des"], np.float32)
                           + (lift_offsets[nm] if nm in lift_offsets else 0.0)
                           ).astype(np.float32)
                      for nm, c in pl_clips.items()}
              pl_sw = {nm: ~np.asarray(c["contact"], bool) for nm, c in pl_clips.items()}
              pl_foot, pl_yaw, pl_roll = [], [], []
              pl_switches = np.zeros(n, dtype=int)
              pl_refused = np.zeros(n, dtype=int)
              pl_held = {nm: np.zeros(n, dtype=int) for nm in pl_clips}
              for k in range(n):
                  hxy = np.array([_qri(bq_[k][None, :],
                                       (bp_[k, h_by[l]] - bpos_[k])[None, :])[0, :2]
                                  for l in legs])
                  lev = lever[k] if settle_ok[k] else med
                  fp, ym, rl = {}, {}, {}
                  for nm in pl_clips:
                      vyl, wzl = _log_motion_for(pl_meta, pl_clips[nm]["name"])
                      ymode = "log-cycle" if nm == "TURN" else (
                          args.heading if args.heading != "off" else "off")
                      hc = 0.0 if nm == "TURN" else (
                          HEADING_CAP.get(nm, 0.0) if args.heading != "off" else 0.0)
                      fp[nm] = FootPlacement(
                          t_stance_s=stance_time_s(pl_clips[nm]["contact"], pl_clips[nm]["fs"]),
                          lever_m=lev, hip_x_m=hxy[:, 0], hip_y_m=hxy[:, 1],
                          cap_rad=args.foot_clip_rad, cycle_len=len(pl_q[nm]),
                          yaw_mode=ymode, heading_cap_rad=hc, vy_log=vyl, wz_log=wzl)
                      ycap = (args.yaw_moment_cap_nm if nm == "TROT"
                              and args.yaw_moment != "off" else 0.0)
                      ym[nm] = (YawMoment(lever_m=lev, hip_x_m=hxy[:, 0], hip_y_m=hxy[:, 1],
                                          gain_nm_per_rad=args.yaw_moment_gain,
                                          bias_nm=args.yaw_moment_nm, cap_nm=ycap,
                                          effort_limit_nm=hip_effort_limit_nm)
                                if ycap > 0 else None)
                      # Shares the hip with the yaw couple, so it is told what that one
                      # already holds and refuses a cap the two cannot both have.
                      rl[nm] = RollCouple(
                          lever_m=lev, hip_x_m=hxy[:, 0], hip_y_m=hxy[:, 1],
                          bias_nm=args.roll_couple_nm,
                          kp_nm_per_rad=args.roll_gain, kd_nm_per_rad_s=args.roll_damp,
                          cap_nm=(args.roll_cap_nm if args.roll_couple != "off" else 0.0),
                          sign=args.roll_sign, effort_limit_nm=hip_effort_limit_nm,
                          shared_with_nm=ycap)
                  pl_foot.append(fp); pl_yaw.append(ym); pl_roll.append(rl)
              if depth is not None:
                  print(f"[bench] *** RULE-PLANNER ARM, DEPTH PERCEPTION: {n} independent "
                        f"planners, tick {pcfg.feature.TICK_HZ:.0f} Hz. The planner sees "
                        f"ONLY the rendered depth image, inverted to a robot-local height "
                        f"map by legged_eval.adapters.depth_terrain -- occluded, noisy, and "
                        f"shaken by the body's own pitch. No ground-truth geometry reaches "
                        f"it. ***")
              else:
                  print(f"[bench] *** RULE-PLANNER ARM: {n} independent planners, "
                        f"tick {pcfg.feature.TICK_HZ:.0f} Hz. The planner reads the "
                        f"TERRAIN's own height field THROUGH THE SENSOR MODEL "
                        f"(planner.features.extract); the low level sees base velocity, "
                        f"heading error and the clip's contact channel only. This is an "
                        f"OPTIMISTIC perception arm: ground-truth geometry put through a "
                        f"sensor model, not a rendered depth image. ***")

          tick_every = (max(int(round((1.0 / pcfg.feature.TICK_HZ) / dt)), 1)
                      if PLANNER_ARM else 0)
          for step in range(ep_steps):
              frame = step % ncyc
              cmd = np.tile(q_seq[frame], (n, 1))
              tau_pl = np.zeros((n, 12))

              if depth is not None:
                  # The fork updates on global_counter % update_interval and then reads a
                  # buffer depth_delay_steps old, so the planner never sees the frame from
                  # the step it is acting on.  Same here: render on the cadence, hold the
                  # frames, hand over the oldest.
                  DC = depth["cfg"]
                  if step % DC["update_interval"] == 0:
                      sim.render()
                      depth["cam"].update(dt, force_recompute=True)
                      fr = depth_normalised()
                      if depth["buf"] is None:
                          depth["hist"] = [fr] * (DC["delay_steps"] + 1)
                      else:
                          depth["hist"].append(fr)
                          depth["hist"] = depth["hist"][-(DC["delay_steps"] + 1):]
                      depth["buf"] = depth["hist"][0]
                      depth["frames"] += 1

              if PLANNER_ARM:
                  pos_w = snap(robot.data.root_pos_w)
                  vb = snap(robot.data.root_lin_vel_b)
                  wb = snap(robot.data.root_ang_vel_b)
                  roll_now, _p_now, yaw_now = quat_to_rpy_deg(snap(robot.data.root_quat_w))
                  roll_now = np.asarray(roll_now, float)
                  yaw_now = np.asarray(yaw_now, float)
                  # psi: deviation from the heading this robot SETTLED at.  This is the
                  # low level's quantity and it stays that way -- heading hold is a
                  # stabiliser and CLAUDE.md 2 keeps terrain and goal information out of it.
                  psi = np.radians((yaw_now - yaw_ref + 180.0) % 360.0 - 180.0)

                  # WHAT THE PLANNER IS ASKED.  RulePlanner._wants_turn documents its
                  # argument as "goal bearing minus heading", and this harness was handing
                  # it psi -- the deviation from the START heading.  Those are different
                  # questions and the difference is not cosmetic:
                  #
                  #   * a goal that is not on the spawn axis is never turned towards.
                  #     staircase_spiral puts goal 0 at 2.6 m ahead and 0.30 m RIGHT, and
                  #     the three cells that get far enough in x all miss to the LEFT.
                  #   * every TURN it did spend was undoing drift that heading hold is
                  #     already correcting at ~0 cost, which is why all 37 cells the
                  #     planner loses to WALK are TURN cells.
                  #
                  # This is goal POSITION, not terrain, so it belongs to the planner layer;
                  # legged_eval's protocol hands every policy the same waypoint bearing
                  # through the command channel, so it is not privileged either.
                  if args.turn_target == "goal":
                      gi = np.minimum(tracker.idx, NUM_GOALS - 1)
                      gxy = goals[np.arange(n), gi]              # world xy of the live goal
                      brg = np.degrees(np.arctan2(gxy[:, 1] - pos_w[:, 1],
                                                  gxy[:, 0] - pos_w[:, 0]))
                      head_err = (brg - yaw_now + 180.0) % 360.0 - 180.0
                  else:
                      head_err = np.degrees(psi)
                  # ---- planner tick -------------------------------------------------
                  if step % tick_every == 0:
                      if depth is not None:
                          pl_tmaps, _cov = depth_maps()
                      for k in range(n):
                          if not alive[k]:
                              continue
                          spd = float(abs(vb[k, 0]))
                          la = lookahead_distance(spd, pcfg)
                          # x,y in the CELL's own frame: the archive's maps are per cell and
                          # each robot sits at its own world offset.
                          if depth is not None:
                              # The observed map is ROBOT-LOCAL: x runs 0..2 m ahead and y
                              # -2..+2 m across, with the robot at its own origin.  Feeding
                              # the cell coordinates here instead would read the map at the
                              # robot's position in the COURSE, which for a robot 1 m along
                              # is a metre of terrain it has already walked over.
                              xk, yk = 0.0, _DTM_Y_HALF
                          else:
                              xk = float(pos_w[k, 0] - offs[k, 0])
                              yk = float(pos_w[k, 1] - offs[k, 1])
                          try:
                              obs = extract(pl_tmaps[k], xk, yk, la, pcfg)
                          except Exception:
                              continue
                          dec = pl[k].step(obs, tick_every * dt, x_m=xk,
                                           heading_err_deg=float(head_err[k]),
                                           speed_m_s=spd)
                          want = dec.active
                          if want is pl_sk[k]:
                              continue
                          if want.value not in pl_clips:
                              # No exception path: CLAUDE.md 2 forbids removing a task the
                              # library cannot serve, so an unserviceable request holds the
                              # current skill and is counted.
                              pl_refused[k] += 1
                              continue
                          # Land in the new clip at a phase whose stance/swing assignment
                          # agrees with the feet actually loaded -- proprioception only, the
                          # same rule run_planner_replay.py uses.
                          nm = want.value
                          cl = pl_sw[nm]
                          cur = pl_q[pl_sk[k].value][pl_ph[k]]
                          agree = (cl == pl_sw[pl_sk[k].value][pl_ph[k]][None, :]).sum(axis=1)
                          cand = np.flatnonzero(agree == agree.max())
                          d_ = np.abs(pl_q[nm][cand] - cur[None, :]).max(axis=1)
                          pl_ph[k] = int(cand[int(np.argmin(d_))])
                          # Leaving a deliberate TURN: the heading the robot now has IS
                          # the heading it meant to have, so that becomes what heading hold
                          # holds.  Without this the low level spends the next seconds
                          # undoing the turn the planner just paid for -- the two terms
                          # would be fighting, and the planner would lose because heading
                          # hold runs every control step and TURN runs once.
                          if args.turn_target == "goal" and pl_sk[k] is _SID.TURN \
                                  and want is not _SID.TURN:
                              yaw_ref[k] = yaw_now[k]
                              pl_foot[k][want.value].reset()
                          pl_sk[k] = want
                          pl_switches[k] += 1
                          pl_foot[k][nm].reset()
                          if pl_yaw[k][nm] is not None:
                              pl_yaw[k][nm].reset()
                          pl_roll[k][nm].reset()
                  # ---- one control step of whatever is playing -----------------------
                  for k in range(n):
                      nm = pl_sk[k].value
                      ph = pl_ph[k] % len(pl_q[nm])
                      # Only while the robot is still up: a fallen robot keeps being commanded
                      # to the shared step budget, and counting those steps made the three
                      # fractions sum to more than 1.
                      if alive[k]:
                          pl_held[nm][k] += 1
                      cmd[k] = pl_q[nm][ph]
                      if alive[k]:
                          cmd[k] = cmd[k] + pl_foot[k][nm].step(
                              float(vb[k, 1]), float(wb[k, 2]), pl_sw[nm][ph],
                              vx=float(vb[k, 0]), psi_err_rad=float(psi[k]))
                          if pl_yaw[k][nm] is not None:
                              tau_pl[k] = pl_yaw[k][nm].step(pl_sw[nm][ph],
                                                             psi_err_rad=float(psi[k]))
                          # ADDED, not replacing: the two couples are duals on one joint
                          # (uniform c gives roll, sign(x)*c gives yaw), so their torques
                          # superpose and the cap check above is what keeps the sum inside
                          # the actuator.
                          tau_pl[k] = tau_pl[k] + pl_roll[k][nm].step(
                              pl_sw[nm][ph], roll_rad=float(np.radians(roll_now[k])),
                              roll_rate_rad_s=float(wb[k, 0]))
                      pl_ph[k] = (pl_ph[k] + 1) % len(pl_q[nm])
              elif foots is not None:
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
                  roll_now, _pitch_now, _ = quat_to_rpy_deg(snap(robot.data.root_quat_w))
                  roll_now = np.asarray(roll_now, float)
                  # CROSS-TRACK HOLD.  Heading hold closes a loop on the yaw ANGLE and
                  # measures 13.26 -> 0.24 deg/m; nothing closes one on lateral POSITION,
                  # and the robot drifts 0.53 m sideways per metre forward with 149 of 200
                  # cells going the same way.  This asks the lateral law for a velocity
                  # that returns to the start line: v_y,target = -gain * (y - y_ref),
                  # bounded.  Body frame, so the error is rotated by the current yaw.
                  # Measured on EVERY run, applied only when the flag is on, so the
                  # paired off-arm reports its own excursion instead of a column of zeros.
                  pw = snap(robot.data.root_pos_w)
                  _, _, _yaw = quat_to_rpy_deg(snap(robot.data.root_quat_w))
                  _yr = np.radians(np.asarray(_yaw, float))
                  ex = pw[:, 0].astype(float) - x_ref
                  ey = pw[:, 1].astype(float) - y_ref
                  # cross-track error expressed in the body's lateral axis
                  ect = -np.sin(_yr) * ex + np.cos(_yr) * ey
                  ct_err[:] = ect
                  ct_abs_max[:] = np.where(alive, np.maximum(ct_abs_max, np.abs(ect)),
                                           ct_abs_max)
                  if args.cross_track != "off":
                      vy_b = np.clip(-args.cross_track_gain * ect,
                                     -args.cross_track_cap, args.cross_track_cap)
                  for k in range(n):
                      if alive[k]:
                          if args.cross_track != "off":
                              foots[k].vy_bias = float(vy_b[k])
                          cmd[k] = cmd[k] + foots[k].step(float(vb[k, 1]), float(wb[k, 2]),
                                                          swing_seq[frame], vx=float(vb[k, 0]),
                                                          psi_err_rad=float(psi[k]))
                          tau_pl[k] = rolls[k].step(
                              swing_seq[frame], roll_rad=float(np.radians(roll_now[k])),
                              roll_rate_rad_s=float(wb[k, 0]))
              tgt = robot.data.default_joint_pos.clone()
              tgt[:, idx_t] = torch.as_tensor(cmd, device=sim.device, dtype=torch.float32)
              robot.set_joint_position_target(tgt)
              if (PLANNER_ARM and args.yaw_moment != "off") or args.roll_couple != "off":
                  # Written EVERY step including the all-zero ones: an effort target is
                  # sticky, so skipping the write leaves the last non-zero torque applied.
                  eff = torch.zeros_like(tgt)
                  eff[:, idx_t] = torch.as_tensor(tau_pl, device=sim.device,
                                                  dtype=torch.float32)
                  robot.set_joint_effort_target(eff)
              for _ in range(decim):
                  robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)

              if video is not None and step % args.video_stride == 0:
                  grab_frame()

              pos = snap(robot.data.root_pos_w)
              idx = tracker.update(pos[:, :2])
              roll, pitch, yaw_t = quat_to_rpy_deg(snap(robot.data.root_quat_w))
              roll = np.asarray(roll, float); pitch = np.asarray(pitch, float)
              yaw_t = np.asarray(yaw_t, float)
              # WHICH AXIS, and how far it got before the cutoff.  The termination test is
              # an OR of three conditions and the row used to record only that one of them
              # fired, which is not enough to aim a controller: "it fell" does not say
              # whether it fell sideways, forwards, or through the floor.  Peaks are kept
              # while the robot is alive only -- a fallen robot keeps rolling.
              live = alive
              upright[live] += 1
              if yaw_prev is None:
                  yaw_prev = yaw_t.copy()
              _d = (yaw_t - yaw_prev + 180.0) % 360.0 - 180.0
              yaw_cum[live] += _d[live]
              yaw_prev = yaw_t
              base_h_max[live] = np.maximum(base_h_max[live], pos[live, 2])
              peak_roll[live] = np.maximum(peak_roll[live], np.abs(roll[live]))
              peak_pitch[live] = np.maximum(peak_pitch[live], np.abs(pitch[live]))
              over_r = np.abs(np.radians(roll)) > ROLL_PITCH_CUTOFF_RAD
              over_p = np.abs(np.radians(pitch)) > ROLL_PITCH_CUTOFF_RAD
              over_z = (pos[:, 2] - ground_z) < HEIGHT_CUTOFF_M
              dead = over_r | over_p | over_z | (idx >= NUM_GOALS)
              newly = alive & dead
              # First cause wins, and they are recorded separately because a robot can trip
              # two at once on the way down.
              end_cause[newly & over_r] = "roll"
              end_cause[newly & over_p & ~over_r] = "pitch"
              end_cause[newly & over_z & ~over_r & ~over_p] = "below"
              end_cause[newly & (idx >= NUM_GOALS) & ~over_r & ~over_p & ~over_z] = "goals"
              end_roll[newly] = roll[newly]; end_pitch[newly] = pitch[newly]
              ended[newly] = idx[newly]
              alive &= ~newly
              if not alive.any():
                  break
          end_cause[alive] = "timeout"
          end_roll[alive] = roll[alive]; end_pitch[alive] = pitch[alive]
          ended[alive] = tracker.idx[alive]        # the rest time out, upstream's way
          # Where each robot finished, and how far it still had to go.  Recorded because the
          # integer goal count has about three and a half units of usable range.
          fin_xy = snap(robot.data.root_pos_w)[:, :2]
          dist_next = np.array([np.linalg.norm(fin_xy[k] - goals[k, min(ended[k], NUM_GOALS - 1)])
                                for k in range(n)])
          dist_last = np.array([np.linalg.norm(fin_xy[k] - goals[k, NUM_GOALS - 1])
                                for k in range(n)])
          # The commanded support pattern each robot actually ran (CLAUDE.md 2.5).  A
          # single-skill arm holds one clip for the whole episode; a planner robot holds
          # its own mixture, so it is weighted by the time that robot spent in each.
          if PLANNER_ARM:
              _sup = {nm: support_stats(c["contact"]) for nm, c in pl_clips.items()}
              cmd_feet = [sum(_frac(pl_held, nm, k) * _sup[nm][0]
                              for nm in pl_held if nm in _sup) for k in range(n)]
              cmd_b3 = [sum(_frac(pl_held, nm, k) * _sup[nm][1]
                            for nm in pl_held if nm in _sup) for k in range(n)]
          else:
              _m, _b = support_stats(clip["contact"])
              cmd_feet = [_m] * n
              cmd_b3 = [_b] * n
          for k, (t, l) in enumerate(cells):
              rows_out.append({"task": t, "task_name": names[t], "level": l, "episode": ep,
                               "goals": int(ended[k]), "settle_ok": int(settle_ok[k]),
                               # Which arm this row is.  A results file whose rows do not
                               # say whether heading hold was on is one that gets read wrong
                               # once: `heading` and `heading-only` are not the same
                               # controller, and neither is `off`.
                               "heading": args.heading, "heading_cap_rad": hcap,
                               # The user asked for resolution beyond the goal count: the
                               # benchmark's useful range is about 1 to 4.5 of 8 (goal 0 sits
                               # 0.50 m from the spawn in 180 of the 200 cells), so a whole
                               # arm can move and the integer score not notice.
                               "final_x_m": float(fin_xy[k, 0] - offs[k, 0]),
                               "final_y_m": float(fin_xy[k, 1] - offs[k, 1]),
                               "travelled_m": float(np.linalg.norm(fin_xy[k] - spawns[k])),
                               "dist_to_next_goal_m": float(dist_next[k]),
                               "dist_to_last_goal_m": float(dist_last[k]),
                               "alive_at_end": int(alive[k]),
                               "end_cause": end_cause[k],
                               "upright_s": float(upright[k] * dt),
                               "yaw_cum_deg": float(yaw_cum[k]),
                               "yaw_rate_deg_s": float(yaw_cum[k] / max(upright[k] * dt, 1e-9)),
                               "base_h_max_m": float(base_h_max[k]),
                               # Forward speed along the CELL's x, over the time the robot
                               # was actually up.  The gait is an open-loop clip replayed at
                               # a fixed rate, so STRIDE cannot move -- the phase advance is
                               # a constant of the harness, not a result.  Speed can, and a
                               # balance term that buys survival by slowing down has bought
                               # nothing, so this is the column that says it did not.
                               "vx_mean_ms": (float((fin_xy[k, 0] - spawns[k][0])
                                                    / max(upright[k] * dt, 1e-9))),
                               # LATERAL speed, the companion to vx.  Heading hold pulls
                               # the yaw angle back to the settle heading and nothing pulls
                               # the lateral POSITION back, so a robot can face straight
                               # down the lane and still leave it sideways.  vx alone
                               # cannot show that; vy and the cross-track column below can.
                               "vy_mean_ms": (float((fin_xy[k, 1] - spawns[k][1])
                                                    / max(upright[k] * dt, 1e-9))),
                               "cross_track_m": float(fin_xy[k, 1] - y_ref[k]),
                               "cross_track_abs_max_m": float(ct_abs_max[k]),
                               # Path curvature, rad per metre travelled: the yaw rate
                               # divided by the forward speed.  A gait that turns because
                               # it is slow and a gait that turns because it is curving
                               # have the same yaw rate and different curvature.
                               "curvature_rad_m": (
                                   float(np.radians(yaw_cum[k])
                                         / (fin_xy[k, 0] - spawns[k][0]))
                                   if abs(fin_xy[k, 0] - spawns[k][0]) > 1e-3 else ""),
                               # Did this robot fall?  end_cause carries the detail; this
                               # is the column a falls-per-minute aggregate divides by, and
                               # a timeout is not a fall.
                               "fell": int(end_cause[k] in ("roll", "pitch", "below")),
                               # This robot's own episode length in seconds -- the time it
                               # was up, which is the episode for everything but a timeout.
                               # mean(episode_s) is the "average episode length" and
                               # sum(episode_s) is the denominator of falls per minute.
                               "episode_s": float(upright[k] * dt),
                               "end_roll_deg": float(end_roll[k]),
                               "end_pitch_deg": float(end_pitch[k]),
                               "peak_roll_deg": float(peak_roll[k]),
                               "peak_pitch_deg": float(peak_pitch[k]),
                               "switches": int(pl_switches[k]) if PLANNER_ARM else "",
                               "refused_ticks": int(pl_refused[k]) if PLANNER_ARM else "",
                               # Fractions of the steps this robot was UP: they sum to 1.
                               "steps_alive": (int(sum(pl_held[s_][k] for s_ in pl_held))
                                               if PLANNER_ARM else ""),
                               "frac_WALK": _frac(pl_held, "WALK", k) if PLANNER_ARM else "",
                               "frac_TROT": _frac(pl_held, "TROT", k) if PLANNER_ARM else "",
                               "frac_TURN": _frac(pl_held, "TURN", k) if PLANNER_ARM else "",
                               "yaw_moment": args.yaw_moment,
                               "yaw_moment_gain": args.yaw_moment_gain,
                               # WHICH TERRAIN.  A legged_eval row and a frozen row are
                               # not two readings of one benchmark and must never be
                               # averaged together; this column is how that stays visible
                               # after the run has scrolled off.
                               "terrain": terr, "planner_set": planner_stamp,
                               "perception": args.perception,
                               "turn_target": args.turn_target,
                               "swing_lift_mm": args.swing_lift,
                               "foot_yaw_turn": args.foot_yaw_turn,
                               "roll_couple": args.roll_couple,
                               "roll_gain": args.roll_gain if args.roll_couple != "off" else "",
                               "roll_damp": args.roll_damp if args.roll_couple != "off" else "",
                               "roll_cap_nm": args.roll_cap_nm if args.roll_couple != "off" else "",
                               "roll_sign": args.roll_sign if args.roll_couple != "off" else "",
                               "partial": int(n < full), "skill": args.skill,
                               "start_phase": args.start_phase, "foot_comp": args.foot_comp,
                               "steps": step + 1,
                               # ---- run conditions, stamped on EVERY row -----------------
                               # so that a results file can be aggregated months later
                               # without the log beside it, and so that two files cannot be
                               # pooled without the difference being visible in the data.
                               "terrain_seed": args.terrain_seed,
                               # There is no separate MEASUREMENT seed.  The arms replay a
                               # recorded clip open loop with no sampling anywhere, which is
                               # why the recorded and unrecorded runs of the same settings
                               # come out bit-identical on every column (SESSION_STATE 5).
                               # The terrain seed is the only randomness in a run.
                               "measurement_seed": "deterministic",
                               "episode_length_s": EPISODE_LENGTH_S,
                               "episodes": args.episodes,
                               # Robots in the scene = cells x episodes-per-batch.  We run
                               # 200 (one per cell); legged_eval's own teacher runs report
                               # num_envs 1000, which is the same 200 cells with 5 episodes
                               # each.  Same protocol, five times the sample.
                               "num_envs": n,
                               "rate": args.rate,
                               "gutter": args.gutter,
                               "spawn_z": args.spawn_z,
                               "settle_s": args.settle_s,
                               "foot_clip_rad": args.foot_clip_rad,
                               # HOW THE ROBOT IS STEERED.  Not "self" (nothing chooses a
                               # direction from what it senses) and not "commander" (no
                               # waypoint or velocity command reaches the gait).  Heading
                               # hold remembers the heading the robot settled at and pulls
                               # the yaw error back to it; the goals are scored against but
                               # never steered toward.  That is a third category and it is
                               # named rather than forced into one of the other two.
                               "steering": ("dead-reckoned-heading-hold"
                                            if args.heading != "off" else "open-loop"),
                               "cross_track": args.cross_track,
                               "cross_track_gain": (args.cross_track_gain
                                                    if args.cross_track != "off" else ""),
                               "cross_track_cap": (args.cross_track_cap
                                                   if args.cross_track != "off" else ""),
                               # The commanded support pattern this clip carries, from
                               # scripts/support_polygon.py.  The CLAUDE.md 2.5 gate is
                               # about this quantity, so it travels with the row.
                               "cmd_mean_feet_down": round(float(cmd_feet[k]), 3),
                               "cmd_frac_below_3_feet": round(float(cmd_b3[k]), 4)})
          if args.roll_couple != "off":
              # Key off the ARM, not off `rolls is not None`.  `rolls` is built whenever
              # --foot-comp is on, which includes the planner arm, where it is never
              # stepped -- reading it there reported "0 stance-leg-steps driven" for a term
              # that was working, which is the shape of failure this project keeps hitting:
              # a plausible number about the wrong object.  And count EVERY per-clip
              # object, not the final skill's: a robot that ends on TURN did not spend the
              # episode there.
              _rcs = ([v for d in pl_roll for v in d.values()] if PLANNER_ARM
                      else [rolls[k] for k in range(n)])
              _ap = sum(r.applied for r in _rcs)
              _ch = sum(r.cap_hits for r in _rcs)
              _mx = max((r.max_abs_nm for r in _rcs), default=0.0)
              print(f"[bench] roll couple: {_ap} stance-leg-steps driven, "
                    f"{100.0*_ch/max(_ap,1):.1f}% of them at the cap, peak hip torque from "
                    f"this term {_mx:.2f} N.m against the {hip_effort_limit_nm:.2f} N.m "
                    f"limit. A term that sits at its cap has no room to regulate; a term "
                    f"that never reaches it is not the binding constraint.")
          print(f"[bench] episode {ep+1}/{args.episodes}: goals reached "
                f"min {ended.min()} median {np.median(ended):.1f} max {ended.max()}, "
                f"{int((ended >= NUM_GOALS).sum())}/{n} cells completed the course "
                f"({time.time()-t0:.0f}s)")
    except Exception as _exc:
        _loud(_exc)
        raise
    finally:
        if video is not None and "writers" in video:
            for w in video["writers"].values():
                w.close()
            print(f"[bench] video: {video['frames']} frames per clip -> {args.video}")
            print("[bench] REGRESSION CHECK: re-run these cells without --video and "
                  "compare `goals` and `steps` per row. Recording only adds render() "
                  "calls between control steps; if any row moves, it did not.")
    return rows_out, aggregate(rows_out), n < full


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skill", default="TROT",
                    help="clip to hold on every cell, or PLANNER for the "
                         "Rule-Planner arm (each robot picks its own)")
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo")
    ap.add_argument("--terrain", choices=TERRAIN_SOURCES, default="legged_eval",
                    help="legged_eval (default): the courses generated by legged_eval, "
                         "WITH its roughness and its border walls, seeded off --terrain-seed. "
                         "frozen: data/benchmark_frozen.npz, which has neither and draws "
                         "from upstream's own seed. The two are different terrain (180 of "
                         "200 cells differ), so their scores may not be compared; every "
                         "row is stamped with which one it was.")
    ap.add_argument("--terrain-seed", type=int, default=1,
                    help="legged_eval's terrain draw. Seed 1 is the same 20 courses for "
                         "every model; 1/2/3 are three draws whose spread is the error bar.")
    ap.add_argument("--no-roughness", action="store_true",
                    help="drop legged_eval's uniform noise. Off the protocol -- for "
                         "isolating the noise's effect, not for scoring.")
    ap.add_argument("--no-border-walls", action="store_true",
                    help="drop legged_eval's 0.1 m / 0.5 m rim. Off the protocol.")
    ap.add_argument("--tasks", type=int, nargs="*", default=None)
    ap.add_argument("--levels", type=int, nargs="*", default=None)
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--gutter", type=float, default=1.0)
    ap.add_argument("--spawn-z", type=float, default=0.42)
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--start-phase", choices=("first", "stance", "level", "measured"),
                    default="first")
    ap.add_argument("--foot-comp", choices=("off", "on"), default="on")
    ap.add_argument("--yaw-moment", choices=("off", "hold"), default="off",
                    help="TROT's stance-leg yaw couple (sim/yawmoment.py), planner arm only. "
                         "off (default) is every earlier benchmark run.")
    ap.add_argument("--yaw-moment-gain", type=float, default=5.0,
                    help="N.m of hip torque per rad of heading error; 5 is the measured "
                         "operating point (trot_yaw_moment.md 3)")
    ap.add_argument("--yaw-moment-nm", type=float, default=0.0)
    ap.add_argument("--yaw-moment-cap-nm", type=float, default=2.0)
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
    ap.add_argument("--cross-track", choices=("off", "hold"), default="off",
                    help="CROSS-TRACK HOLD. off (default) reproduces every earlier run bit "
                         "for bit. hold: ask the lateral placement law for a velocity that "
                         "returns to the y the robot SETTLED at. Heading hold closes a loop "
                         "on the yaw ANGLE (13.26 -> 0.24 deg/m) and nothing closes one on "
                         "lateral POSITION, so a robot faces straight down the lane and "
                         "leaves it sideways: measured 0.53 m of drift per metre forward, "
                         "149 of 200 cells the same way. Same standing as heading hold -- "
                         "the datum is the robot's own start line, no goal and no terrain "
                         "is read, and the planner never sees it.")
    ap.add_argument("--cross-track-gain", type=float, default=0.5, metavar="PER_S",
                    help="m/s of lateral velocity target per metre of cross-track error. "
                         "Sign and value are to be settled by measurement, not derivation.")
    ap.add_argument("--cross-track-cap", type=float, default=0.10, metavar="MS",
                    help="bound on the requested lateral velocity, m/s. WALK's own logged "
                         "v_y is a few cm/s, so a cap well above that is asking the gait "
                         "for something it has never done.")
    ap.add_argument("--allow-partial", action="store_true",
                    help="run fewer than all 200 cells. The printed mean is then a wiring "
                         "check and NOT a benchmark score; every row is stamped partial=1")
    ap.add_argument("--video", default=None,
                    help="directory for side-view mp4s, one per --video-cells entry. "
                         "NEEDS A GPU and --enable_cameras; without a card Isaac Lab does "
                         "not fail, it hangs in carb.cudainterop. Record in its own run "
                         "and check the scores against the unrecorded one.")
    ap.add_argument("--video-cells", type=int, nargs="*", default=None,
                    help="which cells to film, as indices into the SELECTED cell list "
                         "(--plan prints it). Default: cell 0.")
    ap.add_argument("--video-width", type=int, default=960)
    ap.add_argument("--video-height", type=int, default=540)
    ap.add_argument("--video-stride", type=int, default=1,
                    help="capture one frame every N control steps; fps is set from it so "
                         "the clip stays real time")
    ap.add_argument("--video-side-m", type=float, default=2.4,
                    help="camera offset across the lane, metres. The lane is 4 m wide, so "
                         "2.4 puts the camera 0.4 m OUTSIDE the patch -- and legged_eval's "
                         "0.1 m rim, raised 0.5 m, is then between it and the robot.")
    ap.add_argument("--video-eye-m", type=float, default=0.95,
                    help="camera height floor, metres. THE RIM IS WHY THIS EXISTS. At the "
                         "default 2.4 m offset the sight line crosses the wall at y = 3.95, "
                         "which is 0.1875 of the way from eye to target; at the old eye "
                         "height of base+0.12 that crossing is 0.40 m and the rim is "
                         "0.495 m, so the first recording was 20 s of a grey wall. 0.95 m "
                         "puts the crossing at 0.82 m -- clear by 0.33 -- for a 16 deg "
                         "look-down, which still reads as a side view.")
    ap.add_argument("--foot-yaw-turn", choices=("auto", "off"), default="auto",
                    help="auto (default): a TURN clip's placement law is given the log's own "
                         "yaw rate, which is what turn_target.md established and what the "
                         "planner arm already does. off: reproduces the single-skill TURN "
                         "rows measured before 2026-09-02, which had it at zero.")
    ap.add_argument("--swing-lift", type=float, default=0.0, metavar="MM",
                    help="raise every swing foot's apex to this height, mm. The earlier "
                         "ladder (0/40/60/80) found the untouched recording best, but that "
                         "was open loop -- run this PAIRED with --roll-couple.")
    ap.add_argument("--swing-lift-asym", action="store_true",
                    help="per-leg rather than mirrored pairs")
    ap.add_argument("--turn-target", choices=("settle", "goal"), default="settle",
                    help="what the planner's TURN rule is asked. settle (default, and what "
                         "every earlier run used): deviation from the heading the robot "
                         "settled at. goal: bearing to the live waypoint minus heading, "
                         "which is what RulePlanner._wants_turn documents as its argument. "
                         "goal also re-baselines heading hold when a TURN ends, so the low "
                         "level holds the heading the planner just chose instead of "
                         "undoing it.")
    ap.add_argument("--roll-couple", choices=("off", "hold"), default="off",
                    help="LEVEL 2. The stance-leg roll couple (sim/attitude.py): uniform "
                         "feed-forward hip torque on the legs the recording has down, "
                         "which is a roll moment and (to the 3%% front/rear mismatch) no "
                         "yaw. off (default) reproduces every earlier run bit for bit. "
                         "Aimed at the only failure the grid has: 200 of 200 terminations "
                         "are roll, none is pitch.")
    ap.add_argument("--roll-gain", type=float, default=8.0,
                    help="N.m of hip torque per rad of roll error")
    ap.add_argument("--roll-damp", type=float, default=0.8,
                    help="N.m per rad/s of roll rate. The disturbance is a 2.7 Hz texture, "
                         "so the rate term is the half that can see it coming.")
    ap.add_argument("--roll-cap-nm", type=float, default=2.0,
                    help="magnitude bound on the couple. Refused if it plus the yaw "
                         "couple's cap exceeds half the hip's effort limit.")
    ap.add_argument("--roll-couple-nm", type=float, default=0.0,
                    help="constant open-loop bias, for the SIGN probe: run with --roll-gain 0 "
                         "and see which way the robot rolls. The derived sign is written "
                         "down in sim/attitude.py and is not trusted (footcomp.py).")
    ap.add_argument("--roll-sign", type=float, default=1.0, choices=(1.0, -1.0),
                    help="flip the derived sign if the open-loop probe says so")
    ap.add_argument("--perception", choices=("optimistic", "depth"), default=None,
                    help="what the Rule-Planner is allowed to see. DEFAULT depth: the "
                         "rendered depth image, inverted by legged_eval's depth_terrain -- "
                         "occluded, noisy, and what the experiment is about. Needs a GPU "
                         "and --enable_cameras. optimistic: the terrain's own height field "
                         "through planner.features' sensor model, never occluded and never "
                         "noisy -- the perception UPPER BOUND, a control arm, not the "
                         "result. Ignored by the single-skill arms, which read no terrain.")
    ap.add_argument("--planner-set", nargs="*", default=None, metavar="group.FIELD=VALUE",
                    help="override planner thresholds for this run only, e.g. "
                         "--planner-set skill.STEP_TROT_MAX=0.03. planner/config.py is NOT "
                         "written to: a CALIBRATION_NEEDED placeholder stays one. Every row "
                         "is stamped with what was overridden.")
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
    z, _grid = load_terrain(args)
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
