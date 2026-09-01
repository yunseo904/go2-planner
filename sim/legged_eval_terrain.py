"""The benchmark grid, built by ``legged_eval`` instead of read from our archive.

WHY THIS FILE EXISTS
--------------------
``data/benchmark_frozen.npz`` and ``legged_eval`` are the same 20 eurekaverse courses at
the same 18 x 4 m / 0.05 m geometry, generated from the same upstream module.  They are
not the same terrain:

    roughness      archive: none.        legged_eval: random_uniform_terrain, amplitude
                                         drawn from (0.02, 0.04) m, downsampled 0.075 m
    border walls   archive: none.        legged_eval: 0.1 m rim raised 0.5 m, all four
                                         sides of every patch
    terrain seed   archive: upstream's   legged_eval: crc32 of (course name, difficulty)
                   set_seed(variation*1e3 plus the run seed, so seed 1 is the same
                   + difficulty*1e6)     draw for every model and seeds 1/2/3 are three
                                         draws whose spread is the error bar

Both of the first two make the course HARDER and both are what eurekaverse's own
``terrain_gpt.py`` puts on every patch, so the archive was measuring an easier benchmark
than the one the E2E numbers come from.  CLAUDE.md 3 said as much ("동결본은 upstream
대비 random_uniform_terrain 노이즈와 셀 테두리 0.5m 패드가 미적용").  This module closes
that gap by taking the terrain from legged_eval rather than reproducing it.

HOW IT ATTACHES
---------------
Not by copying legged_eval's code.  ``legged_eval.terrain.build_terrain_class`` wants a
legged_gym ``Terrain`` to subclass, and the only things it asks of that base are a cfg, a
patch size in pixels, and an ``add_terrain_to_map`` to hand each finished patch to.
``_CollectingHost`` below is exactly that and nothing more: it collects.  Every decision
that defines the terrain -- which course, which difficulty, the seeding, the noise, the
rim, the goals, the spawn -- runs inside legged_eval.

That matters more than the twenty lines it saves.  The thing legged_eval exists to
prevent is two copies of a benchmark drifting apart with no record in the result files;
re-implementing its epilogue here would be a third copy.

WHAT WE STILL OWN
-----------------
Layout.  legged_eval places its patches edge to edge because its hosts' robots live in
one legged_gym terrain grid; ``run_benchmark.py`` lays cells out with a gutter between
them, in Isaac Lab, on a mesh it builds itself.  So the world offsets legged_eval
computes are discarded and the LOCAL fields are kept -- height field, goals, spawn --
which is the part that has to be identical.

legged_eval is READ ONLY (it is someone else's).  Nothing here writes to it.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

#: Where legged_eval lives.  It is pip install -e'd on the host, but the Isaac Lab
#: container has its own site-packages, so a path is needed there.  Order: an explicit
#: env var, then an ordinary import, then the two known checkouts.
_CANDIDATES = ("/opt/legged_eval", "~/legged_eval", "~/ev_image/legged_eval")


def _import_legged_eval():
    root = os.environ.get("LEGGED_EVAL_ROOT")
    if root:
        root = os.path.expanduser(root)
        if root not in sys.path:
            sys.path.insert(0, root)
    try:
        from legged_eval import terrain as _t          # noqa: F401
        return _t
    except ImportError:
        pass
    for cand in _CANDIDATES:
        cand = os.path.expanduser(cand)
        if os.path.isfile(os.path.join(cand, "legged_eval", "terrain.py")):
            if cand not in sys.path:
                sys.path.insert(0, cand)
            from legged_eval import terrain as _t
            return _t
    raise SystemExit(
        "legged_eval not importable.  It is the benchmark's definition, not an optional\n"
        "dependency: set LEGGED_EVAL_ROOT, or mount ~/legged_eval into the container\n"
        "(scripts/isaac_docker_run.sh does this at /opt/legged_eval).")


# --------------------------------------------------------------------------- #
# The protocol, read off legged_eval rather than restated
# --------------------------------------------------------------------------- #

def protocol_defaults() -> dict:
    """legged_eval's own Protocol() defaults, so a divergence here is impossible."""
    _import_legged_eval()
    from legged_eval.protocol import Protocol
    p = Protocol()
    return {
        "terrain_length": p.terrain_length,
        "terrain_width": p.terrain_width,
        "terrain_resolution": p.terrain_resolution,
        "rows": p.rows,
        "difficulty_range": tuple(p.difficulty_range),
        "border_walls": p.border_walls,
        "roughness": p.roughness,
        "roughness_height": tuple(p.roughness_height),
        "roughness_downsampled_scale": p.roughness_downsampled_scale,
        "num_goals": p.num_goals,
        "episode_length_s": p.episode_length_s,
        "next_goal_threshold": p.next_goal_threshold,
        "reach_goal_delay": p.reach_goal_delay,
        "roll_pitch_cutoff": p.roll_pitch_cutoff,
        "height_cutoff": p.height_cutoff,
        "spawn_height": p.spawn_height,
        "goal_speed": p.goal_speed,
        "steps": p.steps,
    }


#: Isaac Lab needs a vertical quantisation for the int16 field.  Not in legged_eval's
#: Protocol because there it comes from the host repository's own terrain cfg; upstream
#: extreme-parkour uses 0.005 m and so does our frozen archive, so the mesh is unchanged.
VERTICAL_SCALE = 0.005


@dataclass
class Cell:
    """One (course, difficulty) patch, in patch-local metres."""

    course: str
    task: int                  # index into ``BenchmarkGrid.names``
    level: int                 # difficulty index, 0..rows-1
    difficulty: float
    hf_raw: np.ndarray         # (nx, ny) int16, units of vertical_scale, walls + noise in
    hf_clean_m: np.ndarray     # (nx, ny) float64 metres, course only -- what goals mean
    goals: np.ndarray          # (num_goals, 2) metres, eurekaverse's own coordinates
    goal_z: np.ndarray         # (num_goals,) ground height under each goal
    spawn: Tuple[float, float, float]


@dataclass
class BenchmarkGrid:
    names: List[str]
    difficulties: np.ndarray
    cells: Dict[Tuple[int, int], Cell]
    horizontal_scale: float
    vertical_scale: float
    terrain_length: float
    terrain_width: float
    seed: int
    roughness: bool
    border_walls: bool
    roughness_height: Tuple[float, float]
    nx: int
    ny: int

    @property
    def num_rows(self) -> int:
        return len(self.difficulties)

    @property
    def num_cols(self) -> int:
        return len(self.names)

    def hf(self, task: int, level: int) -> np.ndarray:
        return self.cells[(task, level)].hf_raw

    def fingerprint(self) -> dict:
        """What has to match for two runs to be comparable.  Goes in the result file."""
        return {
            "source": "legged_eval",
            "courses": list(self.names),
            "rows": self.num_rows,
            "difficulty_range": [float(self.difficulties[0]), float(self.difficulties[-1])],
            "terrain_length": self.terrain_length,
            "terrain_width": self.terrain_width,
            "terrain_resolution": self.horizontal_scale,
            "vertical_scale": self.vertical_scale,
            "border_walls": self.border_walls,
            "roughness": self.roughness,
            "roughness_height": list(self.roughness_height),
            "terrain_seed": self.seed,
            "terrain_seed_mode": "follows-run-seed",
        }


class _Cfg(object):
    """The four fields legged_eval's grid builder reads off a legged_gym terrain cfg."""

    def __init__(self, hs, vs, num_rows, num_cols):
        self.horizontal_scale = hs
        self.vertical_scale = vs
        self.num_rows = num_rows
        self.num_cols = num_cols


class _CollectingHost(object):
    """The smallest thing ``build_terrain_class`` can subclass.

    A real legged_gym Terrain would turn the grid into a trimesh here.  We do not want
    its mesh -- ``run_benchmark.py`` builds one with gutters between cells -- so this
    keeps the patches and stops.  Everything upstream of ``add_terrain_to_map`` is
    legged_eval's.
    """

    def __init__(self, cfg, x_px, y_px):
        self.cfg = cfg
        self.length_per_env_pixels = x_px     # x, the 18 m travel direction
        self.width_per_env_pixels = y_px      # y, the 4 m lane
        self.collected = {}
        # legged_eval binds this name (and four other spellings) to its grid builder.
        self.curiculum()

    def curiculum(self):                      # replaced by the subclass
        raise NotImplementedError

    def add_terrain_to_map(self, terrain, row, col):
        rec = getattr(terrain, "_benchmark", None)
        if rec is None:
            raise RuntimeError(
                "legged_eval handed back a patch with no _benchmark record at "
                "(row {}, col {}).  That is its own marker that the course generator "
                "ran, so its absence means the grid was NOT built from the benchmark."
                .format(row, col))
        self.collected[(row, col)] = (np.array(terrain.height_field_raw, dtype=np.int16),
                                      rec)


def build(seed: int = 1,
          courses: Optional[Sequence[str]] = None,
          levels: Optional[Sequence[int]] = None,
          roughness: bool = True,
          border_walls: bool = True,
          vertical_scale: float = VERTICAL_SCALE) -> BenchmarkGrid:
    """Generate the benchmark grid.  ``seed`` is the run seed; the terrain follows it."""
    T = _import_legged_eval()
    d = protocol_defaults()
    hs = d["terrain_resolution"]
    rows = d["rows"] if levels is None else len(levels)

    all_names = T.course_names()
    names = list(all_names) if courses is None else list(courses)
    for n in names:
        if n not in all_names:
            raise SystemExit("unknown course {!r}.  legged_eval has: {}"
                             .format(n, ", ".join(all_names)))

    x_px = int(round(d["terrain_length"] / hs))
    y_px = int(round(d["terrain_width"] / hs))

    # The full ladder of difficulties, so that asking for a subset of levels still puts
    # each one at the difficulty it has in a full run.  A level's terrain must not depend
    # on how many levels were requested.
    lo, hi = d["difficulty_range"]
    n_full = d["rows"]
    full_diffs = np.array([lo + (hi - lo) * (i / max(n_full - 1, 1)) for i in range(n_full)])
    lv = list(range(n_full)) if levels is None else list(levels)

    cls = T.build_terrain_class(
        names, _CollectingHost,
        difficulty_range=d["difficulty_range"],
        border_walls=border_walls,
        seed=seed,
        roughness=roughness,
        roughness_height=d["roughness_height"],
        roughness_downsampled_scale=d["roughness_downsampled_scale"])

    cfg = _Cfg(hs, vertical_scale, n_full, len(names))
    host = cls(cfg, x_px, y_px)

    cells = {}
    for col, name in enumerate(names):
        for level in lv:
            hf_raw, rec = host.collected[(level, col)]
            if rec["name"] != name:
                raise RuntimeError(
                    "cell (level {}, col {}) holds course {!r}, expected {!r} -- the "
                    "column ordering is not what this module assumes"
                    .format(level, col, rec["name"], name))
            cells[(col, level)] = Cell(
                course=name, task=col, level=level,
                difficulty=float(full_diffs[level]),
                hf_raw=hf_raw,
                hf_clean_m=np.asarray(rec["height_field"], dtype=np.float64),
                goals=np.asarray(rec["goals"], dtype=np.float64),
                goal_z=np.asarray(rec["goal_z"], dtype=np.float64),
                spawn=tuple(float(v) for v in rec["spawn"]))

    return BenchmarkGrid(
        names=names, difficulties=full_diffs[lv], cells=cells,
        horizontal_scale=hs, vertical_scale=vertical_scale,
        terrain_length=d["terrain_length"], terrain_width=d["terrain_width"],
        seed=seed, roughness=roughness, border_walls=border_walls,
        roughness_height=d["roughness_height"], nx=x_px, ny=y_px)


def as_archive(grid: BenchmarkGrid) -> dict:
    """``grid`` in the key layout ``data/benchmark_frozen.npz`` uses.

    The harness, ``planner.features.maps_from_archive`` and the offline tools all read
    the archive's keys.  Presenting the generated grid the same way keeps one reader for
    both, so the only thing that changes between a legged_eval run and a frozen run is
    where the numbers came from -- which is exactly what we want to be able to A/B.

    ``height_fields_before_fix`` is the RASTERISED field: course + noise + rim.  That is
    what the robot's feet are on, so it is also what the optimistic-perception arm must
    be given.  Handing the planner the clean course instead would be more than optimistic,
    it would be a different terrain -- the noise is +-0.04 m and WALK's measured step
    limit is 0.04-0.06 m, so the noise is inside the range the planner makes decisions in.
    The clean course is kept alongside, under a name of its own, for diagnostics only.
    """
    T, L = grid.num_cols, grid.num_rows
    levels = sorted({l for (_, l) in grid.cells})
    hf = np.zeros((T, L, grid.nx, grid.ny), dtype=np.int16)
    clean = np.zeros((T, L, grid.nx, grid.ny), dtype=np.int16)
    goals = np.zeros((T, L, grid.cells[(0, levels[0])].goals.shape[0], 2), dtype=np.float64)
    for (t, l), c in grid.cells.items():
        li = levels.index(l)
        hf[t, li] = c.hf_raw
        clean[t, li] = np.rint(c.hf_clean_m / grid.vertical_scale).astype(np.int16)
        goals[t, li] = c.goals
    spawn = grid.cells[(0, levels[0])].spawn
    return {
        "task_names": np.array(grid.names),
        "num_rows": np.array(L), "num_cols": np.array(T),
        "horizontal_scale": np.array(grid.horizontal_scale),
        "vertical_scale": np.array(grid.vertical_scale),
        "terrain_length_m": np.array(grid.terrain_length),
        "terrain_width_m": np.array(grid.terrain_width),
        "x_cells": np.array(grid.nx), "y_cells": np.array(grid.ny),
        "num_goals": np.array(goals.shape[2]),
        "difficulties": np.asarray(grid.difficulties),
        "height_fields_before_fix": hf, "height_fields": hf,
        "clean_course_fields": clean,
        "goals_before_fix": goals, "goals": goals,
        "spawn_x": np.array(spawn[0]), "spawn_y": np.array(spawn[1]),
        "spawn_z": np.array(spawn[2]),
    }
