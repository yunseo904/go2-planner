"""Synthetic probe terrains for calibrating the planner's skill limits.

Why these exist
---------------
Ten planner parameters are ``CALIBRATION_NEEDED`` placeholders (see
``planner.config``): nothing in the curated logs measures them, because every
recorded session is on flat floor.  `CLAUDE.md` §2 forbids settling them by
tuning against benchmark scores -- that would be fitting the measuring
instrument to the thing being measured.  It requires a *separate* calibration
terrain instead, and this module generates it.

Isolation from the benchmark
----------------------------
Nothing here imports ``set_terrain_benchmark`` or any upstream module, and no
generator is shared with ``terrain_toolkit.freeze``.  The geometry is written
directly from the parameters below: flat ground with one obstacle, no noise, no
randomness (there is no RNG call in this file at all, so there is no seed to
collide with the benchmark's).  A probe cannot leak benchmark structure into a
threshold because it contains none.

Probe design
------------
**One obstacle per terrain.**  A staircase of increasing steps would let a
failure at 0.14 m contaminate the reading at 0.16 m; an isolated obstacle gives a
clean pass/fail per level.  Each probe is a flat lane with a run-up, the
obstacle, and a landing area long enough to recover on:

    spawn 1.0 m ---- approach ---- obstacle at 4.0 m ---- landing ---- 8.0 m

``step_up``    plane at 0, rising to +h after the obstacle line (h = 0.02..0.30 by 0.02)
``step_down``  plane at 0, dropping to -h after it            (same 15 heights)
``gap``        plane at 0 with a pit at -1.0 m of width w     (w = 0.05..0.60 by 0.05)

The pit depth matches the benchmark generators' -1.0 m so that "gap" means the
same thing to ``planner.features`` in both places; that is a shared *constant*,
not shared code.

Goals mirror the benchmark convention (spawn, then goals in metres) so the same
traversal harness runs unchanged: one goal before the obstacle, one just past it,
one at the end.  Reaching goal 2 is what "cleared it" means.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np

#: Geometry shared by every probe.  Cell is smaller than the benchmark's 18 m
#: because a probe needs one obstacle, not eight goals' worth of course.
PROBE_LENGTH_M = 8.0
PROBE_WIDTH_M = 4.0
HORIZONTAL_SCALE = 0.05
VERTICAL_SCALE = 0.005
SPAWN_X, SPAWN_Y = 1.0, 2.0
OBSTACLE_X = 4.0
#: Benchmark generators carve pits at exactly this depth; matching it keeps
#: ``FeatureParams.PIT_THRESH`` meaningful on both.
PIT_DEPTH_M = -1.0

STEP_HEIGHTS_M = np.round(np.arange(0.02, 0.30 + 1e-9, 0.02), 3)
GAP_WIDTHS_M = np.round(np.arange(0.05, 0.60 + 1e-9, 0.05), 3)


@dataclass
class Probe:
    """One calibration terrain."""

    family: str
    level: int
    param_m: float
    height_field: np.ndarray          # int16, units of VERTICAL_SCALE, [x, y]
    goals_m: np.ndarray               # (3, 2) metres
    description: str

    @property
    def name(self) -> str:
        return f"{self.family}_{self.param_m:.2f}".replace(".", "p")


def _blank() -> np.ndarray:
    x_cells = int(round(PROBE_LENGTH_M / HORIZONTAL_SCALE))
    y_cells = int(round(PROBE_WIDTH_M / HORIZONTAL_SCALE))
    return np.zeros((x_cells, y_cells), dtype=np.int16)


def _to_raw(metres: float) -> int:
    return int(round(metres / VERTICAL_SCALE))


def _goals(clear_x: float) -> np.ndarray:
    """Approach goal, cleared goal, and the far end."""
    return np.array([
        [OBSTACLE_X - 1.0, SPAWN_Y],
        [clear_x, SPAWN_Y],
        [PROBE_LENGTH_M - 1.0, SPAWN_Y],
    ], dtype=np.float64)


def make_step_probe(height_m: float, down: bool = False) -> Probe:
    """Flat lane with a single full-width step at ``OBSTACLE_X``."""
    hf = _blank()
    i = int(round(OBSTACLE_X / HORIZONTAL_SCALE))
    hf[i:, :] = _to_raw(-height_m if down else height_m)
    family = "step_down" if down else "step_up"
    verb = "down onto" if down else "up onto"
    return Probe(
        family=family,
        level=int(round(height_m / 0.02)) - 1,
        param_m=float(height_m),
        height_field=hf,
        goals_m=_goals(OBSTACLE_X + 1.0),
        description=f"flat lane, single full-width step {verb} a {height_m:.2f} m change at x={OBSTACLE_X} m",
    )


def make_gap_probe(width_m: float) -> Probe:
    """Flat lane with a single full-width pit starting at ``OBSTACLE_X``."""
    hf = _blank()
    i0 = int(round(OBSTACLE_X / HORIZONTAL_SCALE))
    i1 = i0 + max(1, int(round(width_m / HORIZONTAL_SCALE)))
    hf[i0:i1, :] = _to_raw(PIT_DEPTH_M)
    return Probe(
        family="gap",
        level=int(round(width_m / 0.05)) - 1,
        param_m=float(width_m),
        height_field=hf,
        goals_m=_goals(OBSTACLE_X + width_m + 0.5),
        description=f"flat lane, full-width {width_m:.2f} m pit at {PIT_DEPTH_M:.1f} m starting at x={OBSTACLE_X} m",
    )


def build_probes() -> List[Probe]:
    """Every probe, in a stable order: step_up, step_down, gap."""
    probes = [make_step_probe(h) for h in STEP_HEIGHTS_M]
    probes += [make_step_probe(h, down=True) for h in STEP_HEIGHTS_M]
    probes += [make_gap_probe(w) for w in GAP_WIDTHS_M]
    return probes


# --------------------------------------------------------------------------- #
# Which placeholder each probe settles
# --------------------------------------------------------------------------- #
#: ``parameter -> (probe family, what to measure, how to read it off)``.
#: Parameters with family ``None`` are NOT covered by these two probes.
CALIBRATION_MAP: Dict[str, Tuple[str, str, str]] = {
    "skill.STEP_WALK_MAX": (
        "step_up",
        "largest step_up the WALK gait clears",
        "run every step_up level in WALK; take the largest height cleared on all repeats, "
        "then back off one level (0.02 m) for margin",
    ),
    "skill.STEP_TROT_MAX": (
        "step_up",
        "largest step_up the TROT gait clears",
        "same sweep held in TROT",
    ),
    "skill.STEP_RUN_MAX": (
        "step_up",
        "largest step_up the RUN gait clears",
        "same sweep held in RUN (trot_run + speed_level 0)",
    ),
    "skill.STEP_JUMP_MAX": (
        "step_up",
        "largest step_up front_jump mounts",
        "balance_stand -> front_jump aimed at the step; the working estimate from the "
        "curated logs is 0.12-0.15 m against a 0.25 m ballistic ceiling, so this probe is "
        "the first direct test of it",
    ),
    "robot.FOOT_SPAN_X": (
        "gap",
        "widest gap crossed *without* a jump",
        "walk every gap level; the largest crossed by stepping bounds the fore-aft foot span "
        "from below. Distinct from GAP_MAX (measured: 0.0 m), which is what a *jump* clears",
    ),
    "skill.ROUGHNESS_TROT_MAX": (None, "residual RMS to a local plane", "needs a roughness probe (not in this set)"),
    "skill.ROUGHNESS_RUN_MAX": (None, "residual RMS to a local plane", "needs a roughness probe (not in this set)"),
    "skill.SLOPE_WALK_MAX": (None, "sustained incline in degrees", "needs a ramp probe (not in this set)"),
    "skill.SLOPE_TROT_MAX": (None, "sustained incline in degrees", "needs a ramp probe (not in this set)"),
    "skill.SLOPE_RUN_MAX": (None, "sustained incline in degrees", "needs a ramp probe (not in this set)"),
}

#: ``step_down`` measures no placeholder directly.  It is included because the
#: rules apply ``step_max_m`` to descents as well as climbs
#: (``planner.rules.RulePlanner._fits`` checks ``step_down_m`` against the same
#: limit), and that symmetry is an assumption nothing has tested.
STEP_DOWN_NOTE = (
    "step_down probes no placeholder on its own. The rules currently check step_down_m "
    "against the same step_max_m as step_up_m; running both families says whether that "
    "symmetry holds. If descent limits come out materially different, the rules need a "
    "separate descent limit rather than a re-tuned shared one."
)
