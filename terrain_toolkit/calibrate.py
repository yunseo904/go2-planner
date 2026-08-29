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
#: Incline angles, degrees.  Spans the placeholder range (RUN 10, TROT 20, WALK 35).
SLOPE_ANGLES_DEG = np.round(np.arange(5.0, 40.0 + 1e-9, 5.0), 3)
#: Square-wave amplitudes, m.  Bottom of the ladder is one VERTICAL_SCALE unit, and the
#: top is twice the largest placeholder (ROUGHNESS_TROT_MAX 0.03).
ROUGHNESS_AMPLITUDES_M = np.round(np.arange(0.005, 0.060 + 1e-9, 0.005), 3)
#: How long the varying patch runs, m.  Long enough that a gait takes several strides
#: inside it rather than crossing it in one.
RAMP_RUN_M = 2.0
ROUGH_RUN_M = 2.0


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


def make_slope_probe(angle_deg: float) -> Probe:
    """Flat lane, then a constant incline of ``angle_deg``, then flat again.

    One condition per terrain, like the others: the only thing that varies down the
    family is the angle.  The ramp is ``RAMP_RUN_M`` long, which at the steepest angle
    is a rise of about 1.1 m, and the lane is flat before and after so a failure is on
    the incline rather than on getting to it.

    The rise is quantised to ``VERTICAL_SCALE`` like every other probe, so the surface
    is a staircase of 5 mm risers rather than a true plane.  That is deliberate and
    matches how the archive stores everything else; the planner's own slope feature
    ignores windows containing a cell step larger than ``STEP_THRESH``, and a 5 mm
    riser is far below it, so the probe reads as slope and not as a series of steps.
    """
    hf = _blank()
    i0 = int(round(OBSTACLE_X / HORIZONTAL_SCALE))
    n_ramp = int(round(RAMP_RUN_M / HORIZONTAL_SCALE))
    rise = np.tan(np.radians(angle_deg)) * HORIZONTAL_SCALE
    for k in range(n_ramp):
        hf[i0 + k, :] = _to_raw(rise * (k + 1))
    hf[i0 + n_ramp:, :] = _to_raw(rise * n_ramp)
    return Probe(
        family="slope",
        level=int(round(angle_deg / 5.0)) - 1,
        param_m=float(angle_deg),                  # degrees, not metres, for this family
        height_field=hf,
        goals_m=_goals(OBSTACLE_X + RAMP_RUN_M + 0.5),
        description=f"flat lane, {RAMP_RUN_M:.1f} m incline at {angle_deg:.0f} deg from "
                    f"x={OBSTACLE_X} m, then flat",
    )


def make_roughness_probe(amplitude_m: float) -> Probe:
    """Flat lane, then a patch of deterministic square-wave roughness.

    NO RNG.  The pattern is a fixed four-cell triangle wave -- 0, +A, 0, -A, repeating --
    so the archive is reproducible from the amplitude alone and two runs of the generator
    cannot disagree.  Random noise would be closer to the benchmark's own terrain but
    would put an unseeded draw inside a frozen archive, which is the thing this toolkit
    exists to avoid.

    **Roughness and step are not independent at short wavelength**, and the wave shape is
    chosen to minimise the coupling rather than to pretend it away.  A two-cell square
    wave of amplitude A has a cell-to-cell jump of 2A, so at the top of the ladder the
    planner reads a 0.12 m STEP as well as 0.057 m of roughness and a failure could be
    attributed to either -- that breaks the one-condition-per-terrain rule the whole
    toolkit rests on.  The triangle halves the jump to A for the same amplitude.  It
    cannot be removed entirely: roughness at this scale IS local steps.

    The planner measures roughness as the RMS residual to a local straight line
    (``features._detrended_rms``); for this wave that residual is about 0.7 A, and the
    measured value is recorded next to the nominal one when the archive is frozen.
    """
    hf = _blank()
    i0 = int(round(OBSTACLE_X / HORIZONTAL_SCALE))
    n_patch = int(round(ROUGH_RUN_M / HORIZONTAL_SCALE))
    raw = _to_raw(amplitude_m)
    pattern = (0, raw, 0, -raw)
    for k in range(n_patch):
        hf[i0 + k, :] = pattern[k % 4]
    return Probe(
        family="roughness",
        level=int(round(amplitude_m / 0.005)) - 1,
        param_m=float(amplitude_m),
        height_field=hf,
        goals_m=_goals(OBSTACLE_X + ROUGH_RUN_M + 0.5),
        description=f"flat lane, {ROUGH_RUN_M:.1f} m of two-cell square-wave roughness at "
                    f"+-{amplitude_m*1000:.0f} mm from x={OBSTACLE_X} m",
    )


def build_probes() -> List[Probe]:
    """Every probe, in a stable order: step_up, step_down, gap, slope, roughness."""
    probes = [make_step_probe(h) for h in STEP_HEIGHTS_M]
    probes += [make_step_probe(h, down=True) for h in STEP_HEIGHTS_M]
    probes += [make_gap_probe(w) for w in GAP_WIDTHS_M]
    probes += [make_slope_probe(a) for a in SLOPE_ANGLES_DEG]
    probes += [make_roughness_probe(a) for a in ROUGHNESS_AMPLITUDES_M]
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
    "skill.ROUGHNESS_TROT_MAX": (
        "roughness",
        "residual RMS to a local plane",
        "run every roughness level in TROT; the family's amplitude IS the feature the planner measures, so the level that fails is the limit directly",
    ),
    "skill.ROUGHNESS_RUN_MAX": (
        "roughness",
        "residual RMS to a local plane",
        "same sweep held in RUN",
    ),
    "skill.SLOPE_WALK_MAX": (
        "slope",
        "sustained incline in degrees",
        "run every slope level in WALK; take the steepest cleared on all repeats, back off one level (5 deg) for margin",
    ),
    "skill.SLOPE_TROT_MAX": (
        "slope",
        "sustained incline in degrees",
        "same sweep held in TROT",
    ),
    "skill.SLOPE_RUN_MAX": (
        "slope",
        "sustained incline in degrees",
        "same sweep held in RUN",
    ),
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
