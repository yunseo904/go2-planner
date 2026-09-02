"""Every planner parameter, with its provenance recorded next to it.

`CLAUDE.md` §2 forbids tuning thresholds on performance data and forbids burying
switch delays or thresholds as literals in the code.  This module is the single
place they live, and each one carries a :class:`Provenance` tag so a reader can
tell a measurement from a guess without reading the git history:

``MEASURED``
    Read off the curated logs or the upstream config.  ``source`` names the file
    the number came from.  Do not change these without re-deriving them.
``DERIVED``
    Computed from ``MEASURED`` values or from the frozen terrain geometry
    (``outputs/terrain_profile.csv``).  ``source`` gives the derivation.
``CALIBRATION_NEEDED``
    **Not yet established.**  The value present is a provisional placeholder so
    the pipeline runs end to end; it is not evidence of anything.  Per
    `CLAUDE.md` these must be settled on a separate calibration terrain, never by
    tuning against benchmark scores.
``CONVENTION``
    A free choice of the implementation (tick rate, corridor width).  Changing it
    changes behaviour but nothing claims it is the right value.

``PlannerConfig.report()`` prints the whole table; ``needs_calibration()`` lists
what is still open.  Nothing in this module imports Isaac Lab / torch.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from enum import Enum
from typing import Dict, List, Tuple


class Provenance(Enum):
    MEASURED = "measured"
    DERIVED = "derived"
    CALIBRATION_NEEDED = "CALIBRATION_NEEDED"
    CONVENTION = "convention"


#: ``"<group>.<field>" -> (provenance, unit, source / justification)``
PROVENANCE: Dict[str, Tuple[Provenance, str, str]] = {}


def _p(key: str, prov: Provenance, unit: str, source: str) -> None:
    PROVENANCE[key] = (prov, unit, source)


# --------------------------------------------------------------------------- #
# Skill library
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SkillLimits:
    """What the four skills can do, from the curated-log measurements."""

    # -- gait identity ------------------------------------------------------
    DUTY_WALK: float = 0.64
    DUTY_TROT: float = 0.52
    DUTY_RUN: float = 0.31
    DUTY_TURN: float = 0.59

    # -- achieved forward speed (steady state, middle 60 % of the motion) ----
    SPEED_WALK: float = 0.20
    SPEED_TROT: float = 0.55
    SPEED_RUN: float = 0.48
    SPEED_TURN: float = 0.0075

    # -- turning -------------------------------------------------------------
    YAW_RATE_TURN: float = -0.3954
    HEADING_ERR_TURN_DEG: float = 12.0

    # -- where in the TURN cycle a replay may start -- MEASURED, not a criterion ----
    # A cyclic clip is a loop and the frame it is entered at is a free choice.  For
    # WALK and TROT that choice is made by a kinematic rule at run time
    # (verify_skill_replay.level_start, the most coplanar pose).  For TURN the rule
    # picks a phase that does not turn, so this skill carries its entry frame as a
    # measured constant instead.  -1 means "no measurement, use the rule".
    ENTRY_FRAME_TURN: int = 6

    # -- clip playback rate, per skill -- NOT ADOPTED.  See the warning below. --------
    #
    # MEASURED 2026-09-02, and it stops this table from being used: `hi` and `lo` are the
    # SAME cycle sampled at different rates -- WALK is 37 frames at 50.6 Hz and 306 at
    # 418 Hz -- and every harness here plays ONE FRAME PER CONTROL STEP at 50 Hz.  So
    # selecting `hi` does not play the clip faster, it stretches it by the sample-rate
    # ratio: 0.74 s per cycle becomes 6.12 s, a slowdown of 8.3x (TROT 8.4x, TURN 8.3x).
    #
    # Measured on the benchmark, 200 cells, no roughness, roll couple on:
    #
    #   WALK rate lo   score 1.090   v_x  0.1070 m/s   alive 74/200
    #   WALK rate hi   score 0.005   v_x -0.0045 m/s   alive  3/200
    #
    # That is not a gait.  SESSION_STATE 12's "TURN hi is worth 72 % -> 83 % of the logged
    # yaw rate" was measured on a different harness and CANNOT be reconciled with a pure
    # 8.3x slowdown as it stands -- a longer cycle gives a LOWER yaw rate in deg/s, not a
    # higher one.  Whoever revisits it should establish what that measurement was per:
    # per second, or per cycle.  Until then this table is inert.
    # The archive carries each clip resampled two ways.  Which one is better is a per-clip
    # question and the evidence is per clip (SESSION_STATE 12, "TURN"):
    #
    #   TURN  hi is worth 72 % -> 83 % of the logged yaw rate with stride cv 0.224 ->
    #         0.068.  The largest single improvement measured on TURN.
    #   TROT  hi is free: v_x +1 %, stride cv halved.
    #   WALK  hi moves v_x +13 %, outside the +-10 % gate every other edit is held to, so
    #         WALK keeps lo.  This is why the setting cannot be global.
    #
    # RUN and JUMP are unresolved and carry lo, the archive's default, so nothing about
    # them changes silently.
    RATE_WALK: str = "lo"
    RATE_TROT: str = "lo"
    RATE_TURN: str = "lo"
    RATE_RUN: str = "lo"
    RATE_JUMP: str = "lo"

    # -- how close the body has to be to the incoming gait's speed to switch ----
    SPEED_MATCH_MAX: float = 0.25

    # -- vertical capability -------------------------------------------------
    JUMP_APEX: float = 0.25
    JUMP_FLIGHT_S: float = 0.451
    JUMP_HORIZONTAL: float = 0.026
    GAP_MAX: float = 0.0

    # -- obstacle limits per gait -- NOT ESTABLISHED -------------------------
    STEP_WALK_MAX: float = 0.10
    STEP_TROT_MAX: float = 0.08
    STEP_RUN_MAX: float = 0.04
    STEP_JUMP_MAX: float = 0.15

    ROUGHNESS_TROT_MAX: float = 0.03
    ROUGHNESS_RUN_MAX: float = 0.015

    SLOPE_WALK_MAX: float = 35.0
    SLOPE_TROT_MAX: float = 20.0
    SLOPE_RUN_MAX: float = 10.0


_p("skill.DUTY_WALK", Provenance.MEASURED, "-", "skill_profile.md §1 classic_walk/cross_step (0.638/0.680)")
_p("skill.DUTY_TROT", Provenance.MEASURED, "-", "skill_profile.md §1 default+move:forward / static_walk (0.516-0.529)")
_p("skill.DUTY_RUN", Provenance.MEASURED, "-", "skill_profile.md §1 trot_run+move:forward (0.310 ± 0.017)")
_p("skill.SPEED_WALK", Provenance.MEASURED, "m/s", "skill_profile.csv speed_steady_mean, classic_walk 0.19 / walk_fwd 0.19")
_p("skill.SPEED_TROT", Provenance.MEASURED, "m/s", "skill_profile.csv speed_steady_mean, run_10 0.71 / run_06 0.45; 0.55 is the group mean")
_p("skill.SPEED_RUN", Provenance.MEASURED, "m/s",
   "skill_profile.md §5: move x=1.5-2.0 + speed_level 0 -> 0.48 m/s measured. "
   "NB this is *below* SPEED_TROT - trot_run buys an aerial phase, not speed")
_p("skill.DUTY_TURN", Provenance.MEASURED, "-",
   "skill_clips.meta.json TURN duty_clip 0.591, session turn_right_20260824_223951")
_p("skill.SPEED_TURN", Provenance.MEASURED, "m/s",
   "skill_profile.csv vx_steady_mean 0.0075 for the TURN session - it turns on the spot")
_p("skill.YAW_RATE_TURN", Provenance.MEASURED, "rad/s",
   "skill_profile.csv yaw_rate_steady_mean -0.3954 (-22.66 deg/s) for turn_right_20260824_223951. "
   "This is the MEASURED rate, not the -0.6 rad/s that was commanded to produce it, and it is "
   "the same number the low level uses as its foot-placement target (outputs/turn_target.md)")
_p("skill.RATE_WALK", Provenance.CALIBRATION_NEEDED, "-",
   "NOT ADOPTED: hi is an 8.3x slowdown in this harness, not a faster playback")
_p("skill.RATE_TROT", Provenance.CALIBRATION_NEEDED, "-",
   "NOT ADOPTED: hi is an 8.4x slowdown in this harness")
_p("skill.RATE_TURN", Provenance.CALIBRATION_NEEDED, "-",
   "NOT ADOPTED: hi is an 8.3x slowdown here; SESSION_STATE 12 needs re-reading")
_p("skill.RATE_RUN", Provenance.CALIBRATION_NEEDED, "-", "unresolved skill; archive default")
_p("skill.RATE_JUMP", Provenance.CALIBRATION_NEEDED, "-", "unresolved skill; archive default")
_p("skill.ENTRY_FRAME_TURN", Provenance.MEASURED, "frame",
   "outputs/turn_entry_phase.md. All 45 phases of the 45-frame TURN clip were run as the "
   "in-place flat control, 9 identical cells each, in both foot-comp arms -- 810 runs. "
   "22 of 45 phases complete the 90 deg with --foot-comp on and 5 of 45 with it off; frame 6 "
   "is the only phase that passes 9/9 in BOTH arms with both its neighbours doing the same, "
   "so it is robust to a one-frame phase error and to the compensator being on or off. "
   "This is measured on the CALIBRATION terrain's flat run-up, not on the benchmark "
   "(CLAUDE.md 2), and it is a property of the skill rather than a threshold on terrain")
_p("skill.HEADING_ERR_TURN_DEG", Provenance.CALIBRATION_NEEDED, "deg",
   "placeholder. How far off the goal bearing before TURN is worth the switch. It trades the "
   "switch cost against the heading a straight gait would give away, and neither side has been "
   "measured on terrain yet, so this number is arbitrary and is here to make the wiring testable")
_p("skill.SPEED_MATCH_MAX", Provenance.CALIBRATION_NEEDED, "m/s",
   "placeholder. Transition safety condition: refuse a switch while the body's own speed "
   "is further than this from the incoming skill's measured speed. There is no measurement "
   "behind the number yet -- the point of having it is that the library's speeds are "
   "0.19 (WALK), 0.44 (TROT) and 0.008 (TURN), so any band under 0.43 forbids WALK<->TROT "
   "outright, and that is a fact about the library rather than about the threshold")
_p("skill.JUMP_APEX", Provenance.MEASURED, "m", "jump_profile.csv apex_rise_ballistic_m, 0.250 ± 0.031 over 5 front_jump sessions")
_p("skill.JUMP_FLIGHT_S", Provenance.MEASURED, "s", "jump_profile.csv flight_s, 0.451 ± 0.028")
_p("skill.JUMP_HORIZONTAL", Provenance.MEASURED, "m", "jump_profile.csv dx_jump_m, 26 ± 4 mm - front_jump is a vertical hop in place")
_p("skill.GAP_MAX", Provenance.MEASURED, "m",
   "skill_profile.md §3: take-off v_x is ~0, so ballistic range is ~0. "
   "No skill in the library launches with forward speed")
_p("skill.STEP_WALK_MAX", Provenance.CALIBRATION_NEEDED, "m",
   "placeholder. The *walking* step limit has never been measured - every curated session is flat "
   "floor. It must stay below STEP_JUMP_MAX or the jump rule has an empty domain and JUMP can "
   "never fire; that ordering is a structural requirement, not evidence about the value")
_p("skill.STEP_TROT_MAX", Provenance.CALIBRATION_NEEDED, "m", "placeholder; unmeasured")
_p("skill.STEP_RUN_MAX", Provenance.CALIBRATION_NEEDED, "m", "placeholder; unmeasured")
_p("skill.STEP_JUMP_MAX", Provenance.CALIBRATION_NEEDED, "m",
   "placeholder = jump working estimate (half of the 0.25 m ceiling). The logs contain no "
   "landing onto a raised surface, so the controller's behaviour on a step is unobserved")
_p("skill.ROUGHNESS_TROT_MAX", Provenance.CALIBRATION_NEEDED, "m", "placeholder; unmeasured")
_p("skill.ROUGHNESS_RUN_MAX", Provenance.CALIBRATION_NEEDED, "m", "placeholder; unmeasured")
_p("skill.SLOPE_WALK_MAX", Provenance.CALIBRATION_NEEDED, "deg", "placeholder; unmeasured (flat floor only)")
_p("skill.SLOPE_TROT_MAX", Provenance.CALIBRATION_NEEDED, "deg", "placeholder; unmeasured")
_p("skill.SLOPE_RUN_MAX", Provenance.CALIBRATION_NEEDED, "deg", "placeholder; unmeasured")


# --------------------------------------------------------------------------- #
# Depth sensor
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SensorLimits:
    """The depth camera as configured upstream (`legged_robot_config.py: CustomDepthCfg`)."""

    SENSOR_NEAR: float = 0.25
    SENSOR_FAR: float = 2.0
    RELIABLE_RANGE: float = 1.0
    UPDATE_HZ: float = 10.0
    DELAY_S: float = 0.02
    RES_ANCHOR_NEAR: Tuple[float, float] = (0.7, 0.027)
    RES_ANCHOR_FAR: Tuple[float, float] = (2.0, 0.24)
    MOUNT_FORWARD: float = 0.272
    MOUNT_UP: float = 0.092
    MOUNT_PITCH_RAD: float = 0.52


_p("sensor.SENSOR_NEAR", Provenance.MEASURED, "m", "CLAUDE.md §3: near_clip=0/far_clip=2 give an effective 0.25-2.0 m")
_p("sensor.SENSOR_FAR", Provenance.MEASURED, "m", "CustomDepthCfg far_clip = 2")
_p("sensor.RELIABLE_RANGE", Provenance.DERIVED, "m", "CLAUDE.md §4: beyond ~1 m the per-pixel ground footprint smears the geometry")
_p("sensor.UPDATE_HZ", Provenance.MEASURED, "Hz", "CustomDepthCfg update_interval=5 at 50 Hz control")
_p("sensor.DELAY_S", Provenance.MEASURED, "s", "CustomDepthCfg depth_delay_steps=1")
_p("sensor.RES_ANCHOR_NEAR", Provenance.MEASURED, "(m, m)", "CLAUDE.md §3: ~2.7 cm of ground per pixel at 0.7 m")
_p("sensor.RES_ANCHOR_FAR", Provenance.MEASURED, "(m, m)", "CLAUDE.md §3: ~24 cm of ground per pixel at 2.0 m")
_p("sensor.MOUNT_FORWARD", Provenance.MEASURED, "m", "CustomDepthCfg position")
_p("sensor.MOUNT_UP", Provenance.MEASURED, "m", "CustomDepthCfg position")
_p("sensor.MOUNT_PITCH_RAD", Provenance.MEASURED, "rad", "CustomDepthCfg angle, 0.52 rad = 29.8 deg down")


# --------------------------------------------------------------------------- #
# Robot geometry
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class RobotGeometry:
    BODY_WIDTH: float = 0.31
    FOOT_SPAN_X: float = 0.35
    STAND_HEIGHT: float = 0.32


_p("robot.BODY_WIDTH", Provenance.MEASURED, "m", "Go2 body width")
_p("robot.FOOT_SPAN_X", Provenance.CALIBRATION_NEEDED, "m",
   "fore-aft foot span; bounds the gap the robot can step over rather than jump. Not measured "
   "here - foot_pos_* is all zero on this firmware (CLAUDE.md §6)")
_p("robot.STAND_HEIGHT", Provenance.MEASURED, "m", "skill_profile.csv height_mean_m over the locomotion sessions, 0.30-0.35")


# --------------------------------------------------------------------------- #
# Switching behaviour
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SwitchParams:
    SWITCH_DELAY: float = 0.21
    MIN_HOLD_S: float = 0.5
    BASE_MARGIN: float = 0.05
    HYSTERESIS: float = 0.25
    JUMP_LOCKOUT_S: float = 0.66
    JUMP_RETRIGGER_M: float = 0.5
    SETTLE_WORST_S: float = 1.63

    #: Sensitivity sweep required by CLAUDE.md §3.
    DELAY_SWEEP: Tuple[float, ...] = (0.0, 0.21, 2.4, 4.06)


_p("switch.SWITCH_DELAY", Provenance.MEASURED, "s",
   "skill_transition.md: post-transition settle, 0.21 s median. The 4.06 s send-to-motion figure "
   "includes ~2.4 s of run.sh startup, which does not exist in sim (CLAUDE.md §3)")
_p("switch.MIN_HOLD_S", Provenance.CONVENTION, "s", "minimum time a skill is held before another switch is allowed")
_p("switch.BASE_MARGIN", Provenance.CONVENTION, "m", "lookahead floor so the window never collapses to zero at zero speed")
_p("switch.HYSTERESIS", Provenance.CONVENTION, "-",
   "fraction of a threshold band by which conditions must improve before switching back to a faster gait")
_p("switch.JUMP_LOCKOUT_S", Provenance.DERIVED, "s",
   "JUMP_FLIGHT_S (0.451) + measured front_jump settle (0.15, jump_profile) + margin; the jump "
   "cannot be cancelled once launched")
_p("switch.JUMP_RETRIGGER_M", Provenance.CONVENTION, "m",
   "distance the robot must cover after a jump before another may be triggered; without it a "
   "jump that fails to clear the step retriggers forever")
_p("switch.SETTLE_WORST_S", Provenance.MEASURED, "s", "skill_transition.md worst-case settle")
_p("switch.DELAY_SWEEP", Provenance.CONVENTION, "s", "CLAUDE.md §3 asks for 0 / 0.21 / 2.4 / 4.06")


# --------------------------------------------------------------------------- #
# Feature extraction
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class FeatureParams:
    PIT_THRESH: float = -0.5
    CORRIDOR_HALF_WIDTH: float = 0.5
    CROSS_HALF_WIDTH: float = 2.0
    STEP_THRESH: float = 0.08
    SLOPE_WINDOW: float = 0.2
    ROUGHNESS_WINDOW: float = 0.3
    DECISION_WINDOW_M: float = 1.0
    TICK_HZ: float = 10.0


_p("feature.PIT_THRESH", Provenance.MEASURED, "m", "benchmark generators carve pits at exactly -1.0 m")
_p("feature.CORRIDOR_HALF_WIDTH", Provenance.CONVENTION, "m", "same value profile.py uses to follow the nearest walkable surface")
_p("feature.CROSS_HALF_WIDTH", Provenance.CONVENTION, "m", "lateral cross-section extent; half the 4 m cell")
_p("feature.STEP_THRESH", Provenance.CONVENTION, "m", "per-cell change above which a rise is a step, not a slope (profile.py)")
_p("feature.SLOPE_WINDOW", Provenance.CONVENTION, "m", "slope baseline (profile.py)")
_p("feature.ROUGHNESS_WINDOW", Provenance.CONVENTION, "m", "window for the residual-to-local-plane roughness estimate")
_p("feature.DECISION_WINDOW_M", Provenance.CONVENTION, "m",
   "depth of the band beyond the commitment point that a decision looks at. 1.0 m matches the "
   "reliable range in CLAUDE.md §4 - past that the pixel footprint smears the geometry anyway")
_p("feature.TICK_HZ", Provenance.DERIVED, "Hz", "planner tick, matched to the depth update rate (sensor.UPDATE_HZ)")


# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class PlannerConfig:
    skill: SkillLimits = field(default_factory=SkillLimits)
    sensor: SensorLimits = field(default_factory=SensorLimits)
    robot: RobotGeometry = field(default_factory=RobotGeometry)
    switch: SwitchParams = field(default_factory=SwitchParams)
    feature: FeatureParams = field(default_factory=FeatureParams)

    _GROUPS = ("skill", "sensor", "robot", "switch", "feature")

    def items(self) -> List[Tuple[str, object, Provenance, str, str]]:
        """``(key, value, provenance, unit, source)`` for every parameter."""
        out = []
        for group in self._GROUPS:
            obj = getattr(self, group)
            for f in fields(obj):
                key = f"{group}.{f.name}"
                prov, unit, source = PROVENANCE.get(key, (Provenance.CONVENTION, "", "undocumented"))
                out.append((key, getattr(obj, f.name), prov, unit, source))
        return out

    def needs_calibration(self) -> List[Tuple[str, object, str]]:
        return [(k, v, s) for k, v, p, _u, s in self.items() if p is Provenance.CALIBRATION_NEEDED]

    def report(self) -> str:
        rows = ["| parameter | value | unit | provenance | source |", "| :--- | ---: | :--- | :--- | :--- |"]
        for key, value, prov, unit, source in self.items():
            mark = "**CALIBRATION_NEEDED**" if prov is Provenance.CALIBRATION_NEEDED else prov.value
            rows.append(f"| `{key}` | {value} | {unit} | {mark} | {source} |")
        return "\n".join(rows)

    def replace(self, **overrides) -> "PlannerConfig":
        """Copy with ``group.FIELD=value`` overrides, e.g. ``cfg.replace(**{'switch.SWITCH_DELAY': 2.4})``."""
        import dataclasses

        groups = {g: getattr(self, g) for g in self._GROUPS}
        for key, value in overrides.items():
            group, _, name = key.partition(".")
            if group not in groups or not name:
                raise KeyError(f"unknown parameter {key!r}")
            groups[group] = dataclasses.replace(groups[group], **{name: value})
        return PlannerConfig(**groups)


DEFAULT = PlannerConfig()
