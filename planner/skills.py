"""The discrete skill library and the policy interface behind it.

The rule planner chooses a :class:`SkillId`; something else has to execute it.
That executor is a :class:`SkillPolicy` -- in the real experiment a replayed
joint trajectory or a learned low-level controller, here
:class:`StandStillPolicy`, which does nothing at all.  The dummy exists so the
planner, the feature extraction and the offline simulation can be wired together
and tested end to end on CPU before any policy exists.

Four skills, each a gait the curated logs actually contain:

======  =====  ==========  ===========================================
skill   duty   speed m/s   what it is
======  =====  ==========  ===========================================
WALK    0.64   0.20        classic_walk / cross_step; long stance, no flight
TROT    0.52   0.55        the default `move` gait; no flight
RUN     0.31   0.48        trot_run + speed_level 0; the only periodic aerial phase
JUMP    -      ~0          front_jump; a vertical hop in place, one-shot
======  =====  ==========  ===========================================

`RUN` is *slower* than `TROT` in the measurements.  That is not a typo: at
``speed_level 0`` the running trot buys an aerial phase, not ground speed
(`skill_profile.md` §5).  `JUMP` covers 26 ± 4 mm of ground, so it is modelled as
stationary.

Nothing here imports Isaac Lab / torch.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Protocol

import numpy as np

from .config import DEFAULT, PlannerConfig


class SkillId(Enum):
    WALK = "WALK"
    TROT = "TROT"
    RUN = "RUN"
    JUMP = "JUMP"

    def __str__(self) -> str:  # keeps CSV output readable
        return self.value


@dataclass(frozen=True)
class Skill:
    """Static capability record for one skill, filled from :mod:`planner.config`."""

    id: SkillId
    duty: float
    speed_m_s: float
    step_max_m: float
    roughness_max_m: float
    slope_max_deg: float
    gap_max_m: float
    one_shot: bool = False
    lockout_s: float = 0.0

    @property
    def name(self) -> str:
        return self.id.value


def build_library(cfg: PlannerConfig = DEFAULT) -> Dict[SkillId, Skill]:
    """The four skills, with every limit read from ``cfg``.

    Several of the limits are ``CALIBRATION_NEEDED`` placeholders -- see
    ``cfg.needs_calibration()``.  They are wired through rather than inlined so a
    calibration run can replace them without touching this module.
    """
    s = cfg.skill
    return {
        SkillId.WALK: Skill(SkillId.WALK, s.DUTY_WALK, s.SPEED_WALK, s.STEP_WALK_MAX,
                            np.inf, s.SLOPE_WALK_MAX, s.GAP_MAX),
        SkillId.TROT: Skill(SkillId.TROT, s.DUTY_TROT, s.SPEED_TROT, s.STEP_TROT_MAX,
                            s.ROUGHNESS_TROT_MAX, s.SLOPE_TROT_MAX, s.GAP_MAX),
        SkillId.RUN: Skill(SkillId.RUN, s.DUTY_RUN, s.SPEED_RUN, s.STEP_RUN_MAX,
                           s.ROUGHNESS_RUN_MAX, s.SLOPE_RUN_MAX, s.GAP_MAX),
        SkillId.JUMP: Skill(SkillId.JUMP, float("nan"), 0.0, s.STEP_JUMP_MAX,
                            np.inf, np.inf, s.GAP_MAX,
                            one_shot=True, lockout_s=cfg.switch.JUMP_LOCKOUT_S),
    }


#: Slowest/most conservative first.  "Rougher terrain -> higher duty factor."
SAFETY_ORDER = (SkillId.WALK, SkillId.TROT, SkillId.RUN)


@dataclass
class Command:
    """What a policy emits for one control tick."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    note: str = ""


class SkillPolicy(Protocol):
    """Executor for one skill.  Real implementations replay or infer a gait."""

    skill: SkillId

    def reset(self) -> None: ...

    def act(self, obs, dt: float) -> Command: ...

    def done(self) -> bool:
        """One-shot skills report completion; periodic ones never finish."""
        ...


class StandStillPolicy:
    """Wiring test: accepts any skill, commands nothing, never finishes.

    Deliberately *not* a locomotion stub -- a dummy that quietly walked would
    make an unwired pipeline look like it worked.
    """

    def __init__(self, skill: SkillId) -> None:
        self.skill = skill
        self._elapsed = 0.0

    def reset(self) -> None:
        self._elapsed = 0.0

    def act(self, obs, dt: float) -> Command:
        self._elapsed += dt
        return Command(0.0, 0.0, 0.0, note=f"{self.skill} stub, no policy attached")

    def done(self) -> bool:
        return False

    @property
    def elapsed_s(self) -> float:
        return self._elapsed


class KinematicPolicy:
    """Advances at the skill's *measured* speed.  For offline sequence studies only.

    This is not a controller: it asserts that a skill moves the body at its
    measured steady-state speed and nothing else.  It exists so
    ``scripts/simulate_planner_offline.py`` can sweep the terrain and produce a
    skill sequence without a simulator.  Any claim about success or failure needs
    the real thing.
    """

    def __init__(self, skill: SkillId, library: Dict[SkillId, Skill]) -> None:
        self.skill = skill
        self._skill = library[skill]
        self._elapsed = 0.0

    def reset(self) -> None:
        self._elapsed = 0.0

    def act(self, obs, dt: float) -> Command:
        self._elapsed += dt
        return Command(vx=self._skill.speed_m_s, note=f"{self.skill} kinematic")

    def done(self) -> bool:
        return self._skill.one_shot and self._elapsed >= self._skill.lockout_s

    @property
    def elapsed_s(self) -> float:
        return self._elapsed


def make_policy(skill: SkillId, library: Optional[Dict[SkillId, Skill]] = None,
                kinematic: bool = False) -> SkillPolicy:
    if kinematic:
        return KinematicPolicy(skill, library or build_library())
    return StandStillPolicy(skill)
