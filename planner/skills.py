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
    TURN = "TURN"
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
    #: Measured steady yaw rate, rad/s.  0 for the straight gaits; TURN's is the
    #: log's own -0.3954 and is what the low level aims its foot placement at.
    yaw_rate_rad_s: float = 0.0

    @property
    def supported(self) -> bool:
        return self.id in SUPPORTED

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
        SkillId.TURN: Skill(SkillId.TURN, s.DUTY_TURN, s.SPEED_TURN, s.STEP_WALK_MAX,
                            np.inf, s.SLOPE_WALK_MAX, s.GAP_MAX,
                            yaw_rate_rad_s=s.YAW_RATE_TURN),
        SkillId.JUMP: Skill(SkillId.JUMP, float("nan"), 0.0, s.STEP_JUMP_MAX,
                            np.inf, np.inf, s.GAP_MAX,
                            one_shot=True, lockout_s=cfg.switch.JUMP_LOCKOUT_S),
    }


#: What the low level can actually execute today.  RUN and JUMP stay in
#: :class:`SkillId` and in the library -- they are coming back -- but the replay
#: cannot produce them yet and a policy asked for one says so instead of
#: pretending.  RUN: the clip has no flight phase in sim, the body never leaves
#: the ground, so foot placement has no footfall to choose
#: (``outputs/run_collapse.md``).  JUMP: the sim robot is 1.10-1.20x short of the
#: torque the recording used, and ``effort_limit`` is not ours to raise
#: (``outputs/jump_torque.md``).
SUPPORTED = (SkillId.WALK, SkillId.TROT, SkillId.TURN)
UNSUPPORTED_REASON = {
    SkillId.RUN: "no flight phase in replay: the base never leaves the ground, so there is "
                 "no footfall for placement to choose (outputs/run_collapse.md)",
    SkillId.JUMP: "sim robot is torque-short by 1.10-1.20x and effort_limit is upstream's "
                  "(outputs/jump_torque.md)",
}


#: Slowest/most conservative first.  "Rougher terrain -> higher duty factor."
SAFETY_ORDER = (SkillId.WALK, SkillId.TROT, SkillId.RUN)


@dataclass
class BaseState:
    """The proprioception a low-level policy is allowed to read.

    Base velocity in the BODY frame and the yaw rate, nothing else.  No terrain,
    no depth, no goal: those belong to the planner, and a low level that could
    see them would make the planner ornamental.
    """

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass
class Command:
    """What a policy emits for one control tick.

    ``q`` is the real output for a replay policy: 12 joint angles in the clip's
    leg-major order.  ``vx/vy/wz`` stay for the kinematic and stub policies the
    offline sweep uses, and are advisory otherwise.
    """

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    note: str = ""
    q: Optional[np.ndarray] = None
    supported: bool = True


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


class ClipPolicy:
    """Replays one frozen clip, with the foot-placement correction on top.

    This is the executor the planner has been missing.  It is deliberately dumb:
    it walks the clip's frames in a loop and adds the correction that
    ``sim.footcomp`` computes from the base's own velocity.  It does not decide
    anything, it cannot see terrain, and it holds no state that outlives a reset
    beyond the phase it is at.

    ``clip`` is what ``scripts/verify_skill_replay.load_clip`` returns: at least
    ``q_des`` (n, 12), ``contact`` (n, 4), ``fs`` and ``name``.  ``foot`` is a
    ``sim.footcomp.FootPlacement`` or None; with None this is an open-loop replay
    and the caller has to say so.

    The swing gate is the RECORDING's contact channel, phase-locked to the frame
    being played, which is deterministic and identical run to run.  The
    alternative -- the robot's own contacts -- is proprioception too and is
    supported by the harness; it is not the default because for a clip that
    reproduces, the recording's labels are the definition of the gait's phase.
    """

    def __init__(self, skill: SkillId, clip: dict, foot=None,
                 hip_sign: float = 1.0, start_frame: int = 0) -> None:
        self.skill = skill
        self.clip = clip
        self.foot = foot
        self.name = str(clip["name"])
        self._q = np.asarray(clip["q_des"], dtype=np.float32).copy()
        self._q[:, 0::3] *= hip_sign
        self._swing = ~np.asarray(clip["contact"], dtype=bool)
        self._n = self._q.shape[0]
        self._start = int(start_frame) % self._n
        self._i = 0
        self._elapsed = 0.0

    def reset(self) -> None:
        self._i = 0
        self._elapsed = 0.0
        if self.foot is not None:
            self.foot.reset()

    @property
    def frame(self) -> int:
        return (self._start + self._i) % self._n

    @property
    def cycles_played(self) -> float:
        return self._i / self._n

    def act(self, obs, dt: float) -> Command:
        f = self.frame
        q = self._q[f].astype(np.float32, copy=True)
        if self.foot is not None:
            vy = float(getattr(obs, "vy", 0.0))
            wz = float(getattr(obs, "wz", 0.0))
            vx = float(getattr(obs, "vx", 0.0))
            q = q + self.foot.step(vy, wz, self._swing[f], vx=vx)
        self._i += 1
        self._elapsed += dt
        return Command(note=f"{self.skill} clip {self.name} frame {f}/{self._n}", q=q)

    def done(self) -> bool:
        return False

    @property
    def elapsed_s(self) -> float:
        return self._elapsed


class UnsupportedPolicy:
    """A skill the low level cannot execute yet.  Says so once; never pretends.

    Kept because RUN and JUMP are coming back and removing them from the
    interface would mean re-deriving the wiring when they do.  ``act`` returns no
    joint targets and ``supported=False``, and the executor is expected to hold
    whatever it was already playing rather than fall over.
    """

    def __init__(self, skill: SkillId) -> None:
        self.skill = skill
        self.reason = UNSUPPORTED_REASON.get(skill, "no policy attached")
        self._logged = False
        self._elapsed = 0.0

    def reset(self) -> None:
        self._elapsed = 0.0

    def act(self, obs, dt: float) -> Command:
        self._elapsed += dt
        first, self._logged = not self._logged, True
        return Command(note=f"{self.skill} UNSUPPORTED: {self.reason}",
                       supported=False, q=None)

    def done(self) -> bool:
        return False

    @property
    def elapsed_s(self) -> float:
        return self._elapsed


def make_policy(skill: SkillId, library: Optional[Dict[SkillId, Skill]] = None,
                kinematic: bool = False, clips: Optional[Dict[SkillId, dict]] = None,
                foot_for=None, hip_sign: float = 1.0) -> SkillPolicy:
    """Build the executor for ``skill``.

    With ``clips`` given this returns a real :class:`ClipPolicy` for the skills the
    low level can execute and an :class:`UnsupportedPolicy` for the ones it cannot.
    ``foot_for(skill, clip)`` supplies the foot-placement stepper, or None for an
    open-loop replay.  Without ``clips`` the old stub/kinematic behaviour is kept,
    because ``simulate_planner_offline.py`` runs with no simulator and no archive.
    """
    if clips is not None:
        if skill not in SUPPORTED or skill not in clips:
            return UnsupportedPolicy(skill)
        foot = foot_for(skill, clips[skill]) if foot_for is not None else None
        return ClipPolicy(skill, clips[skill], foot=foot, hip_sign=hip_sign)
    if kinematic:
        return KinematicPolicy(skill, library or build_library())
    return StandStillPolicy(skill)
