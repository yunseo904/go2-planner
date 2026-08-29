"""The rule engine: geometry in, one of four discrete skills out.

Selection principle
-------------------
**Rougher terrain takes a higher duty factor.**  The curated logs put the three
periodic gaits on one axis -- WALK 0.64, TROT 0.52, RUN 0.31 -- and the aerial
phase only exists below ~0.40 (`skill_profile.md` §5).  More feet on the ground
for more of the cycle is the thing that buys robustness, so the rule is: take the
fastest skill whose limits the observed window still satisfies, and fall back
towards WALK as soon as one is exceeded.  JUMP is not on that axis; it is a
one-shot answer to a single obstacle.

Four mechanisms keep this from thrashing
----------------------------------------
1. **Hysteresis.**  Downgrading (towards WALK) happens the moment a limit is
   exceeded.  Upgrading requires the feature to be back under the limit by
   ``HYSTERESIS`` of the band, so a value hovering on a threshold does not
   oscillate.  Asymmetric on purpose: safety is cheap, speed is not.
2. **Minimum hold time.**  ``MIN_HOLD_S`` after a switch, no further switch is
   accepted -- except a downgrade forced by an exceeded limit, which is always
   allowed.
3. **Transition safety conditions.**  An upgrade needs a fresh, confident
   observation; on stale or low-confidence data the planner may only hold or
   downgrade.  Nothing may interrupt a JUMP.
4. **Switch delay.**  A decision taken now takes effect ``SWITCH_DELAY`` later.
   The planner owns that queue, so ``RulePlanner.active`` is always what the
   robot is really doing, not what was last requested.  A queued switch is **not
   recallable**: once the command is out, the gait changes when it changes.  That
   is the measured behaviour -- ``skill_send`` has no counterpart that cancels it
   -- and it is what makes a long delay expensive rather than merely late.  The
   channel holds one command at a time: requests arriving while something is in
   flight are dropped and counted (``requests_dropped_busy``), not queued behind
   it and not substituted for it.

JUMP is one-shot
----------------
Once launched it cannot be cancelled: the robot is committed for
``JUMP_LOCKOUT_S`` (flight 0.451 s + settle), and during that time every
requested change is refused and logged.  This mirrors the measurement, not a
software convenience -- there is no way to abort a ballistic flight.

Terrain with no matching skill
------------------------------
`CLAUDE.md` §2 forbids excepting those tasks out.  A gap wider than
``GAP_MAX`` (measured: 0.0 m -- no skill in the library launches with forward
speed) or a corridor narrower than ``BODY_WIDTH`` is recorded as an
:class:`Unsupported` event with a reason, and the planner carries on with the
safest skill it has.  "Tried and failed" is the result; a silent skip is not.

Nothing here imports Isaac Lab / torch.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .config import DEFAULT, PlannerConfig
from .features import Observation
from .skills import SAFETY_ORDER, Skill, SkillId, build_library
from .tracking import JumpGate, ObstacleTracker


@dataclass
class Unsupported:
    """Terrain the skill library has no answer for.  Logged, never skipped."""

    reason: str
    feature: str
    value: float
    limit: float
    x_m: float = float("nan")
    fatal: bool = True          # False = "might work, unverified"

    def __str__(self) -> str:
        kind = "UNSUPPORTED" if self.fatal else "UNVERIFIED"
        return (f"{kind} at x={self.x_m:.2f} m: {self.reason} "
                f"({self.feature}={self.value:.3f} vs limit {self.limit:.3f})")


@dataclass
class Decision:
    """One planner tick."""

    active: SkillId
    requested: SkillId
    reason: str
    switched: bool = False
    pending: Optional[SkillId] = None
    pending_in_s: float = 0.0
    locked: bool = False
    unsupported: List[Unsupported] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


def _exceeds(value: float, limit: float) -> bool:
    """NaN-safe 'over the limit'.  An unknown value is *not* treated as a breach.

    Unknown features come from a collapsed or blind window; the caller already
    refuses to upgrade on those, so treating NaN as a breach here would only make
    the planner downgrade on missing data and hide the real cause.
    """
    return bool(np.isfinite(value) and np.isfinite(limit) and value > limit)


class RulePlanner:
    """Stateful rule-based skill selector.

    ``step(obs, dt, x_m)`` is called once per planner tick (``feature.TICK_HZ``,
    matched to the depth update rate) and returns a :class:`Decision`.
    """

    def __init__(self, cfg: PlannerConfig = DEFAULT, initial: SkillId = SkillId.TROT,
                 jump_gate: JumpGate = JumpGate.NEAR_EDGE) -> None:
        self.cfg = cfg
        self.jump_gate = jump_gate
        self.tracker = ObstacleTracker(cfg) if jump_gate is JumpGate.TRACKING else None
        self._t = 0.0
        self.library: Dict[SkillId, Skill] = build_library(cfg)
        self.active = initial
        self._requested = initial
        self._pending: Optional[SkillId] = None
        self._pending_timer = 0.0
        self._hold_timer = cfg.switch.MIN_HOLD_S
        self._jump_lock = 0.0
        self._jump_block_until_x: Optional[float] = None
        self.unsupported_log: List[Unsupported] = []
        self._reported: set = set()
        self.switches = 0
        self.jumps = 0
        self.jump_band_ticks = 0
        self.jump_blocked_far = 0
        self.jump_blocked_retrigger = 0
        self.jump_detections = 0
        self.jump_missed = 0
        self.requests_dropped_busy = 0

    # -- capability test ---------------------------------------------------
    def _fits(self, skill_id: SkillId, obs: Observation, margin: float = 0.0) -> Tuple[bool, str]:
        """Does ``skill_id`` survive the observed window?  ``margin`` tightens the limits."""
        sk = self.library[skill_id]
        f = obs.features
        checks = (
            ("step_up_m", f["step_up_m"], sk.step_max_m),
            ("step_down_m", f["step_down_m"], sk.step_max_m),
            ("roughness_m", f["roughness_m"], sk.roughness_max_m),
            ("slope_max_deg", f["slope_max_deg"], sk.slope_max_deg),
        )
        for name, value, limit in checks:
            eff = limit * (1.0 - margin) if np.isfinite(limit) else limit
            if _exceeds(value, eff):
                return False, f"{name}={value:.3f} over {skill_id.value} limit {eff:.3f}"
        return True, ""

    def _best_fit(self, obs: Observation, margin: float) -> Tuple[SkillId, str]:
        """Fastest skill that fits, else WALK.  Ordered slowest-first, so scan back."""
        for skill_id in reversed(SAFETY_ORDER):
            ok, why = self._fits(skill_id, obs, margin)
            if ok:
                return skill_id, f"fastest fit ({skill_id.value})"
        ok, why = self._fits(SkillId.WALK, obs, 0.0)
        return SkillId.WALK, ("floor: WALK, " + why if not ok else "floor: WALK")

    # -- unsupported terrain -----------------------------------------------
    def _check_unsupported(self, obs: Observation, x_m: float) -> List[Unsupported]:
        out: List[Unsupported] = []
        f = obs.features
        gap = f.get("gap_m", 0.0)
        span = self.cfg.robot.FOOT_SPAN_X
        if _exceeds(gap, self.cfg.skill.GAP_MAX):
            if gap <= span:
                out.append(Unsupported(
                    "gap wider than any jump in the library can clear, but narrower than the "
                    "fore-aft foot span - it may be steppable; FOOT_SPAN_X is CALIBRATION_NEEDED",
                    "gap_m", gap, self.cfg.skill.GAP_MAX, x_m, fatal=False))
            else:
                out.append(Unsupported(
                    "gap crossing needs a jump with forward velocity; no skill in the library has one",
                    "gap_m", gap, self.cfg.skill.GAP_MAX, x_m, fatal=True))
        width = f.get("width_min_m", np.nan)
        if np.isfinite(width) and width < self.cfg.robot.BODY_WIDTH:
            out.append(Unsupported(
                "corridor narrower than the body; no squeeze skill exists",
                "width_min_m", width, self.cfg.robot.BODY_WIDTH, x_m, fatal=True))
        step = max(f.get("step_up_m", 0.0), 0.0)
        if _exceeds(step, self.library[SkillId.JUMP].step_max_m):
            out.append(Unsupported(
                "step taller than the jump can reach",
                "step_up_m", step, self.library[SkillId.JUMP].step_max_m, x_m, fatal=True))
        return out

    # -- jump trigger --------------------------------------------------------
    def _wants_jump(self, obs: Observation, x_m: float) -> bool:
        """Dispatch to the configured jump gate."""
        if self.jump_gate is JumpGate.TRACKING:
            return self._wants_jump_tracking(x_m)
        return self._wants_jump_near_edge(obs, x_m)

    def _wants_jump_tracking(self, x_m: float) -> bool:
        """Fire when a tracked obstacle's time-to-arrival has fallen to SWITCH_DELAY.

        The tracker has already been fed the full visible range this tick (see
        :meth:`observe`), so this only has to ask whether anything is due.  The
        retrigger guard is kept even though ``mark_fired`` makes it redundant --
        both gates then run through identical rule machinery, which is the point
        of the comparison.
        """
        tracker = self.tracker
        assert tracker is not None
        due = tracker.due(x_m, self.speed, self._t, 1.0 / self.cfg.feature.TICK_HZ)
        if due is None:
            return False
        self.jump_band_ticks += 1
        if self._jump_block_until_x is not None:
            if not np.isfinite(x_m) or x_m < self._jump_block_until_x:
                self.jump_blocked_retrigger += 1
                return False
            self._jump_block_until_x = None
        tracker.mark_fired(due)
        return True

    def observe(self, full_obs: Observation, x_m: float, hs: float) -> None:
        """Feed the tracking gate the whole visible range.  No-op for NEAR_EDGE."""
        if self.tracker is not None:
            self.tracker.update(full_obs, x_m, self._t, hs)
            self.jump_detections = self.tracker.detections

    def _wants_jump_near_edge(self, obs: Observation, x_m: float) -> bool:
        """A step too tall to walk up but inside the jump's reach, and close enough to aim at.

        Three separate gates, counted separately so a downstream study can tell
        *which* one suppressed a jump rather than guessing:

        ``jump_band_ticks``
            the observed step landed in ``(STEP_WALK_MAX, STEP_JUMP_MAX]`` at all.
        ``jump_blocked_far``
            it did, but the decision window no longer starts at the blind-zone
            edge.  ``front_jump`` covers 26 mm of ground, so it is only useful
            against something the robot is already standing at; once the switch
            delay pushes the commitment point outwards, the jump stops being
            aimable at all.  **This gate is structural: no threshold value can
            re-open it.**
        ``jump_blocked_retrigger``
            it did, and the window is near, but a previous jump has not yet been
            cleared by ``JUMP_RETRIGGER_M`` of travel.
        """
        f = obs.features
        step = f.get("step_up_m", 0.0)
        if not np.isfinite(step):
            return False
        walk_max = self.library[SkillId.WALK].step_max_m
        jump_max = self.library[SkillId.JUMP].step_max_m
        if not (step > walk_max and step <= jump_max):
            return False
        self.jump_band_ticks += 1

        if obs.observed_from_m > self.cfg.sensor.SENSOR_NEAR + 1e-9:
            self.jump_blocked_far += 1
            return False

        if self._jump_block_until_x is not None:
            if not np.isfinite(x_m) or x_m < self._jump_block_until_x:
                self.jump_blocked_retrigger += 1
                return False
            self._jump_block_until_x = None
        return True

    # -- turn trigger --------------------------------------------------------
    def _wants_turn(self, heading_err_deg: float) -> bool:
        """Is the body pointed far enough off the goal bearing to spend a TURN?

        TURN is not on the duty axis the other gaits are ranked by -- it answers a
        heading question, not a roughness one -- so it is selected here rather
        than in ``_best_fit``.  The threshold is asymmetric for the same reason
        every other threshold here is: leaving a turn early costs a second turn.

        ``heading_err_deg`` is the planner's own quantity (goal bearing minus
        heading).  A NaN means nobody is supplying one, and then TURN is never
        requested -- which is what the offline sweep and every existing caller
        get, unchanged.
        """
        lim = self.cfg.skill.HEADING_ERR_TURN_DEG
        if not (np.isfinite(heading_err_deg) and np.isfinite(lim)):
            return False
        err = abs(heading_err_deg)
        if self.active is SkillId.TURN:
            return err > lim * (1.0 - self.cfg.switch.HYSTERESIS)
        return err > lim

    # -- main tick ------------------------------------------------------------
    def step(self, obs: Observation, dt: float, x_m: float = float("nan"),
             heading_err_deg: float = float("nan")) -> Decision:
        cfg = self.cfg
        self._t += dt
        self._hold_timer += dt
        warnings = list(obs.warnings)

        # 1. a launched jump owns the robot until it lands
        if self._jump_lock > 0.0:
            self._jump_lock -= dt
            self._pending = None
            return Decision(self.active, self.active,
                            f"JUMP locked out, {max(self._jump_lock, 0.0):.2f} s to land",
                            locked=True, warnings=warnings)

        if self.active is SkillId.JUMP:      # lockout just expired
            self.active = SkillId.WALK
            self._hold_timer = 0.0

        # 2. terrain with no answer -- logged once per obstacle, never skipped.
        #    Edge-triggered on (feature, 0.1 m bin) so a 3 m stretch of pit is one
        #    event per position rather than one per tick.
        unsupported = self._check_unsupported(obs, x_m)
        fresh_events = []
        for u in unsupported:
            key = (u.feature, u.fatal, round(u.x_m, 1) if np.isfinite(u.x_m) else None)
            if key not in self._reported:
                self._reported.add(key)
                self.unsupported_log.append(u)
                fresh_events.append(u)
        unsupported_new = fresh_events

        # 3. a decision needs something to decide on
        if not obs.valid:
            return Decision(self.active, self.active,
                            "no observation (window collapsed or blind) - holding",
                            unsupported=unsupported_new, warnings=warnings)

        fresh = (not obs.stale) and obs.confidence > 0.0

        # 4. what does the terrain ask for?
        if self._wants_jump(obs, x_m):
            requested, reason = SkillId.JUMP, "step within jump reach at the near edge"
        elif self._wants_turn(heading_err_deg):
            requested, reason = SkillId.TURN, (
                f"heading off by {heading_err_deg:+.1f} deg, over "
                f"{self.cfg.skill.HEADING_ERR_TURN_DEG:.1f}")
        else:
            requested, reason = self._best_fit(obs, margin=0.0)
            if any(u.fatal for u in unsupported):
                requested = SkillId.WALK
                reason = "unsupported terrain ahead - falling back to WALK and attempting it anyway"

        # 5. upgrades are conditional, downgrades are not
        # TURN is off the duty axis: entering it is never an "upgrade" (so it is not
        # gated behind MIN_HOLD_S -- a heading error is a downgrade-shaped event),
        # and leaving it for a straight gait is, so it goes through the hold timer
        # and the hysteresis check like any other speed-up.
        cur_rank = SAFETY_ORDER.index(self.active) if self.active in SAFETY_ORDER else 0
        req_rank = SAFETY_ORDER.index(requested) if requested in SAFETY_ORDER else 0
        upgrade = requested is not SkillId.JUMP and req_rank > cur_rank

        if upgrade:
            if not fresh:
                return Decision(self.active, requested,
                                "upgrade refused: observation is stale or zero-confidence",
                                unsupported=unsupported_new, warnings=warnings)
            if self._hold_timer < cfg.switch.MIN_HOLD_S:
                return Decision(self.active, requested,
                                f"upgrade refused: {self._hold_timer:.2f} s < MIN_HOLD_S",
                                unsupported=unsupported_new, warnings=warnings)
            ok, why = self._fits(requested, obs, margin=cfg.switch.HYSTERESIS)
            if not ok:
                return Decision(self.active, requested,
                                f"upgrade refused by hysteresis: {why}",
                                unsupported=unsupported_new, warnings=warnings)

        # 6. queue it, and let the delay run.
        #    The command channel holds ONE command in flight. While something is
        #    pending, a new request is dropped and counted rather than replacing
        #    it -- a sent command cannot be unsent, so overwriting the queue would
        #    model a recall that does not exist. Getting this wrong is not
        #    cosmetic: with a 2.4 s delay a queued JUMP survives ~24 ticks, and
        #    letting the next tick's request overwrite it deleted every jump at
        #    long delay while leaving the short-delay numbers untouched.
        if requested is not self.active:
            if self._pending is None:
                self._pending = requested
                self._pending_timer = cfg.switch.SWITCH_DELAY
                self._requested = requested
            elif requested is not self._pending:
                self.requests_dropped_busy += 1

        switched = False
        if self._pending is not None:
            self._pending_timer -= dt
            if self._pending_timer <= 0.0:
                if self._pending is not self.active:
                    self.active = self._pending
                    self.switches += 1
                    switched = True
                    self._hold_timer = 0.0
                    if self.active is SkillId.JUMP:
                        self._jump_lock = self.library[SkillId.JUMP].lockout_s
                        self.jumps += 1
                        if np.isfinite(x_m):
                            self._jump_block_until_x = x_m + cfg.switch.JUMP_RETRIGGER_M
                self._pending = None

        return Decision(self.active, requested, reason, switched=switched,
                        pending=self._pending, pending_in_s=max(self._pending_timer, 0.0),
                        unsupported=unsupported_new, warnings=warnings)

    @property
    def speed(self) -> float:
        """Measured steady speed of the active skill (0 during a jump)."""
        return self.library[self.active].speed_m_s
