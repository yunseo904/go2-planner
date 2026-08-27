"""Obstacle tracking for the alternative JUMP gate.

The question this module exists to answer
-----------------------------------------
`front_jump` covers 26 mm of ground, so it must be *launched while the robot is
standing at the step*.  The command has to go out ``SWITCH_DELAY`` earlier, which
means the decision is taken ``speed x SWITCH_DELAY`` metres before the obstacle.

The default gate (``JumpGate.NEAR_EDGE``) refuses to jump unless the decision
window still starts at the blind-zone edge.  Past a switch delay of about 0.45 s
it never does, and the jump count drops to zero at every threshold -- which looks
like a physical limit but might just be the window's shape.  The window is
``[lookahead, lookahead + DECISION_WINDOW_M]``: it slides *away* from the robot as
the delay grows, so the band containing "the thing I am about to reach" leaves it.

``JumpGate.TRACKING`` tests the alternative.  It watches the **whole visible
range** (0.25-2.0 m) every tick, converts each detected step into a world-frame
position, keeps it in a track list, and fires when the estimated time of arrival
falls to ``SWITCH_DELAY``.  Nothing about the sensor model changes: detection runs
on the blurred profile, inside the same range limits.  Only *which part of what
the camera already sees* the planner is allowed to act on changes.

If tracking restores the jumps, the zero was the window's doing.  If it does not,
the limit is real.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np

from .config import DEFAULT, PlannerConfig
from .features import Observation


class JumpGate(Enum):
    """How the planner decides a jump is aimable."""

    NEAR_EDGE = "near_edge"     # default: only when the window starts at the blind-zone edge
    TRACKING = "tracking"       # watch the full range, fire on time-to-arrival

    def __str__(self) -> str:
        return self.value


@dataclass
class TrackedObstacle:
    """One step-up seen at a world-frame x, refined as the robot closes on it."""

    x_m: float
    height_m: float
    first_seen_s: float
    last_seen_s: float
    n_obs: int = 1
    fired: bool = False

    def merge(self, x_m: float, height_m: float, t: float, alpha: float = 0.5) -> None:
        """Blend a new sighting in.  Closer sightings are the better ones, so the
        position is pulled towards the new estimate rather than averaged flat."""
        self.x_m = (1 - alpha) * self.x_m + alpha * x_m
        self.height_m = max(self.height_m, height_m)
        self.last_seen_s = t
        self.n_obs += 1


class ObstacleTracker:
    """World-frame track list of jumpable step-ups, built from blurred profiles."""

    def __init__(self, cfg: PlannerConfig = DEFAULT, merge_tol_m: float = 0.20,
                 forget_s: float = 5.0) -> None:
        self.cfg = cfg
        self.merge_tol_m = merge_tol_m
        self.forget_s = forget_s
        self.tracks: List[TrackedObstacle] = []
        self.detections = 0

    # -- detection ---------------------------------------------------------
    def _detect(self, obs: Observation, hs: float) -> List[Tuple[float, float]]:
        """``(distance, rise)`` of every step-up in the profile, blur included.

        The rise is measured over +-``RISE_SPAN`` rather than cell to cell: the
        pixel-footprint blur spreads a real step across several samples, and a
        cell-to-cell difference would under-read exactly the obstacles this gate
        is supposed to catch.
        """
        if obs.profile is None:
            return []
        dists, h = obs.profile
        if h.size < 5:
            return []
        span = max(1, int(round(0.10 / hs)))
        idx = np.arange(span, h.size - span)
        if idx.size == 0:
            return []
        rise = h[idx + span] - h[idx - span]
        ok = np.isfinite(rise) & (rise > 0)
        if not ok.any():
            return []

        walk_max = self.cfg.skill.STEP_WALK_MAX
        jump_max = self.cfg.skill.STEP_JUMP_MAX
        cand = ok & (rise > walk_max) & (rise <= jump_max)
        out: List[Tuple[float, float]] = []
        # one detection per contiguous run, at its peak
        run_start = None
        for i in range(cand.size + 1):
            inside = bool(cand[i]) if i < cand.size else False
            if inside and run_start is None:
                run_start = i
            elif not inside and run_start is not None:
                seg = slice(run_start, i)
                k = int(np.argmax(rise[seg])) + run_start
                out.append((float(dists[idx[k]]), float(rise[k])))
                run_start = None
        return out

    # -- track maintenance -------------------------------------------------
    def update(self, obs: Observation, robot_x: float, t: float, hs: float) -> None:
        for dist, rise in self._detect(obs, hs):
            self.detections += 1
            x_world = robot_x + dist
            hit = min(
                (tr for tr in self.tracks if abs(tr.x_m - x_world) <= self.merge_tol_m),
                key=lambda tr: abs(tr.x_m - x_world),
                default=None,
            )
            if hit is None:
                self.tracks.append(TrackedObstacle(x_world, rise, t, t))
            else:
                hit.merge(x_world, rise, t)
        # drop what is behind us or long unseen
        self.tracks = [
            tr for tr in self.tracks
            if tr.x_m > robot_x - 0.5 and (t - tr.last_seen_s) <= self.forget_s
        ]

    # -- trigger -----------------------------------------------------------
    def due(self, robot_x: float, speed_m_s: float, t: float, tick_s: float) -> Optional[TrackedObstacle]:
        """The obstacle whose time-to-arrival has fallen to ``SWITCH_DELAY``.

        Firing exactly one tick of tolerance early is deliberate: the planner is
        sampled at ``TICK_HZ``, so an exact equality test would miss the crossing
        whenever it falls between two ticks.
        """
        if speed_m_s <= 1e-6:
            return None
        delay = self.cfg.switch.SWITCH_DELAY
        best = None
        for tr in self.tracks:
            if tr.fired:
                continue
            ahead = tr.x_m - robot_x
            if ahead <= 0:
                continue
            eta = ahead / speed_m_s
            if eta <= delay + tick_s:
                if best is None or ahead < best.x_m - robot_x:
                    best = tr
        return best

    def mark_fired(self, tr: TrackedObstacle) -> None:
        tr.fired = True

    # -- diagnostics --------------------------------------------------------
    @property
    def n_tracks(self) -> int:
        return len(self.tracks)

    def n_missed(self, robot_x: float) -> int:
        """Tracks the robot has already passed without ever firing at them."""
        return sum(1 for tr in self.tracks if not tr.fired and tr.x_m < robot_x)
