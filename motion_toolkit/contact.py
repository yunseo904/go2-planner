"""Per-session contact detection and gait metrics from ``foot_force``.

INDEX.md caveat 3: the logged ``contact_*`` columns use a fixed 20 N threshold
that does not hold across speeds, so contact is re-derived per session, per leg,
from that leg's force distribution *inside the motion window*:

    p5, p95  = 5th / 95th percentile of the leg force over the active samples
    span     = p95 - p5
    enter    = p5 + 0.45*span        leave = p5 + 0.25*span

The two levels are used as a Schmitt trigger (contact latches on above ``enter``
and only releases below ``leave``), then runs shorter than ``min_run_s`` (40 ms)
are absorbed into their neighbour.  Without that hysteresis+despeckle step the
force ripple inside a stance phase double-counts touchdowns and inflates stride
frequency by up to 2x (seen on the slow-walk sessions).

Caveat 2 still applies: force clips at ~210 N, so contact *timing* is sound but
impact *magnitude* is an underestimate.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .session import LEGS
from .window import Segment, runs_of

ENTER_FRAC = 0.45
LEAVE_FRAC = 0.25
MIN_RUN_S = 0.04


def schmitt(x: np.ndarray, leave: float, enter: float) -> np.ndarray:
    """Latching threshold: on above ``enter``, off below ``leave``.

    Vectorised: a sample is unambiguous when it is above ``enter`` (on) or below
    ``leave`` (off); samples in the band inherit the last unambiguous state.
    """
    on = x > enter
    off = x < leave
    state = np.where(on, 1, np.where(off, 0, -1))
    idx = np.flatnonzero(state >= 0)
    if idx.size == 0:
        return np.zeros(len(x), dtype=bool)
    # forward-fill the last decided state; before the first decision use it too
    pos = np.searchsorted(idx, np.arange(len(x)), side="right") - 1
    pos = np.clip(pos, 0, None)
    return state[idx][pos].astype(bool)


def despeckle(mask: np.ndarray, min_len: int) -> np.ndarray:
    """Remove interior True/False runs shorter than ``min_len`` samples."""
    if min_len < 2:
        return mask
    m = mask.copy()
    for _ in range(3):
        changed = False
        for value in (True, False):
            for s, e in runs_of(m == value):
                if (e - s) < min_len and s > 0 and e < len(m):
                    m[s:e] = not value
                    changed = True
        if not changed:
            break
    return m


@dataclass
class ContactResult:
    contact: np.ndarray        # (n, 4) bool, native LEGS order
    thresholds: np.ndarray     # (4,) the ``enter`` level per leg, N
    leave: np.ndarray          # (4,) the ``leave`` level per leg, N


def detect_contact(force: np.ndarray, mask: np.ndarray, fs: float) -> ContactResult:
    """Per-leg contact from ``(n, 4)`` foot force, calibrated on ``mask``."""
    ref = force[mask] if mask.any() else force
    with np.errstate(invalid="ignore"):
        p5, p95 = np.nanpercentile(ref, [5, 95], axis=0)
    span = np.maximum(p95 - p5, 1e-6)
    enter = p5 + ENTER_FRAC * span
    leave = p5 + LEAVE_FRAC * span
    min_run = int(round(MIN_RUN_S * fs))
    contact = np.zeros(force.shape, dtype=bool)
    for i in range(force.shape[1]):
        col = force[:, i]
        col = np.where(np.isfinite(col), col, np.nan)
        # NaN samples inherit the neighbouring state via the Schmitt band
        filled = np.where(np.isfinite(col), col, (enter[i] + leave[i]) / 2.0)
        contact[:, i] = despeckle(schmitt(filled, leave[i], enter[i]), min_run)
    return ContactResult(contact, enter, leave)


def touchdown_times(contact_leg: np.ndarray, t: np.ndarray, segments: List[Segment]) -> np.ndarray:
    """Touchdown (rising-edge) times of one leg, restricted to active segments."""
    out: List[float] = []
    for s, e in segments:
        c = contact_leg[s:e].astype(np.int8)
        rise = np.flatnonzero(np.diff(c) == 1) + 1
        out.extend(t[s:e][rise].tolist())
    return np.asarray(out, dtype=float)


def stride_intervals(contact: np.ndarray, t: np.ndarray, segments: List[Segment]) -> tuple:
    """(pooled touchdown intervals over all legs, per-leg interval lists)."""
    pooled: List[float] = []
    per_leg: List[List[float]] = []
    for i in range(contact.shape[1]):
        td = touchdown_times(contact[:, i], t, segments)
        iv: List[float] = []
        for s, e in segments:
            in_seg = td[(td >= t[s]) & (td < t[e - 1])]
            if in_seg.size >= 2:
                iv.extend(np.diff(in_seg).tolist())
        pooled.extend(iv)
        per_leg.append(iv)
    return np.asarray(pooled, dtype=float), per_leg


def stride_frequency(contact: np.ndarray, t: np.ndarray, segments: List[Segment]) -> tuple:
    """``(stride Hz, per-leg Hz, interval CV, n_intervals)``.

    Intervals are pooled across legs and segments and reduced with a median, so a
    single mis-detected touchdown cannot move the answer.  The coefficient of
    variation of the pooled intervals says whether the motion was periodic at all
    -- for the aperiodic skills (``front_jump``, ``stand_down``) it blows up and
    the frequency/duty numbers must not be read as gait parameters.
    """
    pooled, per_leg_iv = stride_intervals(contact, t, segments)
    per_leg = np.asarray(
        [1.0 / float(np.median(iv)) if len(iv) >= 1 else np.nan for iv in per_leg_iv], dtype=float
    )
    if pooled.size == 0:
        return np.nan, per_leg, np.nan, 0
    med = float(np.median(pooled))
    cv = float(pooled.std() / pooled.mean()) if pooled.mean() > 0 else np.nan
    return (1.0 / med if med > 0 else np.nan), per_leg, cv, int(pooled.size)


def relative_phases(contact: np.ndarray, t: np.ndarray, segments: List[Segment], cycle_s: float) -> np.ndarray:
    """Touchdown phase of each leg relative to leg 0 (FR), in [0, 1)."""
    ref = touchdown_times(contact[:, 0], t, segments)
    phases = np.full(contact.shape[1], np.nan)
    phases[0] = 0.0
    if ref.size < 2 or not np.isfinite(cycle_s) or cycle_s <= 0:
        return phases
    for i in range(1, contact.shape[1]):
        td = touchdown_times(contact[:, i], t, segments)
        if td.size == 0:
            continue
        # nearest preceding reference touchdown for each of this leg's touchdowns
        idx = np.searchsorted(ref, td, side="right") - 1
        keep = idx >= 0
        if not keep.any():
            continue
        dphi = ((td[keep] - ref[idx[keep]]) / cycle_s) % 1.0
        # circular median
        ang = 2 * np.pi * dphi
        phases[i] = (np.arctan2(np.sin(ang).mean(), np.cos(ang).mean()) / (2 * np.pi)) % 1.0
    return phases


def pair_correlations(contact: np.ndarray, mask: np.ndarray) -> dict:
    """Pearson correlation between the four legs' contact signals over ``mask``.

    Correlation is used instead of touchdown phase because at duty factors above
    0.5 the legs overlap and a touchdown-phase estimate becomes ambiguous, while
    "which legs are on the ground at the same time" stays well defined.
    """
    c = contact[mask].astype(float)
    out = {}
    for i in range(4):
        for j in range(i + 1, 4):
            a, b = c[:, i], c[:, j]
            sa, sb = a.std(), b.std()
            out[f"{LEGS[i]}{LEGS[j]}"] = float(np.mean((a - a.mean()) * (b - b.mean())) / (sa * sb)) if sa > 1e-9 and sb > 1e-9 else np.nan
    return out


#: Leg pairings that define the classic quadruped patterns (native order FR,FL,RR,RL).
PAIRINGS = {
    "trot": ("FRRL", "FLRR"),          # diagonal pairs
    "pace": ("FRRR", "FLRL"),          # same-side pairs
    "bound": ("FRFL", "RRRL"),         # front pair / rear pair
}


def classify_pattern(contact: np.ndarray, mask: np.ndarray, margin: float = 0.15) -> tuple:
    """``(pattern name, scores dict)`` from the contact correlation structure."""
    corr = pair_correlations(contact, mask)
    if not np.isfinite(list(corr.values())).all():
        return "degenerate", corr
    scores = {k: float(np.mean([corr[a], corr[b]])) for k, (a, b) in PAIRINGS.items()}
    if min(corr.values()) > 0.5:
        return "four-leg-sync", {**corr, **scores}
    best = max(scores, key=scores.get)
    rest = max(v for k, v in scores.items() if k != best)
    if scores[best] > 0.4 and (scores[best] - rest) > margin:
        return best, {**corr, **scores}
    return "asymmetric", {**corr, **scores}
