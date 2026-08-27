"""Motion-window detection from joint velocity.

Per INDEX.md caveat 4 the ``events.jsonl`` ``skill_send`` timestamps are *not*
usable as motion bounds: ``run.sh`` needs ~2.4 s to start, so the command lands
well after its logged send time (measured lag to first leg motion is 3-6 s).
The window is therefore taken from the robot's own joint velocity.

Detector
--------
``speed[i] = mean_j |dq_ij|`` over the 12 joints (nan-safe), box-smoothed over
``smooth_s`` (50 ms).  Standing still this sits at ~0.03 rad/s and is extremely
quiet, so a two-level (Schmitt-style) split works:

    idle = p10(speed)          peak = p99.5(speed)
    lo   = idle + 0.05*(peak-idle)      hi = idle + 0.25*(peak-idle)

Candidate runs are the samples above ``lo``; runs separated by less than
``bridge_s`` (0.30 s) are merged, runs shorter than ``min_seg_s`` (0.15 s) or
never reaching ``hi`` are dropped.  What survives are the *active segments*.

A session gets one **window** ``[first segment start, last segment end]`` and an
**active mask** = the union of the segments.  Statistics use the mask, so the
quiet gap between e.g. ``stand_down`` and ``recovery_stand`` never counts as
"stance" or as zero body velocity.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from .session import JOINTS, LEGS, Session

Segment = Tuple[int, int]  # [start, stop) sample indices


def runs_of(mask: np.ndarray) -> List[Segment]:
    """Contiguous ``True`` runs of a boolean array as ``[start, stop)`` pairs."""
    m = np.asarray(mask, dtype=np.int8)
    d = np.diff(np.concatenate([[0], m, [0]]))
    return list(zip(np.flatnonzero(d == 1).tolist(), np.flatnonzero(d == -1).tolist()))


def box_smooth(x: np.ndarray, width: int) -> np.ndarray:
    """Nan-safe centred box filter (NaNs are ignored, not spread)."""
    width = max(1, int(width))
    if width < 3:
        return np.nan_to_num(x, nan=0.0)
    ok = np.isfinite(x)
    xf = np.where(ok, x, 0.0)
    k = np.ones(width)
    num = np.convolve(xf, k, mode="same")
    den = np.convolve(ok.astype(float), k, mode="same")
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(den > 0, num / den, 0.0)


def joint_speed(sess: Session) -> np.ndarray:
    """Mean |joint velocity| over the 12 joints, rad/s, nan-safe."""
    dq = sess.joint_matrix("dq")
    with np.errstate(invalid="ignore"):
        return np.nanmean(np.abs(dq), axis=1)


@dataclass
class MotionWindow:
    segments: List[Segment]
    mask: np.ndarray
    speed: np.ndarray          # smoothed joint speed used for the decision
    idle_level: float
    enter_level: float
    exit_level: float

    @property
    def ok(self) -> bool:
        return bool(self.segments)

    def start_stop(self) -> Segment:
        return (self.segments[0][0], self.segments[-1][1])

    def longest(self) -> Segment:
        return max(self.segments, key=lambda se: se[1] - se[0])


def detect_motion(
    sess: Session,
    smooth_s: float = 0.05,
    enter_frac: float = 0.25,
    exit_frac: float = 0.05,
    bridge_s: float = 0.30,
    min_seg_s: float = 0.15,
) -> MotionWindow:
    fs = sess.fs
    speed = box_smooth(joint_speed(sess), int(round(smooth_s * fs)))
    idle = float(np.nanpercentile(speed, 10))
    peak = float(np.nanpercentile(speed, 99.5))
    span = max(peak - idle, 1e-9)
    lo = idle + exit_frac * span
    hi = idle + enter_frac * span

    merged: List[Segment] = []
    for s, e in runs_of(speed > lo):
        if merged and (s - merged[-1][1]) < bridge_s * fs:
            merged[-1] = (merged[-1][0], e)
        else:
            merged.append((s, e))
    segs = [(s, e) for s, e in merged if (e - s) >= min_seg_s * fs and speed[s:e].max() > hi]

    mask = np.zeros(sess.n, dtype=bool)
    for s, e in segs:
        mask[s:e] = True
    return MotionWindow(segs, mask, speed, idle, hi, lo)
