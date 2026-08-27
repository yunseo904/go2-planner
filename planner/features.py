"""Geometry of the lookahead window, as the depth camera would see it.

The planner never gets the frozen height field directly.  It gets what a camera
with `far_clip = 2 m`, a 0.25 m blind zone and a ground footprint that grows from
~2.7 cm/px at 0.7 m to ~24 cm/px at 2.0 m would report.  Skipping that model
would hand the rule planner information the E2E policy does not have, and the
comparison would stop meaning anything.

The decision window
-------------------
``lookahead = BASE_MARGIN + SWITCH_DELAY · speed`` is the *commitment* distance:
terrain closer than that will be underfoot before a switch ordered now could take
effect, so it is no longer actionable.  The window a decision is made on is
therefore

    from = max(SENSOR_NEAR, lookahead)
    to   = min(SENSOR_FAR,  lookahead + DECISION_WINDOW_M)

A longer delay does not blind the camera, it slides the actionable band outwards
into the blurrier part of the range and eventually off the end of it -- which is
the cost the delay sweep is meant to expose.  When the band collapses (lookahead
at or past ``far_clip``) that is reported as a warning, never as an empty result
the caller might mistake for flat ground.

Sampling
--------
Along the heading, every ``horizontal_scale`` (0.05 m), across that window.
At each position a lateral cross-section is
taken and the **nearest walkable surface within ±CORRIDOR_HALF_WIDTH** of the
centre line is followed -- the same rule ``terrain_toolkit.profile`` uses,
because stepping-stone goals sit inside the pit and a strict centre-line sample
would call the whole task a gap.

Sensor degradation
------------------
1. **Blind zone.**  Nothing closer than ``SENSOR_NEAR`` (0.25 m) is observable.
   What is under the robot right now was seen a moment ago, so
   :class:`FeatureMemory` carries the last valid observation forward instead of
   letting the planner see nothing.
2. **Range cut.**  Nothing beyond ``SENSOR_FAR`` (2.0 m) exists at all.  A
   lookahead at or past it is not silently truncated -- it raises a warning
   through :class:`Observation.warnings` and the observation is marked invalid.
3. **Resolution.**  Ground footprint per pixel is interpolated as a power law
   through the two anchors in `CLAUDE.md` §3, ``res(d) = a·d^b`` with
   ``a ≈ 0.057``, ``b ≈ 2.08``.  The followed surface is box-averaged over
   ``res(d)`` at each distance, so far-away steps blur out exactly the way they
   would in the depth image -- a 0.10 m step at 1.8 m is smeared across roughly
   one pixel and stops being detectable as a step.
4. **Confidence.**  1.0 out to ``RELIABLE_RANGE`` (1 m), falling linearly to 0 at
   ``SENSOR_FAR``.  Reported per feature so a rule can refuse to act on a
   low-confidence reading rather than acting on noise.

Nothing here imports Isaac Lab / torch.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .config import DEFAULT, FeatureParams, PlannerConfig, SensorLimits

FEATURE_NAMES = (
    "step_up_m", "step_down_m", "step_up_true_m", "step_down_true_m",
    "gap_m", "gap_corridor_m", "gap_start_m",
    "slope_max_deg", "slope_mean_deg", "width_min_m", "roughness_m",
    "rise_m", "pit_fraction",
)


# --------------------------------------------------------------------------- #
# Terrain access
# --------------------------------------------------------------------------- #
@dataclass
class TerrainMap:
    """One frozen ``(task, level)`` height field in metres, ``[x, y]``, x forward."""

    height_m: np.ndarray
    horizontal_scale: float
    task: str = ""
    level: int = -1
    goals_m: Optional[np.ndarray] = None
    spawn_m: Tuple[float, float] = (1.0, 2.0)

    @property
    def extent_m(self) -> Tuple[float, float]:
        X, Y = self.height_m.shape
        return (X * self.horizontal_scale, Y * self.horizontal_scale)

    def lookup(self, xy_m: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Nearest-cell heights for ``(..., 2)`` metre coordinates; NaN outside."""
        X, Y = self.height_m.shape
        hs = self.horizontal_scale
        ix = np.rint(xy_m[..., 0] / hs).astype(int)
        iy = np.rint(xy_m[..., 1] / hs).astype(int)
        inside = (ix >= 0) & (ix < X) & (iy >= 0) & (iy < Y)
        out = np.full(ix.shape, np.nan)
        out[inside] = self.height_m[ix[inside], iy[inside]]
        return out, inside

    def route(self) -> np.ndarray:
        """``(num_goals + 1, 2)`` waypoint route: spawn followed by the goals."""
        spawn = np.asarray(self.spawn_m, dtype=float)[None, :]
        if self.goals_m is None:
            X, _ = self.extent_m
            return np.vstack([spawn, [[X - 0.5, spawn[0, 1]]]])
        return np.vstack([spawn, np.asarray(self.goals_m, dtype=float)])


def maps_from_archive(z: Dict[str, np.ndarray], key: str = "height_fields_before_fix") -> List[TerrainMap]:
    """Every ``(task, level)`` in the frozen archive.

    ``height_fields_before_fix`` is the default on purpose: at the frozen commit
    the upstream benchmark branch never calls ``fix_terrain``, so the pre-fix
    array is what the simulator rasterises (see ``terrain_toolkit/README.md``).
    """
    hf = z[key].astype(np.float64) * float(z["vertical_scale"])
    goals = z["goals_before_fix"] if key.endswith("before_fix") else z["goals"]
    names = [str(n) for n in z["task_names"]]
    hs = float(z["horizontal_scale"])
    spawn = (float(z["spawn_x"]), float(z["spawn_y"]))
    out = []
    for ti in range(hf.shape[0]):
        for li in range(hf.shape[1]):
            out.append(TerrainMap(hf[ti, li], hs, names[ti], li, goals[ti, li], spawn))
    return out


# --------------------------------------------------------------------------- #
# Sensor model
# --------------------------------------------------------------------------- #
def ground_resolution(distance_m: np.ndarray, sensor: SensorLimits) -> np.ndarray:
    """Ground footprint of one depth pixel, metres, at the given range.

    Power law through the two anchors documented in `CLAUDE.md` §3.  The growth
    is faster than linear because the camera looks at the ground obliquely
    (mounted 0.52 rad down), so range and incidence both worsen with distance.
    """
    (d0, r0), (d1, r1) = sensor.RES_ANCHOR_NEAR, sensor.RES_ANCHOR_FAR
    b = math.log(r1 / r0) / math.log(d1 / d0)
    a = r0 / (d0 ** b)
    return a * np.power(np.maximum(distance_m, 1e-3), b)


def confidence(distance_m: np.ndarray, sensor: SensorLimits) -> np.ndarray:
    """1.0 inside ``RELIABLE_RANGE``, falling linearly to 0 at ``SENSOR_FAR``."""
    span = max(sensor.SENSOR_FAR - sensor.RELIABLE_RANGE, 1e-6)
    return np.clip(1.0 - (distance_m - sensor.RELIABLE_RANGE) / span, 0.0, 1.0)


def _blur_by_resolution(values: np.ndarray, dist: np.ndarray, hs: float, sensor: SensorLimits) -> np.ndarray:
    """Box-average each sample over the ground footprint of a pixel at its range.

    Variable-width running mean, done with prefix sums so the cost does not grow
    with the window: NaNs are excluded from both the sum and the count.
    """
    res = ground_resolution(dist, sensor)
    half = np.maximum((res / hs / 2.0).astype(int), 0)
    n = values.size
    idx = np.arange(n)
    lo = np.maximum(idx - half, 0)
    hi = np.minimum(idx + half + 1, n)

    finite = np.isfinite(values)
    filled = np.where(finite, values, 0.0)
    csum = np.concatenate([[0.0], np.cumsum(filled)])
    ccnt = np.concatenate([[0], np.cumsum(finite.astype(np.int64))])
    total = csum[hi] - csum[lo]
    count = ccnt[hi] - ccnt[lo]
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(count > 0, total / np.maximum(count, 1), np.nan)


def _sliding(a: np.ndarray, win: int) -> np.ndarray:
    """``(n - win + 1, win)`` read-only sliding window view."""
    return np.lib.stride_tricks.sliding_window_view(a, win)


def _detrended_rms(values: np.ndarray, win: int) -> np.ndarray:
    """RMS residual to the best-fit line over every ``win``-long window.

    Closed form rather than ``polyfit`` per window: with ``x = 0..win-1`` fixed,
    the slope and intercept are sums, so the whole thing is three sliding sums.
    """
    if values.size < win or win < 3:
        return np.empty(0)
    w = _sliding(values, win)
    x = np.arange(win, dtype=float)
    x_mean = x.mean()
    sxx = float(((x - x_mean) ** 2).sum())
    y_mean = w.mean(axis=1, keepdims=True)
    sxy = ((x - x_mean)[None, :] * (w - y_mean)).sum(axis=1, keepdims=True)
    slope = sxy / sxx
    resid = w - (y_mean + slope * (x - x_mean)[None, :])
    return np.sqrt((resid ** 2).mean(axis=1))


# --------------------------------------------------------------------------- #
# Features
# --------------------------------------------------------------------------- #
@dataclass
class Observation:
    """What the planner is allowed to know at one tick."""

    features: Dict[str, float]
    confidence: float
    lookahead_m: float
    observed_from_m: float
    observed_to_m: float
    n_samples: int
    warnings: List[str] = field(default_factory=list)
    stale: bool = False
    stale_age_s: float = 0.0
    #: ``(dists, h_seen)`` of the followed surface, populated only when
    #: ``with_profile=True``.  ``h_seen`` is post-blur -- it is what the sensor
    #: would report, so a tracker built on it stays inside the sensor model.
    profile: Optional[Tuple[np.ndarray, np.ndarray]] = None

    def __getitem__(self, key: str) -> float:
        return self.features[key]

    @property
    def valid(self) -> bool:
        return self.n_samples >= 2


def _empty_features() -> Dict[str, float]:
    f = {k: 0.0 for k in FEATURE_NAMES}
    f["width_min_m"] = np.nan
    f["slope_max_deg"] = np.nan
    f["slope_mean_deg"] = np.nan
    f["roughness_m"] = np.nan
    return f


def _runs(mask: np.ndarray) -> List[Tuple[int, int]]:
    if mask.size == 0:
        return []
    padded = np.concatenate([[False], mask, [False]]).astype(np.int8)
    d = np.diff(padded)
    return list(zip(np.nonzero(d == 1)[0].tolist(), np.nonzero(d == -1)[0].tolist()))


def lookahead_distance(speed_m_s: float, cfg: PlannerConfig = DEFAULT) -> float:
    """``BASE_MARGIN + SWITCH_DELAY · speed`` -- how far ahead a decision must be made.

    At ``SWITCH_DELAY`` 0.21 s this is ~0.10 m at trot speed; at 2.4 s it is
    ~1.15 m; at 4.06 s ~1.95 m, which is at the edge of ``far_clip``.
    """
    return cfg.switch.BASE_MARGIN + cfg.switch.SWITCH_DELAY * max(speed_m_s, 0.0)


def decision_window(lookahead_m: float, cfg: PlannerConfig = DEFAULT) -> Tuple[float, float]:
    """``(from_m, to_m)`` of the band a decision is actually made on.

    Empty (``from >= to``) when the commitment distance has slid past
    ``far_clip``; callers must treat that as "cannot see far enough to decide",
    not as "nothing there".
    """
    sensor, fp = cfg.sensor, cfg.feature
    near = max(sensor.SENSOR_NEAR, lookahead_m)
    far = min(sensor.SENSOR_FAR, lookahead_m + fp.DECISION_WINDOW_M)
    return near, far


def extract(
    tmap: TerrainMap,
    x_m: float,
    y_m: float,
    lookahead_m: float,
    cfg: PlannerConfig = DEFAULT,
    heading: Tuple[float, float] = (1.0, 0.0),
    window: Optional[Tuple[float, float]] = None,
    with_profile: bool = False,
) -> Observation:
    """Sensor-limited geometry of the decision window ahead of ``(x_m, y_m)``.

    ``window`` overrides the delay-derived decision window with an explicit
    ``(from_m, to_m)``.  Used by the tracking jump gate, which watches the whole
    visible range instead of only the band it can still commit to.

    ``heading`` is a unit vector; the cross-sections are always taken along the
    terrain's lateral (y) axis, because the course is x-forward and every
    obstacle is built axis-aligned (same reasoning as ``profile.py``).
    """
    fp: FeatureParams = cfg.feature
    sensor: SensorLimits = cfg.sensor
    hs = tmap.horizontal_scale
    warnings: List[str] = []

    near, far = decision_window(lookahead_m, cfg) if window is None else window
    near = max(near, sensor.SENSOR_NEAR)
    far = min(far, sensor.SENSOR_FAR)

    if window is not None:
        pass                                   # caller chose the band deliberately
    elif lookahead_m > sensor.SENSOR_FAR:
        warnings.append(
            f"lookahead {lookahead_m:.2f} m is beyond far_clip {sensor.SENSOR_FAR:.2f} m - "
            f"the commitment point is outside the sensor range, decision would be blind"
        )
    elif lookahead_m > sensor.RELIABLE_RANGE:
        warnings.append(
            f"lookahead {lookahead_m:.2f} m is past the {sensor.RELIABLE_RANGE:.2f} m reliable "
            f"range - geometry at the commitment point is blurred by pixel footprint"
        )
    if lookahead_m < sensor.SENSOR_NEAR:
        warnings.append(
            f"lookahead {lookahead_m:.2f} m is inside the {sensor.SENSOR_NEAR:.2f} m blind zone - "
            f"the decision window starts at the blind-zone edge instead"
        )

    if far - near < hs:
        obs = Observation(_empty_features(), 0.0, lookahead_m, near, far, 0, warnings)
        obs.warnings.append(
            f"decision window [{near:.2f}, {far:.2f}] m has collapsed - nothing observable to decide on"
        )
        return obs

    u = np.asarray(heading, dtype=float)
    u = u / max(float(np.hypot(*u)), 1e-9)
    dists = np.arange(near, far + hs / 2, hs)
    pts = np.array([x_m, y_m])[None, :] + dists[:, None] * u[None, :]

    offs = np.arange(-fp.CROSS_HALF_WIDTH, fp.CROSS_HALF_WIDTH + hs / 2, hs)
    K = offs.size
    k0 = int(np.argmin(np.abs(offs)))
    lateral_dir = np.array([0.0, 1.0])
    grid = pts[:, None, :] + offs[None, :, None] * lateral_dir[None, None, :]
    H, inside = tmap.lookup(grid)
    N = H.shape[0]

    f = _empty_features()
    conf = float(confidence(dists, sensor).mean())
    if N < 2:
        return Observation(f, conf, lookahead_m, near, far, N, warnings)

    walkable = inside & (H > fp.PIT_THRESH)

    corridor_cells = int(round(fp.CORRIDOR_HALF_WIDTH / hs))
    lateral = np.abs(np.arange(K) - k0)
    cand = np.where(walkable, lateral[None, :], K + 1)
    k_near = cand.argmin(axis=1)
    near_dist = cand[np.arange(N), k_near]
    gap = near_dist > corridor_cells
    h_walk = np.where(gap, np.nan, H[np.arange(N), k_near])

    # --- gaps ---------------------------------------------------------------
    # Measured on the RAW centre line, with no route correction and no blurring.
    # Following the nearest walkable surface within +-CORRIDOR_HALF_WIDTH is right
    # for step/slope/width -- it is how the robot would actually place its feet --
    # but applied to gaps it hides them: a line of stepping stones has a stone
    # within half a metre of almost every point, so the corridor rule reports
    # continuous ground where there is a pit. A gap is a topology fact about the
    # ground under the path, so it is read straight off the height field.
    center_gap = ~(inside[:, k0] & (H[:, k0] > fp.PIT_THRESH))
    f["pit_fraction"] = float(center_gap.mean())
    center_runs = _runs(center_gap)
    if center_runs:
        widths = [(e - s_) * hs for s_, e in center_runs]
        k = int(np.argmax(widths))
        f["gap_m"] = float(widths[k])
        f["gap_start_m"] = float(dists[center_runs[k][0]])
    else:
        f["gap_m"] = 0.0
        f["gap_start_m"] = np.nan

    # The corridor-following version is kept alongside it: the difference between
    # the two is exactly "how far off the straight line is there something to
    # stand on", which is what tells a stepping-stone field from a plain pit.
    corridor_runs = _runs(gap)
    if corridor_runs:
        widths = [(e - s_) * hs for s_, e in corridor_runs]
        f["gap_corridor_m"] = float(max(widths))
    else:
        f["gap_corridor_m"] = 0.0

    # --- sensor blur, then the height-derived features -------------------------
    h_seen = _blur_by_resolution(h_walk, dists, hs, sensor)

    # *_true_m are the same steps *without* the pixel-footprint blur. They are a
    # DIAGNOSTIC ONLY -- they say how much geometry the sensor model removed at
    # this range. The rules must never read them: the planner does not get to see
    # the height field, that is the whole point of the sensor model.
    true_ok = np.isfinite(h_walk)
    if true_ok.sum() >= 2:
        dtrue = np.diff(h_walk[true_ok])
        f["step_up_true_m"] = float(max(dtrue.max(), 0.0)) + 0.0
        f["step_down_true_m"] = float(max(-dtrue.min(), 0.0)) + 0.0

    ok = np.isfinite(h_seen)
    if ok.sum() >= 2:
        hw = h_seen[ok]
        dh = np.diff(hw)
        f["step_up_m"] = float(max(dh.max(), 0.0)) + 0.0
        f["step_down_m"] = float(max(-dh.min(), 0.0)) + 0.0
        f["rise_m"] = float(hw[-1] - hw[0])

        win = max(2, int(round(fp.SLOPE_WINDOW / hs)))
        if hw.size > win:
            step_in_win = np.abs(_sliding(dh, win)).max(axis=1)
            rise = hw[win:] - hw[:-win]
            # a window is slope only if no single cell jumps, and the rise is
            # distributed rather than concentrated in one cell (that is a step)
            usable = (step_in_win <= fp.STEP_THRESH) & (step_in_win <= np.abs(rise) / 2 + 1e-9)
            if usable.any():
                ang = np.degrees(np.arctan2(rise[usable], win * hs))
                f["slope_max_deg"] = float(np.abs(ang).max())
                f["slope_mean_deg"] = float(np.abs(ang).mean())
            else:
                f["slope_max_deg"] = f["slope_mean_deg"] = 0.0
        else:
            f["slope_max_deg"] = f["slope_mean_deg"] = 0.0

        # roughness = RMS residual to a local straight line, i.e. what is left
        # after slope and a single step are taken out
        rwin = max(3, int(round(fp.ROUGHNESS_WINDOW / hs)))
        resid = _detrended_rms(hw, rwin)
        f["roughness_m"] = float(resid.mean()) if resid.size else 0.0

    # --- lateral clearance -----------------------------------------------------
    # Two lateral cells belong to the same walkable span when both are walkable
    # and the height change between them is under STEP_THRESH (a cliff or a wall
    # ends the span). Labelling the spans with a cumulative count of breaks turns
    # "how wide is the span containing the followed sample" into one comparison.
    link = walkable[:, :-1] & walkable[:, 1:] & (np.abs(np.diff(H, axis=1)) <= fp.STEP_THRESH)
    seg_id = np.concatenate([np.zeros((N, 1), dtype=np.int64),
                             np.cumsum(~link, axis=1)], axis=1)
    # non-walkable cells get an id of their own so they never merge into a span
    seg_id = np.where(walkable, seg_id, -1 - np.arange(K)[None, :])
    own = seg_id[np.arange(N), k_near]
    span_cells = (seg_id == own[:, None]).sum(axis=1)
    usable = ~gap
    f["width_min_m"] = float(span_cells[usable].min() * hs) if usable.any() else np.nan

    prof = (dists.copy(), h_seen.copy()) if with_profile else None
    return Observation(f, conf, lookahead_m, near, far, N, warnings, profile=prof)


# --------------------------------------------------------------------------- #
class FeatureMemory:
    """Carries the last valid observation across the 0-0.25 m blind zone.

    `CLAUDE.md` §4: the robot cannot see its own feet, so a planner that reacts
    only to what is currently visible would forget an obstacle at the moment it
    matters most.  ``update`` returns the fresh observation when there is one and
    the remembered one -- flagged ``stale`` with its age -- when there is not.
    """

    def __init__(self, max_age_s: float = 2.0) -> None:
        self.max_age_s = max_age_s
        self._last: Optional[Observation] = None
        self._age: float = 0.0

    def update(self, obs: Observation, dt: float) -> Observation:
        if obs.valid:
            self._last = obs
            self._age = 0.0
            return obs
        self._age += dt
        if self._last is None or self._age > self.max_age_s:
            obs.stale, obs.stale_age_s = True, self._age
            return obs
        held = Observation(
            dict(self._last.features), self._last.confidence, obs.lookahead_m,
            self._last.observed_from_m, self._last.observed_to_m, self._last.n_samples,
            list(obs.warnings) + [f"holding last observation, {self._age:.2f} s old"],
            stale=True, stale_age_s=self._age,
        )
        return held
