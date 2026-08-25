"""Per-goal-segment terrain features for the rule-based skill planner.

A terrain has ``num_goals`` (8) goals. The robot spawns at ``(spawn_x, spawn_y)``
so the route is ``spawn -> goal0 -> goal1 -> ... -> goal7`` = 8 segments.
Segment *k* is the straight line from waypoint *k* to goal *k* (waypoint 0 is
the spawn).

Sampling model
--------------
Along each segment we take positions every ``hs`` (= horizontal_scale, one
cell) and, at every position, a lateral cross-section along the terrain's y
axis (``+-cross_half_width``, one sample per cell, nearest-cell lookup). The
course is x-forward and all obstacles are axis-aligned, so lateral clearance
is the meaningful "width"; a perpendicular section of a slightly diagonal
segment would clip platform corners.

The benchmark generators carve *pits* at exactly -1.0 m to force jumping, and
some of them (stepping stones) place goals *inside* the pit between stones, so
the straight centre line is not always on a surface.  We therefore follow the
**nearest walkable surface**: at each position the walkable sample closest to
the centre line, searched within ``+-corridor_half_width`` (0.5 m).  If there
is none, the position is a *gap*.  ``h_walk`` is the height of that surface.

Feature definitions (lengths in metres, angles in degrees)
-----------------------------------------------------------
* **max_gap_width_m** – longest run of gap positions (``run * hs``);
  ``n_gaps`` counts runs; ``gap_landing_dz_m`` = ``h_walk`` after the widest
  run minus ``h_walk`` before it (landing minus take-off; NaN if the run
  touches a segment end).
* **max_step_up_m / max_step_down_m / max_step_m** – largest change of
  ``h_walk`` between consecutive non-gap positions (a jump over a pit
  therefore contributes take-off -> landing, not the pit depth).
* **mean_slope_deg / max_slope_deg** – slope of the *continuous* surface over
  a ``slope_window`` (0.2 m = 4 cells) baseline.  A window counts only if it
  contains no gap, no single-cell change above ``step_thresh`` (0.08 m), and
  its rise is distributed (largest single-cell change <= half of the window's
  total rise) – so stairs are steps, ramps and domes are slope.
* **min_width_m / median_width_m** – at every non-gap position, the
  contiguous lateral span (along the terrain's y axis) around the followed
  sample that is bounded by a discontinuity: pit, terrain edge, or a height
  change > ``step_thresh`` between neighbouring lateral cells (cliff, wall).
  Positions at the x-edge of an obstacle (followed sample not walkable one
  position before/after) are skipped.  Minimum / median over the segment.
  Bounded by the terrain width (4 m); NaN if every position is a gap or an edge.
* **lateral_offset_max_m** – how far (perpendicular) the followed surface
  strays from the straight line (0 for box jumps, ~0.2-0.3 for stepping stones).
* **center_pit_fraction** – fraction of positions whose *centre-line* sample is pit.
* **rise_m** – ``h_walk`` at segment end minus at segment start.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from .stub import BenchmarkTerrainCfg


@dataclass(frozen=True)
class ProfileParams:
    pit_thresh: float = -0.5          # [m] below this a sample is pit (generators use exactly -1.0)
    corridor_half_width: float = 0.5  # [m] how far from the straight line a surface may be followed
    step_thresh: float = 0.08         # [m] per cell; larger single-cell changes never count as slope
    slope_window: float = 0.2         # [m] baseline for slope estimation
    cross_half_width: float = 2.0     # [m] lateral cross-section extent on each side of the centre line


FEATURE_COLUMNS = [
    "path_len_m", "max_step_up_m", "max_step_down_m", "max_step_m",
    "max_gap_width_m", "n_gaps", "gap_landing_dz_m",
    "mean_slope_deg", "max_slope_deg", "min_width_m", "median_width_m",
    "lateral_offset_max_m", "center_pit_fraction",
    "rise_m", "max_height_m", "min_height_m",
]


def waypoints(goals_m: np.ndarray, cfg: BenchmarkTerrainCfg) -> np.ndarray:
    """(num_goals + 1, 2) route: spawn followed by the goals."""
    return np.vstack([[cfg.spawn_x, cfg.spawn_y], goals_m])


def _sample_line(p0: np.ndarray, p1: np.ndarray, hs: float):
    """Points along p0->p1 spaced ~hs apart (inclusive), the unit normal and the length."""
    d = p1 - p0
    dist = float(np.hypot(*d))
    n = max(2, int(np.ceil(dist / hs)) + 1)
    t = np.linspace(0.0, 1.0, n)
    pts = p0[None, :] + t[:, None] * d[None, :]
    if dist < 1e-9:
        normal = np.array([0.0, 1.0])
    else:
        u = d / dist
        normal = np.array([-u[1], u[0]])
    return pts, normal, dist


def _lookup(hf_m: np.ndarray, xy_m: np.ndarray, hs: float):
    """Nearest-cell heights for (..., 2) metre coordinates; NaN outside the field."""
    X, Y = hf_m.shape
    ix = np.rint(xy_m[..., 0] / hs).astype(int)
    iy = np.rint(xy_m[..., 1] / hs).astype(int)
    inside = (ix >= 0) & (ix < X) & (iy >= 0) & (iy < Y)
    out = np.full(ix.shape, np.nan)
    out[inside] = hf_m[ix[inside], iy[inside]]
    return out, inside


def _runs(mask: np.ndarray) -> List[tuple]:
    """(start, end_exclusive) of True runs."""
    if mask.size == 0:
        return []
    padded = np.concatenate([[False], mask, [False]]).astype(np.int8)
    d = np.diff(padded)
    return list(zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]))


def _empty_features(dist: float) -> Dict[str, float]:
    f: Dict[str, float] = {k: np.nan for k in FEATURE_COLUMNS}
    f["path_len_m"] = dist
    return f


def profile_segment(hf_m: np.ndarray, p0: np.ndarray, p1: np.ndarray, hs: float,
                    params: ProfileParams = ProfileParams()) -> Dict[str, float]:
    pts, _normal, dist = _sample_line(p0, p1, hs)
    f = _empty_features(dist)

    offs = np.arange(-params.cross_half_width, params.cross_half_width + hs / 2, hs)
    K = offs.size
    k0 = int(np.argmin(np.abs(offs)))
    # Cross-sections run along the terrain's lateral (y) axis: the course is x-forward and
    # every obstacle is built axis-aligned, so lateral clearance is what "width" means here
    # (a perpendicular section of a slightly diagonal segment would clip platform corners).
    lateral_dir = np.array([0.0, 1.0])
    xs = pts[:, None, :] + offs[None, :, None] * lateral_dir[None, None, :]  # (N, K, 2)
    H, inside = _lookup(hf_m, xs, hs)                                        # (N, K)
    N = H.shape[0]
    if N < 2:
        return f

    walkable = inside & (H > params.pit_thresh)
    center_pit = inside[:, k0] & ~walkable[:, k0]
    f["center_pit_fraction"] = float(center_pit.mean())

    # nearest walkable sample to the centre line, within the corridor
    corridor_cells = int(round(params.corridor_half_width / hs))
    lateral = np.abs(np.arange(K) - k0)
    cand = np.where(walkable, lateral[None, :], K + 1)
    k_near = cand.argmin(axis=1)
    near_dist = cand[np.arange(N), k_near]
    gap = near_dist > corridor_cells
    h_walk = np.where(gap, np.nan, H[np.arange(N), k_near])
    f["lateral_offset_max_m"] = float(np.where(gap, 0, near_dist).max() * hs)

    # --- gaps -------------------------------------------------------------
    runs = _runs(gap)
    f["n_gaps"] = float(len(runs))
    f["max_gap_width_m"] = 0.0
    f["gap_landing_dz_m"] = 0.0
    if runs:
        widths = [(e - s) * hs for s, e in runs]
        k = int(np.argmax(widths))
        s, e = runs[k]
        f["max_gap_width_m"] = float(widths[k])
        if s > 0 and e < N:
            f["gap_landing_dz_m"] = float(h_walk[e] - h_walk[s - 1])
        else:
            f["gap_landing_dz_m"] = np.nan

    walk_idx = np.nonzero(~gap)[0]
    if walk_idx.size == 0:
        f["max_height_m"] = float(np.nanmax(H)) if np.isfinite(H).any() else np.nan
        f["min_height_m"] = np.nan
        return f
    hw = h_walk[walk_idx]
    f["max_height_m"] = float(hw.max())
    f["min_height_m"] = float(hw.min())
    f["rise_m"] = float(hw[-1] - hw[0])

    # --- steps (gaps removed) --------------------------------------------
    if hw.size >= 2:
        dh = np.diff(hw)
        f["max_step_up_m"] = float(max(dh.max(), 0.0))
        f["max_step_down_m"] = float(max(-dh.min(), 0.0))
        f["max_step_m"] = float(np.abs(dh).max())
    else:
        f["max_step_up_m"] = f["max_step_down_m"] = f["max_step_m"] = 0.0

    # --- slope of the continuous surface ---------------------------------
    w = max(1, int(round(params.slope_window / hs)))
    f["mean_slope_deg"] = f["max_slope_deg"] = 0.0
    if N > w:
        dh1 = np.diff(h_walk)                       # NaN where either side is a gap
        big = ~(np.abs(dh1) <= params.step_thresh)  # True for big edges and NaN edges
        big_cum = np.concatenate([[0], np.cumsum(big)])
        i = np.arange(N - w)
        ok = big_cum[i + w] - big_cum[i] == 0        # edges i..i+w-1 all small & finite
        if ok.any():
            ii = i[ok]
            rise = h_walk[ii + w] - h_walk[ii]
            # distributed rise: largest single edge <= half the total rise (flat windows pass trivially)
            seg_max = np.max(np.abs(np.stack([dh1[ii + j] for j in range(w)], axis=1)), axis=1)
            distributed = seg_max <= 0.5 * np.abs(rise) + 1e-9
            if distributed.any():
                slopes = np.degrees(np.arctan2(np.abs(rise[distributed]), w * hs))
                f["mean_slope_deg"] = float(slopes.mean())
                f["max_slope_deg"] = float(slopes.max())

    # --- walkable width of the followed surface ---------------------------
    # The span is bounded by lateral discontinuities: a pit, the terrain edge, or a
    # height change > step_thresh between neighbouring lateral samples (cliff / wall).
    # Positions where the followed sample is not also walkable at the same lateral
    # index one position before and after are skipped (x-edges of obstacles).
    kn = k_near[walk_idx]
    prev_ok = np.array([walkable[i - 1, k] if i > 0 else True for i, k in zip(walk_idx, kn)])
    next_ok = np.array([walkable[i + 1, k] if i + 1 < N else True for i, k in zip(walk_idx, kn)])
    interior = prev_ok & next_ok
    if interior.any():
        rows = walk_idx[interior]
        kn = kn[interior]
        Hr = H[rows]
        wr = walkable[rows]
        edge_ok = wr[:, :-1] & wr[:, 1:] & (np.abs(np.diff(Hr, axis=1)) <= params.step_thresh)  # (M, K-1)
        bad = ~edge_ok
        j = np.arange(K - 1)[None, :]
        left_bad = np.where(bad & (j < kn[:, None]), j, -1).max(axis=1)      # edge j is between j and j+1
        right_bad = np.where(bad & (j >= kn[:, None]), j, K - 1).min(axis=1)
        span_cells = right_bad - left_bad                                     # (right_bad) - (left_bad + 1) + 1
        f["min_width_m"] = float(span_cells.min() * hs)
        f["median_width_m"] = float(np.median(span_cells) * hs)
    return f


def profile_terrain(hf_units: np.ndarray, goals_m: np.ndarray, cfg: BenchmarkTerrainCfg,
                    params: ProfileParams = ProfileParams()) -> List[Dict[str, float]]:
    """Features for every goal segment of one terrain. ``hf_units`` is int16 in vertical_scale units."""
    hf_m = hf_units.astype(np.float64) * cfg.vertical_scale
    wp = waypoints(goals_m, cfg)
    rows = []
    for k in range(goals_m.shape[0]):
        p0, p1 = wp[k], wp[k + 1]
        f = profile_segment(hf_m, p0, p1, cfg.horizontal_scale, params)
        f.update({"segment": k, "x0_m": p0[0], "y0_m": p0[1], "x1_m": p1[0], "y1_m": p1[1]})
        rows.append(f)
    return rows


def cfg_from_archive(z: Dict[str, np.ndarray]) -> BenchmarkTerrainCfg:
    return BenchmarkTerrainCfg(
        terrain_length=float(z["terrain_length_m"]), terrain_width=float(z["terrain_width_m"]),
        horizontal_scale=float(z["horizontal_scale"]), vertical_scale=float(z["vertical_scale"]),
        num_goals=int(z["num_goals"]), num_rows=int(z["num_rows"]), num_cols=int(z["num_cols"]),
        spawn_x=float(z["spawn_x"]), spawn_y=float(z["spawn_y"]),
    )


def profile_archive(z: Dict[str, np.ndarray], cfg: Optional[BenchmarkTerrainCfg] = None,
                    params: ProfileParams = ProfileParams()):
    """Profile every (task, level) in a frozen archive -> pandas.DataFrame."""
    import pandas as pd

    cfg = cfg or cfg_from_archive(z)
    names = [str(n) for n in z["task_names"]]
    records = []
    for ti, task in enumerate(names):
        for li in range(len(z["difficulties"])):
            for r in profile_terrain(z["height_fields"][ti, li], z["goals"][ti, li], cfg, params):
                r.update({
                    "task_idx": ti, "task": task, "level": li,
                    "difficulty": float(z["difficulties"][li]),
                    "scaled_difficulty": float(z["scaled_difficulties"][ti, li]),
                    "fix_desc": str(z["fix_descs"][ti, li]),
                })
                records.append(r)
    cols = ["task_idx", "task", "level", "difficulty", "scaled_difficulty", "segment",
            "x0_m", "y0_m", "x1_m", "y1_m"] + FEATURE_COLUMNS + ["fix_desc"]
    return pd.DataFrame.from_records(records)[cols]


def summarize_by_task(df):
    """Per-task summary: value at the easiest level, at the hardest level, and the extreme over all."""
    import pandas as pd

    lo, hi = df["level"].min(), df["level"].max()
    spec = {
        "max_step_m": "max", "max_gap_width_m": "max", "max_slope_deg": "max",
        "mean_slope_deg": "mean", "min_width_m": "min", "max_height_m": "max",
        "lateral_offset_max_m": "max",
    }
    out = []
    for ti, g in df.groupby("task_idx", sort=True):
        row = {"task_idx": ti, "task": g["task"].iloc[0]}
        for col, how in spec.items():
            for tag, lvl in (("lo", lo), ("hi", hi)):
                row[f"{col}@{tag}"] = getattr(g[g["level"] == lvl][col], how)()
            row[f"{col}@all"] = getattr(g[col], how)()
        row["n_fixed_levels"] = int((g.groupby("level")["fix_desc"].first() != "").sum())
        out.append(row)
    return pd.DataFrame(out)
