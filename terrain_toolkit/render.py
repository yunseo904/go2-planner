"""Matplotlib (Agg) rendering of frozen terrains and profile statistics.

Colour usage follows a small fixed palette:
  * height fields: diverging map, blue = below ground (pits), neutral = 0, orange = above.
  * feature heatmaps: single-hue sequential (light -> dark).
  * histograms / paths: one categorical hue; text in ink colours, never series colours.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Iterable, Optional, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm  # noqa: E402

from .profile import FEATURE_COLUMNS, cfg_from_archive, waypoints  # noqa: E402,F401
from .stub import BenchmarkTerrainCfg  # noqa: E402

# Palette (light surface). Series-1 blue / series-2 orange from the reference palette.
INK = "#0b0b0b"
INK_2 = "#52514e"
INK_MUTED = "#8a8985"
SURFACE = "#fcfcfb"
GRID = "#e6e4df"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
NEUTRAL = "#ece9e3"

HEIGHT_CMAP = LinearSegmentedColormap.from_list("height_div", [(0.0, "#123f80"), (0.5, NEUTRAL), (1.0, "#8a2e0e")])
SEQ_CMAP = LinearSegmentedColormap.from_list("seq_blue", ["#eef4fc", "#9cc3ee", "#2a78d6", "#0f3f7a"])

HIST_FEATURES = ["max_step_up_m", "max_step_down_m", "max_gap_width_m", "mean_slope_deg", "max_slope_deg",
                 "min_width_m", "lateral_offset_max_m", "rise_m", "max_height_m"]
HEATMAP_FEATURES = [
    ("max_step_m", "max", "max step [m]"),
    ("max_gap_width_m", "max", "max gap width [m]"),
    ("max_slope_deg", "max", "max slope [deg]"),
    ("mean_slope_deg", "mean", "mean slope [deg]"),
    ("min_width_m", "min", "min width [m]"),
    ("max_height_m", "max", "max height [m]"),
    ("lateral_offset_max_m", "max", "lateral offset of surface [m]"),
    ("max_step_up_m", "max", "max step up [m]"),
    ("max_step_down_m", "max", "max step down [m]"),
]


def _style(ax):
    ax.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK_2, labelsize=8)
    ax.xaxis.label.set_color(INK_2)
    ax.yaxis.label.set_color(INK_2)
    ax.title.set_color(INK)


def draw_terrain(ax, hf_units: np.ndarray, goals_m: np.ndarray, cfg: BenchmarkTerrainCfg,
                 vmax: Optional[float] = None, show_route: bool = True, title: str = ""):
    hf_m = hf_units.astype(np.float64) * cfg.vertical_scale
    vmin = min(-1.0, float(hf_m.min()))
    vmax = max(0.5, float(hf_m.max())) if vmax is None else vmax
    norm = TwoSlopeNorm(vmin=vmin, vcenter=0.0, vmax=vmax)
    extent = (0, cfg.terrain_length, 0, cfg.terrain_width)
    im = ax.imshow(hf_m.T, origin="lower", cmap=HEIGHT_CMAP, norm=norm, extent=extent,
                   aspect="equal", interpolation="nearest")
    if show_route:
        wp = waypoints(goals_m, cfg)
        ax.plot(wp[:, 0], wp[:, 1], "-", color="white", lw=2.6, solid_capstyle="round", zorder=3)
        ax.plot(wp[:, 0], wp[:, 1], "-", color=INK, lw=1.2, zorder=4)
        ax.scatter(goals_m[:, 0], goals_m[:, 1], s=26, facecolor=SURFACE, edgecolor=INK, lw=1.0, zorder=5)
        ax.scatter([cfg.spawn_x], [cfg.spawn_y], s=40, marker="^", facecolor=ORANGE, edgecolor="white", lw=0.8, zorder=6)
        for k, (gx, gy) in enumerate(goals_m):
            ax.annotate(str(k), (gx, gy), xytext=(0, 5), textcoords="offset points", ha="center",
                        fontsize=6, color=INK, zorder=7)
    ax.set_xlim(0, cfg.terrain_length)
    ax.set_ylim(0, cfg.terrain_width)
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color(GRID)
    if title:
        ax.set_title(title, fontsize=8, color=INK, loc="left", pad=3)
    return im


def render_task(z: Dict[str, np.ndarray], task_idx: int, out_path: Path, cfg: Optional[BenchmarkTerrainCfg] = None) -> Path:
    """One PNG per task: every difficulty level stacked vertically, shared colour scale."""
    cfg = cfg or cfg_from_archive(z)
    task = str(z["task_names"][task_idx])
    levels = len(z["difficulties"])
    hf_all = z["height_fields"][task_idx]
    vmax = max(0.5, float(hf_all.max()) * cfg.vertical_scale)
    # each panel keeps the true 18:4 aspect; size the figure so panels fill the width
    panel_w = 9.0
    panel_h = panel_w * cfg.terrain_width / cfg.terrain_length
    fig = plt.figure(figsize=(panel_w + 1.0, (panel_h + 0.32) * levels + 0.6))
    fig.patch.set_facecolor(SURFACE)
    gs = fig.add_gridspec(levels, 2, width_ratios=[panel_w, 0.18], wspace=0.04, hspace=0.45,
                          left=0.02, right=0.93, top=0.96, bottom=0.01)
    axes = [fig.add_subplot(gs[li, 0]) for li in range(levels)]
    for li, ax in enumerate(axes):
        d = float(z["difficulties"][li])
        sd = float(z["scaled_difficulties"][task_idx, li])
        fix = str(z["fix_descs"][task_idx, li])
        if len(fix) > 70:
            fix = fix[:67] + "..."
        title = f"level {li}  difficulty {d:.2f}  (scaled {sd:.2f})" + (f"  ·  fixed: {fix}" if fix else "")
        im = draw_terrain(ax, hf_all[li], z["goals"][task_idx, li], cfg, vmax=vmax, title=title)
    cax = fig.add_subplot(gs[levels // 4: levels - levels // 4, 1])
    cb = fig.colorbar(im, cax=cax)
    cb.set_label("height [m]", color=INK_2, fontsize=8)
    cb.ax.tick_params(colors=INK_2, labelsize=7)
    fig.suptitle(f"{task_idx:02d}  {task}", x=0.02, y=0.985, ha="left", fontsize=11, color=INK)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    return out_path


def render_overview(z: Dict[str, np.ndarray], level: int, out_path: Path, cfg: Optional[BenchmarkTerrainCfg] = None,
                    cols: int = 2) -> Path:
    """All tasks at one difficulty level on a single page."""
    cfg = cfg or cfg_from_archive(z)
    names = [str(n) for n in z["task_names"]]
    n = len(names)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(6.2 * cols, 1.55 * rows + 0.6), constrained_layout=True)
    fig.patch.set_facecolor(SURFACE)
    axes = np.atleast_2d(axes)
    for ti in range(rows * cols):
        ax = axes[ti // cols][ti % cols]
        if ti >= n:
            ax.axis("off")
            continue
        hf = z["height_fields"][ti, level]
        draw_terrain(ax, hf, z["goals"][ti, level], cfg, title=f"{ti:02d} {names[ti]}")
    fig.suptitle(f"benchmark terrains · level {level} (difficulty {float(z['difficulties'][level]):.2f})",
                 x=0.01, ha="left", fontsize=11, color=INK)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    return out_path


def plot_feature_histograms(df, out_path: Path, features: Sequence[str] = HIST_FEATURES) -> Path:
    n = len(features)
    cols = 3
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 3.0 * rows), constrained_layout=True)
    fig.patch.set_facecolor(SURFACE)
    for ax, feat in zip(np.ravel(axes), features):
        vals = df[feat].dropna().to_numpy()
        ax.hist(vals, bins=30, color=BLUE, edgecolor=SURFACE, linewidth=0.6)
        ax.set_title(feat, fontsize=9, loc="left")
        ax.set_ylabel("segments", fontsize=8)
        ax.grid(axis="y", color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        _style(ax)
        med = float(np.median(vals)) if vals.size else np.nan
        ax.axvline(med, color=INK_2, lw=1, ls="--")
        ax.annotate(f"median {med:.2f}", (med, ax.get_ylim()[1]), xytext=(3, -10), textcoords="offset points",
                    fontsize=7, color=INK_2)
    for ax in np.ravel(axes)[n:]:
        ax.axis("off")
    fig.suptitle(f"feature distributions over {len(df)} goal segments (20 tasks × 10 levels × 8 segments)",
                 x=0.01, ha="left", fontsize=10, color=INK)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120, facecolor=SURFACE)
    plt.close(fig)
    return out_path


def plot_feature_heatmaps(df, out_path: Path, features=HEATMAP_FEATURES) -> Path:
    tasks = df.groupby("task_idx")["task"].first()
    levels = sorted(df["level"].unique())
    n = len(features)
    cols = 3
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5.0 * cols, 0.32 * len(tasks) * rows + 1.2), constrained_layout=True)
    fig.patch.set_facecolor(SURFACE)
    for ax, (feat, how, label) in zip(np.ravel(axes), features):
        piv = df.pivot_table(index="task_idx", columns="level", values=feat, aggfunc=how).reindex(index=tasks.index, columns=levels)
        data = piv.to_numpy(dtype=float)
        cmap = SEQ_CMAP.reversed() if feat == "min_width_m" else SEQ_CMAP  # narrow = dark
        im = ax.imshow(data, cmap=cmap, aspect="auto", interpolation="nearest")
        ax.set_xticks(range(len(levels)))
        ax.set_xticklabels(levels, fontsize=7)
        ax.set_yticks(range(len(tasks)))
        ax.set_yticklabels([f"{i:02d} {t}" for i, t in tasks.items()], fontsize=7)
        ax.set_xlabel("difficulty level", fontsize=8)
        ax.set_title(label + f"  ({how} over segments)", fontsize=9, loc="left")
        for s in ax.spines.values():
            s.set_visible(False)
        ax.tick_params(colors=INK_2, length=0)
        # direct labels for every cell (values are the point of the table)
        finite = data[np.isfinite(data)]
        thr = np.nanpercentile(finite, 60) if finite.size else 0
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                v = data[i, j]
                if np.isfinite(v):
                    dark = (v < thr) if feat == "min_width_m" else (v > thr)
                    ax.text(j, i, f"{v:.2f}" if abs(v) < 10 else f"{v:.0f}", ha="center", va="center", fontsize=5.5,
                            color="white" if dark else INK)
        cb = fig.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
        cb.ax.tick_params(colors=INK_2, labelsize=7)
    for ax in np.ravel(axes)[n:]:
        ax.axis("off")
    fig.suptitle("terrain features by task and difficulty level", x=0.01, ha="left", fontsize=10, color=INK)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=130, facecolor=SURFACE)
    plt.close(fig)
    return out_path


def plot_task_summary_table(summary, out_path: Path) -> Path:
    cols = [
        ("max_step_m", "max step [m]"),
        ("max_gap_width_m", "max gap [m]"),
        ("max_slope_deg", "max slope [°]"),
        ("mean_slope_deg", "mean slope [°]"),
        ("min_width_m", "min width [m]"),
        ("max_height_m", "max height [m]"),
    ]
    header = ["#", "task"] + [f"{lbl}\nlvl0 → lvl9" for _, lbl in cols] + ["fixed\nlevels"]
    cells = []
    for _, r in summary.iterrows():
        row = [f"{int(r['task_idx']):02d}", r["task"]]
        for c, _ in cols:
            row.append(f"{r[f'{c}@lo']:.2f} → {r[f'{c}@hi']:.2f}")
        row.append(str(int(r["n_fixed_levels"])))
        cells.append(row)
    fig, ax = plt.subplots(figsize=(15, 0.36 * len(cells) + 1.4))
    fig.patch.set_facecolor(SURFACE)
    ax.axis("off")
    tbl = ax.table(cellText=cells, colLabels=header, loc="center", cellLoc="center",
                   colWidths=[0.035, 0.24] + [0.115] * len(cols) + [0.06])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8)
    tbl.scale(1.0, 1.55)
    for (i, j), cell in tbl.get_celld().items():
        cell.set_edgecolor(GRID)
        cell.set_facecolor(SURFACE if i % 2 else "#f3f1ec")
        cell.get_text().set_color(INK)
        if i == 0:
            cell.set_facecolor("#e9e6df")
            cell.get_text().set_fontweight("bold")
            cell.set_height(cell.get_height() * 1.6)
        elif j == 1:
            cell.get_text().set_ha("left")
            cell._loc = "left"
    ax.set_title("per-task terrain summary  (extreme over the 8 goal segments; easiest level → hardest level)",
                 fontsize=10, color=INK, loc="left")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=130, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)
    return out_path
