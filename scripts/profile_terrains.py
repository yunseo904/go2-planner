#!/usr/bin/env python
"""Profile every frozen benchmark terrain per goal segment.

Writes:
    outputs/terrain_profile.csv      one row per (task, level, segment)
    outputs/task_summary.csv         per-task extremes at easiest/hardest level
    outputs/feature_histograms.png   distribution of each feature
    outputs/feature_heatmaps.png     task x level heatmap per feature
    outputs/task_summary.png         summary table rendered as an image

Usage:
    python scripts/profile_terrains.py [--no-plots]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from terrain_toolkit import paths  # noqa: E402
from terrain_toolkit.freeze import load_archive  # noqa: E402
from terrain_toolkit.profile import profile_archive, summarize_by_task  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    z = load_archive()
    df = profile_archive(z)
    paths.OUTPUTS_DIR.mkdir(parents=True, exist_ok=True)
    df.to_csv(paths.PROFILE_CSV, index=False, float_format="%.4f")
    print(f"[profile] wrote {paths.PROFILE_CSV} ({len(df)} rows)")

    summary = summarize_by_task(df)
    summary.to_csv(paths.TASK_SUMMARY_CSV, index=False, float_format="%.3f")
    print(f"[profile] wrote {paths.TASK_SUMMARY_CSV}")

    if not args.no_plots:
        from terrain_toolkit.render import plot_feature_histograms, plot_feature_heatmaps, plot_task_summary_table
        p = plot_feature_histograms(df, paths.OUTPUTS_DIR / "feature_histograms.png")
        print(f"[profile] wrote {p}")
        p = plot_feature_heatmaps(df, paths.OUTPUTS_DIR / "feature_heatmaps.png")
        print(f"[profile] wrote {p}")
        p = plot_task_summary_table(summary, paths.OUTPUTS_DIR / "task_summary.png")
        print(f"[profile] wrote {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
