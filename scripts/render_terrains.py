#!/usr/bin/env python
"""Render the frozen benchmark height fields to PNG (CPU only).

Writes:
    outputs/terrains/<idx>_<task>.png        all 10 difficulty levels of one task
    outputs/terrains_overview_level<L>.png   all 20 tasks at one level (default: 0 and 9)

Usage:
    python scripts/render_terrains.py                 # everything
    python scripts/render_terrains.py --task 5 --task squeeze
    python scripts/render_terrains.py --overview-levels 0 4 9
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from terrain_toolkit import paths  # noqa: E402
from terrain_toolkit.freeze import load_archive  # noqa: E402
from terrain_toolkit.render import cfg_from_archive, render_overview, render_task  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", action="append", default=None, help="task index or name (repeatable); default all")
    ap.add_argument("--overview-levels", type=int, nargs="*", default=None,
                    help="levels for the overview pages (default: first and last)")
    ap.add_argument("--no-overview", action="store_true")
    args = ap.parse_args()

    z = load_archive()
    cfg = cfg_from_archive(z)
    names = [str(n) for n in z["task_names"]]

    if args.task:
        sel = []
        for t in args.task:
            sel.append(int(t) if t.isdigit() else names.index(t))
    else:
        sel = list(range(len(names)))

    for ti in sel:
        p = render_task(z, ti, paths.TERRAIN_RENDER_DIR / f"{ti:02d}_{names[ti]}.png", cfg)
        print(f"[render] {p}")

    if not args.no_overview:
        levels = args.overview_levels if args.overview_levels is not None else [0, len(z["difficulties"]) - 1]
        for lv in levels:
            p = render_overview(z, lv, paths.OUTPUTS_DIR / f"terrains_overview_level{lv}.png", cfg)
            print(f"[render] {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
