#!/usr/bin/env python
"""Profile every session in the read-only curated Go2 log set.

Writes:
    outputs/skill_profile.csv      one row per session (36), ~100 columns
    outputs/jump_profile.csv       one row per four-leg-sync session (8)
    outputs/skill_profile.md       per-skill roll-up + method + cross-check
    outputs/skill_transition.md    skill_send -> skill_done / -> motion / settle

Usage:
    python scripts/profile_skills.py [--curated DIR]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motion_toolkit import report  # noqa: E402
from motion_toolkit.jump import jump_table  # noqa: E402
from motion_toolkit.profile import profile_all  # noqa: E402
from motion_toolkit.session import iter_sessions  # noqa: E402
from motion_toolkit.transitions import (  # noqa: E402
    predecessor_table,
    summarize,
    transition_table,
)
from terrain_toolkit import paths  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--curated", type=Path, default=None, help="curated log root (default: $GO2_CURATED_ROOT or ../curated)")
    args = ap.parse_args()

    root = args.curated.expanduser().resolve() if args.curated else paths.require_curated()
    sessions = iter_sessions(root)
    if not sessions:
        print(f"[skills] no sessions under {root}", file=sys.stderr)
        return 1
    print(f"[skills] {len(sessions)} sessions from {root}")

    paths.OUTPUTS_DIR.mkdir(parents=True, exist_ok=True)

    df = profile_all(sessions)
    df.to_csv(paths.SKILL_PROFILE_CSV, index=False, float_format="%.6g")
    print(f"[skills] wrote {paths.SKILL_PROFILE_CSV} ({len(df)} rows x {len(df.columns)} cols)")

    jumps = [s for s in sessions if s.group.startswith("06_")]
    jdf = jump_table(jumps)
    jdf.to_csv(paths.JUMP_PROFILE_CSV, index=False, float_format="%.6g")
    print(f"[skills] wrote {paths.JUMP_PROFILE_CSV} ({len(jdf)} rows)")

    tdf = transition_table(sessions)
    summary = summarize(tdf)
    preds = predecessor_table(tdf)

    paths.SKILL_PROFILE_MD.write_text(
        report.skill_profile_md(df, jdf, paths.SKILL_PROFILE_CSV.name, str(root)), encoding="utf-8"
    )
    print(f"[skills] wrote {paths.SKILL_PROFILE_MD}")

    paths.SKILL_TRANSITION_MD.write_text(
        report.transition_md(tdf, summary, preds, df), encoding="utf-8"
    )
    print(f"[skills] wrote {paths.SKILL_TRANSITION_MD} ({len(tdf)} skill_send events)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
