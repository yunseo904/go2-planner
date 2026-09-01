"""Which of the review's per-cell fields each existing results CSV can already fill.

    python3 scripts/csv_coverage.py outputs/bench_le/*.csv

Groups the files by column set and names what is missing from each, so a run does not have
to be repeated to find out that it cannot answer the question being asked of it.  The
aggregator reconstructs `fell` from `end_cause` and `episode_s` from `upright_s`; anything
listed as missing here is missing for real.
"""
import csv, glob, os, sys
from collections import defaultdict

#: field the review asks for -> the columns that can supply it, best first
WANT = {
    "course/level": ["task_name", "level"], "goals": ["goals"],
    "falls": ["fell", "end_cause"], "episode length": ["episode_s", "upright_s"],
    "finished": ["alive_at_end"], "final x/y": ["final_x_m", "final_y_m"],
    "dist next/last": ["dist_to_next_goal_m", "dist_to_last_goal_m"],
    "skill shares": ["frac_WALK"], "switches": ["switches"], "refusals": ["refused_ticks"],
    "travelled": ["travelled_m"], "vx": ["vx_mean_ms"], "vy": ["vy_mean_ms"],
    "yaw rate": ["yaw_rate_deg_s"], "curvature": ["curvature_rad_m"],
    "end cause": ["end_cause"], "cross-track": ["cross_track_m"],
    "terrain seed stamp": ["terrain_seed"], "episode-length stamp": ["episode_length_s"],
    "num_envs stamp": ["num_envs"], "steering stamp": ["steering"],
    "support stamp": ["cmd_mean_feet_down"],
}


def main():
    pats = sys.argv[1:] or ["outputs/bench_le/*.csv"]
    files = [p for pat in pats for p in sorted(glob.glob(pat))]
    groups = defaultdict(list)
    for p in files:
        try:
            hdr = next(csv.reader(open(p)))
        except Exception:
            continue
        groups[tuple(sorted(hdr))].append(os.path.basename(p))
    print(f"{len(files)} results CSVs, {len(groups)} distinct column sets\n")
    for i, (key, names) in enumerate(sorted(groups.items(), key=lambda kv: -len(kv[1])), 1):
        cols = set(key)
        miss = [k for k, v in WANT.items() if not any(c in cols for c in v)]
        print(f"--- set {i}: {len(names)} files, {len(cols)} columns")
        print(f"    e.g. {', '.join(sorted(names)[:5])}{' ...' if len(names) > 5 else ''}")
        print(f"    MISSING: {', '.join(miss) if miss else '(nothing)'}")


if __name__ == "__main__":
    main()
