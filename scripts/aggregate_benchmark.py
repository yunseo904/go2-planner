"""Turn one (or several) run_benchmark.py results CSV into the review's markdown tables.

    python3 scripts/aggregate_benchmark.py outputs/bench_le/h2r_rough_on.csv
    python3 scripts/aggregate_benchmark.py outputs/bench_le/*.csv --split-by roll_couple

CPU only, no Isaac, no re-running: it reads what a run already wrote.  Three tables --
overall, by difficulty, by course -- with the run conditions as a footnote above them, read
off the rows themselves so a file can be aggregated long after its log has scrolled away.

TWO THINGS IT REFUSES TO DO
    * pool rows whose `terrain` differs.  A legged_eval score and a frozen score are not
      two readings of one benchmark (CLAUDE.md 2), and averaging them is the single
      easiest way to publish a number that means nothing.  Use --split-by terrain.
    * call a partial run a score.  Any row with partial=1 makes the header say so.

FALLS PER MINUTE is falls divided by the summed upright time, not by wall clock and not by
episodes x 20 s: a robot that fell at 3 s did not contribute 20 s of exposure.  Older CSVs
have no `fell` or `episode_s` column; both are reconstructed from `end_cause` and
`upright_s`, and the header says when that happened.
"""
import argparse, csv, glob, statistics as st
from collections import defaultdict

FALL_CAUSES = ("roll", "pitch", "below")
# Columns that describe the RUN rather than the cell.  Printed as the footnote; any that
# varies inside one table is printed as "mixed(...)", which is the warning.
COND = ("terrain", "terrain_seed", "measurement_seed", "skill", "perception", "steering",
        "heading", "heading_cap_rad", "episode_length_s", "episodes", "num_envs", "rate",
        "start_phase", "foot_comp", "foot_clip_rad", "spawn_z", "settle_s", "gutter",
        "roll_couple", "roll_gain", "roll_damp", "roll_cap_nm", "roll_sign",
        "yaw_moment", "yaw_moment_gain", "swing_lift_mm", "turn_target", "foot_yaw_turn",
        "planner_set", "cmd_mean_feet_down", "cmd_frac_below_3_feet", "partial")


def num(r, key, default=0.0):
    v = r.get(key, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def fell(r):
    if r.get("fell", "") != "":
        return int(float(r["fell"]))
    return int(r.get("end_cause", "") in FALL_CAUSES)      # older CSVs


def episode_s(r):
    for k in ("episode_s", "upright_s"):
        if r.get(k, "") != "":
            return num(r, k)
    return 0.0


def block(rows):
    """The numbers every table row is made of."""
    n = len(rows)
    goals = [num(r, "goals") for r in rows]
    up = [episode_s(r) for r in rows]
    falls = sum(fell(r) for r in rows)
    finished = sum(1 for r in rows if str(r.get("alive_at_end", "")) in ("1", "1.0"))
    tot_min = sum(up) / 60.0
    return dict(n=n, score=sum(goals) / n if n else 0.0,
                falls=falls, fpm=(falls / tot_min if tot_min else float("nan")),
                ep_len=st.mean(up) if up else 0.0, sim_s=sum(up),
                finished=finished, finished_pct=100.0 * finished / n if n else 0.0)


def footnote(rows, missing):
    vals = {}
    for c in COND:
        seen = sorted({r.get(c, "") for r in rows if r.get(c, "") != ""})
        if not seen:
            continue
        vals[c] = seen[0] if len(seen) == 1 else f"mixed({'/'.join(seen[:4])})"
    line = ", ".join(f"`{k}`={v}" for k, v in vals.items())
    out = [f"> **Conditions.** {line}"]
    if any(str(r.get("partial", "0")) not in ("0", "0.0", "") for r in rows):
        out.append("> **PARTIAL RUN — the mean is a wiring check, not a benchmark score.**")
    if missing:
        out.append(f"> Reconstructed for older rows: {', '.join(sorted(missing))}.")
    terr = {r.get("terrain", "") for r in rows}
    if len(terr) > 1:
        out.append(f"> **REFUSED TO POOL: {len(terr)} terrains in one table — "
                   f"{sorted(terr)}. Re-run with `--split-by terrain`.**")
    return "\n".join(out)


def table(title, groups, keycol):
    hdr = (f"| {keycol} | goals /8 | falls/min | mean episode s | episodes | finished % |\n"
           f"|---|---:|---:|---:|---:|---:|")
    lines = [f"### {title}", "", hdr]
    for key, b in groups:
        lines.append(f"| {key} | {b['score']:.3f} | {b['fpm']:.2f} | {b['ep_len']:.2f} | "
                     f"{b['n']} | {b['finished_pct']:.1f} |")
    return "\n".join(lines)


def emit(rows, label):
    missing = set()
    if not any(r.get("fell", "") != "" for r in rows):
        missing.add("`fell` (from `end_cause`)")
    if not any(r.get("episode_s", "") != "" for r in rows):
        missing.add("`episode_s` (from `upright_s`)")
    print(f"\n## {label}\n")
    print(footnote(rows, missing))
    print()
    b = block(rows)
    print(table("1. Overall", [("all cells", b)], "arm"))
    by_l = defaultdict(list)
    for r in rows:
        by_l[int(num(r, "level"))].append(r)
    print()
    print(table("2. By difficulty", [(f"level {l}", block(by_l[l]))
                                     for l in sorted(by_l)], "difficulty"))
    by_c = defaultdict(list)
    for r in rows:
        by_c[r.get("task_name", "?")].append(r)
    print()
    print(table("3. By course", [(c, block(by_c[c]))
                                 for c in sorted(by_c, key=lambda c: -block(by_c[c])["score"])],
                "course"))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="results CSVs; globs are expanded")
    ap.add_argument("--split-by", default=None, metavar="COLUMN",
                    help="emit one set of tables per distinct value of this column "
                         "(e.g. terrain, roll_couple, skill)")
    args = ap.parse_args()

    rows, srcs = [], []
    for pat in args.csv:
        for p in sorted(glob.glob(pat)) or [pat]:
            got = list(csv.DictReader(open(p)))
            for r in got:
                r["_src"] = p
            rows += got
            srcs.append(f"{p} ({len(got)} rows)")
    print("# Benchmark aggregate\n")
    print("Sources: " + "; ".join(srcs))

    if args.split_by:
        groups = defaultdict(list)
        for r in rows:
            groups[r.get(args.split_by, "")].append(r)
        for key in sorted(groups):
            emit(groups[key], f"{args.split_by} = {key or '(blank)'}")
    else:
        emit(rows, "all rows")


if __name__ == "__main__":
    main()
