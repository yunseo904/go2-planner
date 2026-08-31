# Session state — 2026-08-31, calibration on TURN, TROT and the v2 families

**Read this first next session.** It records what was finished, what was left in the air,
and what to do first. Everything below is committed.

All runs this session were CPU (`--device cpu`), no GPU at any point. The WMP training
containers were never touched.

---

## Done

| # | Asked | Result | Written to |
|---|---|---|---|
| 1 | TURN probe limits, criterion first | Criterion designed, flat control run, all 5 families swept. **TURN's limit is below every ladder's floor** — a 0.02 m edge under the footprint stops a 90° turn. | `outputs/turn_probes.md` |
| 2 | TROT straight line: 3 sub-questions | All three answered and measured. The step-length half is built (`--heading-len`, default off). **No condition fixes TROT.** | `outputs/trot_straight.md` |
| 3 | TROT probes, once it goes straight | **Still not measurable**, but the reason moved: cross-track offset against a point goal, not lane departure. | `trot_straight.md` §6 |
| 4 | v2 probes, slope + roughness, WALK/TURN | Archive frozen and run. **First brackets this project has produced**: WALK slope 4/5 at 5°, 0/5 at 10°; WALK roughness 5/5 at 0.005, 3/5 at 0.010, 0/5 at 0.015. | `outputs/v2_probes.md` |
| 5 | RUN/JUMP recorded as not measurable | Recorded, with two more that belong beside them. | `outputs/unmeasurable.md` |

Also: `outputs/harness_findings.md` §12 (the grid's cells are not replicates — TROT turns
0.3 mm of float rounding into 1 m) and §13 (`freeze_calibration.py --verify` **writes**).

## The one thing to be careful about carrying forward

`trot_straight.md` **§4 reports a large win and §4b withdraws it.** A constant step-length
feed-forward gave +72% survival and −43% curvature on one controlled cell, and paired across
all fifteen cells it is 7/15, 6/15, 5/15 — a coin flip, with the mean slightly the wrong way.
The cell §4 was run on turned out to be the best of the fifteen on every column.

**Do not quote §4 without §4b.** The mechanism and the DC-vs-P reasoning in §3 stand; the
effect size does not.

---

## Left unfinished

**One run was killed mid-flight, deliberately, for the shutdown.** `robot.FOOT_SPAN_X`: the
gap family re-run under heading hold. The 60 gap rows that scored 0 predate heading hold, and
WALK now holds a line at 0.24 °/m, so those rows may simply be stale. Nothing was written;
`outputs/gap_heading.csv` does not exist.

    # this is the first thing to run next session -- ~15 min, CPU
    NAME=g2_gap scripts/isaac_docker_run.sh scripts/run_calibration_grid.py \
      --headless --device cpu --params robot.FOOT_SPAN_X --reps 5 \
      --foot-comp on --heading heading-only --results-csv outputs/gap_heading.csv

It is the only `CALIBRATION_NEEDED` field with a plausible route to a number right now.

## What to do next, in order

1. **The gap re-run above.** Cheapest remaining threshold.
2. **Re-cut the v2 ladders — they start above the limits.** Both families were sized against
   the placeholders and the placeholders are where the error is. `slope` 1°–10° in 1° (it
   currently starts at 5° and WALK is already 4/5 there); `roughness` 0.001–0.010 m in 0.001
   (it starts at 0.005 and WALK is 5/5 there, 0/5 two rungs later). One edit to
   `terrain_toolkit/calibrate.py`, one re-freeze to `--out data/calibration_probes_v3.npz`,
   one run. Nothing else has to change.
3. **Decide whether TROT gets cross-track control.** `trot_straight.md` §5 is the argument:
   heading authority is spent three different ways, and what is left is a *position* error
   against a path, which only the planner has. That crosses a line the low level has kept —
   no goal reaches `FootPlacement` today — so it is a design decision, not a tuning one.
4. **`turn_probes.md` §6 is a planner constraint, not a threshold**: TURN must be issued on
   ground flat under the whole footprint, i.e. the switch to TURN happens *before* the
   obstacle, never on it. That needs to reach `planner/skills.py` eventually.

## Deliberately not done

- **Nothing was applied to `planner/config.py`.** Every placeholder is still
  `CALIBRATION_NEEDED`. The brackets in `v2_probes.md` §5 are evidence, not values; under the
  protocol's own margin rule both land at 0, which is a statement about the ladders.
- **`skill.STEP_TURN_MAX` was not invented.** The TURN rows are stamped
  `probe.<family>@TURN`. Name the parameter after the measurement produces a number.
- **`data/calibration_probes.npz` (the pinned 42) is untouched** — verified by hash. The v2
  archive is a separate file whose first 42 probes are byte-identical terrain, names and
  goals, so every earlier result stays comparable.
- The `--verify` trap in `harness_findings.md` §13 was **recorded, not renamed**: changing
  the flag mid-investigation would be changing the instrument.

## New flags and files this session

    sim/footcomp.py            heading_len, heading_len_cap_rad (default OFF; the self-test
                               asserts the hips are bit-identical without it)
    run_calibration_grid.py    --probes --families --skill --score {goal,inplace}
                               --inplace-at {obstacle,spawn} --inplace-yaw-deg
                               --heading-len --heading-len-cap
                               --foot-yaw-bias --foot-len-bias
    scripts/analyze_heading_ab.py       grid trace -> lane drift / curvature at the lip
    scripts/analyze_trot_heading.py     heading conditions, trimmed to before the fall
    scripts/reduce_turn_inplace.py      in-place sweep reduced against its own flat control

All three self-tests pass: `sim/footcomp.py`, `run_calibration_grid.py --self-test`,
`scripts/test_sim_contracts.py`, and `verify_skill_replay.py --self-test`.
