# The three arms under the legged_eval protocol

Re-run of `benchmark_planner.md` with the terrain taken from `legged_eval` instead of
`data/benchmark_frozen.npz`.  Same 200 cells, same 20 s episode, same goal rule, same
equal-weight-over-cells mean, **same settings in all three arms** (`--heading heading-only
--start-phase first --foot-comp on`, planner additionally `--yaw-moment hold`).  CPU, no
GPU.  Terrain seed 1.

What changed under the robot: eurekaverse's uniform noise (amplitude drawn from
0.02–0.04 m, interpolated from a 0.075 m grid), its 0.1 m rim raised 0.5 m around every
patch, and a terrain draw that follows the run seed instead of upstream's.

## 1. The result

| arm | legged_eval | frozen (old) | Δ | median | max | zeros | alive at 20 s | travelled (med) |
|---|---|---|---|---|---|---|---|---|
| **Rule-Planner** | **0.57** | 1.07 | **−0.51** | 1.0 | 2 | 89/200 | 9/200 | 0.69 m |
| **WALK fixed** (lower bound) | **0.76** | 1.11 | **−0.36** | 1.0 | 2 | 60/200 | 29/200 | 0.79 m |
| TROT fixed | **0.29** | 0.97 | −0.68 | 0.0 | 2 | 143/200 | 1/200 | 0.69 m |
| *E2E teacher, legged_eval protocol* | *4.83 (seed 1) / 4.98 (seed 3)* | | | | | | | |

**Everything drops and the ordering is unchanged.**  The planner is still below its own
lower bound and by more than before: paired per cell it is now **better in 1, worse in 37,
tied in 162** (was 0 / 6 / 194).  The floor caveat still governs — goal 0 sits 0.50 m from
the spawn in 180 of 200 cells, and the median robot travels 0.7–0.8 m.

The E2E row is **not** the 4.48 that used to sit in this table.  See §4.

## 2. Which of the three changes did it — ablation, WALK, all 200 cells

| | rim off | rim on |
|---|---|---|
| **roughness off** | 1.09 | 1.09 |
| **roughness on** | 0.76 | **0.76** |

*(frozen archive, which is "neither" on a different draw: 1.11)*

Read it three ways and it says the same thing:

* **Roughness is the whole drop.**  −0.33 with the rim on and −0.33 with it off.
* **The border walls do nothing measurable.**  1.09 vs 1.09, 0.76 vs 0.76 — not a small
  effect, an *identical* one.
* **The different terrain draw does nothing measurable either.**  Our "neither" corner is
  1.09 against the frozen archive's 1.11, and the archive is a different draw of every
  course.  0.02 on a 200-cell mean is noise.

So of the three things that changed, one of them accounts for all of it.

## 3. Against the three predictions that were made in advance

| prediction | outcome |
|---|---|
| "roughness amplitude is up to 0.04 m and WALK's step limit is 0.04–0.06 m, so flat ground disappears and the score falls" | **Right, and it is the only thing that mattered.** −0.33 of a −0.35 total. |
| "10 of 20 courses are PIT with a 1 m drop and terminate immediately, so half the cells may be 0" | **Did not happen.**  WALK's PIT-course mean is 0.84 against 0.67 on the non-PIT courses — the pit courses score *higher*.  The robots die at a median 0.79 m and never reach a gap; the nearest one is 0.10 m wide and the median is metres away. The pits are not binding at this performance level. |
| "the border walls may help, since lane departure was the main failure" | **No effect at all**, to the digit, in both ablation rows.  For the same reason: a robot that stops after 0.79 m in a 4 m lane never reaches a wall. |

The two predictions that failed, failed for one shared reason, and it is worth stating on
its own: **at 0.7–0.8 m of travel, nothing beyond the first metre of the course is being
measured.**  Pits, walls and the terrain draw are all properties of ground the robot never
stands on.  Roughness is the only one of the three that is under the robot at the spawn.

## 4. The E2E teacher's 4.48 is not a legged_eval number

Asked: whether the 4.48 in this project's tables came from this protocol.  **It did not**,
on two counts:

* **Different tool.**  4.48 is from `ev-go2/docs/RUN_RESULTS.md`, the M6 depth-distillation
  report of 2026-07-10, produced by eurekaverse's own `evaluate.py`.  legged_eval's own
  results for the teacher are in `~/eval_out/teacher_lab_*.json` (`"schema":
  "legged_eval/1"`) and read **4.83** at seed 1 and **4.98** at seed 3.
* **Different checkpoint.**  4.48 is teacher `_10_3`.  The legged_eval-measured teacher is
  `2026-07-15_12-17-17_4_1`.  They are not the same policy.

The *scoring rule* does agree — `evaluate.py` also means over episodes within a cell and
then over cells with equal weight, which is what legged_eval reproduces — so the gap is
not the metric.  What differs is the terrain draw (upstream's is unseeded and depends on
how much RNG the host consumed while parsing its config; legged_eval's follows the run
seed) and the simulator/fleet the run was made in.

**So the number to quote beside our arms is 4.83–4.98, not 4.48**, and it should carry its
seed.  The planner's 0.57 is 12% of the teacher's 4.83.

## 5. What the planner is still doing

Share of upright time, planner arm: WALK 0.731 (200/200 cells), TURN 0.256 (174/200),
**TROT 0.014 (14/200)**.  Median 1 switch per cell, mean 1.7, max 8.  TURN is used in more
cells than before (174 vs 119) and takes a bigger share (0.256 vs 0.098) — the rougher
ground turns the heading over faster, so the planner re-aims more often, and the extra
detours are where the 37 losing cells are.

Unchanged conclusion, more sharply: **the arm is WALK plus TURN detours, and the detours
cost.**

## 6. Reproducing

```bash
scripts/isaac_docker_run.sh scripts/run_benchmark.py --headless --device cpu \
    --terrain legged_eval --terrain-seed 1 \
    --heading heading-only --start-phase first --foot-comp on --episodes 1 \
    --skill WALK --results-csv outputs/bench_le/walk.csv
#   ... --skill TROT ... ;  --skill PLANNER --yaw-moment hold ...
#   ablation: add --no-roughness and/or --no-border-walls
```

CSVs in `outputs/bench_le/`.  Every row carries `terrain=legged_eval-seed1`; the old ones
in `outputs/bench/` are the frozen archive and carry no such column, which is itself the
tell.  **Do not average across the two.**
