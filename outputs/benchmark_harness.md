# The benchmark scorer: the parts that did not exist

`data/benchmark_frozen.npz` had never been simulated. Every script that opened it read it
with numpy — the offline planner sweep, the planner's feature extractor, the freezer
itself. `scripts/run_benchmark.py` is the path from the archive into Isaac Lab, and it
scores the result the way `eurekaverse`'s `evaluate.py` does, so that a number here and
the 4.48 in `docs/RUN_RESULTS.md` are the same quantity.

Everything reusable was reused: the mesh conversion (`sim/heightfield.to_trimesh`), the
one-mesh/one-robot-per-cell grid, the settle, and the foot-placement law all come from
`run_calibration_grid.py`. What follows is what had to be new.

---

## 1. The goal rule, read off upstream and not approximated

`legged_robot._update_goals` and `check_termination`:

| | value | upstream name |
|---|---|---|
| goal radius | 0.20 m | `env.next_goal_threshold` |
| dwell before it counts | 0.10 s | `env.reach_goal_delay` |
| goals per cell, consumed **in order** | 8 | `terrain.num_goals` |
| episode length | 20 s | `env.episode_length_s` |
| terminate on attitude | \|roll\| or \|pitch\| > 1.5 rad | `check_termination` |
| terminate on height | base z < −0.25 m | `check_termination` |
| all 8 goals reached | ends the episode as a **timeout**, not a termination | `reset_time_out \|= reach_goal_cutoff` |

An episode's score is the goal index it ended on, 0–8.

**Forward progress in x is not the rule.** The smoke that preceded this harness scored a
robot 9.13 m outside a 3.95 m lane as having passed 5 of 8 goals on an x-progress proxy.
`self_test` asserts that case directly: a robot parked past every goal in x but 9.13 m
aside scores **0**. It also asserts that standing on goal 3 without having taken goal 0
scores 0, and that the index cannot run past 8.

## 2. The aggregation: cells are the unit, not robots and not episodes

`evaluate.py` accumulates per env, divides by that env's episode count, then averages over
**cells** — 20 tasks × 10 levels, each weighted equally, via `aggregate_cells(...,
granularity="overall")`. A cell that produces ten episodes does not outvote a cell that
produces one. `aggregate()` does that and the self-test pins it: three cells scoring 8, 0
and 4 give **4.00** whether the 8 came from one episode or ten.

## 3. Two guards, both from things that have already gone wrong

**The infinite ground plane.** `TerrainImporterCfg(terrain_type="plane")` lays a
2000 km plane at z = 0 *under* the imported mesh. On the calibration probes that floored
every pit and produced 60/60 passes for a robot walking on an invisible floor. On the
benchmark it is worse: **11 of the 20 tasks carry −1.0 m pits over 16–48% of their cell
area**. The plane is deleted and the deletion is **verified** — the run raises rather than
starting with it present. Measured on one cell before this harness existed: with the plane
kept, `balance_beam` level 0 "survived" 20 s and covered 6.07 m in x while travelling 9.13 m
in y, outside a 3.95 m lane, on the invisible floor; with it removed the same run falls at
2.30 s.

**The denominator.** A mean over a subset of the grid is not the benchmark score. Upstream
has this trap live: `evaluate.py:187` clamps `num_envs` up to the 200 cells and then
`helpers.py:91` overwrites it from `--num_envs`, so asking for fewer envs than cells
silently shrinks the denominator and the printed mean still looks like a score. Here a run
with fewer than `tasks × levels` cells is **refused** unless `--allow-partial`, and a
partial run stamps `partial=1` on every row and prints "PARTIAL RUN — this mean is NOT the
benchmark score" above the number.

## 4. A third guard the first full run made necessary

The 200-cell mesh as one collider is 5.76 M vertices and 11.34 M triangles. PhysX cooked it
without complaint and collided with nothing: every cell scored 0 on control step 1 because
the robots had fallen through during the settle. `settle_ok` — the hip-to-foot lever test
inherited from the calibration grid — does not catch this, because the lever is perfectly
valid in free fall.

Two changes: the terrain is imported as **one mesh per task** (20 colliders of ~567 k
triangles, identical geometry, different ownership), and the harness now refuses to score
if any robot sits under 0.15 m above its own cell's ground after the settle. Recorded in
`harness_findings.md` §15, and worth repeating here: **a benchmark score of 0.00 on all 200
cells is exactly what a single-skill lower bound might legitimately produce.** It would have
been believed.

## 5. What it does not do

It drives **one clip**, named on the command line — the single-skill lower bound CLAUDE.md
§2 requires ("Single-skill 하한선은 필수다. 없으면 Rule-Planner 점수가 해석되지 않는다").
The rule planner's own choice is not wired in; `run_planner_replay.py` owns that and it
currently runs on a scripted terrain rather than a height field. Depth is not wired in
either — nothing in this project renders a camera yet, and `planner/features.py` works on a
*model* of the camera applied to the frozen height field.

Three arms need one scorer. This is the scorer.

## 6. Usage

    # the lower bound, all 200 cells
    scripts/isaac_docker_run.sh scripts/run_benchmark.py --headless --device cpu \
        --skill TROT --episodes 3 --results-csv outputs/benchmark_single_TROT.csv

    # one task, wiring only -- refuses without --allow-partial, stamps partial=1
    scripts/isaac_docker_run.sh scripts/run_benchmark.py --headless --device cpu \
        --skill TROT --tasks 0 --allow-partial

    # no Isaac Lab needed
    python3 scripts/run_benchmark.py --plan
    python3 scripts/run_benchmark.py --self-test

CPU throughout; nothing here renders, so nothing here needs a GPU (CLAUDE.md §8).

---

## 7. The first benchmark number this project has produced

`--skill TROT --episodes 3`, all 200 cells, 600 episodes, 161 s of wall clock on CPU with
no GPU.

**Single-skill TROT: 0.98 goals of 8. Single-skill WALK: 1.11.**

| | benchmark score |
|---|---|
| **this project, single-skill TROT (the lower bound)** | **0.98** |
| **this project, single-skill WALK** | **1.11** |
| upstream `walk_pretrain` baseline (`docs/RUN_RESULTS.md`) | 1.05 |
| upstream E2E teacher `_10_3` | 4.48 |
| upstream depth student `_10_3_distill` | 3.72 |

TROT per level: 1.00, 0.90, 0.95, 1.05, 0.95, 1.00, 1.00, 1.00, 0.95, 1.00 — **flat across
all ten difficulties**, range 0.00 (`staircase_spiral`) to 2.00
(`stepping_stones_randomly_arranged`), with 15 of 20 tasks at exactly 1.00.

WALK per level: 1.30, 1.25, 1.10, 1.15, 1.10, 1.05, 1.10, 1.05, 1.00, 1.05 — **a shallow
but monotone gradient**, which TROT does not have. WALK's best tasks are `squeeze` 2.50,
`box_jump_even` and `stepping_stones_randomly_arranged` 2.00, `balance_beam` 1.40; its
worst are `staircase_spiral` and `staircase_walking_full_width`, both **0.00**.

So the slower gait scores higher, and it is the only one of the two whose score responds to
difficulty at all — the first evidence in this project that the benchmark can distinguish
anything the rule planner might choose between.

### Read this before quoting it: 1.0 is the floor, not a score

**Goal 0 sits 0.50 m from the spawn in 180 of the 200 cells** (range 0.50–2.69 m, median
0.50). Goal 1 is at a median 1.59 m. So a score of 1.0 means *the robot moved half a metre
and then stopped mattering*, and any policy that walks at all collects it.

That is what makes the comparison legible rather than what undermines it: upstream's own
untrained baseline scores **1.05** — the same floor. Our fixed-TROT lower bound is
statistically indistinguishable from a policy that has not been trained, which is exactly
what a lower bound should look like, and it means the useful dynamic range of this
benchmark runs from about 1 to about 4.5 rather than from 0 to 8.

TROT's flatness across difficulty says the same thing from the other side: **it dies before
it reaches anything the difficulty parameter controls.** No cell's obstacle is being tested;
the score is a measurement of how far the gait gets on the flat run-up. WALK's 1.30 → 1.05
gradient means WALK is at least occasionally reaching something that gets harder.

### Conditions, so this is reproducible and comparable

`--foot-comp on --foot-clip-rad 0.05`, `--start-phase first`, no heading hold. Heading hold
is a `run_calibration_grid.py` option that has **not** been wired into this harness, so this
number is the bare Raibert lateral law and is weaker than the configured condition the
calibration sweeps use. That is a knob, not a defect, but a later number run with heading
hold will not be comparable to this one and should say so.

