# Session state — 2026-09-01 (3), the benchmark moved to legged_eval and the roughness took a third of the score

**Read this first next session.**  Everything below is committed.  Sections 1–4 are CPU
only; section 5 used GPU 1 (which was free — the WMP training had been stopped and its
watchdog paused at 20:28, see §7).

---

## The headline

| | before | after |
|---|---|---|
| evaluation terrain | `data/benchmark_frozen.npz` — no roughness, no rim, upstream's seed | **`legged_eval`**, generated per run, with eurekaverse's noise and rim, seeded off the run seed |
| how it attaches | — | `sim/legged_eval_terrain.py`, which copies **none** of legged_eval's logic: it hands `build_terrain_class()` a 30-line host that collects |
| Rule-Planner / WALK / TROT | 1.07 / 1.11 / 0.97 | **0.57 / 0.76 / 0.29** |
| what caused the drop | — | **the roughness, all of it.**  The rim is worth 0.00 and the different terrain draw 0.02 |
| the E2E number to compare against | 4.48 | **4.83 (seed 1) / 4.98 (seed 3)** — 4.48 was never a legged_eval number |
| JUMP's `effort_limit` | "the spec sheet", read as a rating | **the published *peak*.**  Unitree publishes no rated torque for these joints |

---

## 1. The switch — `sim/legged_eval_terrain.py`

`legged_eval` (`~/legged_eval`, someone else's, **read only**) is now the definition of the
benchmark: terrain, seeding, roughness, rim, spawn, episode rules, aggregation.  The
attachment is one file and it deliberately contains no terrain logic.
`legged_eval.terrain.build_terrain_class()` wants a legged_gym `Terrain` to subclass and
asks that base for a cfg, a patch size in pixels, and an `add_terrain_to_map`;
`_CollectingHost` is exactly that.  Course choice, difficulty scaling, the crc32 seeding,
`random_uniform_terrain`, the rim, the goals and the spawn all run inside legged_eval.

Re-implementing its epilogue here would have been the third copy of a benchmark that
exists to stop there being a second.

Three protocol details the archive had been hiding, now fixed:

* **Spawn height is measured from the patch datum z = 0**, not from the ground under the
  robot.  eurekaverse resets to `base_init_state + env_origin` with `origin_zero_z`;
  sampling the terrain was equivalent while the spawn strip was exactly flat and is not
  once roughness moves it by ±0.025 m.
* **The pit cutoff uses that same datum.**  It used to subtract a per-cell ground sample,
  which meant a robot walking along the floor of a 1 m pit could never trip `z < −0.25`.
* The pit-count banner tests `< PIT_DEPTH`, not `< 0`, which with roughness on had started
  calling all 20 tasks pitted.

`--terrain frozen` still reaches the archive, and every results row carries a `terrain`
column.  **A frozen score and a legged_eval score may not be averaged**: of 200 cells only
20 are the same terrain, and eight courses differ by over a metre in peak height.

## 2. The result, and the one cause — `outputs/benchmark_legged_eval.md`

| arm | legged_eval | frozen | Δ | zeros | alive at 20 s |
|---|---|---|---|---|---|
| **Rule-Planner** | **0.57** | 1.07 | −0.51 | 89/200 | 9/200 |
| **WALK fixed** | **0.76** | 1.11 | −0.36 | 60/200 | 29/200 |
| TROT fixed | 0.29 | 0.97 | −0.68 | 143/200 | 1/200 |

Ordering unchanged; the planner is still under its lower bound and by more (paired: better
1, worse 37, tied 162, was 0 / 6 / 194).

**The ablation is unusually clean.**  WALK, all 200 cells:

| | rim off | rim on |
|---|---|---|
| roughness off | 1.09 | 1.09 |
| roughness on | 0.76 | **0.76** |

The rim changes the score **not at all** — 1.09 vs 1.09 and 0.76 vs 0.76, identical, not
merely close.  The different terrain draw is worth 0.02 (our "neither" corner 1.09 against
the archive's 1.11).  Roughness is −0.33 in both rows and is the entire difference.

**Two of the three things predicted in advance did not happen, for one shared reason.**
The pit courses were expected to zero out half the grid; they actually score *higher* than
the non-pit ones (WALK 0.84 vs 0.67).  The rim was expected to help, since lane departure
was the main failure; it does nothing.  Both because **the median robot travels 0.79 m**.
Pits, rims and the terrain draw are properties of ground it never reaches.  Roughness is
the only one of the three that is under it at the spawn.

That also says where the headroom is: nothing past the first metre of any course is being
measured yet.

## 3. `4.48` was never a legged_eval number

* **Different tool.**  It is from `ev-go2/docs/RUN_RESULTS.md` (M6 distillation, 2026-07-10),
  produced by eurekaverse's own `evaluate.py`.  legged_eval's own teacher results are in
  `~/eval_out/teacher_lab_*.json` (`"schema": "legged_eval/1"`) and read **4.83 / 4.98**.
* **Different checkpoint.**  4.48 is teacher `_10_3`; the legged_eval-measured teacher is
  `2026-07-15_12-17-17_4_1`.

The scoring *rule* does agree (evaluate.py also means within a cell then over cells with
equal weight).  What differs is the terrain draw — upstream's is unseeded and depends on
how much RNG the host burned parsing its config, which legged_eval measured as changing the
peak height of 14 of 20 courses.  **Quote 4.83–4.98 with its seed.  The planner's 0.57 is
12% of it.**

## 4. JUMP torque: peak, not rated — `outputs/jump_torque.md`

Investigated only; nothing applied.

* Unitree's own labels: the GO joint motor page says "**Maximum** Torque: 23.7NM" at
  reduction 1:6.33; the Go2 page says "**Peak** Joint Torque: Approx. 45N.m", footnoted as
  the *largest* joint motor's.  The config's 23.70 / 45.43 are those two numbers.
  **Unitree publishes no continuous rating**, so there is nothing underneath for a peak to
  be 1.5–3× of.  The log's 54.67 N·m is 1.20× above the published *peak*.
* This withdraws one sentence of the earlier section ("that is its rated point").
* **The USD asset carries no limit**: PhysX shows 1.0e+09 on all twelve joints.  The clip
  exists only in `IdealPDActuatorCfg.effort_limit`, in Python.
* **No Isaac Lab actuator allows a momentary overshoot.**  All seven models end in
  `ActuatorBase._clip_effort`, a static symmetric clip — the actuator *networks* included.
  `DCMotor` is the only override and its dual limit is peak-vs-**speed**, not peak-vs-time,
  and stricter at speed than the flat clip.  Modelling the headroom needs a custom subclass.
* **And it would not open the pit courses.**  `front_jump` travels 26 ± 4 mm horizontally;
  the narrowest gap on the grid is 0.10 m and the widest 5.15 m.  More torque buys apex,
  and apex is not what is binding.  Ten of twenty courses stay at zero either way.

## 5. Videos — `outputs/video_le/`, and the regression check passed exactly

Side-view mp4s, ported into `run_benchmark.py` from `run_calibration_grid.py`'s camera:
created before `sim.reset()`, `update_period 0.0`, and `grab_frame()` calls `sim.render()`
and nothing else, so the substep loop keeps its `phys_dt`, decimation and write order.
Five 20 s clips at 960x540, 25 fps, mixing switching with pits — `sideways_ramp` L6
(8 switches, pit), `box_jump_even` L6 (pit, the furthest any planner robot got at 2.09 m),
`staircase_spiral` L3 (the largest TROT share on the grid, 0.42), `flat_circle_jump` L0
(pit, 2.18 m), `jump_on_and_off_box` L9 (6 switches, alive at 20 s).

**The regression check is the strongest form available and it passed.**  The recorded run
is the full 200 cells at the same seed and settings, so it was compared against
`outputs/bench_le/planner.csv` row by row: **200 rows x 27 columns, every value identical**
— `goals`, `steps`, `settle_ok`, `alive_at_end`, `switches`, the skill fractions and the
positions to the last digit.  Note that the recorded run also had `GPU=1` while the scored
one had `GPU=none`, so this re-confirms `isaac_docker_run.sh`'s claim that the card changes
nothing when nothing renders.

**The first recording was 20 s of a grey wall, and the arithmetic says why.**  The camera
sits 2.4 m across the lane, which is 0.4 m *outside* the 4 m patch — and legged_eval's rim
stands between it and the robot.  The sight line crosses the wall at y = 3.95, which is
0.1875 of the way from eye to target; at the old eye height of base + 0.12 m that crossing
is at 0.40 m and the rim is 0.495 m.  `--video-eye-m` (default 0.95 m) puts the crossing at
0.82 m, clear by 0.33, for a 16 deg look-down that still reads as a side elevation.  Worth
remembering as a shape: **the rim was invisible in every number and immediately visible the
first time anyone looked.**

The clips also show the roughness directly — the floor is visibly faceted, which is the
thing §2 says costs a third of the score.

## 6. `benchmark_frozen.npz` — kept, retired, labelled

Not deleted.  Every number before 2026-09-01 came off it (`outputs/bench/`,
`terrain_profile.csv`, the offline sweep, the threshold evidence), and deleting it makes
those unreproducible.  It is now reachable only through `--terrain frozen`, its rows stamp
`terrain=frozen-no-roughness-no-walls`, `outputs/benchmark_planner.md` opens with a banner
saying what it is, and CLAUDE.md §2 records the change.

## 7. Two things about the machine, neither ours

* **The WMP training is down and paused until 2026-09-02 20:28.**  `nvidia-smi` at 20:14
  showed it running (GPU1 22,861 MiB, `wmp_lab_train` up 19 h, iteration 10026 at 20:00);
  by 20:30 `wmp_status.log` reads `정지` and `~/.wmp_lab_pause` had been written at 20:28
  with a 24 h expiry.  Checkpoint 10151 is saved.  It will not come back on its own before
  tomorrow evening — `~/wmp_lab_resume.sh` is the way back.
* **`legged_eval`'s `RULE_ORACLE=1` now reads the wrong terrain.**  The rule_walker adapter's
  oracle mode loads `go2-planner/data/benchmark_frozen.npz` as its privileged height map
  while the simulator runs legged_eval's terrain underneath — different draw, no roughness,
  no rim.  The adapter already marks oracle runs as unscorable, so nothing published is
  wrong, but the comparison it exists to support no longer holds.  **Not fixed: legged_eval
  is read only.**  Reported, not patched.

## 8. The oracle ceiling, depth, and the threshold — `outputs/oracle_and_depth.md`

| | score / 8 |
|---|---|
| E2E teacher, same protocol, seed 1 | **4.83** |
| **ORACLE — best single skill per cell** | **0.81** |
| WALK fixed, no perception at all | 0.76 |
| Rule-Planner, **depth** | 0.60 |
| Rule-Planner, `STEP_TROT_MAX` corrected to the measured 0.03 | 0.59 |
| Rule-Planner, as configured | 0.57 |
| TROT fixed | 0.29 |
| TURN fixed (new — completes the envelope) | 0.17 |

**A perfect chooser gains 0.06 over always walking.**  Some skill beats WALK in 12 of 200
cells; in the other 188 the library has nothing better.  The rules are not choosing badly,
they are choosing among options that are not there.  **Planner redesign is not the route.**

**The depth arm is 0.03 better than the privileged one**, because it never selects TROT
(0 cells against 14) and TROT is the gait that survives 1 cell of 200.  Both numbers say
the same thing: sees-everything 0.57, sees-what-the-robot-sees 0.60, sees-nothing 0.76.
Perception is not the limit either.

**The threshold correction is real and small.**  0.57 → 0.59, better in 5 cells worse in 1,
TROT usage 14 cells → 5.  0.02 and 0.03 give the same score, so the choice inside the
measured bracket does not matter and `planner/config.py` keeps its 0.08 placeholder and its
`CALIBRATION_NEEDED` mark — the override is a run argument (`--planner-set`) and is stamped
on every row.

Everything between 0.57 and 0.81 is planner engineering and the whole of it is 0.24.
Everything between 0.81 and 4.83 is the skill library and it is 4.02.

## 9. The 37 cells the planner loses to WALK

**All 37 used TURN.  None was lost while running WALK alone.**  `frac_TURN` median 0.472
against 0.256 over the whole grid; the planner travels 0.31 m less than WALK in those cells
and ends with 0.19 goals against WALK's 1.22.  It is upright for a median 3.42 s and spends
**1.52 s of that turning on the spot at 0.0075 m/s**.

Same mechanism as the old 6 cells, six times as many of them — the rougher ground turns the
heading over faster, so the planner re-aims more often, and each re-aim is most of the life
it has left.  Across the whole grid the relation is not monotone (no TURN 0.42, TURN < 0.3
0.64, TURN >= 0.3 0.52), which is a correlation on cells of differing difficulty and not a
demonstration that TURN causes the loss — but the paired within-cell comparison above is.

## 10. Level 2 — designed, not built — `outputs/level2_design.md`

A stance-phase attitude regulator on the channel `sim/yawmoment.py` already proved
(feed-forward hip torque into the legs the recording has on the ground; 6x the authority of
moving a swing foot; additive to the PD, measured; 11.95 N·m peak against the 23.70 clip
with 0 of 2000 steps clipped, so there is room for a second term).

The evidence says it recovers about **two thirds** of the roughness loss and not the rest:
of the 64 cells WALK survives without roughness, 44 stop surviving with it — that part is
balance — but in the 136 that fall either way the roughness still costs 0.23 m of travel
and 0.33 of a goal, and there is no survival left to lose there.  Estimated ceiling ~0.98
against WALK's 0.76, with the oracle at 0.81.  Worth having; not a route to 4.83.

Falsifiable as specified: excursion must fall monotonically with gain on a flat-plus-noise
rig, and stride and forward speed must not move (the yaw couple's claim to be free was
v_x 0.664 → 0.661 and stride unchanged).

## 11. Session 4 (2026-09-02) — depth is the arm, the oracle is 0.81, level 2 is built

**One document for the review: `outputs/REPORT_2026-09-02.md`.**

### The rule that changed
`--skill PLANNER` now defaults to `--perception depth`, and that is the arm's number.
`--perception optimistic` is the perception UPPER BOUND and has to be asked for by name;
the single-skill arms carry `perception=n/a` because they read no terrain.  CLAUDE.md §2
has it as a rule.  Leaving optimistic as the default after a depth path existed would have
gone on publishing the control as the result.

### The table
| arm | score / 8 |
|---|---|
| E2E teacher | 4.83 |
| **ORACLE — best single skill per cell** | **0.81** |
| WALK fixed + roll couple (3-seed mean) | 0.82 |
| WALK fixed | 0.76 |
| **Rule-Planner, depth + roll couple** | **0.69** |
| Rule-Planner, depth | 0.60 |
| *Rule-Planner, optimistic (control)* | *0.57* |
| TROT / TURN fixed | 0.29 / 0.17 |

### The two courses at 0.00 — `outputs/zero_courses.md`
The teacher scores **4.49 and 4.87** there, so it is not the terrain.  They are the only
two courses that do not put goal 0 at 0.50 m: `staircase_walking_full_width` puts it at
1.00 m **on top of the first riser** (0.06 m at difficulty 0, already above WALK's limit),
and `staircase_spiral` puts it **2.6 m ahead and 0.30 m right, over perfectly flat ground**.
Not a structural limit — the same limits everything else has, against a differently placed
goal.  What it shows is how much the 0.50 m placement is doing for the other eighteen.

### Level 2, built and measured — `outputs/level2_results.md`
Roll only: **200 of 200 terminations are roll, none is pitch**, in 400 episodes.  The pitch
term was dropped before it was written.  `sim/attitude.py` is the exact dual of the yaw
couple on the same hip joint.

WALK three seeds paired: 0.76→0.83, 0.77→0.82, 0.69→0.81.  **The arm on depth: 0.60→0.69.**
Sign settled by measurement (+1 → 0.83, −1 → 0.66, i.e. worse than off).  Forward speed
−6/−5/−1 %, inside the ±10 % criterion; at gain 16/cap 4 it scores 0.85 and costs −19 %,
so **the recommended point is deliberately not the highest-scoring one.**  WALK-only by
table: TROT gains but loses 32 % of its speed, TURN is worse.

Design predicted ≈0.98, delivered 0.82.  The estimate assumed the term recovers the 44
cells that stop surviving when roughness is added; it recovers 6–18.

### The threshold correction
`STEP_TROT_MAX` 0.08 (config, `CALIBRATION_NEEDED`) vs 0.02–0.04 (measured): overridden by
run argument only, worth **0.57 → 0.59**, TROT usage 14 cells → 5.  0.02 and 0.03 score the
same, so the choice inside the bracket does not matter.

### Two harness bugs found this session
* The depth camera read `PLANNER_ARM` before it was assigned — Kit exits 0 with no
  traceback (harness_findings §17), **again**.  Resolution order in `run_isaac` is fixed.
* The roll-couple end-of-run report read `rolls` (built whenever `--foot-comp` is on,
  including the planner arm where it is never stepped) instead of `pl_roll`, and printed
  "0 stance-leg-steps driven" for a term that was working.  Scores never affected.

## 12. What to do next, in order

1. **The skill library is the whole question now, and the oracle proves it.**  0.81 is what
   a perfect chooser gets; 4.83 is the teacher.  Nothing in the planner closes that.  What
   is missing is concrete: a gait that holds on 40 mm of noise, and a forward-travelling
   jump (`front_jump` moves 26 mm horizontally; the narrowest gap on the grid is 0.10 m).
2. **Level 2 (§10) is the first piece of item 1** and is designed and costed.  It needs a
   decision, not more measurement.
3. **TURN costs more than it buys in its current form** (§9).  It is 1.5 s of a 3.4 s life
   at 0.0075 m/s.  Either a faster re-aim, or cross-track correction inside WALK instead —
   `SESSION_STATE` (2) §3 already measured the yaw couple taking WALK's curvature to
   0.24 °/m, so the heading may not need TURN at all on this terrain.
4. **`effort_limit`** — closed as far as evidence goes (§4); the decision is the user's and
   45.43 is confirmed to be the published peak.  Recorded, not acted on.
5. `staircase_walking_full_width` and `staircase_spiral` score **0.00 in every arm**,
   including the oracle.  Two courses of twenty that nothing in the library touches at all.
