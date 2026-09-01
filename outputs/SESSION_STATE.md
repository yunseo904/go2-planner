# Session state — 2026-09-02 (7), three hypotheses tested: one confirmed, two corrected

**Read this first next session.**  Everything below is committed.

**The newest work is §14 (session 7) and the current to-do list is §15.**  Sections 1–13 are
earlier sessions, kept in order; where a later section corrects an earlier one it says so,
and §14 corrects §13 on level 2 and the swing-lift mechanism.

Machine notes for session 7: the WMP training was stopped with `~/.wmp_lab_pause` set to
epoch 1788289200 = **2026-09-02 04:00 KST** (an earlier note in §7 read this as 20:28 and was
wrong).  GPU 1 was free for the whole session and was used only for the depth runs, which
RENDER; every other run was `GPU=none` on CPU.  GPU 0 was not touched.

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

## 12. Session 5 (2026-09-02) — skills pushed, four hypotheses closed

Full detail: `outputs/skill_push_2026-09-02.md`.

### The table now

| arm | score / 8 |
|---|---|
| E2E teacher | 4.83 |
| **ORACLE on the improved skills** | **0.88** (was 0.81) |
| WALK + roll couple | 0.83 |
| WALK | 0.76 |
| **Rule-Planner, depth + roll couple** | **0.63** (3-seed mean; seed 1 alone was 0.69) |
| Rule-Planner, depth | 0.55 (3-seed mean) |
| TROT / TURN + couple | 0.34 / 0.14 |

**Level 2 on depth, three seeds: 0.60→0.69, 0.54→0.62, 0.52→0.59.**  Positive in all three,
better-than-worse in all three.  **Quote 0.63, not 0.69** — seed 1 was the top of the spread.

### What was tried and what it cost

* **WALK swing lift — refuted, with a new reason.**  Score flat (0.83/0.76/0.82/0.83 at
  0/20/40/60 mm) while survival collapses (31→25→20→12) and **v_x rises +10/+19/+32 %**.  It
  is a *speed* edit, which is exactly what the `sin²` shape with zero slope at both endpoints
  was built not to be.  Fails the v_x criterion, so rejected on the criterion.  The
  hypothesis that the roll couple would change the trade is **wrong** — with the couple on
  it costs survival at the same rate.
* **TURN — the clip is genuine, the replay is not.**  `SPEED_TURN = 0.0075 m/s` is the real
  robot's own logged value.  But the sim delivers **31–36 % of the logged −22.66 °/s** and
  **translates backwards at 10× the logged forward speed**.  Foot placement excluded (off is
  worse), entry phase excluded (36 % vs 34 %) — it is the open-loop replay itself.  A 90°
  re-aim costs 4.0 s logged and **11.6 s replayed**, against a 3.4 s median life.
  **This is now the largest single unexplained gap in the library.**
* **RUN — one more route excluded.**  The roll couple cannot lift the body: peak base height
  0.313 → 0.311 m against WALK's 0.330 m standing.  A hip torque makes a force perpendicular
  to the hip→foot vector, which on a near-vertical leg is horizontal.  No vertical component
  exists to give.
* **JUMP** — unchanged, recorded.

### The planner's cross-track problem: diagnosed right, fix is a null

`RulePlanner._wants_turn` documents *"goal bearing minus heading"* and the harness was
handing it the drift from the **start** heading.  So the planner has been restoring its
launch heading, never aiming at a goal.  `--turn-target goal` fixes the wiring — and
measures **0.63 → 0.42 over three seeds**, worse in 45–50 cells of 200, and it does not
help either course it was diagnosed on.

The two findings interlock: aiming correctly makes the planner turn **more** (TURN share
0.24 → 0.39), and each turn costs 11.6 s.  **A better aim is worthless while the aiming
mechanism is 3× too slow.**  Kept, default `settle`; re-run it when the replay gap closes.

### The oracle, on the improved skills

| | WALK | ORACLE | headroom | cells where a non-WALK skill wins |
|---|---|---|---|---|
| couple off | 0.76 | **0.81** | +0.06 | 12 / 200 (TROT 7, TURN 5) |
| couple on | 0.83 | **0.88** | +0.05 | **9 / 200** (TROT 8, TURN 1) |

**Improving a skill raised the ceiling by 0.07 and left the planner's room where it was** —
and the cells where choosing matters *fell*, because the couple helps WALK more than the
alternatives.  Same conclusion as §8, now demonstrated by an intervention that worked.

### Three more harness bugs, all in `swing_lift_offsets`' free-floating measurement

1. It held the robot at a hard-coded **z = 1.5 m**, which is above the flat rig and *inside*
   this grid (staircase_climbing reaches 3.96 m).  The measurement was a collision response
   and returned 21.1 mm where the free measurement says 19.8.  `air_z` added.
2. It wrote every instance to `default_root_state`'s x-y — **the same point for all of
   them** — so a 200-robot fleet measured itself.  PhysX ran out of contact blocks and the
   run sat at 121 % CPU emitting nothing for twenty minutes.  `spread` added; both defaults
   preserve the one-robot rig bit-for-bit.
3. Between them they wrote **11.5 GB** of one repeated PhysX warning.
   `isaac_docker_run.sh` now drops that line at the source (`LOGCAP=0` restores it).

## 13. Session 6 (2026-09-02) — three layers, and they disagree

Full detail: `outputs/three_layer_2026-09-02.md`.  Judging is now (a) flat, (b) probes,
(c) benchmark, in that order.

### The finding that matters most

**The probe rig and the benchmark point opposite ways about the roll couple.**  On the
benchmark it is +0.07 on WALK, repeated over three seeds.  On the probe rig it **lowers
WALK's roughness limit from 0.010 to 0.005 m** (3/3 at 0.010 without it, 1/3 with), and the
benchmark's terrain is made of roughness at 0.02–0.04 m.  Its `step_up` goes 0/3 → 1/3 at
0.040 m — a direction, not a threshold.

The gain is real; **the mechanism is not established.**  Do not describe level 2 as "better
on rough ground" until this is resolved.  Either the benchmark gain comes from something
else (the rim, the obstacles, the goal geometry) or the two roughnesses are not the same
disturbance.

### TURN

* **Hip sign settled on the skill it matters for.**  `flip` does not turn: +1.09 °/s, 5 % of
  the log, wrong direction, stride 1.12 → 1.94 Hz.  TURN's hips move ±0.23 rad, the most of
  any clip, so this is a stronger test than the WALK verdict that set the default.  `keep`.
* **`--rate hi` is worth 72 % → 83 %** of the logged yaw rate with stride cv 0.224 → 0.068,
  and the benchmark has never used it.  **Not adopted globally**: WALK's v_x moves +13 %,
  outside ±10 %.  TROT is free (+1 %, cv halved).
* **A real bug fixed**: the single-skill TURN arm ran its placement law with `wz_log = 0`,
  because `--heading off` (the only way to run TURN) skips the block that loads it.  That is
  `turn_target.md`'s bug surviving in the one arm that never got the fix.  Worth 34 → 35 %.
* **Half the flat→benchmark gap is the measurement window.**  The turn is a slow transient:
  4 cycles 57 %, 25 cycles 73 %, 61 cycles 75 %.  Benchmark TURN robots are upright 3.4 s.
  **Still open: 57 % → 35 % at a matched window.**  Excluded: roughness, obstacles, rate,
  ω_log, hip sign, entry phase, settle mode.  Left: the per-cell lever, the 0.42 m spawn
  drop onto rough ground, the 200-robot scene.
* `convention_verified` is a literal `False` written by every extractor and computed by
  nothing — a placeholder for "nobody has checked leg order and joint signs against this
  robot".  The flat rig's identity check **is** that test; on TURN it passes at r = 0.380
  with a 0.054 margin, which the tool flags as thin.

### TROT — nothing found

**0/3 at the lowest rung of every probe family**, with and without the couple: 0.020 m step,
0.005 m roughness, the shallowest slope.  The 0.02–0.04 m `STEP_TROT_MAX` bracket came from
a different rig and does not reproduce here.

### RUN / JUMP — closed

RUN: the couple cannot lift the body (0.313 → 0.311 m against 0.330 standing); a hip torque
has no vertical component.  JUMP: published maxima, 4–200× short on gaps.

### Videos — `outputs/video_turn/`

Flat TURN at half speed, side and **top-down** (added: from above a turn is the motion, not
a silhouette), plus the placement law off.  Regression identical on every column, including
`yaw_rate_deg_s` to 16 digits.  Not done: the in-air replay of the log's joint angles, and
the WALK couple on/off pair on a probe step.

## 14. Session 7 (2026-09-02) — three hypotheses tested, one confirmed, two corrected

Full detail: `outputs/support_gate.md`, `outputs/level2_verdict.md`, `outputs/cross_track.md`.

**The list from §14 of the previous session is superseded by what follows.**

### The headline

| question | answer |
|---|---|
| Is the roll couple's benchmark gain about roughness? | **Yes — entirely.** +0.080 with roughness, −0.005 without. The benchmark is right, the probe's criterion is wrong for this term. **0.83 stands.** |
| Does WALK hold 3 feet down? | **No, and it cannot.** duty 0.662 → 56.8 % of the cycle on 2 feet. The proposed gate would reject the only skill that works. |
| Does swing lift remove support? | **No — it ADDS it** (2.374 → 2.512 feet). It was never a support edit; it is a speed edit (+16 to +20 % v_x). |
| Can the robot go straight? | **No.** 0.53 m of lateral drift per metre travelled, 149/200 cells the same way. |
| Can foot placement fix that? | **No.** Every gain, either sign, and three times the authority all reproduce the control. The channel does not move lateral position. |

### 1. The support gate had to be written relative, not absolute

`scripts/support_polygon.py`, on our own clips:

| clip | duty | mean feet | min | below 3 feet | benchmark |
|---|---|---|---|---|---|
| WALK | 0.662 | 2.65 | 2 | **56.8 %** | 0.76 |
| TROT | 0.578 | 2.31 | 2 | 78.1 % | 0.29 |
| **TURN** | 0.672 | **2.69** | 2 | 48.9 % | **0.17** |
| RUN | 0.344 | 1.38 | 0 | 100 % | 0.00 |

Two things follow. **Mean feet = 4 × duty**, so "3 feet at all times" requires duty ≥ 0.75;
our `03_slow_walk` recording is 0.66 and therefore *must* spend part of its cycle on two
feet. And **support does not order the scores** — TURN has the widest support of any clip
and the worst score. CLAUDE.md 2.5 is therefore a *relative* gate: a foot-moving
intervention must not reduce support below the unedited clip's own measurement, measured
paired. Torque-only terms are exempt by construction and still stamped.

The static/dynamic account itself survives and still explains why WALK is the only skill
that works open loop. What the measurement removes is the number "3", not the idea.

### 2. Swing lift — rejected, for a different reason than proposed

Flat rig, WALK, 60 cycles, contact measured from force:

| lift | mean feet | min | below 3 | v_x |
|---|---|---|---|---|
| 0 mm | 2.374 | 2 | 70.8 % | 0.2332 |
| 20 mm | 2.406 | 2 | 64.1 % | +16 % |
| 40 mm | 2.421 | **1** | 61.7 % | +20 % |
| 60 mm | 2.512 | **1** | 54.5 % | +19 % |

Support *rises* monotonically; the rear feet are not being lifted off, the gait is spending
longer in stance. What appears is single-foot support the reference never has. **Rejected on
the v_x criterion and on min support, not on mean support.**

**Both adopted foot-moving terms pass and neither is marginal**: foot placement 2.374 →
2.385 at v_x −0.7 %, plus heading hold 2.438 at −5.7 %, minimum 2 throughout.

### 3. The roll couple: (b), and the description changes

WALK, seed 1, 200 cells, `--heading heading-only`, recommended point:

| | couple off | couple on | Δ | paired |
|---|---|---|---|---|
| roughness ON | **0.755** | **0.835** | **+0.080** | 24 b / 12 w / 164 t |
| roughness OFF | **1.095** | **1.090** | **−0.005** | 3 b / 3 w / 194 t |

**Two regression checks passed exactly** (0.755 and 1.095 are the published arms, reproduced
by code three sessions newer, with `level2_results.md`'s own 24/12 paired counts). Repeated
on the `--heading off` arm as a control: +0.080 / +0.000.

So **(a) is refuted, (b) holds, (c) does not apply** — `sim/attitude.py` writes effort
targets on legs the recording already has down and never writes `q_des`. **0.83 stands, and
level 2 may now be described as better on rough ground** — the opposite of the caution
session 6 imposed.

Why the probe disagreed, as far as this goes: its roughness ladder is 0.005–0.015 m and the
benchmark's is 0.02–0.04 m, above the top of it everywhere; and its rungs are n = 3, so the
headline 3/3-vs-1/3 is a one-to-two-episode difference against 200 paired cells. **Direction
and dependence established; mechanism not.** Re-run the probe at 0.02–0.04 m with more
repeats before claiming it.

### 4. Cross-track: the drift is real, the channel does not touch it

Heading hold fixes the ANGLE (13.26 → 0.24 °/m) and nothing fixes the POSITION. WALK,
no roughness: forward 1.006 m median, |cross-track| 0.562 m, **0.531 m/m**, and **149/200
cells drift the same way**. That is a bias, not scatter, and it is a candidate cause for
`staircase_spiral`'s 0.00 (goal 0 sits 0.30 m to one side). The depth arm carries the same
drift (|cross-track| 0.551 m).

`--cross-track hold` asks the lateral law for `v_y = −gain × error` against the robot's own
settle-line datum. Paired against off, gains 0.5 / 1.0 / 2.0 / **−1.0**, caps 0.03 / 0.10 /
0.30: **every arm lands inside 0.720–0.765 against the control's 0.755**, with peak
cross-track inside 0.398–0.429. **The sign flip is indistinguishable from off** — the test
that settled the roll couple's sign, failed here.

Checked offline: `vy_bias = −0.10 m/s` produces exactly **0.0500 rad** of hip command and
`--foot-clip-rad` is **0.05**, so the term saturates the law's authority at the smallest cap
tried. **But that is not the explanation.** At `--foot-clip-rad 0.15` — three times the
authority, run as a pair — the term still moves peak cross-track by **+0.1 %** and final
|cross-track| by **+1.3 %**, and costs 0.780 → 0.750 with survival 47 → 28.

> **The lateral placement channel does not move lateral position on this terrain, at any
> gain, either sign, or three times the authority.** Whatever produces the drift is not the
> body-frame lateral velocity this law regulates.

**Not adopted; flag kept, default off.** An earlier draft blamed the 0.05 rad cap and linked
it to TURN's 31–36 %; the wider-cap arm refutes that and it is withdrawn.

One measurement worth chasing: `cross_track_m` (0.598) and `cross_track_abs_max_m` (0.408)
disagree because the first is read at the end of the episode and includes post-mortem
sliding while the second stops at the last upright step. `final_x_m` and `travelled_m` have
always had that property too.

### 4b. The by-product: survival and score have decoupled

**`--foot-clip-rad` has been 0.05 since the placement law was written and has never been
swept.** Swept now, WALK, seed 1, roll couple on — the adopted configuration:

| clip | score | alive at 20 s | v_x |
|---|---|---|---|
| **0.05** (the adopted default) | **0.835** | 31 | 0.0875 |
| 0.10 | 0.800 | 40 | −3.8 % |
| 0.15 | 0.800 | 49 | −7.4 % |
| **0.25** | **0.835** | **53** | −6.2 % |

**Survival rises by 22 cells and the score ends exactly where it started**, with the
intermediate points no better (0.800 at both 0.10 and 0.15). Without the roll couple the
same widening reads 0.755 → 0.780 / 29 → 47 alive, which is the same effect through a
different baseline.

> **Survival and score have decoupled.** The roll couple and the placement cap both buy time
> upright and past a point neither converts it into distance; v_x falls 6–7 %, which costs
> goals outright.

Consistent with the rest of the grid — median robot dies at 0.79 m, goal 0 sits at 0.50 m in
180 of 200 cells, and §2 already recorded that nothing past the first metre is being
measured. **Interventions that buy balance have run out of score to buy.** Same conclusion
the oracle reached from the other side (0.88 against 4.83).

Worth a proper sweep — a 22-cell survival move from an unexamined default should be
understood — but it is **not** a lead on the score.

### 5. Results wiring for the review (§8 of the request)

Columns are only ever appended, so older files stay readable and new ones are supersets.
Per cell: `vy_mean_ms`, `cross_track_m`, `cross_track_abs_max_m`, `curvature_rad_m`, `fell`,
`episode_s`. Stamped on every row: `terrain_seed`, `measurement_seed`, `episode_length_s`,
`episodes`, `num_envs`, `rate`, `gutter`, `spawn_z`, `settle_s`, `foot_clip_rad`, `steering`,
and the commanded-support pair.

**The two condition checks, answered from the data:**

* **Episode length AGREES.** Ours is 20.0 s and `~/eval_out/teacher_lab_*.json` reports
  `protocol.episode_length_s` 20.0. No change needed.
* **Env count DOES NOT.** Those runs report `num_envs` **1000** — the same 200 cells with
  **5 episodes each**. We run 200 with `--episodes 1`. Same protocol, **one fifth the
  sample**. `--episodes 5` closes it at 5× the cost; until then the stamp makes it visible.

**Steering is a third category and is named as one**: `dead-reckoned-heading-hold`. Heading
hold remembers the heading the robot settled at and pulls the yaw error back to it; no
waypoint or velocity command reaches the gait, and the goals are scored against but never
steered toward. Neither "self" nor "commander" fits.

`scripts/aggregate_benchmark.py` (CPU only) turns any results CSV into the three tables —
overall, by difficulty, by course — with conditions as a footnote read off the rows. It
refuses to pool differing terrains and says when it has reconstructed a column.
`scripts/csv_coverage.py` answers what the existing runs can already fill: **68 results CSVs
in 9 column sets, and only the two newest have everything.** 50 of 68 can fill tables 1–3
(they carry `end_cause` + `upright_s`); the 11 earliest legged_eval files cannot produce
falls or episode length at all.

### 6. The review run

`outputs/bench_le/depth_review_s1.csv` -- the representative arm (Rule-Planner, depth, roll
couple, heading hold) re-measured in the section-8 format. **It reproduces the published arm
on 200 of 200 cells, 0.685.** `outputs/aggregate_depth_review.md` is its three-table
aggregate. Seeds 2 and 3 follow for the error bar.

Cost note for planning: the depth run took **4.7 minutes**, not the ~20 assumed. Depth is
much cheaper than it has been treated as.

### 7. Asked for and already done before this session

* **§3's transient discovery is already recorded** — SESSION_STATE 13 carries the 4 / 11 /
  25 / 61-cycle table (57 / 61 / 73 / 75 % of the logged yaw rate) and states that the
  benchmark gives TURN a 3.4 s median life. The implication the request asks to write down
  is written down. **Not repeated.**
* **§5's measurement is already done** — SESSION_STATE 13 has `--rate hi` per clip on flat
  (TURN 72 → 83 % with stride cv 0.224 → 0.068; TROT +1 % and cv halved; WALK v_x +13 %,
  outside the band) and the conclusion not to adopt it globally. **What is missing is the
  per-clip implementation, not the evidence.**
* **§4's "a hip torque has no vertical component"** was closed in sessions 5 and 6. The
  leg-extension hypothesis is new and is now partly measured (§4 above, `run_extension.md`).

### 8. Not started this session

§3 (TURN in-air replay and the remaining per-cell-lever / spawn-drop / 200-robot
candidates), §5 (per-clip `--rate`), §6 (TROT's 4 cm failure mode frame by frame), §7 (the
fall-after-climb roll divergence), §7-b (WALK+lift against TROT).

**§7-b's premise did not survive a first look and is worth restating before it is run.** It
assumes swing lift makes WALK's contact pattern resemble TROT's. Measured, WALK+lift goes
the *other* way — mean feet down 2.374 → 2.512, while TROT's own clip is 2.31 commanded.
Lift makes WALK's support *wider*, not narrower, so the natural experiment as described does
not exist. A flat TROT reference was attempted and is unusable: bare `--clip TROT` fell at
0.60 s on a roll after one cycle, because the default entry phase is not one of the four
`trot_yaw_moment.md` found TROT survives at.

---

## 15. What to do next, in order

1. **Take the decoupling seriously: measure TRAVEL, not survival.** Two independent terms
   now buy time upright and neither buys goals (§4b). Before adding a third balance term,
   establish what stops a robot that stays up from going anywhere — the distance-to-next-goal
   and cross-track columns are already in the CSV and have never been the target of an
   intervention. **This reframes items 2–4 below rather than replacing them.**
2. **Build the in-air replay.** It is the kinematic control for TURN's missing 57 % → 35 %
   (§3, open since session 6) *and* the only way to settle RUN's posture question (§4). One
   rig, two open questions.
3. **Where does the cross-track drift come from?** Not heading, not the lateral placement
   law at any authority. Candidates: foot slip, yaw-coupled sliding, and post-mortem drift
   (`cross_track_m` 0.598 against `cross_track_abs_max_m` 0.408 — the gap is the fall).
4. **Re-run the probe rig at the benchmark's own roughness (0.02–0.04 m) with more repeats**,
   to close level 2's mechanism. Direction and dependence are settled; mechanism is not.
5. **`--episodes 5`** to match legged_eval's 1000-env sample, now that a depth run is known
   to cost 4.7 minutes.
6. **Sweep `--foot-clip-rad` properly** — three seeds, both skills, support gate. An 18-cell
   survival move from a never-examined default is worth understanding even though it is not
   a score lead.
7. **The library is still the ceiling** — 0.88 against 4.83. Nothing this session moved it.
8. `effort_limit` is the user's decision; the evidence is closed.
