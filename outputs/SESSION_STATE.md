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

## 5. Videos — `outputs/video_le/`

Side-view mp4s, ported into `run_benchmark.py` from `run_calibration_grid.py`'s camera:
created before `sim.reset()`, `update_period 0.0`, and `grab_frame()` calls `sim.render()`
and nothing else, so the substep loop keeps its `phys_dt`, decimation and write order.
Cells filmed mix switching with pits — `sideways_ramp` L6 (8 switches, pit),
`box_jump_even` L6 (pit, the furthest any planner robot got at 2.09 m), `staircase_spiral`
L3 (the largest TROT share on the grid, 0.42), `flat_circle_jump` L0 (pit, 2.18 m),
`jump_on_and_off_box` L9 (6 switches, alive at 20 s).

**Regression check: the recorded run is the full 200 cells with the same seed and settings,
so its CSV is compared row by row against `outputs/bench_le/planner.csv`.**  See §8 for the
outcome.

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

## 8. What to do next, in order

1. **Roughness is now the binding constraint, and it is 40 mm of noise.**  WALK's measured
   step bracket is 0.04–0.06 m, so the terrain is at the limit everywhere at once.  That is
   a sharper target than "the library is the ceiling" and it is testable: raise WALK's
   ground clearance, or add the terrain-following the recording does not have.
2. **The depth arm is not wired.**  `run_benchmark.py` has no camera path;
   `legged_eval`'s `rule_walker` adapter has one, but its locomotion is a *generated* gait,
   not our clip replay — its `RULE_MODE=clip` scored **0.00 goals on 30 cells** with the
   open-loop clip falling at 3.68 s, matching `open_loop_replay_limit.md`.  So it does not
   answer "our Rule-Planner with real depth"; that needs a `TiledCamera` in our harness
   feeding `legged_eval.adapters.depth_terrain.local_maps_batch`.  Timing measured on GPU 1:
   30 envs × 400 steps with depth is ~25 s of stepping, so 200 envs × 2000 steps is roughly
   **15 min plus 4 min of startup**.
3. **`effort_limit`** — still the user's call, now with the labels resolved (§4).
4. The 21 zero-scoring cells of the old grid are now 60–143 depending on the arm; the
   question "which cells and why" is worth asking again on this terrain.
