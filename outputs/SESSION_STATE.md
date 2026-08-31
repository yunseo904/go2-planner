# Session state — 2026-09-01, TURN solved on flat, the benchmark gets a scorer and a number

**Read this first next session.** Everything below is committed.

CPU (`--device cpu`) for everything except the last hour: the nine side-view recordings
needed GPU1 and the WMP training was paused, stopped, filmed around, and resumed by the
procedure in `~/WMP_중단_재개.md`. See "The GPU borrow" at the end — there is one thing
about the watchdog to check.

---

## The headline

| | before | after |
|---|---|---|
| TURN on flat | 2 of 5 sampled entry phases complete a 90° turn; the phase the planner actually uses was never checked | **the planner's phase never turns**, and a measured one does: 9/9 cells in both compensator arms, 0.19 m of drift, 62 sustained cycles in the second harness |
| the benchmark | never simulated — every script that opened the archive read it with numpy | **scored**: single-skill TROT **0.98 / 8**, WALK **1.11 / 8**, all 200 cells, upstream's own goal rule |
| RUN's swing gate | named as the untried candidate in `run_collapse.md` §6 item 3 | **excluded**: identical collapse time to fourteen decimals |
| JUMP's torque gap | "is the fork's `effort_limit` even right?" | **it is the Go2 spec sheet**, and the real robot's excursion above it is 11–23 ms. A decision, framed, not taken |
| the capture-point rear term | untested import | **null**, on a paired 75-run design, all three parts separately |

---

## 1. TURN — solved on flat, and the criterion was the problem

`outputs/turn_entry_phase.md`. 810 grid runs: all 45 entry phases of the 45-frame clip,
9 identical flat cells each, with `--foot-comp on` and again with it `off`.

- **22 of 45 phases pass with the compensator on, 5 of 45 with it off.** Foot placement is
  doing most of the work for TURN — a much larger effect than it has on the translating gaits.
- **`level_start`, the rule the planner and `verify_skill_replay` both use, picks frame 24**,
  which is inside a contiguous ten-frame dead band (20–29) that fails in **both** arms. So
  "the compensator breaks those phases" is excluded.
- **No pose property predicts the outcome.** Feet-down at entry: 4/4 is 3 pass, 3 fail,
  2 mixed. Joint speed does not separate them. Coplanarity actively misleads — frame 24 has
  the *best* foot spread in the cycle.
- **Frame 6 is the only phase that passes 9/9 in both arms with both neighbours doing the
  same.** It is now `planner.config.skill.ENTRY_FRAME_TURN = 6` (provenance MEASURED) and
  reachable as `--start-phase measured`. Default stays `first`; nothing earlier changes.

**The second harness disagrees about two of the four phases, and that is the real finding.**
`verify_skill_replay` has **no drift term**: frame 24 turns steadily for 63 cycles while
translating 0.41–0.58 m in the first 8 s, and its PASS verdict cannot see that. A turn in
place that walks half a metre away is a curved walk with the right yaw rate. Frame 6 is the
only phase two independent instruments both call good.

Against the user's own gate: stride 1.121 Hz against an expected 1.121 (exact), duty 0.613
against the clip's 0.591, yaw −21.33 °/s against the log's −22.66. **Residual**: `vx_mean`
0.021 m/s against the log's 0.0075 — the sim TURN still translates 2.8× the recording, in
both passing phases. Not fixed by the entry phase and not chased.

**This does not change `turn_probes.md` §4.** That reduction already used the flat control
as its own denominator, so TURN's terrain limit — below every ladder floor, under a 0.02 m
edge — stands unaltered.

## 2. The benchmark now has a scorer, and the first numbers

`scripts/run_benchmark.py`, `outputs/benchmark_harness.md`. Upstream's rule exactly: goal
within **0.20 m held for 0.10 s**, eight goals **in order**, 20 s episodes, per-cell episode
mean, then the **equal-weight mean over all 200 cells**.

| | score |
|---|---|
| single-skill TROT (the lower bound CLAUDE.md §2 requires) | **0.98 / 8** |
| single-skill WALK | **1.11 / 8** |
| upstream `walk_pretrain` baseline | 1.05 |
| upstream E2E teacher `_10_3` | 4.48 |

**Read the floor before quoting either.** Goal 0 sits **0.50 m from the spawn in 180 of the
200 cells**. A score of 1.0 means the robot moved half a metre. That is why upstream's
untrained baseline is 1.05, and it means the benchmark's useful range runs from about 1 to
about 4.5, not from 0 to 8.

TROT is **flat across all ten difficulties** (0.90–1.05) — it dies before reaching anything
the difficulty parameter controls. WALK has a shallow monotone gradient (1.30 → 1.05) and is
the first thing in this project whose benchmark score responds to difficulty at all.

Three guards are in the harness, each from something that has already gone wrong: the
infinite ground plane is deleted **and verified** (11 of 20 tasks are 16–48% pit); a run with
fewer than 200 cells is **refused** without `--allow-partial`; and a robot that has fallen
through the terrain is **refused** rather than scored (see §5).

## 3. RUN and JUMP — both closed as far as they go, neither fixed

**RUN**: `outputs/run_swing_gate.md`. `--foot-swing-source sim` moves the correction
(cap-hit 26.5% → 36.8%, overwrite 66.2% → 59.6%) and the collapse time is **identical to
fourteen decimal places**. Across three entry phases the mean change is +2.9% on a run that
needs 19 s. `run_collapse.md` §6's ordering holds: item 1 (the base never becomes ballistic)
is upstream of items 2 and 3. **Item 2, the loop seam, is also not the way in** — the uncut
full-session replay already on disk survives 10.28 s but at duty 0.96 and 0.035 m/s, i.e. by
standing rather than running.

Collapse time moves **0.93 → 1.64 s across three entry phases**. The 1.13 s quoted
everywhere is one draw.

**JUMP**: `outputs/jump_torque.md`, appended. The config's 23.70 / 45.43 N·m **are the Go2
spec sheet**, and the ratio proves it: 45.43/23.70 = 1.9169 and 30.1/15.7 = 1.9172 — the same
motor behind a 1.917:1 knee reduction. The actuator is `IdealPDActuatorCfg`, a static clip
chosen *over* `DCMotorCfg` to avoid torque-speed derating, so the sim is already more
permissive at speed than a real motor.

The log's 54.67 N·m is **LowState's measured torque, not a command** (the calf's peak
*command* is 38.2, inside the limit), and it reproduces on all five takes: calf 53.8–56.2,
thigh 26.2–28.0. Time above the limit: **7–51 ms per jump, longest single bout 11–23 ms.**
It is a momentary peak, and the user's reading was right.

**This withdraws a line in `jump_torque.md`**: "the p99.9 is over the limit too, so it is the
shape of the push-off and not one spike" does not follow — p99.9 of a 16 s recording at
420 Hz is the top ~7 ms, so one 20 ms excursion puts it over by itself.

**`effort_limit` was not changed.** The three options are laid out at the end of that file;
it is the user's call and it changes the E2E arm too.

## 4. The capture-point import — null, and separated so the null is readable

`outputs/trot_capture_point.md`. Three differences, three flags, all default off, self-test
asserts the default path is bit-identical:

| | ours | theirs | paired result over 75 runs |
|---|---|---|---|
| gain | `T_stance/2` = 0.186 s | `sqrt(h/g)` = 0.178 s | **4.4% apart — not where the laws differ** |
| velocity | instantaneous | 20-sample average | 69 of 75 identical |
| clip | ±0.05 **rad** = ±0.0155 m | ±0.05 **m** | the only visible arm; 8 fewer falls of 75, p ≈ 0.11 |

Curvature medians across the five arms: 11.3–14.0 °/m, non-monotone, against a within-arm
per-cell spread of 1.7–48.9. `reached` is 0/75 in every arm. **Nothing here makes TROT go
straight**, and `trot_straight.md` §3 predicted it: the disturbance is DC and all of this is
still proportional.

The sweep did surface that **entry phase moves TROT's fall rate from 11/15 cells to 15/15**.
TROT's 32 phases have not been swept. That is the cheapest untried thing left.

## 5. Two harness traps, both recorded

`harness_findings.md` §14 and §15.

- **`steps` in the grid CSV is the repeat's shared loop length, not the robot's.** Read as
  survival it produced "median 901, min 901, max 901" across fifteen different step heights.
  Per-robot survival is `fell`; progress is `final_dist_m`.
- **An 11.3 M-triangle terrain cooks without error and collides with nothing.** The first
  200-cell benchmark run scored 0 on every cell at control step 1 because the robots fell
  through during the settle. `settle_ok` cannot catch it — the hip-to-foot lever is valid in
  free fall. Fixed with one collider per task; guarded by a base-height check. **A 0.00 across
  all 200 cells is exactly what a single-skill lower bound might legitimately produce.**

## 6. Swing height — the three numbers converge once the basis is named

`outputs/swing_height_basis.md`. Measured twice, the second time in a floored replay where
lift-off is a physical event and the sim's contact sensor and the clip's channel agree
within 6%:

| quantity | ours (TROT) | theirs |
|---|---|---|
| apex above **lift-off** | **57 mm** | `quadruped_pympc` 0.2 × 0.28 m = **56 mm** |
| the same rule for **our** body | 0.2 × 0.333 = 67 mm | — |
| apex above **ground** | **79 mm** | Unitree `footRaiseHeight` **0.08 m** |

**All three agree and the apparent dispute was entirely the basis.** There is no swing-height
deficit in TROT.

WALK's rear feet come out at **0.0 mm above lift-off** on both gates while the front pair
manages 44–50. Recorded, not chased — WALK is deferred by request — but it is a front/rear
asymmetry rather than a body attitude, so a level body would not fix it.

The early-touchdown reflex was **not** imported: a skill that regenerates its trajectory on
contact is reacting to terrain. Recorded as outside confirmation that a fixed trajectory does
not climb.

## 7. What to do next, in order

1. **Sweep TROT's 32 entry phases** the way TURN's 45 were. It is the largest lever
   measured on TROT this session and it costs one grid run.
2. **Wire heading hold into `run_benchmark.py`.** The 0.98 / 1.11 are the bare Raibert
   lateral law; the calibration sweeps use heading hold and the two are not comparable.
3. **Decide `effort_limit`** (`jump_torque.md`, three options). It gates JUMP entirely.
4. **The gap re-run** from the last session is still not done — it was the first item then
   and stayed unfinished.
5. `run_planner_replay.py` still uses `level_start` for every clip; it now has a measured
   frame for TURN available and does not read it. One line, but it changes what the planner
   executes, so it is listed rather than done.

## 8. The GPU borrow — and one thing to check

Procedure followed exactly: `pause.sh 180` → `stop.sh` (saved `model_4272.pt`, nothing lost)
→ nine recordings on GPU1 → `resume.sh`. GPU0 never touched.

**`~/.wmp_lab_watchdog_state` was missing before the borrow**, and `~/wmp_lab_watchdog.log`
ends at 2026-08-31 12:23 with fifteen consecutive "재시작 3회 동안 iteration 이 늘지
않았습니다 … 멈춥니다" lines — the loop-prevention trip from 함정 1. Training itself was
healthy at that moment (checkpoints every ~1.5 h, `model_4000` 30 minutes before the borrow),
so the trip is stale, and with the state file gone the counter is not latched. **Worth a look
anyway**: if the watchdog is meant to be the safety net, its state file should exist.

**Regression check on the recordings** (CLAUDE.md §8: rendering must not change physics):
every recorded run's termination matches its un-recorded CPU baseline. Table in
`outputs/video/README.md`.
