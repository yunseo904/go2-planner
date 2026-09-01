# Session state — 2026-09-01 (2), TROT's heading is solved on flat, and the entry phase is the bigger number

**Read this first next session.** Everything below is committed. All CPU (`--device cpu`),
**no GPU at any point** — GPU 1 was left alone; `nvidia-smi` showed 22.8 of 24.5 GB in use
by `wmp_lab_train` and nothing here needed a card.

---

## The headline

| | before | after |
|---|---|---|
| TROT heading on flat | 3.20 °/m, "authority spent", cross-track named as the only route left | **0.004–0.170 °/m**, inside the 0.565 budget at every entry phase where TROT survives, at no cost in stride or forward speed |
| the mechanism | three routes, all foot PLACEMENT, all bounded by the trot's footfall margin | a **stance-leg yaw couple**: feed-forward hip torque, dynamics not kinematics, bounded by the friction cone and the effort limit instead |
| TROT's entry phase | never swept ("the cheapest untried thing left") | **swept: TROT walks at 4 of its 32 phases on FLAT ground**, and the ladder has been sampling four dead ones out of five |
| `run_benchmark.py` | no heading hold, so its scores were not comparable with the calibration sweeps | `--heading` wired, banner, per-row stamp |
| `run_planner_replay.py` TURN | `level_start`'s frame 24, which `turn_entry_phase.md` measured as never turning | `--entry-turn measured` reaches `ENTRY_FRAME_TURN = 6` |

---

## 1. TROT heading — `outputs/trot_yaw_moment.md`

`trot_straight.md` was right about the routes it tried and wrong about the premise under
them. All three moved a **swing foot**, and all three were bounded by how far a trot's
footfall can be displaced before the trot stops being one — a bound that was already
measured to get worse when widened.

`sim/yawmoment.py` moves no foot. It adds feed-forward torque to the hips of the legs the
**recording** has on the ground, where the load path is through the contact, so the
torque buys a lateral ground reaction force and the forces buy a couple about the
vertical. `tau_i = c·sign(x_i)` makes every stance leg contribute with the same sign,
which matters because a trot only ever has a diagonal pair down — the couple does not
switch off twice a cycle. Net roll moment is exactly zero; net lateral force is the 3.2%
front/rear lever mismatch and nothing else.

**Nothing new is asked of the simulator.** `isaac_actuator_probe.json` already recorded
it: explicit `IdealPDActuator`, effort target **additive** to the PD, 5 N·m in → 5.0 N·m
out, PD not replaced.

**Open loop** (heading hold off, baseline +5.17 °/s): linear, bidirectional, crosses zero
at **c ≈ 0.49 N·m**, about **9–10 °/s per N·m** and safe to at least ±2 N·m — **20 °/s of
authority against foot placement's 3.1**. Across the whole working range v_x moves
0.664 → 0.661 and stride does not move at all.

**Closed loop**, gain 5 N·m/rad, cap 2.0: curvature **3.20 → 0.01 °/m**, peak hip torque
11.95 against the actuator's 23.70, **0 of 2000 control steps clipped**, and the term never
reaches its own cap.

**Read a second way** (`scripts/analyze_yaw_moment.py`, from the trajectory rather than
the mean yaw rate, because a heading loop drives the mean to zero by construction):
net yaw 84.4° → 0.92°, **heading excursion 89.9° → 15.4°** (a bound, not a mean), lateral
drift 19.09 m → 3.49 m, distance along the launch heading 15.0 → 26.3 m of the same 26.6 m
path. The two readings agree.

**Why a P law works here when `trot_straight.md` §3 says it cannot.** §3 is right: a P law
holds a standing error and this disturbance is DC. The question is only whether that
standing error is small. Nulling the drift costs 0.49 N·m against a 2.0 N·m cap, so the
loop has somewhere to sit instead of pinning. **The law did not change. The authority did.**

## 2. TROT's entry phase — the larger number, and it was not the target

All 32 phases, flat, 40 s, both arms. **TROT survives 4 of 32 with the term off** (phases
0, 16, 17, 29) **and 3 of 32 with it on**. It falls at the other 28 in 0.9–2.9 s on flat
ground with heading hold and nothing else.

`entry_frames(rep)` picks **0, 6, 13, 19, 26** at `--reps 5`, and only **0** survives. So
four of every five `STEP_TROT_MAX` repeats this project has run started at a phase where
TROT falls on flat in under three seconds. Visible in the old CSVs: `steps` is 901 for rep
0 and 68 / 146 / 84 / 57 for reps 1–4, at every level.

Where the question is askable, the couple wins **3 of 3** (0 of 3 inside budget without it,
3 of 3 with it). **The one run it cost is phase 29**, which survives with the term off and
rolls at 2.54 s with it on; the cap-hit column says why — 0.0% of stance steps at the cap
on the surviving phases, **19.9%** on phase 29.

## 3. What the couple does NOT do

- **The paired 15-cell `step_up` ladder is a null**: curvature 11.97 vs 12.06 °/m, drift
  0.77 vs 0.81 m, 11/15 fell in both. Read it next to the off-arm's own 11.97 against the
  flat rig's 3.20 for the same controller: there TROT is being knocked over and the
  curvature is the collapse.
- **Re-running the ladder at the surviving phases does not rescue it either**: 90 runs,
  **one cell of ninety reaches goal 2**. `STEP_TROT_MAX` stays unmeasurable, with the
  reason narrowed — not lane departure, not heading, not entry phase. Phase 17 reaches
  **3.85–3.88 m at every one of the fifteen levels**, the same on a 0.02 m step as on a
  0.30 m one, which is not a step limit but something at a fixed place ~0.1 m short of
  goal 2. **That is the next thing to look at.**
- **It does not remove the lateral offset entirely**, though the feed-forward takes most
  of what was left. At gain 5 the robot ends 3.49 m to the side of a 26.6 m path (down
  from 19.09); adding the measured open-loop zero underneath the loop
  (`--yaw-moment-nm 0.49`) takes it to **1.25 m**, with the heading excursion 15.4° → 13.4°
  and net curvature 0.035 → 0.111 °/m — both far inside the 0.565 budget, so the trade is
  on a metric already met against the one that actually fails goals. Survives all three
  phases (0.07 / 0.19 / 0.13 °/m). **This is `trot_straight.md` §4's idea working**: there
  the constant was in placement space and the P loop fought it; here they are the same
  channel and add.

## 4. Control group

WALK on the same rig, both arms: **0.24 °/m, yaw +0.06, v_x 0.235, stride 1.35 — identical.**
The term is TROT-only by table (`YAW_MOMENT_CAP_NM`), and `--yaw-moment off` reproduces
`heading_hold.md`'s TROT row exactly (3.20, +2.12, 0.663, 1.56).

## 5. Wiring, both verified end to end

- **`run_benchmark.py --heading` / `--heading-cap`.** The 0.98 / 1.11 scores were the bare
  Raibert law while every calibration sweep runs with heading hold, so they were never
  numbers about the same controller. Each robot holds the heading **it** settled at.
  Asking for `--heading` on a skill with no measured cap is **refused** — a cap of 0 would
  stamp 200 rows `heading=heading-only` on a run where the term was identically zero.
  Smoke (6 cells, TROT): banner fires, both arms score 1.00, which at 6 cells is the floor
  and not a comparison.
- **`run_planner_replay.py --entry-turn measured`**: prints `entry frame 6 <- MEASURED,
  overriding the rule's 24`. Default stays `rule`.

## 6. A guard that was always open

The first version read the hip effort limit from `robot.data.joint_effort_limits` and got
**1e9** — PhysX's limit, not the one an explicit `IdealPDActuator` enforces in Python. The
headroom check accepted any cap and the saturation counter could never fire. Caught by the
banner printing it next to a peak of 11.79 N·m. All three harnesses now read it off the
**actuator** and refuse `--yaw-moment` outright if they cannot.

Related: the first ladder run recorded `tau_hip_max_nm` only in the arm with the term on,
which made the two rows at the 23.70 clip unreadable. With the control column added, peak
hip torque is **higher with the couple off** (23.66 / 22.77) than on (18.03 / 20.31) — the
terrain puts the hips near their clip, not the couple.

## 7. What to do next, in order

1. **Phase 17's 3.85–3.88 m**, identical at all fifteen levels. A distance that does not
   respond to the level is not a step limit. That is where `STEP_TROT_MAX` actually dies
   and nothing has looked at it.
2. **The full 200-cell benchmark with `--heading heading-only`**, paired with off. The
   flag is wired and smoke-tested; the run has not been done.
3b. **`effort_limit`** (`jump_torque.md`, three options) is still the user's call and still
   gates JUMP entirely.
5. RUN, JUMP and WALK-4cm were **not touched this session** — deferred by the priority
   order, not by a finding.

## 8. Untouched

`--effort_limit` unchanged. No clip was edited. GPU 0 never touched, GPU 1 never touched.
