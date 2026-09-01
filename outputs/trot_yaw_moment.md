# TROT's heading: the authority was never in the feet

`outputs/trot_straight.md` closed with heading authority "spent" and named cross-track
control — a position error against a path, which only the planner has — as the one thing
left. That conclusion was right about the *routes it had tried* and wrong about the
premise underneath all three of them.

All three were **foot placement**:

- **(a)** differential lateral placement, capped at ±0.02 rad because TROT falls in both
  directions at ±0.04 (`heading_candidates.md` §2). At 155–175 °/s per rad that ceiling
  is worth about 3.1 °/s against a +5.32 °/s drift.
- **(b)** differential step length, strong one way and destructive the other.
- **(c)** either of them fed forward as a constant — a coin flip over fifteen paired
  cells (`trot_straight.md` §4b).

They share a bound, and it is not the actuator's: it is **how far a trot's footfall can
be displaced before the trot stops being one**. Widening it was already measured to make
heading worse (`lip_failure.md` §3). Three routes into the same wall is one route.

This is a fourth thing, and it is on the other side of that wall. It **moves no foot**.

---

## 1. The mechanism, in the robot's own numbers

The hip is the abduction joint — `heading_candidates.md` §1 settled that this robot has
no yaw joint at all — and `lever_i`, the hip-to-foot vertical drop, is exactly the moment
arm that turns a hip torque into a lateral force at the foot. On a **stance** leg the
foot is on the ground, so the load path is through the contact and the torque buys a
ground reaction force:

    f_y,i = -tau_i / lever_i          on the body
    M_z   = sum_i x_i * f_y,i         x_i = the hip's fore-aft offset, body frame

Take `tau_i = c * sign(x_i)` and every stance leg contributes to `M_z` with the **same
sign**:

    M_z = -c * sum_over_stance |x_i| / lever_i

That last property is not cosmetic. A trot has only a **diagonal pair** on the ground at
any moment and the pair alternates twice a cycle; a rule needing all four feet would
produce a moment that switched off twice a cycle. This one does not care which legs are
down — measured at **1.241 N·m of yaw couple per N·m of hip torque on a diagonal pair**,
2.483 with all four, identical for both diagonals.

Two side effects, both bounded in the self-test rather than asserted away:

- **net roll moment is exactly zero** (`sum_i lever_i * f_y,i = -c * sum sign(x_i)`), for
  any stance set with as many front legs as rear.
- **net lateral force is not zero** and the first draft of `sim/yawmoment.py` claimed it
  was. It cancels only if the front and rear levers are equal, and they are 0.306 and
  0.316 m. What is left is that **3.2% mismatch and nothing else** — 0.207 N against
  6.433 N of force in play on a diagonal pair, a couple-to-residual ratio of 12 N·m per N.
  The self-test now bounds it against the lever asymmetry, which is the honest test.

### Nothing new is asked of the simulator

The channel was probed a week ago and written down. `outputs/isaac_actuator_probe.json`:
the actuator is an explicit `IdealPDActuator`, `set_joint_effort_target` exists,
`q3_effort_is_additive` is **true** (5 N·m commanded produced exactly 5.0 N·m of extra
torque), and `q3_effort_replaces_pd` is **false**. The feed-forward adds to the PD that
plays the clip, and the actuator clips their **sum** at the hip's 23.70 N·m.

### What it does not see

The stance mask is the **clip's own contact channel**, phase-locked — the same gate
`ClipPolicy` already uses for foot placement, and the same information class. The sim's
contact sensor is deliberately not an input: a term that changed with what the feet
actually hit would be a contact reflex, which the skill layer is forbidden. Heading
error, hip geometry, clip phase. No terrain, no depth, no goal.

---

## 2. The actuator's gain, measured open loop before any loop was closed

The methodology is `heading_candidates.md` §2's, for the reason given there: a controller
that fails tells you almost nothing about which part failed, so measure the actuator
first. Rig: `run_planner_replay.py --schedule hold --initial TROT`, 40 s, flat, CPU —
the rig `heading_hold.md`'s 3.20 °/m came from. **Heading hold off**, so nothing fights
the constant. Baseline +5.17 °/s.

| c (N·m) | −2.0 | −1.0 | −0.5 | −0.25 | 0 | +0.25 | +0.4 | +0.5 | +0.6 | +0.75 | +1.0 | +2.0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| yaw °/s | +24.19 | +7.50 | +8.82 | +6.86 | **+5.17** | +2.98 | +1.34 | **−0.12** | −0.87 | −2.31 | −4.13 | −15.54 |
| v_x | 0.323 | 0.442 | 0.662 | 0.663 | 0.664 | 0.663 | 0.662 | 0.661 | 0.660 | 0.658 | 0.653 | 0.639 |
| stride Hz | 1.47 | 1.52 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 |
| survived | **2.28 s** | **3.58 s** | 40 s | 40 s | 40 s | 40 s | 40 s | 40 s | 40 s | 40 s | 40 s | 40 s |

- **The sign came out as derived**, which is worth saying because it is the first time in
  this project it has (`heading_hold.md` had to flip its heading term after a run, and
  `sim/footcomp.py` carries the note). Positive c → negative yaw moment.
- **It crosses zero, at c ≈ 0.49 N·m**, interpolating 0.4 → +1.34 and 0.5 → −0.12.
- **Roughly 9–10 °/s per N·m**, safe to at least ±2 N·m in the working direction: **20 °/s
  of demonstrated authority against foot placement's 3.1**, six times as much, and the
  cap could go to 11.85 N·m before the actuator guard refuses it.
- **It costs nothing measurable.** Across the entire working range v_x moves 0.664 → 0.661
  and stride does not move at all. Compare (a), whose ±0.04 rad costs a quarter of the
  forward speed *and* falls.

### The one asymmetry, stated rather than buried

The two large **negative** amplitudes fall, and −1.0 (+7.50 °/s, fell at 3.58 s) falls
while +2.0 (−15.54 °/s, survived 40 s) does not — so it is **not** simply "too much yaw".
Something about driving the drift further in its own direction ends the run, and this is
not yet explained. It is also not on the path: TROT's drift is one-signed at +5.17 °/s and
the correcting direction is the surviving one. But a bidirectional controller does have
to enter the other half occasionally, which is what §3's cap is for, and −0.25 and −0.5
both run the full 40 s — so the safe window is at least [−0.5, +2.0] and the closed loop
below never leaves it.

---

## 3. Closed loop: `c = gain × heading error`

On top of heading hold, same rig, cap 2.0 N·m.

| gain N·m/rad | off | 5 | 10 | 20 | 40 |
|---|---|---|---|---|---|
| **curvature °/m** | **3.20** | **0.01** | **0.02** | **0.03** | **0.13** |
| yaw °/s | +2.12 | +0.01 | +0.01 | +0.02 | −0.08 |
| v_x | 0.663 | 0.664 | 0.664 | 0.653 | 0.645 |
| stride Hz | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 |
| commanded \|c\| max | — | 1.32 | 2.00 | 2.00 | 2.00 |
| at its cap | — | 0.0% | 0.5% | 1.1% | 2.9% |
| peak hip torque | — | 11.95 | 11.97 | 12.01 | 12.07 N·m |
| actuator clipped | — | 0/2000 | 0/2000 | 0/2000 | 0/2000 steps |

**The budget is 0.565 °/m and every arm is inside it by a factor of four to fifty.**

`gain = 5` is the operating point: curvature 0.01 °/m, forward speed and stride
**unchanged** (0.663 → 0.664, 1.56 → 1.56), and the term never reaches its own cap at
all, so the cap is not what is producing the result.

### Why a P law works here when `trot_straight.md` §3 says it cannot

§3's argument is exactly right and this does not contradict it. A proportional law must
hold a standing error to emit an output, and this disturbance is DC — so the question is
only whether the standing error it needs is small enough to *be* small. For the placement
term it is not: it needs 3.7° to reach a cap worth 3.1 °/s against a 5.3 °/s drift, so it
saturates permanently and the heading error grows without bound. For this term, nulling
the drift costs **0.49 N·m against a 2.0 N·m cap**, so at gain 5 the equilibrium sits at
0.49/5 = 0.098 rad and the loop has somewhere to sit. **The law did not change. The
authority did.**

Peak hip torque is 11.95–12.07 N·m against 23.70 and the actuator clipped on **zero** of
2000 control steps in every arm, so none of this is being paid for out of the position
tracking that produces the gait.

---

## 4. The paired 15-cell ladder: a null, and the null explains itself

`trot_straight.md` §4b killed this term's predecessor on the `step_up` ladder, so the
same design was run: `--params skill.STEP_TROT_MAX`, 15 cells at **identical world
coordinates** in both arms, five entry phases, `--heading heading-only` throughout.
Measured with `analyze_heading_ab.py`, over the **approach** — spawn to the obstacle line
— which is the heading question.

| | A, `--yaw-moment off` | B, gain 5 |
|---|---|---|
| curvature, median | 11.97 °/m | 12.06 °/m |
| y drift at the lip, median | 0.77 m | 0.81 m |
| fell | 11 / 15 | 11 / 15 |
| reached the lip | 1 / 15 | 2 / 15 |

**A null, and it should be read next to A's own 11.97 °/m against the flat rig's 3.20 for
the same controller.** On this ladder TROT is being knocked over and the curvature is
measuring the collapse, not the drift — `trot_straight.md` §0's trap, at a different
scale. A heading term cannot help a robot falling off a step, and this one does not.

§5 adds the other half of why: four of the five entry phases this grid uses are phases at
which TROT falls on flat ground in under three seconds.

---

## 4b. Regressions and the control group

- **`--yaw-moment off` reproduces the baseline exactly**: 3.20 °/m, yaw +2.12 °/s,
  v_x 0.663, stride 1.56 — every column of `heading_hold.md`'s TROT row.
- **`sim/yawmoment.py` defaults to a cap of 0**, and the self-test asserts a large heading
  error produces zero torque through the default path.
- The term is **TROT-only** by table (`YAW_MOMENT_CAP_NM`), not by argument: WALK already
  reaches 0.24 °/m without it and TURN is trying to *change* a heading, not hold one.

---

## 5. Entry phases: TROT walks at 4 of its 32, and the couple works at every one of them

`trot_straight.md` §4b is the standing warning here: a controlled A/B at one realisation
is a draw from a wide distribution of differences, not a small version of a controlled
A/B at fifteen. So the flat rig gained `--entry-offset` (a measurement knob — it shifts
where in its own cycle a clip is entered and changes no controller) and **all 32 of
TROT's entry phases were run in both arms**, 64 runs, 40 s each.

**The first result is not about the couple at all.**

| | phases surviving the full 40 s, of 32 |
|---|---|
| `--yaw-moment off` | **4** — 0, 16, 17, 29 |
| `--yaw-moment hold` | **3** — 0, 16, 17 |

**TROT falls at 28 of its 32 entry phases on FLAT GROUND**, with heading hold on and
nothing else, in 0.9–2.9 s. This is SESSION_STATE §7's item 1 ("the cheapest untried
thing left") and it is a larger number than anything the heading work has produced.

It also reframes everything scored on the `step_up` ladder. `entry_frames(rep)` sweeps
the cycle uniformly, so at `--reps 5` it picks **0, 6, 13, 19, 26** — and the only one of
those in the surviving set is **0**. Four of every five `STEP_TROT_MAX` repeats this
project has ever run **started at a phase where TROT falls on flat ground in under three
seconds**, and the ladder was scoring that. It is visible in the old CSV once you know
to look: `steps` is 901 for rep 0 and 68 / 146 / 84 / 57 for reps 1–4, at every level.

### Where the question is askable, the answer is 3 for 3

Comparing the arms at a phase where the robot dies at 1 s is comparing two corpses —
`trot_straight.md` §0's trap, where the whole-run curvature was reading the fall. On the
three phases that survive in both arms:

| entry phase | curvature off | curvature hold | v_x off → hold | stride off → hold |
|---|---|---|---|---|
| 0 | 3.204 | **0.014** | 0.663 → 0.664 | 1.56 → 1.56 |
| 16 | 2.720 | **0.170** | 0.665 → 0.664 | 1.56 → 1.56 |
| 17 | 2.814 | **0.004** | 0.665 → 0.665 | 1.56 → 1.56 |

**0 of 3 inside the budget without it, 3 of 3 with it**, better at every phase, and
forward speed and stride do not move at any of them.

### The run it cost, and why that one is legible

**Phase 29 survives 40 s with the term off (1.83 °/m) and rolls at 2.54 s with it on.**
That is one run lost of the four the baseline had, and it is reported rather than
dropped. The cap-hit column says what happened: on phases 0/16/17 the loop reaches its
2.0 N·m cap on **0.0%** of stance-leg steps, and on phase 29 it reaches it on **19.9%**.
The failure is the loop saturating, which is the same shape of failure `lip_failure.md`
§3 found when the placement cap was widened — a saturated corrector is not a corrector.
A lower cap or a rate limit is the obvious next thing and neither was tried.

---

## 6. What this does NOT do, and it is a different quantity

**It does not fix the ladder.** §4's paired 15-cell null stands: on `step_up`, curvature
is 11.97 °/m with the term off against the flat rig's 3.20 for the same controller,
because there TROT is being knocked over at ~7 s and the curvature is the collapse. A
heading term cannot help a robot falling off a step.

**It does not remove the lateral offset.** On flat at gain 5 the robot ends 3.49 m to the
side over a 26.6 m path — down from 19.09 m, but not gone. The reason is exactly
`trot_straight.md` §3's, now working in the project's favour instead of against it: a P
law holds a standing error, and here that error is small enough to sit inside the cap
rather than pin it. Nulling the drift costs 0.49 N·m, so at gain 5 the equilibrium is
0.49/5 = 0.098 rad ≈ 5.6°, and 26 m at a standing 5.6° is about 2.5 m of offset. The
measured heading excursion is 15.4° and the measured offset 3.49 m, which is the same
story.

Two untried things follow directly and neither is a new mechanism: **feed the measured
open-loop zero (0.49 N·m) forward underneath the loop**, so the loop only carries the
residual and the standing error goes toward zero — this is `trot_straight.md` §4's idea,
which failed there because the constant was in placement space and the loop fought it,
and here the constant and the loop are the *same channel* and compose — or simply raise
the gain, which §3 measures at a cost of 2.9% of forward speed by gain 40.

**It does not make TROT survive the ladder** — but chasing why produced a threshold, and
two of the three claims in the first version of this section were misreadings. They are
corrected in place below and recorded in `harness_findings.md` §16.

### The two misreadings, first

**`final_dist_m` is the distance from the robot to GOAL 2 when the run ended**
(`dist = norm(root_pos_w[:, :2] - goal2)`), not the distance it travelled. Read as
progress it inverts every comparison.

**`--entry-offset` counts from a different frame in the two harnesses.**
`run_planner_replay.py` offsets from `level_start`'s choice — **frame 16** for TROT — and
`run_calibration_grid.py` offsets from frame 0. So the flat sweep's surviving offsets
0/16/17/29 are **clip frames 16, 0, 1 and 13**, and the grid re-run at "phases 0, 16, 17"
was at frames 0, 16 and **17 — which is not one of them**.

Those two together produced a specific, plausible and entirely wrong finding: "phase 17
reaches 3.85–3.88 m at every one of the fifteen levels, so it is failing at a fixed
place." Against the trace those robots travel **0.52 m and fall at 1.50–1.52 s**, three
metres short of the riser. The column was flat across levels for the most ordinary reason
available — the terrain was never reached.

### Re-run at the four correct frames, and TROT has a step limit

15 cells × clip frames 0, 1, 13, 16 × both arms, 120 runs, read as **the furthest the base
got relative to the riser** (3.00 m ahead of the spawn) rather than off a goal column.
Frame 13 is dropped: it survives 40 s on the flat rig but never leaves the spawn on the
grid (0 of 15 travel more than 0.6 m), which is its own open question.

| step | fr0 off | fr0 on | fr16 off | fr16 on | fr1 off | fr1 on | crossed |
|---|---|---|---|---|---|---|---|
| **0.020** | +0.63 | +11.20 | +4.63 | +5.75 | +0.55 | +5.07 | **6/6** |
| **0.040** | −0.11 | +0.22 | −0.04 | +0.16 | +0.12 | +0.18 | **4/6** |
| 0.060 | −0.13 | −0.15 | −0.17 | −0.16 | −0.17 | −0.18 | 0/6 |
| 0.080–0.300 | | | | | | | **0/6 at every level** |

(one stray at 0.120, fr1/on, +0.03 m — three centimetres past a riser is a foot over an
edge, not a crossing.)

**`STEP_TROT_MAX` is between 0.04 and 0.06 m.** Monotone, six independent runs per level,
and it is the first ladder response TROT has produced in this project. It is **not**
applied to `planner/config.py`, which carries 0.08 m as a `CALIBRATION_NEEDED`
placeholder: a threshold becoming a planner guarantee is a decision, and this is a bracket
from three entry phases rather than a calibrated number.

**The couple helps at the margin, which is the level where a straightness term should
help**: at 0.040 the off arm crosses **1 of 3** and the on arm **3 of 3**. n = 3 phases,
so it is a direction and not a measurement.

### What the stall actually is, and it is `lip_failure.md` §1's mechanism

The base stops **0.15–0.29 m short of the riser** — about the front hip's own fore-aft
offset, 0.193 m — pitched −20 to −26°, in one of two modes. Loaded foot height is the 10th
percentile of foot z over the last second, so it is the stance value and not a mean mixed
with swing:

| | front loaded z | rear loaded z | reads as |
|---|---|---|---|
| fr0 @ 0.040 | 0.063 | 0.024 | 0.023 + 0.040 — **front feet ON the step** |
| fr16 @ 0.100 | 0.124 | 0.024 | 0.023 + 0.100 — **on the step** |
| fr1 @ 0.120 / 0.140 / 0.160 | 0.144 / 0.164 / 0.184 | 0.024 | 0.023 + step, exactly — **on the step** |
| fr0 @ 0.060 / 0.100 / 0.120 | 0.023 | 0.024 | both pairs on the **ground** — never mounted |

**The rear feet read 0.024 m in every single case**, which is `lip_failure.md` §1's measured
foot radius (23 mm) — they are on the ground, not swinging over anything.

So the dominant failure is: **the front feet mount the step, the rear feet cannot, and the
body stops.** That is exactly the mechanism `lip_failure.md` §1 measured for WALK at 0.04 m
("the rear feet close on the riser from −0.71 m to −0.05 m and stop there"), now measured
for TROT. And it explains the threshold without fitting anything: the loaded foot centre
rides at the foot's own radius, so a 20 mm riser is an edge a foot rolls over and a 40–60 mm
riser is a wall.

**This is also the WALK-4cm question**, which was priority 5 and is answered here by
accident: "the front feet get up on the step but the body stops" is not a WALK quirk, it
is the rear pair against a riser taller than the foot radius, and TROT does the same thing.

**The saturation question from §4 is also settled, by the control column that was missing
there.** Peak hip torque on this terrain is **higher with the couple off** (23.66 / 22.77
N·m) than with it on (18.03 / 20.31), and nothing saturated in any of the 90 runs. The
terrain, not the couple, is what puts the hips near their clip.

### The feed-forward under the loop — run, and it works

The first of those two was one flag combination away, so it was run:
`--yaw-moment hold --yaw-moment-gain 5 --yaw-moment-nm 0.49`, at all three surviving
phases.

| | off | gain 5 | gain 5 + 0.49 N·m FF |
|---|---|---|---|
| curvature (net) | 3.175 | **0.035** | 0.111 °/m |
| heading excursion | 89.9° | 15.4° | **13.4°** |
| **lateral drift** | **19.09 m** | **3.49 m** | **1.25 m** |
| along the launch heading | 15.00 | 26.31 | **26.38** m (of a 26.45 m path) |
| v_x | 0.663 | 0.664 | 0.662 |
| curvature at phases 0 / 16 / 17 | — | 0.014 / 0.170 / 0.004 | 0.07 / 0.19 / 0.13 |

**The lateral offset falls by another factor of 2.8**, and it survives all three phases.
Net curvature goes slightly *up* (0.035 → 0.111) and both are far inside the 0.565 budget,
so the trade is a metric that is already met against the one `trot_straight.md` §5 says
actually fails the goals. **This is `trot_straight.md` §4's idea working**, and the
difference is that there the constant was in placement space and the P loop fought it
(arm D, 9.10 °/m), while here the constant and the loop are the *same channel* and add.

**Cross-track error is still not available to this controller**, by construction, and
`trot_straight.md` §5 is still right that it is what closes the last of the gap. What has
changed is that heading is no longer the binding one.

---

## 7. Flags

`--yaw-moment off|probe|hold`, `--yaw-moment-nm`, `--yaw-moment-gain`,
`--yaw-moment-cap-nm`, `--yaw-moment-skill`, on `run_planner_replay.py` and
`run_calibration_grid.py`. Default off, banner when on, every setting stamped into the
results row, and `tau_hip_max_nm` / `hip_saturated` recorded per row so a run that paid
for its couple out of the gait cannot be reported as one that did not.

The cap is refused above **half the hip's effort limit**, and that limit is read off the
**actuator**, not off `robot.data`. The first version read `robot.data.joint_effort_limits`
and got 1e9 — PhysX's limit, not the one an explicit `IdealPDActuator` enforces in Python
— which left the headroom guard permanently open and the saturation counter unable to
fire. It was caught by the banner printing "against a 1000000000.00 N·m limit" next to a
peak of 11.79.
