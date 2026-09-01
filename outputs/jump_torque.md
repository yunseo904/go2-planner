# JUMP: the sim robot is 1.10–1.20× short of the torque the recording used

**Status: unresolved, to be retried. Not closed.**

`front_jump` is the only vertical capability in the library and the only answer the
planner has to a step. It is currently not executable in replay, for a reason that is
measured and is not about foot placement.

## The measurement

`scripts/verify_skill_replay.py --headroom --clip JUMP`, offline, no simulator:

| joint | peak logged | p99.9 | sim `effort_limit` | peak / limit |
|---|---|---|---|---|
| hip | 19.17 Nm | 18.05 | 23.70 | 0.81× |
| thigh | 26.15 Nm | 25.69 | 23.70 | **1.10× OVER** |
| calf | **54.67 Nm** | 50.18 | 45.43 | **1.20× OVER** |

The real robot applied more torque through the push-off than `IdealPDActuator` will let
the sim robot apply. The gap is not marginal and it is not noise: the p99.9 is over the
limit too, so it is the shape of the push-off and not one spike.

## Why the limit is not being raised

Raising `effort_limit` for the jump would measure a robot the evaluation never
instantiates. The whole comparison rests on both experimental arms running the same
articulation, and this config is the one the E2E policy is trained and scored on
(CLAUDE.md §2: upstream bugs and upstream limits apply to both arms equally). A JUMP
that only clears an obstacle with the limit relaxed is not a skill the planner may
claim.

So the gap is recorded and JUMP stays out of the executable set.

## What this blocks

- The planner's jump gate has an empty domain in practice. `STEP_JUMP_MAX` is a
  `CALIBRATION_NEEDED` placeholder and cannot be calibrated on a skill that cannot run.
- Every task whose only answer is a step taller than the walking limit is currently
  unsupported end to end. Per CLAUDE.md §2 those are not excepted out — they run and
  score zero, and "tried and failed" is the result.

## Ways back in, none attempted

1. **Play it in TORQUE mode with the log's own feed-forward** and accept a short jump —
   measure how short. The clip is already non-position-controlled at kp 13/3/2 for RUN;
   JUMP's gains and `tau_ff` are in the archive and the harness supports the mode. This
   measures what the *sim* robot's jump is, which is the honest number for a sim planner
   even if it clears less than the real one.
2. **A different jump source.** `front_pounce` is a lunge, not a ballistic flight
   (`skill_profile.md` §3), so it is not a substitute. Nothing else in the 36 sessions
   leaves the ground.
3. **Ask whether the fork's `effort_limit` is right.** 23.70 / 45.43 Nm are what the
   eurekaverse Go2 config declares. If those numbers are below the real Go2's actuator
   spec, the sim robot is wrong rather than weak — but changing them changes the E2E
   arm too and is a decision about the experiment, not a fix to apply quietly.

Related: `outputs/run_collapse.md` (RUN, also unresolved, different cause — no flight
phase in replay at all).

---

# Way back in #3, answered: the limits are the spec sheet, and the excursion is 20 ms

Asked: whether `effort_limit` is right, and what the Go2's calf joint spec is.
**`effort_limit` is not being raised and nothing below changes it** -- CLAUDE.md §2 still
governs, and the decision is the user's. This is the evidence for making it.

## The config is the spec sheet, and it is internally consistent

`extreme-parkour/legged_gym/envs/base/legged_robot_config.py`, `UNITREE_GO2_CFG`:

| actuator group | effort limit | velocity limit | comment in the file |
|---|---|---|---|
| `.*_hip_joint`, `.*_thigh_joint` | 23.70 N·m | 30.1 rad/s | "Torque/velocity limits from the Go2 spec sheet" |
| `.*_calf_joint` | **45.43 N·m** | **15.7 rad/s** | same line |

45.43 / 23.70 = **1.9169** and 30.1 / 15.7 = **1.9172**. The calf is the same motor behind
a ~1.917:1 knee reduction -- torque up by that factor, speed down by it. **These are not
arbitrary numbers and they are not a fork's invention**; the ratio is the check.

The actuator is `IdealPDActuatorCfg`, a **static** clip. The file says `DCMotorCfg` was
tried first and rejected because "its torque-speed derating is an extra constraint the
original never had". That matters for the interpretation: **the sim is already more
permissive than a real motor at speed.** A real Go2 knee turning fast cannot make 45.43
N·m either -- that is its rated point, not a floor. So "the sim clips flat and cannot hold
a momentary peak" is only half right: the clip is flat *at the rating*, and what the real
robot has that the sim does not is headroom **above** the rating, which comes from thermal
and current margin rather than from a curve the sim has flattened.

## The log's 54.67 N·m is a measurement, not a command, and it reproduces on all five takes

The curated jump logs carry two torque families: `*_tau` from LowState (the motor's own
torque estimate) and `*_tau_ff` from LowCmd (the commanded feed-forward). Peak |value| over
all four legs, per session, all five `front_jump` recordings:

| joint | peak `tau` (LowState) over 5 sessions | peak `tau_ff` (LowCmd) | limit | tau / limit |
|---|---|---|---|---|
| hip | 15.73 – 22.24 (median 18.78) | 11.81 | 23.70 | 0.94× |
| thigh | 26.15 – 28.00 (median 27.16) | **38.46** | 23.70 | **1.18×** |
| calf | **53.82 – 56.19** (median 54.67) | 38.20 | 45.43 | **1.24×** |

Three things:

* **It is the measured torque, not the command.** The calf's peak *command* is 38.2 N·m,
  comfortably inside 45.43; the 54.67 is what the joint actually produced once the PD term
  was added to it. So the number is not an artefact of reading the wrong column.
* **It reproduces.** All five takes land in 53.8-56.2 on the calf and 26.2-28.0 on the
  thigh. Not one spike, not one session.
* **The thigh's commanded `tau_ff` is 38.46 on every session, to the digit** -- that is a
  clamp inside Unitree's own sport controller, and it sits *above* the 23.70 the sim
  allows on that joint.

## It IS a momentary peak -- and the earlier argument against that does not hold

Time spent above the published limit, per session, at the log's ~420 Hz:

| joint | samples over | total time | longest single bout |
|---|---|---|---|
| thigh | 12 – 21 | 0.028 – 0.051 s | **0.017 – 0.023 s** |
| calf | 3 – 13 | 0.007 – 0.030 s | **0.007 – 0.017 s** |

**7 to 51 ms per jump, in bouts of 11-23 ms.** That is a momentary peak, and the user's
reading of it is right.

**This withdraws the argument at the top of this file.** "The p99.9 is over the limit too,
so it is the shape of the push-off and not one spike" does not follow: the p99.9 of a
16 s recording at 420 Hz is the top ~7 ms of samples, so a single 20 ms excursion puts the
p99.9 over the limit by itself. The p99.9 was consistent with a spike all along and was
read as evidence against one. Same shape as CLAUDE.md §6.5 -- a statistic that is *almost*
the one the argument needed.

## What this leaves as the decision

The sim robot is faithful to the published Go2 and the real robot exceeded the published
Go2 by 18-24% for about 20 ms per jump. So the choice is not "fix the sim" versus "leave
it"; it is:

1. **Leave `effort_limit` at the spec.** JUMP stays unexecutable, its gate stays empty,
   and tasks whose only answer is a tall step score zero end to end -- which CLAUDE.md §2
   already accepts as a result. This is the status quo and no code has been changed.
2. **Raise it to the measured peak (thigh 28.0, calf 56.2) for BOTH arms.** Defensible as
   "the real robot demonstrably does this", but it re-trains nothing -- the E2E policy was
   trained and scored at 23.70/45.43, so scoring it at a higher limit compares a policy to
   a robot it never had. That is a bigger change than it looks.
3. **Model the headroom rather than the level** -- a short-duration overload allowance.
   Physically the right shape and not available in `IdealPDActuatorCfg`.

Torque mode has been run, for the record: `outputs/reproduction_check.csv` has JUMP at
`mode_used=torque`, `torque_saturated_frac=0.671`, terminating at 1.96 s with a mean base
height of 0.19 m. It saturates the clip two thirds of the time and sinks. That is way back
in #1 attempted, and it does not jump.

---

# Rated or peak? — answered: **they are the peak, and Unitree publishes no rated figure**

Asked 2026-09-01: whether 23.70 / 45.43 N·m are continuous (rated) or instantaneous
(peak) numbers, on the reasoning that a motor's peak is usually 1.5–3× its rating, so a
11–23 ms excursion in the real-robot log could be ordinary peak use rather than the robot
exceeding itself.

**Investigation only. Nothing has been changed and `effort_limit` still reads 23.70 /
45.43.** The decision is still the user's.

## 1. Unitree's own labels

| source | figure | Unitree's own wording |
|---|---|---|
| GO joint motor page (the hip/thigh number) | **23.7 N·m** | "**Maximum** Torque: 23.7NM", at "Reduction Ratio: 1:6.33", "Maximum Rotational Speed: 30rad/s (@24VDC)", "Measured at 24°C" |
| Go2 product page (the calf number) | **≈45 N·m** | "**Peak** Joint Torque: Approx. 45N.m", footnoted "the maximum torque in the table refers to the maximum torque of the **largest** joint motor; the actual maximum torque varies for the 12 joint motors" |

The fork's config carries 23.7 @ 30.1 rad/s and 45.43 @ 15.7 rad/s, commented "taken from
spec sheet".  Those are those two published numbers.  **Both are labelled maximum/peak.
Unitree does not publish a continuous or rated torque for these joints at all.**

So the premise the question rests on does not hold here.  There is no rating underneath
these numbers for a peak to be 1.5–3× of — the config is *already* at the peak, and the
log's 54.67 N·m calf is **1.20× above the published peak**, not inside an unpublished
headroom above a rating.  Same for the thigh at 1.18×.

The footnote is worth keeping: "≈45" is the *largest* joint motor's peak and Unitree says
the other eleven differ.  45.43 for the calf is therefore the most generous reading of
that line, not a conservative one.

**This withdraws one sentence of the previous section.**  "A real Go2 knee turning fast
cannot make 45.43 N·m either — that is its rated point, not a floor" reads 45.43 as a
rating.  It is not one; Unitree labels it a peak.  The rest of that section stands — the
1.9169 / 1.9172 ratio check still shows the two numbers are one motor behind two
reductions, and the 20 ms excursion is still a 20 ms excursion.  What changes is only what
the limit is the limit *of*, and that is the whole question being asked here.

**Two things this does not settle.**  The peak is quoted at 24 V and 24 °C; bus voltage
and winding temperature move it, and a warm pack at full charge is not the datasheet
point.  And the log's 54.67 is `tau` from LowState — the motor's own current-derived
*estimate*, with its own error, not an instrumented torque.  "20% over the published
peak for 20 ms" is a statement about the published number.

## 2. The USD asset does not carry the limit at all

The joint table Isaac Lab prints at startup gives, for all twelve joints:

    effort limit (PhysX)   1.0e+09
    velocity limit         30.100 (hip/thigh)   15.700 (calf)

So the asset is effectively unlimited and the velocity limits come from the cfg.  The
23.70 / 45.43 clip exists **only** in `UNITREE_GO2_CFG`'s `IdealPDActuatorCfg.effort_limit`,
in Python.  In Isaac Lab 3.0's split, `effort_limit` clips the actuator's output and
`effort_limit_sim` is the one PhysX enforces; the fork sets the first and leaves the
second at the asset's default.

This is the same trap `SESSION_STATE.md` §6 already hit from the other side — reading
`robot.data.joint_effort_limits` returns that 1e9 and not the number the robot is
actually held to.

## 3. Isaac Lab has no actuator that allows a momentary overshoot

Everything in `isaaclab/actuators`: `ImplicitActuator`, `IdealPDActuator`, `DCMotor`,
`DelayedPDActuator`, `RemotizedPDActuator`, `ActuatorNetLSTM`, `ActuatorNetMLP`.

Every one of them ends in `ActuatorBase._clip_effort`, which is

    torch.clip(effort, min=-self.effort_limit, max=self.effort_limit)

— static, symmetric, and with no notion of time.  The actuator *networks* clip too, so
even a net trained on real Go2 logs is capped at the same wall.

`DCMotor` is the only model that overrides it, and its dual limit is **not** the one the
question is after: `saturation_effort` is the stall torque of a linear four-quadrant
torque–speed curve, so the clip tightens as the joint turns faster.  That is peak **vs
speed**, not peak **vs time**, and at speed it is *stricter* than the flat IdealPD clip —
which is exactly why the fork's own comment says `DCMotorCfg` was tried and rejected
("its torque-speed derating is an extra constraint the original never had").

So option 3 in this file — model the headroom rather than the level — needs a custom
`ActuatorBase` subclass carrying an I²t or duty budget.  There is nothing off the shelf.

## 4. What this does to the decision

The user's framing was: if the clip is the *rated* value then the sim is conservatively
wrong, and correcting it is plant matching of the same kind as friction 0.5 → 1.3.

**The evidence says the clip is not the rated value, so it is not the same shape.**  The
friction change replaced a default with a measured property of the plant.  Raising 45.43
to 56.2 would replace a manufacturer's *published maximum* with a log that exceeds it —
defensible ("the real robot demonstrably did this"), but it is option 2 above, unchanged,
and it still re-scores the E2E policy on a robot it was never trained on.  Still the
user's call, and now with the labels resolved.

## 5. It would not open the pit courses either

Even granting the most permissive torque decision, `front_jump` does not become a gap
crossing.  It is a standing hop: flight 0.451 ± 0.028 s, apex rise 0.250 ± 0.031 m,
**horizontal travel 26 ± 4 mm** (`skill_profile.md`).  More torque buys apex, and apex is
already not the binding quantity — there is no forward velocity in the recording for extra
force to scale.

Measured on the legged_eval grid, longest pit run along the centre lane (y = 2.0 m), at
difficulty 0.0 / 0.44 / 1.0:

| course | easiest | middle | hardest |
|---|---|---|---|
| `stepping_stones_randomly_arranged` | 0.10 m | 0.15 | 0.15 |
| `stepping_stones_cylinder` | 0.10 | 0.30 | 0.40 |
| `flat_circle_jump` | 0.25 | 0.65 | 1.05 |
| `box_jump_uneven` | 0.50 | 0.70 | 0.90 |
| `box_jump_even` | 0.55 | 0.70 | 0.85 |
| `bump_jump` | 0.80 | 0.90 | 1.10 |
| `sideways_ramp` | 1.65 | 3.25 | 5.15 |

Against 26 mm, the *easiest* gap in the set is 4× too wide and the hardest is 200×.
**10 of the 20 courses carry a 1 m pit** (`--list-courses` marks them PIT), so half the
benchmark stays at zero whatever is decided about `effort_limit`.  What is missing is a
forward-travelling jump in the clip library, not a torque limit — which is
`SESSION_STATE.md` §8's first item ("the library is the ceiling, not the rules"), and the
torque question is not on the path to it.

Sources for §1: <https://www.unitree.com/mobile/go1/motor/>, <https://www.unitree.com/go2/>.
