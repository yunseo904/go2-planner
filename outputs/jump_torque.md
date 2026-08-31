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
