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
