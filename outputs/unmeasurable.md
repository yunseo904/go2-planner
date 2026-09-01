# What is not measurable, and why — the calibration's negative results

Four of the ten `CALIBRATION_NEEDED` fields cannot be measured at all in the current state,
and the reasons are different in kind. Recording them here so a zero in a results CSV is
never read as a capability of zero. Two of these were asked to be closed as "not
measurable"; the other two are the same shape and belong beside them.

| field | skill | status | why |
|---|---|---|---|
| `skill.STEP_RUN_MAX` | RUN | **not measurable** | the gait collapses in 1.13 s on flat |
| `skill.STEP_JUMP_MAX` | JUMP | **not measurable** | the clip needs 1.10–1.20× the actuator's torque limit |
| `skill.STEP_TROT_MAX` | TROT | **partly measured — see the note below** | as a GOAL score, still not measurable; as a crossing, 0.04–0.06 m |
| `robot.FOOT_SPAN_X` | WALK | **not measurable** | WALK does not fall on any gap and never arrives either |

---

## RUN — 1.13 s on flat ground, and the clip has no flight phase

`outputs/run_collapse.md`. RUN falls at **1.13 s** on flat ground, with or without foot
placement — identical to the control step. It is not a step limit that is missing; there is
no gait to put a step under.

Two measured facts, from different instruments:

- **It rolls out, from the first cycle.** \|roll\| goes 6.45° → 27.85° → 61.0° (the fall
  gate) while the base sinks monotonically from 0.302 m to 0.163 m. It does not trip; it
  never gets its weight up.
- **The rear-left leg is never loaded.** 12.6 N mean against 30–34 N on the others, loaded
  19.3% of the time against 40–51%. The support polygon is missing a corner for most of the
  run.

And the clip itself is short of what a run needs: `skill_clips.md` finds **no flight phase**
in the recording, and `outputs/turn_target.md`'s sibling analysis found no footfall for
foot placement to place. RUN is out of `planner.skills.SUPPORTED` and the planner refuses it
by name at runtime.

**What would have to change:** a recording with a flight phase in it, or an explanation for
the unloaded rear-left. Neither is a calibration task.

## JUMP — the recording used more torque than the sim robot is allowed

`outputs/jump_torque.md`. Measured offline from the log, no simulator:

| joint | peak logged | p99.9 | sim `effort_limit` | ratio |
|---|---|---|---|---|
| hip | 19.17 Nm | 18.05 | 23.70 | 0.81× |
| thigh | 26.15 Nm | 25.69 | 23.70 | **1.10×** |
| calf | **54.67 Nm** | 50.18 | 45.43 | **1.20×** |

The p99.9 is over the limit too, so it is the shape of the push-off and not one spike.

**`effort_limit` is not being raised, and that is a decision rather than an oversight.**
CLAUDE.md §2: both experimental arms run the same articulation, and this config is the one
the E2E policy is trained and scored on. A JUMP that only clears an obstacle with the limit
relaxed is not a skill the rule planner may claim against a policy that never got the
relaxation. The consequence is accepted: the planner's jump gate has an empty domain, and
tasks whose only answer is a tall step score zero end to end — "tried and failed" is the
result, per the same section.

## TROT — reaches the obstacle, misses the goal

`outputs/trot_straight.md` §5–6. With the step-length bias fed forward, TROT crosses the
obstacle line and runs to 7.4 m at 1.95 °/m — its best heading number — and still scores
0/5, because goal 2 is a 0.35 m radius and it arrives **1.64 m to the side**. The residual
is lateral offset, not heading error, and a heading-hold controller has no term for it by
construction. Cross-track control is what would move this, and it needs a path from the
planner, which the low level deliberately does not have.

## FOOT_SPAN_X — 60 gap runs, zero falls, zero arrivals

`outputs/calibration_grid.md`. WALK does not fall on a single gap probe at any width and
ends on average 2.21 m from a goal it started 3.95 m from. The probe is not being failed by
the obstacle; the robot is not going where the goal is. Same cause as TROT's, one gait
milder: WALK's curvature is inside the budget now (0.24 °/m) but the calibration runs that
produced those 60 rows predate heading hold.

**This one is the cheapest to fix and has not been re-run.** WALK holds a line; the gap
family should be repeated with `--heading heading-only` before `FOOT_SPAN_X` is called
unmeasurable.


---

## Closing status, 2026-09-01

Decided with the user: **RUN and JUMP are closed as unresolved and not carried further.**
Both causes are established, neither is a missing measurement, and neither is fixable
inside the constraints this project runs under.

### RUN — the clip has no ballistic phase, and that is upstream of everything else

`run_collapse.md`, `run_swing_gate.md`. The base never becomes ballistic, and the three
downstream candidates have each been excluded on their own:

- **the swing gate** (`--foot-swing-source sim`) moves the correction (cap-hit 26.5% →
  36.8%) and changes the collapse time by **fourteen decimal places of nothing**;
- **the loop seam** is not it either — the uncut full-session replay already on disk
  survives 10.28 s, but at duty 0.96 and 0.035 m/s, i.e. by standing rather than running;
- **collapse time is 0.93–1.64 s across three entry phases**, so the 1.13 s quoted
  everywhere is one draw, and no arrangement of the remaining knobs moves the distribution.

What is missing is in the recording: the clip is a duty-0.31 gait whose *body* never leaves
a ballistic arc in replay. That is a re-recording, not a control fix, and re-recording is
not available here.

### JUMP — the sim's torque limit is the spec sheet, and the real robot exceeded it in bursts

`jump_torque.md`. `effort_limit` 23.70 / 45.43 N·m **is the Go2 spec sheet**, and the ratio
proves it: 45.43/23.70 = 1.9169 against the motor's 30.1/15.7 = 1.9172, the same motor
behind a 1.917:1 knee reduction. The actuator is an `IdealPDActuatorCfg`, a static clip
chosen *over* `DCMotorCfg` precisely to avoid torque–speed derating, so the sim robot is
already more permissive at speed than a real one.

The log's 54.67 N·m is **LowState's measured torque, not a command** — the calf's peak
*command* is 38.2, inside the limit — and it reproduces on all five takes. Time above the
limit is **7–51 ms per jump, longest single bout 11–23 ms**: a momentary peak, not the
shape of the push-off.

So the sim robot is torque-short by 1.10–1.20× for a few tens of milliseconds per jump, and
closing that gap means raising `effort_limit` above the spec sheet — which changes the E2E
arm too and is the user's decision, not a measurement. **Not taken. JUMP stays unresolved.**

### STEP_TROT_MAX — the row above changed, and the distinction matters

The original entry said "arrives past the obstacle, 1.6 m beside the goal". That reading was
built on `--foot-len-bias`, which `trot_straight.md` §4b later showed does not replicate.
What is now measured (`trot_yaw_moment.md` §6) is narrower and firmer:

- as a **crossing** — does the base get past the riser — TROT is **6/6 at 0.02 m, 4/6 at
  0.04 m, 0/6 at 0.06 m and above**. A bracket of **0.04–0.06 m**, monotone, over three
  entry phases and both intervention arms.
- as a **goal-2 score** it remains **not measurable**: one cell of ninety reaches goal 2,
  and what stops the rest is the rear pair against a riser taller than the 23 mm foot
  radius (`lip_failure.md` §1's mechanism, now measured on TROT).

**The bracket is not in `planner/config.py`.** `STEP_TROT_MAX` stays 0.08 m with provenance
`CALIBRATION_NEEDED`, by the user's decision: three entry phases is a bracket, not a
calibration, and a threshold becoming a planner guarantee is a decision rather than a
measurement.

### Frame 13 — an open question, deliberately parked

TROT clip frame 13 survives 40 s on the flat rig and **never leaves the spawn on the grid**
(0 of 15 cells travel more than 0.6 m). The two harnesses disagree about the same phase.
Parked, not chased.
