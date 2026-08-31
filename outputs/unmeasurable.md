# What is not measurable, and why — the calibration's negative results

Four of the ten `CALIBRATION_NEEDED` fields cannot be measured at all in the current state,
and the reasons are different in kind. Recording them here so a zero in a results CSV is
never read as a capability of zero. Two of these were asked to be closed as "not
measurable"; the other two are the same shape and belong beside them.

| field | skill | status | why |
|---|---|---|---|
| `skill.STEP_RUN_MAX` | RUN | **not measurable** | the gait collapses in 1.13 s on flat |
| `skill.STEP_JUMP_MAX` | JUMP | **not measurable** | the clip needs 1.10–1.20× the actuator's torque limit |
| `skill.STEP_TROT_MAX` | TROT | **not measurable** | arrives past the obstacle, 1.6 m beside the goal |
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
