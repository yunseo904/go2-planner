# Calibration on the grid: 210 runs in one scene, and not one threshold

The grid harness works at full scale. The calibration it produced is **empty**, and the
reason is the heading problem, not the probes.

`scripts/run_calibration_grid.py --reps 5 --skip-unsupported --foot-comp on`, flat CPU,
no GPU. 42 runs (probe × supported parameter, from the archive's 42 distinct probes and
72 probe/parameter pairs before RUN and JUMP are dropped) × 5 repeats = **210 rows, one
scene build, one articulation of 42 robots stepping together**. The per-episode harness
would have needed 210 process starts and could not get past its first episode at all.

## What the repeats vary, and why they have to vary something

Physics here does not vary: friction is fixed at the env's midpoint rather than sampled
per episode, so five identical repeats of a deterministic sim give five identical rows.
The original protocol's repeats assumed the env's randomisation.

What *is* arbitrary is where in its cycle a clip starts — whoever cut the clip chose
frame 0 — so repeat *r* enters every clip at frame `round(r·n/reps)`: 0, 7, 15, 22, 30 for
a 37-frame WALK. The outcomes do differ between repeats (25–29 of 42 fell), so the
variation is real, and a limit that only held at one entry phase would not have survived.

## The result

| family | skill | n | fell | final distance to goal 2 |
|---|---|---|---|---|
| gap | WALK | 60 | **0%** | 2.21 m (min 1.63) |
| step_up | WALK | 75 | 85% | 2.26 m |
| step_up | TROT | 75 | **97%** | 3.59 m |

    parameter               skill   value   top all-pass   first fail
    robot.FOOT_SPAN_X       WALK     none          none         0.050
    skill.STEP_TROT_MAX     TROT     none          none         0.020
    skill.STEP_WALK_MAX     WALK     none          none         0.020

**Not one level of any family passed all five repeats**, so no value can be derived and
nothing is proposed for `config.py`. Two separable causes, and they are different
problems:

**TROT falls on a 2 cm step, every repeat.** That is not a threshold that needs a finer
ladder; it is below the ladder's floor. The honest reading is that replayed TROT has no
step capability at all, which is consistent with everything else measured about it — it
holds a gait on flat ground and nothing more.

**WALK does not fall on the gap probes and still never arrives.** Zero falls in 60 runs,
and it ends on average **2.21 m from a goal it started 3.95 m from**. It is not being
stopped by the obstacle; it is not going where the goal is. WALK's measured curvature is
11.5–13.0 °/m (`outputs/heading_candidates.md`), which over 3.95 m is **about 47° of
heading error** — far outside the 0.35 m goal radius, and the 20 s budget leaves only
15% margin over the 17 s that 3.95 m at 0.233 m/s needs even in a straight line.

So the calibration is blocked behind the heading corrector, which is measured but not
built. Until a gait can cross 4 m of flat ground and arrive, a probe cannot distinguish
"the step was too tall" from "the robot walked past".

## What this does and does not establish

- **Establishes**: the grid mechanism runs at scale (42 robots, 1.06 M faces, 5 repeats,
  one scene, no restarts), the repeat scheme produces genuine variation, and the protocol
  reduction runs end to end and correctly reports *no value* rather than inventing one.
- **Does not establish**: any threshold. `STEP_WALK_MAX`, `STEP_TROT_MAX` and
  `FOOT_SPAN_X` stay `CALIBRATION_NEEDED`.

## Before re-running

1. **Heading first.** Without it the goal test measures curvature. The two mechanisms are
   already measured (`heading_candidates.md` §2) and the controller is one term.
2. **Then reconsider the time budget.** 20 s was written for a faster gait; WALK needs
   17 s for 3.95 m in a straight line and any deviation exceeds it. It should be derived
   from the skill's own measured speed rather than shared across skills.
3. **A finer step ladder below 0.02 m is not worth cutting yet** — TROT failing at the
   floor is a statement about TROT, and re-cutting the frozen probe archive to chase it
   would change the archive before the thing being measured is fixed.
