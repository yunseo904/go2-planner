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


---

# Re-run with heading hold and a speed-derived budget — the first real numbers

Same 42 runs x 5 repeats, now with `--heading heading-only` and a per-robot time budget of
`distance / skill speed x 2` (18.0–43.9 s instead of a shared 20 s).

| family | skill | reached, before → after | fell | final distance |
|---|---|---|---|---|
| gap | WALK | **0/60 → 60/60** | 0% | 2.21 → 6.62 m (past the goal, having reached it) |
| step_up | WALK | 0/75 → **5/75** | 84% | 2.26 → 2.08 m |
| step_up | TROT | 0/75 → 0/75 | 100% | 3.59 → 3.60 m |

**The heading corrector is what unblocked it.** WALK now crosses 4 m of probe terrain and
arrives at the goal on every one of 60 gap runs, where before it arrived at none. This is
the same finding as `heading_hold.md` seen from the other end: the calibration was
measuring curvature, and once the curvature was gone the probes measured what they were
cut to measure.

(A first re-run looked unimproved and nearly went into the record as "heading did not
help". The `--heading` flag was set but `psi_err` was never passed into the per-robot
stepper, so the heading term was identically zero. A flag that silently does nothing is
the same failure class as `harness_findings.md` 9.)

## The config patch

    parameter               skill   value   top all-pass   first fail   monotone
    robot.FOOT_SPAN_X       WALK    0.550          0.600         none       True
    skill.STEP_TROT_MAX     TROT     none           none        0.020       True
    skill.STEP_WALK_MAX     WALK    0.000          0.020        0.040       True

Read carefully, because two of these three are not what they look like:

- **`STEP_WALK_MAX`.** The measurement is clean and it is per level: **0.02 m passes 5/5,
  0.04 m fails 5/5** — a sharp edge with no ragged band. The protocol's one-level margin
  then takes 0.02 down to **0.000**, which is the rule working as written on a ladder whose
  floor is 0.02: there is no level below to back off to. The honest statement is "WALK
  clears 2 cm reliably and 4 cm never", and the config value that follows from the
  protocol is 0.
- **`FOOT_SPAN_X` = 0.550 m is bounded by the ladder, not by a failure.** `first fail` is
  *none*: WALK cleared every gap probe up to the top of the family, 0.60 m. A walking Go2
  should not cross a 0.60 m gap, and `GAP_MAX` is measured at ~0 because no skill launches
  with forward speed. So this number says the gap probes are not testing what their name
  says — most likely they are steppable rather than jumpable at this width — and it should
  **not** be written into config until the probe geometry is checked. A limit that the
  ladder never challenged is not a limit.
- **`STEP_TROT_MAX` still has no value.** TROT falls at 0.02 m in 75 of 75 runs, heading
  or no heading. Its step capability is below the ladder's floor, which is a statement
  about replayed TROT rather than a threshold waiting for a finer ladder.

Nothing has been applied to `planner/config.py`. `STEP_WALK_MAX` is the one that could be,
on the protocol's own rule; the other two should not be.
