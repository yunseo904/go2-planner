# TURN on terrain: the criterion, and what the flat control did to it

Asked for: TURN's step and gap limits on the existing probes, and — first — how a
turn-in-place skill should be scored at all. The criterion is below; the flat control is
why it looks the way it does.

All runs CPU, `--device cpu`, no GPU at any point.

---

## 1. Why the existing criterion cannot be used

Every other family scores a repeat by *reaching goal 2*, 3.95 m from the spawn past the
obstacle. TURN's measured forward speed is **0.0075 m/s** (`skill_profile.csv`, the
session turns on the spot). That is **527 s** to goal 2 against a budget of 20 s, and the
budget is itself derived from the skill's own speed, so raising it to 1054 s would make
the grid step for a simulated quarter hour to score a robot that never intended to go
anywhere.

The failure would be uniform across every level, which is the tell: **a score that comes
out the same on 0.02 m and 0.30 m is not measuring the terrain.** That is the identical
error `lip_failure.md` §4 found in `STEP_TROT_MAX` — a lane-departure score being read as
a step limit — and it is worth refusing twice.

## 2. The criterion

**The obstacle is brought to the robot, and what is scored is the turn.**

| | |
|---|---|
| **spawn** | on the obstacle, not 3 m short of it: the step edge under the body centre, the pit centred under the footprint, the ramp and the rough patch at their midpoint |
| **pass** | complete **90° of yaw** — a quarter turn, ~4.5 cycles of the clip's own 20.2°/cycle — while never falling and never leaving a **0.35 m** circle around the spawn |
| **budget** | 90° / the clip's own 22.66 °/s × the same slack factor the other families use = 7.9 s |
| **recorded regardless** | the yaw actually completed, the largest drift, and the drift/yaw at every step, so a different threshold can be read off these runs without repeating them |

Two of those numbers are borrowed rather than chosen: 0.35 m is `GOAL_RADIUS_M`, the same
radius the traversal families call "arrived", and the slack factor is the traversal
budget's. The 90° is the one new number and §3 is what justifies its placement.

**This is a different question and it is named as one.** Not *what can TURN cross* —
nothing, it does not translate — but **what can TURN turn on**. The rows are stamped
`probe.<family>@TURN` rather than a `skill.*` parameter name, because no
`CALIBRATION_NEEDED` field in `planner/config.py` holds this quantity and inventing one
before knowing whether the number exists would put a placeholder in config for something
that may not be defined. TURN's only calibration entry is `HEADING_ERR_TURN_DEG`, a
bearing threshold, and this sweep does not settle it.

## 3. The flat control, which had to be run first and changed the protocol

Same scoring, same spawn machinery, robot left on the probe's own **flat run-up** 3 m
short of the obstacle. Nine cells × five repeats; the cells are identical flat ground, so
the nine are a check on the harness and the five repeats are the measurement.

| rep | clip entry frame | passed | yaw completed | drift |
|---|---|---|---|---|
| 0 | 0 | **0/9** | 48.7–51.8° | 0.45–0.54 m |
| 1 | 9 | **9/9** | 90.2–95.7° | **0.12–0.14 m** |
| 2 | 18 | **0/9** | 53.4–65.1° | 0.41–0.59 m |
| 3 | 27 | **0/9** | 47.6–58.3° | 0.38–0.58 m |
| 4 | 36 | **9/9** | 90.6–95.2° | **0.16–0.22 m** |

Three things come out of this, and the third is the one that matters.

**The 0.35 m radius is in the right place, and that is measured rather than argued.** The
repeats that turn sit at 0.12–0.22 m and the ones that walk away sit at 0.38–0.59 m.
There is a clean gap between the two populations and the borrowed threshold falls inside
it. Nothing was tuned to make that true — 0.35 is `GOAL_RADIUS_M` and the populations are
where they are.

**The nine cells agree to within 3° and 0.09 m.** TURN is *reproducible* across cells,
which is not true of TROT on the same harness (`harness_findings.md` §12: 0.3 mm at step 0
becomes 1 m by the end). So a TURN row is a measurement and not a sample of a chaotic
distribution, and n = 9 cells × 5 reps can be read as it stands.

**TURN fails 3 of its 5 entry phases on flat ground, before any terrain is involved.**
It reaches 48–65° of its 90 and then walks out of the circle. Under the protocol's own
"a level passes only if EVERY repeat passed" rule this makes **every level 0/5 forever**,
at any terrain, for a reason that has nothing to do with terrain — the same shape of
error §1 refuses.

So the reduction is re-baselined, and this is the criterion as it stands:

> **The flat run is the control for its own entry phase.** A level passes at phase *p*
> only if it passes at *p* **and** flat passes at *p*. Phases that fail on flat carry no
> information about terrain and are excluded from the denominator. TURN's usable
> denominator is therefore **2 of 5 phases** (entry frames 9 and 36), and a terrain limit
> is the largest level at which both of them still turn.

---

## 4. The result: TURN's terrain limit is below every ladder's floor

42 traversal probes (`step_up` 15, `step_down` 15, `gap` 12) plus the v2 families
(`slope` 8, `roughness` 12), 5 repeats each, `--foot-comp on`, symmetric lift, heading hold
off (TURN uses the rotation-kinematics target from `turn_target.md`, not heading hold —
it is *supposed* to be turning). Reduced against the flat control by
`scripts/reduce_turn_inplace.py`, so the denominator is the 2 phases that turn on flat.

| family | passes | yaw completed at the smallest level | drift there | flat control |
|---|---|---|---|---|
| `step_up` | **0 / 15 levels** | 24.3–52.6° of 90 | 0.49–0.64 m | 90.2–95.7°, 0.12–0.22 m |
| `step_down` | **0 / 15 levels** | 54.2–54.6° | 0.78–0.80 m | same |
| `gap` | **0 / 12 levels** (1 of 2 phases at 0.05 m) | 71.0–101.5° | 0.23–0.50 m | same |
| `roughness` | **0 / 12 levels** (1 of 2 phases at 0.005 m) | 27.6–118.5° | 0.20–0.62 m | same |
| `slope` | non-monotone — see below | 23.3–28.1° at 5° | 0.59–0.63 m | same |

**A 2 cm edge under the body is enough.** On flat, the two working phases turn 90–96° and
stay inside 0.22 m. Put a 0.02 m step edge under the middle of the footprint and the same
two phases reach 24–53° and drift 0.49–0.64 m — out of the circle before a quarter turn.
The limit is below the smallest level in every family, so **no threshold can be read off
this archive**; what can be said is that it is under 0.02 m of step, under 0.05 m of gap
and under 5 mm of roughness.

The response is graded and monotone, which is what says the probe is measuring something
rather than failing uniformly: completed yaw falls 24° → 0° across the `step_up` ladder,
54° → 0° across `step_down`, and 71° → 0° across `gap`.

**`slope` is not readable as a limit and is reported as such.** 5° and 10° fail, 15° passes
both phases (turning 125–166°), 20° and above cannot establish a stance at all. A
non-monotone column means the family did not behave like a ladder, and with a denominator
of 2 phases one inversion is not something to build a threshold on. The 20°+ rows are a
separate fact and a real one: **the robot cannot stand still on a 20° incline** — the settle
never produces a stance, the hip-to-foot lever collapses from a nominal 0.31 m, and the run
is scored failed at settle rather than scored at all.

## 5. What is NOT claimed

- **This is not `STEP_TURN_MAX`.** No such field exists in `planner/config.py` and none was
  added. The rows are `probe.<family>@TURN`. A parameter should be named after the
  measurement produces a number, and this one produced "below the floor".
- **It does not settle `HEADING_ERR_TURN_DEG`**, TURN's only calibration entry, which is a
  bearing threshold and not a terrain one.
- **It says nothing about turning while walking.** The clip is a stationary turn; a turn
  folded into forward motion is a different manoeuvre that this library does not contain.
- **The drift column on a passing row is a whole-run maximum**, so it can exceed 0.35 m on
  a row that passed: the pass is decided by the drift at the moment the 90° was completed,
  and what the robot does afterwards is recorded but not scored.

## 6. Where it leaves TURN in the planner

TURN remains what `heading_candidates.md` §4 concluded — a deliberate reorientation at
20.2° per cycle, too coarse for straightness — and it now has a terrain qualification
attached: **it must be issued on ground flat under the whole footprint.** A turn commanded
while the robot straddles a 2 cm edge does not complete. For a planner that switches skills
at feature boundaries this is a scheduling constraint, not a threshold: the switch to TURN
has to happen before the obstacle, not on it.
