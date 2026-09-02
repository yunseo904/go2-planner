# Climbing then falling: TROT yes at one level, WALK no — and WALK never reaches 6 cm

Asked: the roll time series just before and after a step crossing, against a comparable
flat stretch, for **TROT at 2 cm and WALK at 6 cm**. The probe cells carry a ~3 m flat
run-up before the riser, so the matched flat stretch is **inside the same run, on the same
robot, on the same cell** — no cross-cell comparison is needed.

Rig: `run_calibration_grid.py`, 15-level `step_up` ladder per skill, `--reps 1`,
`--foot-comp on --heading heading-only`, TROT at clip frame 0 (one of the four it survives
at on flat). Riser 3.00 m ahead of the spawn. Windows are 1 s wide; "flat" is 3 s to 1 s
before the crossing.

## TROT at 0.020 m — the pattern is there

| window | \|roll\| mean | \|roll\| max | drift over the window |
|---|---|---|---|
| flat run-up, 3 s → 1 s before | **1.11°** | 2.70° | −1.26° |
| 1 s before the riser | 0.87° | 2.21° | +3.77° |
| **1 s after the riser** | **3.72°** | **13.34°** | **−15.36°** |
| 1 s → 3 s after | 26.80° | 58.29° | −44.49° (terminates) |

It crosses at t = 5.06 s carrying +2.03° of roll, and it is over at 6.82 s. **Roll is flat
and small right up to the riser and is initiated by it** — the robot is not already falling
when it arrives. That is the mechanism the question proposed, and TROT at 2 cm shows it.

## WALK at 0.060 m — the premise does not hold, because it never gets there

**WALK never reaches the 6 cm riser**: max x is 2.81 m against a riser at 3.00 m. It
survives 14.30 s and rolls out **on the flat run-up, 0.19 m short of the step it was being
asked about.** There is no crossing to look at.

That is true at every level from 0.06 m up — 13 of 15 rungs, terminating at 14–43 s having
never arrived.

## And where WALK does climb, the effect is the opposite

| step | crossed | \|roll\| flat | \|roll\| 1 s after | ratio | survived |
|---|---|---|---|---|---|
| 0.020 | yes | 2.31° | **1.36°** | **0.59** | **30.46 s** |
| 0.040 | yes | 2.89° | **2.41°** | **0.83** | **20.06 s** |

**Roll after the crossing is lower than on the flat approach, and these two are the longest
surviving runs in the whole WALK table.** WALK climbs and carries on; it does not climb and
fall.

## Status

- **"Climbing then falling" is TROT-specific here, and it rests on one cell** — TROT crosses
  only at 0.020 m in this ladder (0/15 elsewhere), so the ×3.4 roll amplification is n = 1
  and must not be quoted as a curve.
- **The WALK half of the premise is withdrawn.** At 6 cm there is no climb; at the levels it
  does climb the roll goes the other way.
- **What ends WALK on this rig is not the step.** It dies at 14–16 s on almost every level
  with under 3 m travelled, on the flat approach. That is the same event
  `cross_track_is_the_fall.md` finds ending the benchmark robots — roll, before the terrain
  feature is reached — and it is the open question both files now point at.
