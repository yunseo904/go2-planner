# The 4 mm rear swing was wrong — it was ground clearance, reported as leg lift

`step_and_gap.md` §3 said WALK's rear swing apex is 3.8–5.5 mm and concluded that WALK
fails a 4 cm step because its rear legs cannot lift over it. **That conclusion does not
hold.** The number was real but it was not the leg's motion, and the third check is the
one that matters.

`scripts/check_foot_kinematics.py`, CPU, no GPU. The robot is held in the air and driven
through the clip's own `q_des`, so Isaac's own kinematics answer the question — no
hand-written forward model is involved, and therefore no front/rear formula asymmetry can
hide in it.

## 1. Leg order — correct, and now checked rather than assumed

`robot.find_bodies(".*_foot")` returns, with each foot's body-frame position:

| col | body | x | y | is |
|---|---|---|---|---|
| 0 | FL_foot | +0.183 | +0.177 | front left |
| 1 | FR_foot | +0.183 | −0.177 | front right |
| 2 | RL_foot | −0.268 | +0.173 | rear left |
| 3 | RR_foot | −0.267 | −0.173 | rear right |

So the positional labelling FL, FR, RL, RR is right for this articulation. It was an
assumption in the step-probe analysis — the grid trace stored foot heights without the
body names — and it happens to be correct. Recorded so it is not assumed again.

## 2. The recording is a normal gait

Driving WALK's own `q_des` through the articulation, hip-to-foot drop in mm:

| leg | lowest | at apex | lift | **clearance on a level body** |
|---|---|---|---|---|
| FL | 351.4 | 276.0 | 75.4 | **75.4** |
| FR | 348.8 | 268.1 | 80.6 | **83.3** |
| RL | 332.4 | 279.5 | 52.8 | **71.9** |
| RR | 334.2 | 271.2 | 63.0 | **80.2** |

**All four feet clear 72–83 mm.** The rear "lift" is smaller only because the rear legs
stand more crouched — their lowest extension is 332–334 mm against the front's 349–351 —
so measured from each leg's own lowest point they travel less. Against a common
reference, which is what ground clearance is, all four are the same.

Unitree did not ship a walk with a 4 mm rear step, and the archive does not contain one.

## 3. So where did 4 mm come from? The body, not the legs

In the replay (`walk_cap005`, 60 cycles, the run that passes its gait gate):

| leg | body-frame drop at apex | ground clearance |
|---|---|---|
| FL | 266.5 mm | 50.6 mm |
| FR | 256.9 | 67.1 |
| RL | 262.7 | **10.0** |
| RR | 259.7 | **16.3** |

**The legs are all doing the same thing** — every foot retracts to within 10 mm of the
same body-relative height. What differs is where the body is: the base holds a mean
**pitch of −3.25°**, which over the 0.386 m hip-to-hip span puts the rear hips about
22 mm lower than the front ones, and the rear feet spend their swing 10–16 mm off the
floor instead of 72–80.

Two different quantities were being compared as if they were one: leg lift (the gait) and
ground clearance (the gait *plus* the body's attitude). The 3.8–5.5 mm figure was the
per-bout median of the second.

## What this invalidates

- **"WALK's rear legs cannot lift over a 4 cm step" is withdrawn.** They lift 72–80 mm
  in the recording and retract normally in the replay.
- **The 2 cm WALK step limit has to be re-measured.** As it stands it is a property of the
  replayed *body attitude* — a nose-up pitch that costs the rear feet 50–60 mm of
  clearance — not of the WALK gait. On a level body the same joint trajectory would bring
  ~75 mm of rear clearance to the lip.
- `STEP_WALK_MAX` therefore stays `CALIBRATION_NEEDED`, and the "2 cm reliable, 4 cm not"
  line should be read as *of this replay*, not of this skill.
- The TROT step result is not affected in the same way — its clearance was measured the
  same way but its failure is a roll divergence 0.4 m *past* a step it demonstrably
  climbs, which no clearance number changes.

## A separate inconsistency worth recording

The clip's own contact channel and its own kinematics disagree about which legs are in the
air on **51% of WALK leg-frames** (32% for TROT). The contact channel is what the
foot-placement swing gate reads, so half of WALK's corrections may be applied on the wrong
phase. That is independent of everything above and has not been chased.

## Method note

This is the third time a foot-height number has been wrong here — after the missing
contact rule and the fixed-interval sampling aliasing. All three failures share a shape:
a quantity that is *nearly* the one being reasoned about. The defence that worked was
measuring the same thing two ways in two frames of reference and requiring them to agree.
