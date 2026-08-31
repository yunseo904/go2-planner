# Swing height: three quantities, one number each, and they were being compared to each other

The external comparison (`quadruped_pympc`) defines step height as **0.2 × hip_height =
0.056 m, measured against that leg's own lift-off z**. This project measured against the
**stance plane**. The question was what our number is on their basis.

**Answer: TROT is 57 mm on their basis, against their 56 mm.** And the `footRaiseHeight`
0.08 m argument dissolves: 0.08 m is a third quantity — ground clearance — and our TROT
delivers 79 mm of it. All three numbers are ours, all three are right, and none of them is
the same measurement.

CPU only, two instruments, no GPU.

---

## 1. The bases are not interchangeable, and off the floor they differ by 2×

`scripts/check_swing_basis.py` drives each clip's own `q_des` through Isaac's kinematics
with the robot **held in the air** — the `foot_clearance_check.md` §2 method, which
isolates the recording from the replay's body attitude. Per leg, at the swing apex:

| clip | vs lift-off (contact channel) | vs stance plane | vs the leg's own cycle minimum |
|---|---|---|---|
| WALK | 8.5–28.3 mm | 63.0–71.9 | 53.0–80.5 |
| TROT | 57.0–77.2 | 88.9–117.8 | 94.2–120.9 |
| TURN | 18.9–48.1 | 47.0–67.4 | 41.8–76.1 |
| RUN | 91.9–105.4 | 96.3–110.4 | 103.9–111.7 |

**On the recording, the two bases are a factor of two apart for WALK and TROT.** So a
comparison that does not name its basis is not a comparison.

## 2. But the air-held measurement cannot settle the lift-off basis, and here is why

The takeoff basis needs a lift-off *event*. Held in the air there is none, so the frame has
to come from the clip's contact channel — and the threshold-free geometric check disagrees
with it:

| clip | apex frame | channel's lift-off | where the foot starts rising | offset |
|---|---|---|---|---|
| WALK FL | 11 | 7 | 24 | −17 of 37 |
| TROT RL | 25 | 19 | 3 | +16 of 32 |
| RUN FL | 2 | 14 | 14 | **0 of 16** |

The two agree for RUN and are a third of a cycle apart for WALK and TROT. The reason is
not a bad channel: **with no floor the foot's z is one continuous fall-and-rise**, so
"where it starts rising" lands on the cycle minimum, which on a real gait is *mid-stance*,
not lift-off. The geometric reference is measuring a different event.

(A kinematic stance gate — "within 3 mm of the lowest foot this frame",
`check_foot_kinematics.py`'s corrected rule — was tried as the second reference and is not
usable here either: it calls 70–75% of leg-frames swing for every clip, against duties of
0.31–0.64. It is not calibrated for a body-fixed replay.)

## 3. The second reference frame, where lift-off is a physical event

`scripts/swing_basis_table.py` reads a **floored 60-cycle replay** and takes lift-off from
the simulator's own contact sensor. Median over all swing bouts, [p10, p90]:

| clip | leg | vs lift-off (SIM contact) | vs lift-off (clip channel) | vs stance plane | vs ground |
|---|---|---|---|---|---|
| **TROT** | FL | **56.6** [−0.1, 63.6] | 58.2 | 55.5 | 78.6 |
| | FR | **56.6** [−0.1, 65.0] | 60.5 | 55.9 | 79.1 |
| | RL | **57.1** [−0.1, 60.9] | 58.9 | 55.9 | 79.0 |
| | RR | **56.9** [−0.1, 60.3] | 58.0 | 56.0 | 79.1 |
| **WALK** | FL | 44.0 [36.1, 49.3] | 40.8 | 44.1 | 67.3 |
| | FR | 49.5 [41.1, 60.9] | 39.2 | 49.4 | 72.7 |
| | RL | **−0.0** [−0.0, 3.3] | 2.4 | −0.0 | 23.4 |
| | RR | **−0.0** [−0.0, 6.5] | 4.3 | 0.5 | 23.6 |

**The two gates agree** — within 6% on TROT, within 21% on WALK's front, and both say zero
on WALK's rear. That is the agreement §2 could not produce, and it is what makes the
numbers quotable. (88–98 bouts per leg for TROT, 59–134 for WALK: these are distributions,
not single measurements. The p10 of −0.1 mm on TROT is bouts where a contact chatters; the
median is the honest statistic.)

Note that on the floor the lift-off basis and the stance-plane basis **converge** (56.6 vs
55.5 on TROT), because a foot that is standing on flat ground *is* on the stance plane. The
2× disagreement in §1 is an artefact of measuring a body-fixed replay.

## 4. So the three numbers, side by side

| quantity | ours (TROT) | theirs / the disputed value |
|---|---|---|
| swing apex above **lift-off** | **57 mm** | `quadruped_pympc` 0.2 × 0.28 m = **56 mm** |
| the same rule applied to **our** body | 0.2 × 0.333 m = 67 mm | — |
| swing apex above **ground** | **79 mm** | Unitree `footRaiseHeight` **0.08 m** |

**All three converge, and the apparent disagreement was entirely the basis.** Our TROT
commands what `quadruped_pympc` would command (57 vs 56 mm), sits 15% under what its own
rule would give for our taller body (67 mm), and clears the ground by the 0.08 m
`footRaiseHeight` names. The A1 study's `ztop` search range of 0.00–0.055 m is the same
lift-off basis and brackets it too.

**There is no swing-height deficit in TROT.** That closes the line of enquiry that the
`footRaiseHeight` argument opened.

## 5. WALK's rear feet do not rise at all, and both gates say so

`foot_clearance_check.md` §3 measured WALK's rear ground clearance at 10.0/16.3 mm and
attributed it to a −3.25° body pitch rather than to the legs, which was right. On the
lift-off basis the rear feet come out at **0.0 mm** — the foot leaves the ground and its
apex is level with where it left. The front pair manages 44–50 mm on the same run.

This is recorded and **not chased**: WALK's 4 cm limit is deferred by request. It is
written down because it is a two-reference measurement of the quantity that question is
about, and because the rear legs producing 0 mm while the fronts produce 45 is a
front/rear asymmetry, not a body attitude — a level body would not fix it.

## 6. What was NOT imported

The early-touchdown reflex `quadruped_pympc` needs to climb a step is **not adopted** — a
skill that regenerates its trajectory when a foot hits something is reacting to terrain,
which breaks the fixed-skill-library premise this whole comparison rests on.

Recorded because the finding cuts the other way and is worth keeping: an independent
implementation found it had to add reactive trajectory regeneration to get up a step. That
is outside confirmation of this project's own observation that a fixed trajectory does not
climb — see `turn_probes.md` §4 and `v2_probes.md`. The conclusion is not "we should react";
it is that the fixed-library arm has a ceiling and somebody else hit the same one.

Also worth recording: `quadruped_pympc`'s contact determination is its **gait scheduler's
timer output, not a sensor**. Our clips' contact channel has the same standing — a
phase-locked schedule rather than a measurement — so that structure is not an oddity of
this project.
