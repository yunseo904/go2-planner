# Raising the swing foot: the curve, and it is flat

`--swing-lift` raises each swing foot's arc to a chosen apex above its own
liftoff/touchdown chord, as `A·sin²(π·phase)` — zero value **and zero slope** at both ends,
so the footfall positions that set stride and speed are untouched. Stance is untouched.
The hip is untouched: that is foot placement's and heading hold's joint, and they must not
compete. The vertical displacement becomes thigh and calf offsets through a per-frame
numerical Jacobian solved for (Δz = lift, Δx = 0). Off by default, banner-printed, stamped
into every row, and the archive on disk is unchanged.

The existing apex each leg is raised *from* is measured from `q`, the achieved angles, per
`outputs/commanded_angles.md` — `q_des` is not a pose the robot holds. A leg already above
the target is left alone, which is why TROT at 20 and 40 mm is a byte-identical no-op: its
arcs are already 50–57 mm.

---

## The curve — passable step height vs foot height

5 repeats per level, probe grid, heading hold on, foot placement on.

| swing lift | WALK 0.02 m | WALK 0.04 m | WALK ≥0.06 | TROT, any height |
|---|---|---|---|---|
| **0 (recording)** | **4/5** | 0/5 | 0/5 | 0/5 |
| 20 mm | 0/5 | 1/5 | 0/5 | 0/5 |
| 40 mm | 0/5 | 0/5 | 0/5 | 0/5 |
| 60 mm | 0/5 | 0/5 | 0/5 | 0/5 |
| 80 mm | 0/5 | 0/5 | 0/5 | 0/5 |

**Raising the foot does not raise the passable step. It slightly lowers it.** The best
score in the whole table belongs to the untouched recording.

That is the result, and it is consistent with everything measured before it:

- TROT already clears a 2 cm step by 2.5× at zero lift and still fails it 75/75.
- WALK at 80 mm lift puts its rear feet 50–64 mm above their own chord — well clear of a
  2–4 cm lip — and fails anyway.
- The failure traces show why: WALK **stalls at the lip** (pitch nose-up −10°, x stuck at
  3.85→4.11 m, then roll), and TROT **climbs the step and rolls over 0.4 m past it**.
  Neither is a foot catching on the edge.

**Step-up is not clearance-limited for these gaits.** It is limited by what happens after
the foot is over the lip.

## On flat ground the edit is nearly free — up to a point

60 cycles, foot placement on. Clearance shown in both references, from the replay's own
foot positions.

| clip | lift | survived | stride | vx | rear apex vs own chord |
|---|---|---|---|---|---|
| WALK | 0 | 60+ cyc | 1.37 (−0%) | 0.231 | RL 1.2, RR 0.9 mm |
| WALK | 20 | 60+ cyc | 1.37 (−0%) | 0.257 | RL 0.1, RR 0.0 |
| WALK | 40 | 60+ cyc | 1.37 (−0%) | 0.276 | RL 18.9, RR 0.0 |
| WALK | 60 | 60+ cyc | 1.37 (−0%) | 0.279 | RL 36.2, RR 48.8 |
| WALK | 80 | 60+ cyc | 1.37 (−0%) | **0.279** | RL 50.2, RR 64.4 |
| TROT | 0–40 | 60+ cyc | 1.56 (+0%) | 0.659 | no-op, already 50–57 |
| TROT | 60 | 60+ cyc | 1.56 (+0%) | 0.656 | RL 61.6, RR 63.0 |
| TROT | **80** | **2.37 s** | 1.61 (+3%) | 0.357 | — |
| TURN | 0 | 60+ cyc | 1.12 (−0%) | 0.020 | RL 37.0, RR 26.5 |
| TURN | **20** | **3.81 s** | 1.63 (+45%) | 0.016 | — |
| TURN | 40 / 60 / 80 | 4.07 / 13.03 / 2.48 s | — | — | — |

- **WALK takes the whole range.** Stride stays at +0% and forward speed *rises*,
  0.231 → 0.279 m/s. This is not the stance-widening pattern — nothing is traded away.
- **TROT breaks between 60 and 80 mm**, and when it breaks vx collapses 0.659 → 0.357.
- **TURN breaks below 20 mm.** It is the least tolerant of the three by a wide margin.

### A commanded lift under ~40 mm does not reach the ground

WALK's rear apex goes 1.2 → 0.1 → 18.9 → 36.2 → 50.2 mm for commanded targets of
0 → 20 → 40 → 60 → 80. The first 20 mm buys **nothing**: the rear foot is skimming, and a
small commanded lift does not break contact. This is worth knowing before reading any
small-lift result as "the edit did not help" — below ~40 mm the edit does not happen.

## What it costs, per joint and per phase

Commanded deviation RMS from the untouched clip (rad). Stance is exactly zero everywhere,
by construction — checked, not asserted.

| clip | lift | hip swing | thigh swing | calf swing |
|---|---|---|---|---|
| WALK | 0 | 0.0402 | 0 | 0 |
| WALK | 40 | 0.0386 | 0.0536 | 0.0982 |
| WALK | 80 | 0.0389 | **0.1267** | **0.2397** |
| TROT | 80 | 0.0463 | 0.0330 | 0.0885 |
| TURN | 80 | 0.0467 | 0.1438 | 0.2845 |

The hip column is foot placement's, and it does not move as the lift grows — the two
mechanisms are on separate joints and stay there. At 80 mm the calf is offset by up to
0.47 rad peak (0.24 RMS), which is **five times** the whole stage-2 foot-placement budget
of 0.05 rad. That is the price of the edit, and it should be read next to the fact that it
buys no step height.

## Recommendation

**Leave `--swing-lift` off.** It is implemented, exact at 0, and correctly scoped, and the
curve it produced is the deliverable. But it does not move the quantity it was built to
move, it costs up to 0.47 rad of joint offset, and it destabilises two of the three skills.

The measurement it enables is worth more than the feature: **foot height is not what caps
these gaits on a step.** The next thing to look at is the stall and the post-lip roll,
which is where every step failure actually happens.
