# Stance height: the deficit is real and correctable, and it is not what costs the clearance

## 1. The distinction holds — low from the start, not a decay

| window | WALK replay base height |
|---|---|
| first 1 s | 308.5 mm (still settling) |
| 1–5 s | **316.0** |
| middle | 316.6 |
| last 5 s | 315.9 |

Flat. Nothing decays. This is a different object from the compensation rejected earlier,
which was looking for a body height that falls during the run.

Measured like-for-like — commanded hip-to-foot drop against achieved, on the frames the
clip has that leg in **stance**, both read from `body_pos_w`:

| clip | commanded | achieved | deficit |
|---|---|---|---|
| WALK | 317–338 mm | 280–298 | **37–41 mm (mean 39.5)** |
| TROT | 353–357 | 298–321 | 36–55 (mean 45.5) |
| TURN | 338–354 | 297–334 | 8–42 (mean 28.2) |

kp 40 sags about 40 mm under the robot's weight. (TROT and TURN are measured over 2
cycles: they collapse without foot placement, and a longer window measures the fall.)

## 2. `--plant-comp height`

Thigh and calf are offset by a constant, in the ratio that moves the foot **straight down**
and leaves its fore-aft position alone — solved as a 2×2 from a numerical Jacobian at each
clip's own mean stance pose (`scripts/check_stance_height.py`). It does **not** touch the
hip: widening the stance is what cost a third of the forward speed last time, and the hip
is also the joint foot placement and heading hold use.

`--plant-comp-alpha` scales it; 1.0 asks for the whole measured deficit. Constant, no
feedback, banner-printed, stamped into the results row, archive untouched.

| run | base height | stride | vx | RL rise | RR rise |
|---|---|---|---|---|---|
| WALK α=0 (null) | 316.0 mm | 1.37 (+0%) | 0.231 | 10.0 mm | 16.3 mm |
| **WALK α=0.5** | **348.8** | 1.37 (+0%) | **0.268** | 9.7 | 13.8 |
| WALK α=1.0 | 366.8 | 1.30 (+5%) | 0.338 | — | **fell at 3.32 s** |
| TROT α=0 | 333.5 | 1.56 (+0%) | 0.659 | 63.6 | 63.8 |
| TROT α=1.0 | 382.5 | 1.69 (+8%) | 0.368 | — | **fell at 1.81 s** |
| TURN α=0 | 342.0 | 1.12 (+0%) | 0.020 | 64.9 | 43.5 |
| TURN α=1.0 | 344.1 | 1.80 (+61%) | — | — | **fell at 3.85 s** |

**It does what it was asked to do.** At α=0.5 WALK's base goes 316 → 348.8 mm, closing
the sag, and it costs nothing: stride stays +0% and forward speed goes **up**, 0.231 →
0.268 m/s. This is not the pattern that killed the stance-widening version.

**Full correction destabilises every clip.** α=1.0 falls WALK at 3.32 s, TROT at 1.81,
TURN at 3.85. The Jacobian is kinematic and the sag is a load effect, so α=1 commands more
extension than the leg needs once it is pushing harder; the useful range is the lower half.

**And it does not recover the rear clearance.** RL 10.0 → 9.7 mm, RR 16.3 → 13.8. That is
not a failure of the implementation — a constant leg-lengthening offset is **common mode**:
it raises the body and pushes the swing feet down by the same amount, leaving their
difference, which is what ground clearance is, untouched.

## 3. So the clearance hypothesis is withdrawn

"The body rides 35 mm low, which consumes the rear legs' retraction" was wrong. The +33 mm
of body height bought exactly zero clearance.

What was ruled out along the way, each by measurement:

- **not load** — the rear feet carry 8–15 N during their swing against 46 N in stance
- **not tracking** — swing-phase joint errors are ≤0.06 rad mean (RL thigh 0.057, the worst)
- **not a phase-dependent sag** — the base is no lower when a rear leg swings (317.2 mm)
  than when a front one does (314.6)
- **not body height** — §2

## 4. What the recording implies cannot be settled from `q_des`

Two references disagree, and the disagreement is the finding:

| reference | FL | FR | RL | RR |
|---|---|---|---|---|
| level body | 60.3 | 70.4 | 54.3 | 64.5 mm |
| the plane the stance feet actually define | 46.0 | 123.5 | **−11.9** | 81.9 |

A negative clearance is a "swing" foot below the stance plane, which cannot happen, and the
trunk pitch implied by keeping three stance feet coplanar swings from **−2.7° to +9.0°**
within one cycle. **The commanded trajectory does not describe a rigid trunk standing on
flat ground** — which is unsurprising for a `q_des` stream whose achieved angles differed
under load on the real robot, but it means the earlier "the rear feet should clear 72–80 mm"
figure is **not established**, and neither is any replacement.

Settling it needs the achieved angles (`q`, which the archive also carries) rather than the
commanded ones, and that has not been done.

## 5. WALK's 2 cm probe is still not worth re-running

The limit is set by rear-foot ground clearance, and clearance did not move. Re-running the
probes would measure the same number to five repeats. `STEP_WALK_MAX` stays
`CALIBRATION_NEEDED`.

---

# Recorded separately: the trunk pitch is a plant mismatch

Not the cause of anything above, and logged because it will surface again.

| clip | real robot (steady window of its own session) | sim replay |
|---|---|---|
| WALK | **−0.08° ± 2.66** | −3.25° ± 0.84 |
| TROT | +0.19 ± 0.32 | +1.69 ± 0.74 |
| TURN | +0.67 ± 0.48 | +2.24 ± 1.44 |

The real robot is level in all three; the sim holds 1.5–3.2° of pitch on the same joint
targets. Same family as the stance-width and ground-friction mismatches. Its direction is
*opposite* to what would explain the rear clearance — −3.25° raises the rear hips 11 mm —
so it is recorded rather than corrected.
