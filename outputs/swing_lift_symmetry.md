# Symmetric swing lift, and the roll that was never there

Three things were asked of this: make the lift left-right symmetric, find out why the
Jacobian solved differently per leg, and re-decide whether TURN's break at 20 mm was the
asymmetry or a real limit.

Headline: **the asymmetry was real and the fix works, but it fixes steering, not steps.**
The symmetric rule takes the curvature an asymmetric lift injects under the heading
controller from 6.5–6.8 °/m back to 0.4–1.5 — the no-lift level, inside the benchmark
budget. It changes no step score, does not rescue TURN at 20 mm, and does not rescue TROT
at 80 mm.

And one correction first, because it decides how the rest is read: the **roll** that
motivated all of this was a quaternion-decoding error in the analysis. The robot never
rolled.

---

## 0. Retraction: the "+2° → +55° of roll" was yaw

The lip diagnosis hand-wrote the quaternion-to-Euler formulas assuming scalar-first
`(w, x, y, z)`. Every trace in this project is scalar-**last** `(x, y, z, w)`, and
`sim/replay.py` has a `quat_to_rpy_deg` that knows it. Read correctly, the same runs are:

| | reported | measured |
|---|---|---|
| WALK, 60 mm lift, flat | roll +2° → **+55°**, "an asymmetric lift is a steering input" | roll median **−0.63°**, \|roll\| max **5.0°** |
| same run | — | it walks a wide **circle**: yaw +196°, curvature 15.7 °/m |
| base height throughout | — | **0.311–0.316 m** (nominal 0.32) — which is what a 55° roll could not be |

Recorded as `harness_findings.md` §11 and as a rule in CLAUDE.md §6.5.

**The claim that the lift produces a rolling, crabbing robot is withdrawn**, and with it
the reasoning that "heading hold could not see it because there was no heading error" —
there was, and heading hold saw it. The claim that an asymmetric lift *steers* survives the
correction at a tenth of the size: §3 measures it as 6.5–6.8 °/m, not 40. What follows is
the A/B that should have been run instead of asserting either.

---

## 1. Why the four legs solved differently — measured, and it was not the Jacobian

The 2×2 solved per frame per leg for (Δz = lift, Δx = 0), from a numerical Jacobian at that
leg's own mid-swing pose. WALK, 60 mm:

| leg | ∂z/∂(thigh, calf) | cond(A) |
|---|---|---|
| FL | (−0.0274, −0.1672) | 2.5 |
| FR | (−0.0378, −0.1760) | 2.4 |
| RL | (+0.0300, −0.1352) | 2.6 |
| RR | (+0.0387, −0.1333) | 2.5 |

**Every leg is well conditioned** (2.4–2.6; nothing near singular) and the front/rear sign
flip on the thigh column is geometry — at mid-swing the front and rear thighs are on
opposite sides of vertical. Left and right differ by ~30 % on that column purely because
the two legs are at *different poses* at their own mid-swing. None of that is a defect and
none of it was the asymmetry.

**The asymmetry came from the amplitude rule, not the solve.** The amplitude was
`target − that leg's own existing apex`, and the legs do not start equal:

| WALK, 60 mm target | FL | FR | RL | RR |
|---|---|---|---|---|
| apex the clip already has | 14.6 | 31.2 | 7.2 | 8.1 mm |
| **added, per-leg (old)** | +45.4 | **+28.8** | +52.8 | +51.9 |
| **added, per pair (new)** | +45.4 | **+45.4** | +52.8 | +52.8 |

The recording walks straight *with* the apex asymmetry it has, so the quantity the robot
feels as a change is the **added** one. The fix takes one amplitude per mirror pair, from
the worst bout in the pair, so the added Δz is identical left and right and the lower leg
still reaches the target. Front and rear stay free to differ — that is pitch-symmetric,
not steering. `--swing-lift-asym` restores the old rule for the A/B; `--swing-lift 0` is
still a byte-exact null (checked: WALK/TROT/TURN reproduce the pre-change rows on all 11
metric columns).

---

## 2. Flat ground, 60 cycles, symmetric vs per-leg

`--foot-comp raibert --foot-clip-rad 0.05`, no heading controller (this harness has none),
so curvature here is the **open-loop** number and 12.5 °/m is WALK's own baseline.

| clip | lift | sym | cycles | survived | vx | curvature | rear apex RL / RR |
|---|---|---|---|---|---|---|---|
| WALK | 0 | — | 58 | 60+ | 0.231 | **12.50 °/m** | 1.9 / 5.4 mm |
| WALK | 40 | ✓ | 58 | 60+ | 0.276 | 15.70 | 21.1 / 32.9 |
| WALK | 60 | **✗** | 58 | 60+ | 0.279 | **16.43** | 40.1 / 52.1 |
| WALK | 60 | **✓** | 58 | 60+ | 0.279 | **15.66** | 40.0 / 53.0 |
| WALK | 80 | ✓ | 58 | 60+ | 0.279 | 15.20 | 59.7 / 73.7 |
| TROT | 0 | — | 58 | 60+ | 0.659 | 7.99 | 58.5 / 58.2 |
| TROT | 60 | ✓ | 58 | 60+ | 0.654 | 7.62 | 73.5 / 73.0 |
| TROT | 80 | ✓ | 3 | **roll 2.97 s** | 0.401 | — | — |
| TURN | 0 | — | 63 | 60+ | 0.020 | (turning gait) | 43.5 / 26.5 |
| TURN | 20 | **✗** | 4 | **roll 3.81 s** | 0.016 | — | — |
| TURN | 20 | **✓** | 5 | **roll 3.67 s** | 0.024 | — | — |
| TURN | 40 / 60 / 80 | ✓ | 4 / 7 / 1 | roll 3.61 / 6.54 / 2.34 s | — | — | — |

**Open loop, symmetry looks free and pointless.** At 60 mm on WALK it moves curvature
16.43 → 15.66 °/m — 0.8 of a °/m, against a 3 °/m step that the lift causes at *any*
symmetry and against a 12.5 °/m baseline the gait already has. Survival, stride, vx and
rear apex are unchanged to three digits. §3 shows this harness is the wrong place to look:
a gait that already curves at 12.5 °/m cannot show a 6 °/m effect.

### TURN at 20 mm is a real limit

This was the question the A/B was for, and the answer is clean. TURN's per-leg amplitudes
were genuinely lopsided (FL +10.7 vs FR +17.5, RL +8.3 vs RR +19.5/+0.0) — a fair test.
Equalising them: **3.807 s / 4 cycles → 3.668 s / 5 cycles.** Same failure, same mode
(roll), same time. **TURN cannot take a 20 mm lift, and the asymmetry is not why.**

The same holds for TROT's break between 60 and 80 mm: symmetric 80 mm still rolls at
2.97 s (per-leg: 2.37 s).

---

## 3. Under the heading controller, symmetry does matter

The flat harness has no heading hold, so §2 is open loop and the two rules look alike
there. On the probe grid, which does have it (`--heading heading-only`, WALK ±0.04), they
do not. WALK, flat approach before the lip, one repeat per level:

| WALK, pre-lip approach | 0.02 m probe | 0.04 m probe |
|---|---|---|
| lift 0 | 0.45 °/m | 3.20 °/m |
| lift 60, **per-leg** | **6.78** | **6.51** |
| lift 60, **symmetric** | **1.46** | **0.37** |

**The per-leg rule injects 6.5–6.8 °/m of curvature; the symmetric rule gives it back.**
Symmetric lift holds a line as well as no lift at all — 0.37–1.46 against 0.45–3.20 — and
the benchmark budget is 0.565 °/m.

So the asymmetry *was* a steering input after all. It is 6 °/m, not the 40 °/m the
mis-decoded traces implied, and it only shows up once a heading controller is present to be
degraded — which is why §2's open-loop A/B, where WALK already curves at 12.5 °/m on its
own, could not see it.

This is the one thing the fix genuinely buys, and it is worth keeping for its own sake.
It does not change any step score (§3b).

## 3b. Step probes: the curve is still flat

Probe grid, 5 repeats per level, `--foot-comp on --heading heading-only`, symmetric lift.
Reached / 5:

| level | lift 0 | 40 sym | 60 per-leg | 60 sym | 80 sym |
|---|---|---|---|---|---|
| **0.02 m** | **4/5** | 0/5 | 0/5 | 0/5 | 0/5 |
| 0.04 m | 0/5 | 1/5 | 0/5 | 0/5 | 0/5 |
| 0.06–0.30 m | 0/5 | 0/5 | 0/5 | 0/5 | 0/5 |
| TROT, every level | 0/5 | 0/5 | 0/5 | 0/5 | 0/5 |

**WALK does not clear 4 cm with a symmetric lift.** The single 1/5 at 0.04 m / 40 mm is one
repeat out of five and the same size as the noise in the old sweep's 1/5 at 20 mm.

The lift-0 column reproduces the original sweep exactly (WALK 4/5 at 0.02, 0 everywhere
else), so the harness is unchanged and the comparison is like for like.

The best step score in the table still belongs to the untouched recording. Fixing the
symmetry removed a steering side effect; it did not touch the thing the lift was built for,
because — as `lip_failure.md` established — the step failure is not a clearance failure.

## 4. Conclusion

- **Keep the symmetric rule.** It is now the default. It is free open loop, and under the
  heading controller it removes 6.5–6.8 °/m of curvature that the per-leg rule was
  injecting — from 6.5–6.8 back to 0.4–1.5, which is the no-lift level and inside the
  0.565 °/m benchmark budget.
- **It does not rescue anything else.** WALK still does not clear 4 cm (0/5 at every lift);
  TROT still breaks between 60 and 80 mm on flat ground; TURN still breaks below 20 mm, and
  the A/B says the asymmetry was not why.
- **`--swing-lift` stays off by default.** Nothing here changes the recommendation in
  `swing_lift.md`: the edit does not move step height, and it costs up to 0.47 rad of calf
  offset — 5× the entire foot-placement budget.
