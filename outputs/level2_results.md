# Level 2 measured — the roll couple works on WALK, and the criteria say where it stops

Implementation of `outputs/level2_design.md`.  `sim/attitude.py`, wired into
`run_benchmark.py` behind `--roll-couple` (default **off**, banner, per-row stamp,
paired on/off runs).  All 200 cells, legged_eval terrain, 20 s episodes.

## 0. The design changed before it was built, because the measurement said so

The design proposed regulating roll **and** pitch.  Measured first, WALK, 200 cells, both
roughness arms:

| | roll | pitch | below floor | timeout |
|---|---|---|---|---|
| roughness ON | **165** | **0** | 6 | 29 |
| roughness OFF | **125** | **0** | 11 | 64 |

peak \|roll\| median **88°** against an 85.9° cutoff; peak \|pitch\| median 15°, p90 30–37°.

**Not one pitch termination in 400 episodes.**  So the pitch term was dropped before it was
written.  It would have had nothing to do, and it would have had to live on the *thigh*
channel, which is the one the gait itself drives — the measurement removed that risk
rather than managing it.

## 1. The sign, settled by measurement and not by derivation

`sim/footcomp.py` records a derived sign that turned out to be positive feedback, so this
one was probed.  WALK, seed 1, 200 cells:

| | score | alive at 20 s | upright, median |
|---|---|---|---|
| off (control) | 0.76 | 29 | 6.26 s |
| **sign +1 (derived)** | **0.83** | 31 | **7.49 s** |
| sign −1 (flipped) | **0.66** | 21 | 5.82 s |

Bidirectional and ordered: the wrong sign is **worse than off**, the right sign is better.
A single-armed run could not have said that.

## 2. WALK — three seeds, paired per cell

| seed | off | on | Δ | alive | upright | v_x | cells better / worse |
|---|---|---|---|---|---|---|---|
| 1 | 0.76 | 0.83 | +0.08 | 29 → 31 | 6.26 → 7.49 s | 0.083 → 0.078 (**−6%**) | 24 / 12 |
| 2 | 0.77 | 0.82 | +0.06 | 24 → 42 | 6.13 → 7.32 s | 0.076 → 0.073 (**−5%**) | 29 / 18 |
| 3 | 0.69 | 0.81 | +0.12 | 29 → 35 | 6.38 → 7.87 s | 0.070 → 0.069 (**−1%**) | 31 / 8 |
| **mean** | **0.73** | **0.82** | **+0.09** | | | | |

Seed spread: **off 0.08, on 0.02**.  The effect is the size of the off arm's own seed
spread, which is why three seeds were run and not one — but it is **positive in all three,
paired better-than-worse in all three, and it shrinks the spread.**

## 3. The arm — Rule-Planner on depth

| | score | upright | v_x | cells better / worse |
|---|---|---|---|---|
| depth, roll couple off | 0.60 | 5.04 s | 0.067 | — |
| **depth, roll couple on** | **0.69** | 5.76 s | 0.060 (**−10%**) | 33 / 16 |

+0.09, the same size as on WALK.  v_x is **exactly at the ±10% limit** — see §5.

## 4. The gain and cap sweep, WALK seed 1

| | score | alive | roll-ends | upright | v_x | at the cap |
|---|---|---|---|---|---|---|
| off | 0.76 | 29 | 165 | 6.26 s | 0.083 | — |
| gain 0 (damping only) | 0.79 | 30 | 163 | 6.60 s | 0.077 | 1.8% |
| gain 4, cap 2 | 0.80 | 36 | 157 | 6.82 s | 0.075 | 9.1% |
| **gain 8, cap 2** | **0.83** | 31 | 158 | 7.49 s | 0.078 | 14.3% |
| gain 8, cap 1 | 0.78 | 38 | 156 | 7.06 s | 0.072 | 36.5% |
| gain 8, cap 4 | 0.81 | 41 | 151 | 7.76 s | 0.075 | 7.2% |
| gain 16, cap 2 | 0.85 | 38 | 151 | 7.44 s | 0.079 | 38.7% |
| gain 16, cap 4 | **0.85** | **45** | **145** | **8.82 s** | 0.067 (**−19%**) | 15.0% |

Two things read off this:

* **The damping half alone is worth 0.03 of the 0.07.**  The disturbance is a 2.7 Hz
  texture, so the rate term sees it arriving.
* **The cap, not the gain, is what pins the loop.**  At cap 1 the term is at its limit
  36.5% of stance-leg-steps — bang-bang, not regulation — and scores *worse* than cap 2
  at the same gain.  This is `trot_yaw_moment.md`'s own lesson: a loop needs somewhere to
  sit instead of pinning.

## 5. The falsification criteria, checked

The design said: excursion must fall monotonically with gain, and stride and forward speed
must not move.

| criterion | outcome |
|---|---|
| roll terminations fall with gain | **essentially met**: 165 → 163 → 157 → 158 → 151. Falls, but not strictly — gain 8 is one cell above gain 4. |
| upright time rises with gain | **met to gain 8** (6.26 → 6.60 → 6.82 → 7.49); gain 16 at cap 2 is 7.44, flat. |
| stride does not move | **structurally guaranteed, not measured.** The gait is an open-loop clip replayed at a fixed phase advance; the phase counter is a constant of the harness. Stated rather than claimed as a result. |
| v_x within ±10% | **met at gain 8 / cap 2** (−6%, −5%, −1% across seeds). **Failed at gain 16 / cap 4** (−19%) and **failed badly on TROT** (−32%). |

**So the recommended operating point is gain 8, cap 2.0, damp 0.8, sign +1** — not the
highest-scoring one.  Gain 16 / cap 4 scores 0.85 against 0.83 and it buys the difference
with 19% of the forward speed, which the criterion forbids and which the earlier three foot
heights in this project are a warning about: a number that improves while the thing it was
supposed to measure quietly changes.

## 6. The controls — it is not a general balance improvement

| gait | off | on | Δ | v_x |
|---|---|---|---|---|
| WALK (target) | 0.76 | 0.83 | **+0.08** | −6% |
| TROT | 0.29 | 0.34 | +0.05 | **−32%** |
| TURN | 0.17 | **0.14** | **−0.04** | +12% |

**TROT's gain fails the criterion**: it survives longer by moving slower, which is not what
the term is for.  **TURN is worse with it on.**  TURN is trying to rotate in place and this
term resists exactly the body motion it is trying to produce, which is the expected sign
and is why the control was required.

So the term is **WALK-only by table**, the way the yaw couple is TROT-only.

## 7. Against the prediction

The design estimated the term would recover about two thirds of the roughness loss, i.e.
**≈0.98** against WALK's 0.76.

**Measured: 0.82 (mean of three seeds) at the compliant operating point, 0.85 at the
non-compliant one.**  The prediction was about twice the delivered effect.

Where the estimate went wrong is visible in the split it was built on: of the 64 cells WALK
survives without roughness, 44 stop surviving with it, and the design assumed a balance
term recovers that class.  It recovers **6 to 18 of them** (alive 29→31, 24→42, 29→35).
The rest are not falls the couple can catch — the hip's lateral force can right a lean, and
it cannot put a foot somewhere else, which is what a 4 cm step under one foot asks for.

## 8. What it is worth, in the table that matters

| | score / 8 |
|---|---|
| E2E teacher, same protocol, seed 1 | 4.83 |
| oracle over the whole skill library | 0.81 |
| **Rule-Planner, depth, roll couple ON** | **0.69** |
| WALK fixed, roll couple ON (mean of 3 seeds) | 0.82 |
| WALK fixed, off | 0.76 |
| Rule-Planner, depth, off | 0.60 |

The couple moves the arm from 0.60 to 0.69 and moves the single-skill lower bound from
0.76 to 0.82, so **the planner is still below its own lower bound and the gap is
unchanged**.  It is a real 15% on the arm and it is not a route to 4.83.  §1 of
`oracle_and_depth.md` still governs: the library is the question.
