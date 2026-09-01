# Pushing the skills — what moved, what did not, and what is now excluded

All 200 cells, legged_eval terrain, 20 s episodes.  Rule-Planner on **depth**.  Every arm
paired off/on, seeds where stated.

## 1. Level 2 on depth, three seeds (the missing repeats)

| seed | off | on | Δ | upright | v_x | cells better / worse |
|---|---|---|---|---|---|---|
| 1 | 0.60 | 0.69 | +0.09 | 5.04 → 5.76 s | 0.067 → 0.060 (−10 %) | 33 / 16 |
| 2 | 0.54 | 0.62 | +0.08 | 4.87 → 5.21 s | 0.058 → 0.057 (−1 %) | 31 / 13 |
| 3 | 0.52 | 0.59 | +0.07 | 4.52 → 5.26 s | 0.047 → 0.054 (**+14 %**) | 25 / 11 |
| **mean** | **0.55** | **0.63** | **+0.08** | | | |

Positive in all three, better-than-worse in all three, and seed 3 is *faster* with the term
on, so there is no speed cost across the set.  **The arm's number is 0.63, not the 0.69 of
seed 1** — that one was the top of the spread.

## 2. WALK swing lift — the hypothesis is refuted, and the reason is new

The proposal: the earlier 0/40/60/80 ladder found the untouched recording best, but it was
open loop; with the roll couple holding the roll the trade might land elsewhere.

WALK, seed 1, all 200 cells:

| | score | alive | upright | v_x |
|---|---|---|---|---|
| roll off, lift 0 | 0.76 | 29 | 6.26 s | 0.083 |
| roll off, lift 20 | 0.72 | 18 | 5.47 s | 0.097 (**+17 %**) |
| roll off, lift 40 | 0.78 | 16 | 4.74 s | 0.104 (**+26 %**) |
| **roll on, lift 0** | **0.83** | **31** | **7.49 s** | 0.078 |
| roll on, lift 20 | 0.76 | 25 | 6.29 s | 0.086 (+10 %) |
| roll on, lift 40 | 0.82 | 20 | 5.11 s | 0.092 (**+19 %**) |
| roll on, lift 60 | 0.83 | 12 | 4.77 s | 0.103 (**+32 %**) |

**It does not help, and the flat score is hiding two things moving in opposite directions.**
Survival collapses monotonically (31 → 25 → 20 → 12) and forward speed rises monotonically
(+10 → +19 → +32 %).  The score stays at 0.82–0.83 because the robot covers the same ground
faster before falling over.

That is a **speed edit**, which is exactly what `swing_lift_offsets` was built not to be —
its `A·sin²(πφ)` shape has zero value *and zero slope* at liftoff and touchdown precisely so
that stride length and forward speed are untouched.  The shape does what it claims in
joint space; the outcome in the world is still 32 % more speed.  Worth recording as a
failure of that argument, not of the implementation.

**It fails the acceptance criterion outright** (v_x must stay inside ±10 %), so it is
rejected on the criterion and not on the score.  `--swing-lift` stays, default 0.

The user's hypothesis was that the roll couple would change the trade.  **It does not**:
with the couple on, lift still costs survival at exactly the same rate.

## 3. TURN — the clip is genuine, the replay is not

Two separate questions, both answered.

**Is `SPEED_TURN = 0.0075 m/s` the clip or a replay loss?**  The clip.
`data/skill_clips.meta.json`, session `turn_right_20260824_223951`:
`vx_steady_mean = 0.00751`, `flight_frac = 0`, `duty 0.715`.  **The real robot's own logged
forward speed.**  It really is an in-place turn.

**Is the logged yaw rate reproduced?**  No — about a third of it.

| configuration | yaw rate | of the log | v_x |
|---|---|---|---|
| **real robot log** | **−22.66 °/s** | 100 % | **+0.0075 m/s** |
| sim, foot-comp on, entry `first` | −7.76 °/s | **34 %** | **−0.078 m/s** |
| sim, foot-comp **off** | −7.13 °/s | 31 % | −0.095 m/s |
| sim, foot-comp on, entry `measured` | −8.20 °/s | 36 % | −0.088 m/s |

Two things follow, and one candidate is excluded.

* **It is not the foot-placement law.**  Turning it off makes the yaw rate *worse*, so the
  compensator is not what is eating the turn.  `turn_target.md` fixed its *target*; the
  residual is not there.
* **It is not the entry phase.**  The measured frame gives 36 % against 34 %.
* So it is the **open-loop replay of the clip itself**.  And the sim TURN does not merely
  turn slowly, it **translates backwards at 10× the logged forward speed** — it is not the
  recorded motion at a slower rate, it is a different motion.

**This reframes TURN.**  A 90° re-aim costs 4.0 s at the logged rate and **11.6 s at the
replayed one**, against a median upright time of 3.4 s.  TURN is not inherently too slow to
use; **our replay of it is.**  That is a plant/fidelity question, and it is now the largest
single unexplained gap in the library.

## 4. RUN — the roll couple cannot lift the body, measured

| | peak base height, median | upright | score |
|---|---|---|---|
| WALK reference (standing ≈ 0.33 m) | **0.330 m** | — | 0.76 |
| RUN, couple off | 0.313 m | 1.54 s | 0.00 |
| RUN, couple **on** | **0.311 m** | 1.31 s | 0.00 |

**No.**  The body never rises above its standing height in either arm, and the couple makes
RUN slightly worse.  The mechanism says why and the measurement agrees with it: a hip
(abduction) torque on a stance leg produces a force **perpendicular to the hip→foot vector
in the y–z plane**, which for a near-vertical leg is essentially horizontal.  There is no
vertical component to give.  Level 2 pushes the robot sideways, not up.

RUN stays unresolved for the reason it always had — no ballistic phase in replay — and
this excludes one more candidate route to it.

## 5. JUMP — recorded, unchanged

23.70 / 45.43 N·m are Unitree's published **maxima**, not ratings; no continuous rating is
published.  Gaps on this grid run 0.10–5.15 m against `front_jump`'s 26 ± 4 mm of
horizontal travel — 4× to 200× short.  Nothing changed and nothing was attempted.

## 6. The planner's cross-track problem — diagnosed correctly, and the fix is a null

**The path exists and was wired to the wrong signal.**  `RulePlanner._wants_turn` documents
its argument as *"goal bearing minus heading"*.  `run_benchmark.py` was passing `psi` — the
deviation from the heading the robot **settled at**.  So the planner has been turning to
restore its start heading, never to aim at a goal, which is why `staircase_spiral`'s goal
0.30 m to the right was never turned towards.

`--turn-target goal` computes the real bearing (and re-baselines heading hold when a TURN
ends, so the low level does not undo it).  Three seeds, planner on depth with the roll
couple:

| seed | settle (default) | goal | Δ | frac_TURN | cells better / worse |
|---|---|---|---|---|---|
| 1 | 0.69 | 0.47 | **−0.22** | 0.241 → 0.401 | 9 / 50 |
| 2 | 0.62 | 0.44 | **−0.18** | 0.230 → 0.381 | 11 / 45 |
| 3 | 0.59 | 0.36 | **−0.23** | 0.265 → 0.388 | 4 / 47 |

**Much worse, in all three seeds, and it did not even help the two courses it was diagnosed
on** — `staircase_spiral` and `staircase_walking_full_width` are still 0.00, and the median
distance to the live goal got slightly *worse* (2.25 → 2.43 m, 0.93 → 1.05 m).

The two findings interlock and the explanation is §3.  Aiming correctly makes the planner
turn **more** (TURN share 0.24 → 0.39), and a TURN costs 11.6 s per 90° in replay against a
3.4 s median life.  **A better aim is worthless while the aiming mechanism is 3× too
slow.**  The diagnosis was right about the wiring and wrong about what it was worth.

`--turn-target` stays, default `settle`, and this is recorded as a null.  It becomes worth
re-running the moment §3's replay gap closes — that is the ordering, and it was not
visible before both were measured.

## 7. The oracle, recomputed on the improved skills

| | WALK | TROT | TURN | **ORACLE** | headroom over WALK | cells where a non-WALK skill wins |
|---|---|---|---|---|---|---|
| roll couple **off** (the old number) | 0.76 | 0.29 | 0.17 | **0.81** | +0.06 | **12 / 200** (TROT 7, TURN 5) |
| roll couple **on** | 0.83 | 0.34 | 0.14 | **0.88** | +0.05 | **9 / 200** (TROT 8, **TURN 1**) |

**Improving the skill raises the ceiling by 0.07 and leaves the planner's room exactly where
it was.**  The headroom a perfect chooser has over always-walking is 0.05 against 0.06, and
the number of cells where choosing matters *falls* from 12 to 9 — because the couple helps
WALK more than it helps the alternatives.  TURN's contribution collapses from 5 cells to 1.

This is the same conclusion as before, sharpened by an intervention that worked: **skill
work moves the ceiling, rule work does not.**

## 8. Where this leaves each skill

| skill | before | after | what closed, what did not |
|---|---|---|---|
| WALK | 0.76 | **0.83** | roll couple, inside every criterion.  Swing lift **excluded** — it is a speed edit and costs survival |
| TROT | 0.29 | 0.34 | roll couple helps the score and **fails the v_x criterion** (−32 %), so it is not adopted.  The 0.02–0.04 m step limit is untouched |
| TURN | 0.17 | 0.14 | roll couple is **harmful** here.  New: the replay delivers **31–36 %** of the logged yaw rate and translates backwards — the largest unexplained gap found |
| RUN | 0.00 | 0.00 | one more route excluded by measurement: the couple cannot lift the body (0.313 → 0.311 m) |
| JUMP | 0.00 | 0.00 | closed, recorded |
| planner | 0.55 | **0.63** | level 2.  The goal-bearing fix is correct wiring and a measured null while TURN is slow |
