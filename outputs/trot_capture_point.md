# TROT: the Raibert rear term as a capture point — three changes, separated, all null

Asked: `quadruped_pympc` writes the rear term as `sqrt(com_height/g) × (v_avg − v_ref)`
with the offset clipped to ±0.05 m and `v_avg` a 20-sample moving average; ours is
`(T_stance/2 + k)(v_y − v_y_target)` with `k = 0`, clipped at ±0.05 **rad** of hip. Does
adopting their form make TROT go straight?

**No. None of the three differences moves it, individually or together.** The one that
looks like it does — the metre clip — is a cap widening, and cap widening was already
measured as not helping (`trot_straight.md` §3, `lip_failure.md` §3).

All CPU, no GPU. 375 grid runs, paired.

---

## 1. Three differences, three flags, so they can be attributed

They are not one change and lumping them would make the result unreadable:

| | ours | theirs | flag (default OFF) |
|---|---|---|---|
| gain | `T_stance/2` = **0.186 s** | `sqrt(h/g)` = **0.178 s** | `--foot-gain capture` |
| velocity | instantaneous `v_y` | 20-sample moving average | `--foot-vy-avg-n 20` |
| clip | ±0.05 **rad** of hip | ±0.05 **m** of foot offset | `--foot-offset-clip-m 0.05` |

**The gain is not the difference — it is 4.4% apart.** That is worth saying up front because
`sqrt(h/g)` looks like a different law and is arithmetically the same number. `T_stance/2`
is a property of this gait; `sqrt(h/g)` is a property of this body; on TROT they coincide.

**The clip is a 3× difference and does not look like one.** Our ±0.05 rad through a 0.31 m
hip-to-foot lever is **±0.0155 m** of foot travel. Theirs is ±0.05 m. Same-looking number,
a third of the authority.

`sim/footcomp.py` carries all three; its self-test asserts the default path is
**bit-identical** to the law before they existed.

## 2. The paired result

`run_calibration_grid.py --params skill.STEP_TROT_MAX --reps 5 --foot-comp on --heading
heading-only`: the 15-cell `step_up` ladder × 5 entry phases = 75 runs per arm, and cell
*k* sits at identical world coordinates in every arm. This is the paired design
`trot_straight.md` §4b had to introduce after a single-cell result did not replicate.

| arm | | fell (of 75) | paired vs A: fewer falls / more | final distance to goal 2, median Δ |
|---|---|---|---|---|
| **A** | baseline | 69 | — | — |
| G | capture gain | 72 | 2 / 5 | +0.000 m |
| V | v_y averaged | 71 | 2 / 4 | −0.001 m |
| M | metre clip | **63** | **8 / 2** | −0.017 m |
| F | all three | 70 | 3 / 4 | −0.016 m |

**`reached` is 0/75 in every arm**, as it is for every condition `trot_straight.md` tried.

M's 8-fewer-against-2-more is 10 discordant pairs out of 75; under McNemar that is
p ≈ 0.11, i.e. not distinguishable from a coin. The distance medians are ~0.02 m on a
4 m journey.

## 3. And on the quantity actually asked about — straightness — nothing moves

`scripts/analyze_heading_ab.py` on the rep-0 traces, 15 cells, curvature over the approach:

| arm | curvature, median | y drift at the lip, median | fell | reached the lip |
|---|---|---|---|---|
| A baseline | 11.97 °/m | 0.77 m | 11/15 | 1/15 |
| G capture gain | 13.99 | 0.76 | 12/15 | 2/15 |
| V v_y averaged | 11.58 | 1.01 | 13/15 | 3/15 |
| M metre clip | 11.34 | 0.93 | 9/15 | 1/15 |
| F all three | 12.55 | 0.97 | 10/15 | 3/15 |

The five medians span 11.3–14.0 °/m, non-monotone, on a plant whose **per-cell** curvature
in the baseline arm runs 1.7–48.9 °/m. The between-arm differences are inside the
within-arm spread by an order of magnitude. The benchmark's own budget is 0.565 °/m.

## 4. What the sweep did surface, and it is not about the capture point

Falls in the baseline arm, by entry frame, 15 cells each:

| entry frame | 0 | 6 | 13 | 19 | 26 |
|---|---|---|---|---|---|
| cells that fell | **11/15** | 15/15 | 13/15 | 15/15 | 15/15 |

**The entry phase is a bigger lever on TROT than any of the three imported changes.** It is
the same variable that decides TURN outright (`turn_entry_phase.md`) and that moves RUN's
collapse time 76% (`run_swing_gate.md`). TROT's 32 frames have not been swept; TURN's 45
have, and there the good phases were 22 of 45 and not predictable from any pose property.
That is the cheapest untried thing left for TROT.

## 5. Why this was the expected outcome, stated after the fact but derivable before

`trot_straight.md` §3: TROT's disturbance is **DC**, and a proportional law must hold a
standing error to emit a constant output. The capture-point form is still proportional —
in `v_y` rather than in heading, with a gain 4.4% different. Averaging `v_y` changes the
ripple, not the mean. The metre clip raises the authority, and `lip_failure.md` §3 already
measured raising it (±0.08 rad) as producing 5.0% saturation and every robot still falling.

The import was worth running because the reasoning could have been wrong; it was not.

## 6. What was NOT changed

`effort_limit` untouched. The archive untouched. All three flags default off and the
default path is asserted bit-identical. `foot_gain`, `foot_vy_avg_n`, `foot_offset_clip_m`
and the measured `foot_com_height_m` are stamped on every results row, for the same reason
`heading` is: a capture-point row and a half-stance row are different controllers.

`sim/footcomp.py` refuses to build in capture mode without a **measured** `com_height_m`
rather than substituting a nominal 0.31 — the gain *is* `sqrt(h/g)`, so a nominal height
would be a nominal gain wearing a measurement's name.
