# TURN: the foot-placement target is not zero, it comes out of the rotation

Stage 2 made TURN worse — 8.82 s open loop down to 5.65 s with the compensator on, stride
−37%. The reason was in the law, not in the mechanism: it drove every foot toward
`v_y = 0`, and for an in-place turn a lateral velocity is not an error to null. It is the
motion.

This is that fixed, with no parameter chosen to make it work.

---

## 1. The derivation

A rigid base with body-frame linear velocity **v** and yaw rate ω does not carry all four
hips at the same velocity. The hip at body-frame offset **r**ᵢ = (xᵢ, yᵢ) moves at

    v_i = v + ω ẑ × r_i     →     v_i,y = v_y + ω · x_i
                                  v_i,x = v_x − ω · y_i

So the lateral velocity a given foot has to answer for is not the base's — it is the
base's **plus ω·xᵢ**, which has opposite signs on the front and rear pairs. The
placement law's error term becomes, per leg,

    Δv_i,y = (v_y − v_y_log) + (ω − ω_log) · x_i

Two things go into that and **both are measured**:

| quantity | value | where from |
|---|---|---|
| ω_log (TURN) | **−0.3954 rad/s = −22.66 °/s** | `outputs/skill_profile.csv`, `yaw_rate_steady_mean` for session `turn_right_20260824_223951` |
| v_y_log (TURN) | +0.0143 m/s | same row, `vy_steady_mean` |
| x_i | **±0.193 m** | the articulation's own hip body positions, rotated into the body frame at settle |
| y_i | ±0.047 m | same |

Note ω_log is the **measured** −22.66 °/s, not the −0.6 rad/s (−34.4 °/s) that was
commanded to produce it. CLAUDE.md §3 already says command ≠ measurement for speed; it is
just as true for turn rate, and using the command here would have been choosing a target.

The per-leg targets that fall out for TURN:

    FL, FR (front, x = +0.193):  v_y target = +0.0143 + (−0.3954)(+0.193) = −0.0622 m/s
    RL, RR (rear,  x = −0.193):  v_y target = +0.0143 + (−0.3954)(−0.193) = +0.0908 m/s

The front feet should be tracking right at 62 mm/s while the rear feet track left at
91 mm/s. Driving all four to zero — what stage 2 did — is a command to stop turning, and
the measurement agrees that is exactly what happened: yaw rate fell from −14.70 °/s
uncompensated to **−7.96 °/s** with the compensator on. It was cancelling the skill.

On a straight clip ω_log ≈ 0 and this reduces to the stage-2 law plus a yaw term, so the
change is backward compatible by construction and `--foot-yaw off` reproduces the shipped
stage-2 rows bit for bit (checked: `e_regress_off`, `f_regress`, `h_trot_best`).

---

## 2. What it does

TURN, 60 cycles, flat ground. Expected from the log: stride 1.12 Hz, yaw −22.66 °/s,
in place (|v| < 0.05 m/s).

| target | cap | survived | stride | yaw °/s | vx | hip RMS in swing |
|---|---|---|---|---|---|---|
| — (open loop) | — | 8.82 s | 1.12 (−0%) | −14.70 | 0.003 | 0 |
| v_y = 0 (stage 2) | 0.05 | **5.65 s** | 0.71 (−37%) | **−7.96** | 0.005 | 0.0445 |
| rotation, instant | 0.02 | 19.39 s | 1.20 (+7%) | −14.87 | 0.008 | 0.0189 |
| rotation, instant | 0.03 | 32.46 s | 1.15 (+2%) | −16.99 | 0.016 | 0.0272 |
| rotation, instant | 0.04 | **60+ cycles** | 1.15 (+2%) | −18.91 | 0.019 | 0.0349 |
| rotation, instant | 0.05 | 60+ cycles | 1.26 (+12%) | −18.22 | 0.019 | 0.0424 |
| rotation, instant | 0.10 | 7.40 s | 1.15 (+2%) | −18.09 | 0.025 | 0.0761 |
| rotation, instant | 0.20 | 10.75 s | 1.16 (+3%) | −20.46 | 0.029 | 0.1234 |
| **rotation, cycle-mean** | **0.05** | **60+ cycles** | **1.12 (−0%)** | **−20.66** | 0.020 | 0.0424 |
| rotation, cycle-mean | 0.10 | 60+ cycles | 1.12 (−0%) | −16.32 | 0.017 | 0.0664 |

- **From 8.82 s to 60 cycles.** The budget where it first runs out the clock is
  **0.04 rad** (12 mm of foot travel), against TROT's 0.03. A turn costs more overwriting
  than a trot, which is what you would expect of the skill that is further from the
  compensator's straight-line assumptions.
- **It still turns, and the best configuration turns nearly as fast as the log.** The
  cycle-averaged target at cap 0.05 holds **−20.66 °/s against the log's −22.66 — 91%** —
  while reproducing the stride exactly, on 0.0424 rad of hip RMS in swing. The
  uncompensated replay managed only 65% of the logged turn rate before falling over, and
  the v_y = 0 target managed 35%.
- **It stays in place.** vx 0.017–0.019 m/s against the log's 0.008, inside the 0.05
  in-place gate.
- **The instantaneous variant is non-monotonic in the cap** — fine at 0.04–0.05, worse
  again at 0.10 and 0.20. That is the noise, not the law: see §3. Averaged over one
  cycle, both caps reproduce the stride exactly and cap 0.05 is the best run in the
  table on every count: longest survival, exact stride, closest turn rate, and the
  smallest departure from the recording of any run that survives.

---

## 3. The yaw rate has to be averaged over a cycle, and the window is free

Feeding back the *instantaneous* yaw rate feeds back the stride. Measured on the working
TROT run:

| | mean | sd |
|---|---|---|
| instantaneous yaw rate | +5.32 °/s | **20.70 °/s** |
| averaged over one clip cycle (32 steps) | +5.32 °/s | **3.64 °/s** |

The signal is 3.9× noisier than the bias it is supposed to correct, and the one-cycle
mean removes 6× of that while keeping the mean exactly. **The averaging window is the
clip's own period**, so this introduces no cutoff frequency to tune —
`--foot-yaw log-cycle` reads the cycle length out of the clip being played.

That the noise, not the law, was the problem shows up twice: the instantaneous variant
destabilises TROT completely (60 cycles → 2.71 s at cap 0.05, where it saturates the cap
on 75% of swing samples), and on TURN it makes the cap non-monotonic, because a small cap
was acting as a noise limiter rather than as a budget.

---

## 4. A diagnostic that was calling the working turn a bug

`sim/diagnose.py` warned "yaws at 16.3 deg/s **while commanded straight**" — on the TURN
clip, whose entire content is a yaw rate, and it named a leg-indexing error as the likely
cause. It compared the measured yaw rate against zero for every clip.

It now compares against the log's own yaw rate, which `expected_from_meta` carries for
each clip, and reports `turn rate −16.3 °/s vs the log's −22.7` as an ordinary
measurement. A clip with no recorded turn keeps the old test, because for the straight
clips zero is the right reference.

Worth stating plainly: this misdiagnosis fired on the one run that finally reproduced
TURN's stride. A verdict machine that cannot represent "this skill is supposed to turn"
will call every success a mapping bug.
