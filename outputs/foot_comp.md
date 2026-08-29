# Stage 2 — Raibert foot placement: how far from the recording does a trot have to go?

**Question.** Stage 1 (`--balance-comp pd`, attitude PD on the hips) did not hold any of
these gaits, and the reading it left was that a trot is held up not by where the body is
but by **where the next foot lands**. The constraint that the replay must stay the
recorded trajectory has since been released, on the condition that the skill stays the
same skill. So the recording is no longer the output — it is the **baseline**, and the
result is a budget: *how much of it has to be overwritten before the gait stands up.*

**Answer.** Between **0.02 and 0.03 rad** of hip deviation, in swing only.
At 0.03 rad TROT runs the full 60 cycles at exactly the logged stride, with lateral
drift eliminated and **forward speed higher than the uncompensated replay, not lower**.
The correction touches one joint of four legs for 42% of each cycle; every other joint,
and every loaded leg, plays the recording unmodified.

Everything here is flat ground. No terrain, no depth, no benchmark score, nothing
learned: the law's only inputs are the base's own lateral velocity, the clip's measured
stance time, and the robot's own hip-to-foot geometry.

---

## 1. The law

    foot_y = (T_stance / 2) · v_y  +  k · (v_y − v_y_target),     v_y_target = 0

- The leading term is the **neutral point** and has **no free parameter**:
  `T_stance` is duty × period taken from the clip's own contact channel
  (`stance_time_s`). TROT 0.372 s, WALK 0.484 s, RUN 0.111 s, TURN 0.600 s — each
  within 1% of the same quantity computed from the meta's `duty_mean / stride_hz`.
- `k` (`--foot-k`) is velocity damping on top, and **defaults to 0**. The sweep below
  shows it is not needed.
- `v_y_target = 0` because the clips are straight-line gaits whose logged lateral speed
  is ~0. This assumption is what breaks TURN — see §5.

**Joint mapping.** Every Go2 hip rotates about +x, so a positive hip angle moves that
foot toward +y on *every* leg (which is why the zero-action stance is +0.1/−0.1/+0.1/−0.1).
The lever is therefore the hip-to-foot **vertical drop**, measured once from the robot's
own body positions after the settle: FL/FR 0.306 m, RL/RR 0.316 m. So 0.05 rad of hip
is 15 mm of foot travel, and 0.03 rad is 9 mm.

**Swing only.** The correction is added to a leg's hip target for the frames the
recording has that leg in the air (`--foot-swing-source clip`, phase-locked). A loaded
leg keeps the recording exactly. The correction is therefore released at touchdown with
a step of at most `--foot-clip-rad`, and that step is deliberately not smoothed: the cap
is the quantity being swept, so it must bound every edit the run makes.

**This closes a loop.** A run with `--foot-comp raibert` reads the base's lateral
velocity and is not an open-loop replay. It is banner-printed at the top of every run
and stamped into every results row, exactly as `--balance-comp` is.

---

## 2. The cap ladder (TROT, 60 cycles, flat ground)

Expected from the log: stride 1.56 Hz, vx 0.444 m/s.

| cap (rad) | survived | stride | vx (m/s) | vy (m/s) | hip RMS in swing | max edit | cap bound |
|---|---|---|---|---|---|---|---|
| off        | 1.95 s  | 1.56 (+0%) | 0.383 | −0.216 | 0      | 0      | — |
| 0.00 (null) | 1.95 s | 1.56 (+0%) | 0.383 | −0.216 | 0      | 0      | 100% |
| 0.02       | 2.55 s  | 1.49 (−4%) | 0.434 | −0.189 | 0.0194 | 0.0200 | 91% |
| **0.03**   | **60+ cycles** | **1.56 (+0%)** | **0.660** | +0.016 | 0.0199 | 0.0300 | 20% |
| 0.04       | 60+ cycles | 1.56 (+0%) | 0.658 | +0.014 | 0.0209 | 0.0400 | 9% |
| 0.05       | 60+ cycles | 1.56 (+0%) | 0.659 | +0.015 | 0.0222 | 0.0500 | 2% |
| 0.10       | 60+ cycles | 1.56 (+0%) | 0.659 | +0.014 | 0.0226 | 0.0718 | 0% |
| 0.20       | 60+ cycles | 1.56 (+0%) | 0.659 | +0.014 | 0.0226 | 0.0718 | 0% |
| 0.40       | 60+ cycles | 1.56 (+0%) | 0.659 | +0.014 | 0.0226 | 0.0718 | 0% |

- **The null control holds.** `--foot-clip-rad 0` reproduces `--foot-comp off` to the
  digit — same 1.95 s, same stride, same vx, same vy. The ladder is a comparison.
- **The threshold is between 0.02 and 0.03 rad.** 0.02 buys 0.6 s and no more; 0.03 runs
  out the clock.
- **The law saturates at 0.0718 rad.** Above 0.10 the cap never binds and every run is
  the same run. There is no benefit to allowing more deviation, so the budget is not a
  tuning knob with an open top — it is a measured demand.
- **This is not the failure mode named in advance.** Stance-widening and the attitude PD
  both suppressed roll by taking away the travel. Here roll goes away *and* vx goes
  **up**: 0.383 → 0.659 m/s. Against the log's 0.444 m/s the compensated run is 48%
  fast (same order of magnitude, inside the ×3 gate), and the uncompensated one was
  14% slow — but the uncompensated one was also falling over, so neither is a clean
  reading of speed.
- Roll: |roll| max 63.4° (a fall) → **3.6°**, RMS 18.7° → **0.7°**. Base height 0.324 →
  0.334 m. Stride CV 0.019 over 59 measured cycles.

---

## 3. The direction was measured, not reasoned

Stage 1 found the stabilising sign was not the one reasoning about the joint frame
predicted, so this one was put to the same test. `--foot-sign -1` at cap 0.05:

| | survived | stride | vx | vy |
|---|---|---|---|---|
| sign +1 (geometry's) | 60+ cycles | 1.56 (+0%) | 0.659 | +0.015 |
| sign −1 | **1.65 s** (worse than no compensation) | — | 0.237 | −0.217 |

The geometric prediction and the measurement agree this time. Reversed, the correction
is actively harmful: it falls sooner than the untouched replay and loses 40% of its
forward speed on the way down.

---

## 4. The control: does it break what already worked?

WALK is the gait that survives open loop. It has to come out unchanged, or the
compensator is not a fix but a different gait.

| WALK | survived | stride | vx | vy | roll RMS | hip RMS in swing |
|---|---|---|---|---|---|---|
| off  | 60+ cycles | 1.37 (−0%) | 0.233 | −0.000 | 2.0° | 0 |
| cap 0.05 | 60+ cycles | 1.37 (−0%) | 0.231 | −0.003 | 2.4° | 0.0402 |

**It does not break it.** Forward speed 0.233 → 0.231 m/s — under 1%. Compare what the
two earlier compensators cost WALK: stance-widening 0.233 → 0.167, attitude PD
0.233 → 0.148. Both took a third of the travel; this takes 1%.

WALK is also the like-for-like case for the deviation table, because both runs go the
full 60 cycles. Measured joint deviation from the recording, RMS in rad:

| joint / phase | WALK off | WALK cap 0.05 |
|---|---|---|
| hip, swing | 0.0277 | **0.0492** |
| hip, stance | 0.1005 | 0.1026 |
| thigh, swing | 0.0464 | 0.0464 |
| thigh, stance | 0.0803 | 0.0819 |
| calf, swing | 0.0428 | 0.0421 |
| calf, stance | 0.2316 | 0.2322 |

One cell moves. Everything else is the recording, to three decimal places.

(`walk_off` was run twice, at the start and the end of the sweep, and the two rows are
identical — the harness is deterministic, so the differences above are the compensator's
and not the sim's.)

---

## 5. The same correction on the other skills

| clip | off | cap 0.05 | reading |
|---|---|---|---|
| TROT | 1.95 s | **60+ cycles**, stride +0% | the target case |
| WALK | 60+ cycles | 60+ cycles, unchanged | the control |
| TURN | 8.82 s | **5.65 s**, stride −37% | **worse** |
| RUN  | 1.13 s | 1.13 s | no change |

- **TURN gets worse, and it should.** The law aims for `v_y = 0`. In an in-place turn
  the body's lateral velocity is not an error — it is part of the motion, so nulling it
  fights the skill. This is a property of the target, not of foot placement: a turn
  needs its own `v_y_target`, taken from its own log, before this term means anything
  for it. Left as a finding rather than patched, because patching it here would be
  choosing a target to make a number look better.
- **RUN is untouched by it.** Duty 0.31 with a flight phase; the correction is applied
  on 100% of control steps and the collapse time does not move by one control step.
  Foot placement is not what RUN is missing.

---

## 5b. Two knobs that turned out not to matter

**The damping term is not needed.** `--foot-k` adds `k · (v_y − v_y_target)` on top of
the neutral point. TROT at cap 0.05:

| k (s) | survived | stride | vx | vy | hip RMS in swing |
|---|---|---|---|---|---|
| 0.00 | 60+ cycles | 1.56 (+0%) | 0.659 | +0.015 | 0.0222 |
| 0.05 | 60+ cycles | 1.56 (+0%) | 0.661 | +0.014 | 0.0279 |
| 0.15 | 60+ cycles | 1.56 (+0%) | 0.664 | +0.013 | 0.0345 |

Identical outcomes for 26% and 55% more overwriting. The parameter-free law is the one
to report, and `k` stays at its default of 0 — a knob that buys nothing should not be
carried as if the result depended on it.

**The swing gate can come from either place.** The default reads the recording's own
contact channel, which is phase-locked and identical run to run. Reading the sim's
contact sensor instead (`--foot-swing-source sim`, legs resolved by name, never by
position) gives the same run: 60+ cycles, stride +0%, vx 0.657, vy +0.015, with the
correction on 95.3% of control steps instead of 90.6%. The result does not rest on the
clip's labels being right about the sim's feet — which they are known not always to be
(`level_start`).

---

## 6. What the compensation costs, per joint

TROT at cap 0.05, 60 cycles, RMS in rad. `cmd` is what was overwritten; `meas` is what
the joint actually did against what the recording commanded.

| joint | phase | cmd RMS | cmd max | meas RMS |
|---|---|---|---|---|
| hip | swing | 0.0169 – 0.0273 (per leg) | 0.0500 | 0.0336 |
| hip | stance | **0** | **0** | 0.1162 |
| thigh | swing | **0** | **0** | 0.0535 |
| thigh | stance | **0** | **0** | 0.1209 |
| calf | swing | **0** | **0** | 0.0689 |
| calf | stance | **0** | **0** | 0.2440 |

The zeros are the design claim, checked rather than asserted: only the hip, only in
swing. Per-leg the commanded RMS runs 0.0169 (RR) to 0.0273 (FR) — the front-right and
rear-left legs carry more of the correction, which is the diagonal the run was falling
onto.

**Overwritten time.** 90.6% of control steps contain at least one corrected leg;
42.2% of leg-steps are corrected, which is exactly the clip's swing fraction — every
swing sample gets a correction, none is skipped. Per-run numbers are in the
`overwrite_frac_time` / `overwrite_frac_legsteps` columns.

The `meas` column is *not* a cost of the compensator: the uncompensated TROT deviates
more (thigh swing 0.186 rad against 0.054), because it is falling over while it does it.
Its 98 steps and the compensated run's 1920 are not the same window, which is why WALK
in §4 is the honest comparison.

---

## 7. What this does not fix

The compensated trot travels 15.5 m in 38.6 s and **yaws 5.1 °/s while doing it** —
about 7.8 °/m. The benchmark's heading budget is 0.565 °/m (`analyze_heading_budget.py`),
so this run curves 14× more than a benchmark traverse allows. Foot placement bought the
gait; it did not buy a heading. That was already the open item for WALK (11.8 °/m) and
it is unchanged: **heading correction is a separate mechanism and still missing.**

Also unchanged: the sim trot is 48% faster than the log's 0.444 m/s. Stride is exact, so
this is step length, not step rate — the sim robot covers more ground per stride than the
real one did. Worth naming before any distance-based threshold is derived from it.

---

## 8. Reproducing

    scripts/run_foot_comp_sweep.sh A                    # the cap ladder + the WALK control
    scripts/run_foot_comp_sweep.sh run <tag> <clip> --foot-comp raibert --foot-clip-rad 0.05
    python3 scripts/foot_comp_table.py                  # the table above, from the CSV

Rows land in `outputs/foot_comp.csv` (one per run, every knob stamped) and the per-leg /
per-joint / per-phase departure table in `outputs/foot_comp_dev.csv`. Traces are under
`outputs/traces/footcomp/`. Every run in this document is
`--rate lo --hip-sign keep --start-phase level --settle-mode stand --contact-threshold-n 30
--cycles 60`, physics on CPU at 200 Hz, ground friction 1.3.

## 9. A harness defect found on the way

`SimulationApp.close()` in `main()`'s `finally` was tearing the process down *before*
the traceback of an exception on its way out of the `try` could be printed. A run that
raised looked like a clean `exit 0` that had simply stopped talking half way through its
report — two runs were spent finding that out. `main()` now prints the exception before
closing the app. This is the sixth entry in the pattern `harness_findings.md` exists for:
the failure was silent, and the silence looked like success.
