# `--rate hi`: an 8.3× slowdown that helps TURN and destroys WALK

Two things were true at once and looked contradictory. Both are now measured.

## What `hi` and `lo` are

The **same single gait cycle**, sampled at two rates, and every harness here plays **one
frame per control step at 50 Hz**:

| clip | `lo` | `hi` | cycle at 50 Hz, lo → hi | slowdown |
|---|---|---|---|---|
| WALK | 37 frames @ 50.6 Hz | 306 @ 418.3 Hz | 0.74 s → 6.12 s | **8.3×** |
| TROT | 32 @ 49.8 | 268 @ 416.9 | 0.64 → 5.36 | 8.4× |
| TURN | 45 @ 50.4 | 374 @ 419.1 | 0.90 → 7.48 | 8.3× |

So `lo` is a **decimated** recording — 37 samples of a 418 Hz signal — and `hi` is the full
one, stretched in time because the harness does not resample.

## It destroys any skill that has to cover ground

Benchmark, 200 cells, no roughness, roll couple on:

| | score | v_x | alive |
|---|---|---|---|
| WALK `lo` | **1.090** | +0.1070 m/s | 74/200 |
| WALK `hi` | **0.005** | **−0.0045 m/s** | 3/200 |

An 8.3× slower walk covers nothing in a 20 s episode.

## And it genuinely improves TURN, measured per second

Flat rig, `verify_skill_replay --clip TURN`, 6 cycles:

| rate | yaw rate | **of the log's −22.66 °/s** | stride cv | v_x |
|---|---|---|---|---|
| `lo` | −14.37 °/s | **63 %** | 0.000 | **−0.0296** |
| `hi` | **−19.28 °/s** | **85 %** | 0.046 | **+0.0149** |

**Per second, not per cycle** — and it confirms SESSION_STATE §12's 72 % → 83 % in direction
and size. The sign of `v_x` also comes right: the log is **+0.0075 m/s** and `lo` translates
*backwards* at −0.0296 while `hi` gives +0.0149.

## Why both are true

They are not the same motion at two speeds. **`lo` is a lossy version of the motion.** For a
skill that must travel, the slowdown dominates and `hi` is fatal. For a skill that turns in
place, travel is not the currency, and the fidelity `lo` threw away is worth more than the
time it costs.

## What this does to TURN's diagnosis

`skill_push_2026-09-02.md` §3 calls the replayed yaw rate "the largest single unexplained gap
in the library" at 34–36 % of the log, and reframes TURN as too slow to use (11.6 s per 90°
against a 3.4 s median life). Measured **per second at rate `lo` on the benchmark**. Splitting
it:

| | of the log |
|---|---|
| benchmark, `lo` | **35 %** |
| flat rig, `lo` | **63 %** |
| flat rig, `hi` | **85 %** |

**The gap is two gaps.** About **28 points is the rig** (benchmark vs flat at the same rate)
and about **22 points is the sampling** (`lo` vs `hi` on the same rig). Neither is a plant
mystery, and "the largest single unexplained gap" overstates what is left.

**This does not license `--rate hi`.** It is 8.3× real time; a 90° turn that costs 11.6 s at
`lo` costs far more at `hi`, so a *better* turn per second is still a worse turn per episode
unless the harness resamples. **The fix is resampling `hi` to the control rate, which nothing
here does and which is not a measurement — it is a change to how clips are played, and it
would move every published number.** Recorded, not made.

## Status

`planner.config.RATE_*` stay `lo`, provenance `CALIBRATION_NEEDED`. `--rate per-clip`
**refuses** rather than silently running a skill at 8.3× real time. `--rate hi` remains
available for exactly what it is good for: measuring a clip's motion without the decimation,
on a rig where wall-clock time is not being scored.
