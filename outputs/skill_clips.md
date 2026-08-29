# Skill clips

`skill_clips.npz` — sha256 `c1d4860dcbc8cf60…`, 5 clips.

Built by `scripts/extract_skill_clips.py` from the read-only curated logs.
Leg order stored is **FL,FR,RL,RR** (native log order is FR,FL,RR,RL).

## Clips

| clip | kind | source session | hi rate | n hi | lo rate | n lo | cycle / duration | duty | q_des update | loop seam | alias >25 Hz |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `WALK` | cyclic | `gait_classic_walk_20260824_224559` | 418.3 Hz | 306 | 50.6 Hz | 37 | 732 ms | 0.663 | 43 Hz | 0.089 rad (1.16× max step) | 0.266% |
| `TROT` | cyclic | `run_06_20260824_225352` | 416.9 Hz | 268 | 49.8 Hz | 32 | 643 ms | 0.577 | 43 Hz | 0.019 rad (0.15× max step) | 0.427% |
| `RUN` | cyclic | `run_trot_20_lvl0_20260824_232530` | 425.8 Hz | 138 | 49.4 Hz | 16 | 324 ms | 0.326 | 165 Hz | 0.346 rad (1.34× max step) | 1.256% |
| `TURN` | cyclic | `turn_right_20260824_223951` | 419.1 Hz | 374 | 50.4 Hz | 45 | 892 ms | 0.666 | 43 Hz | 0.053 rad (0.48× max step) | 0.615% |
| `JUMP` | oneshot | `front_jump_20260824_232314` | 410.5 Hz | 1805 | 50.0 Hz | 221 | 4.40 s | — | 158 Hz | n/a | 0.352% |

`loop seam` is the wrap-around jump in `q_des`, in radians and as a multiple of the largest sample-to-sample step the clip already contains. At or below 1× the seam is no faster than a transition inside the cycle. One-shot clips are not looped.

`q_des update` is how often the sport controller actually wrote a new command. The log samples at ~419 Hz but holds the value in between, so a clip whose command rate is already near 50 Hz has little left to decimate and one in the hundreds has a lot.

`alias >25 Hz` is the share of `q_des` variance above the 50 Hz Nyquist — what plain decimation would have folded into the gait band. It was removed by the filter, not by the decimation.

## Selection

| clip | picked from | why | duty | stride Hz | vx steady |
|---|---|---|---|---|---|
| `WALK` | 3 candidates | lowest stride_cv (steadiest cycle) among 3 candidates | 0.638 | 1.38 | 0.187 m/s |
| `TROT` | 4 candidates | lowest stride_cv (steadiest cycle) among 4 candidates | 0.570 | 1.54 | 0.444 m/s |
| `RUN` | 11 candidates | lowest stride_cv (steadiest cycle) among 11 candidates | 0.300 | 2.93 | 0.514 m/s |
| `TURN` | 1 candidates | lowest stride_cv (steadiest cycle) among 1 candidates | 0.715 | 1.04 | 0.008 m/s |
| `JUMP` | 5 candidates | flight fraction closest to the cohort median of 5 | 0.274 | 1.49 | 0.006 m/s |

## Commanded gains — not constant, and not what a replay can ignore

Medians inside the motion window, per joint type (hip / thigh / calf):

| clip | kp | kd | tau_ff RMS (Nm) | position-controlled? |
|---|---|---|---|---|
| `WALK` | 40 / 40 / 40 | 1 / 1 / 1 | 0 / 0 / 0 | yes |
| `TROT` | 40 / 40 / 40 | 1 / 1 / 1 | 0 / 0 / 0 | yes |
| `RUN` | 13 / 3 / 2 | 1.8 / 2 / 1.5 | 3.66692 / 2.59149 / 11.7881 | **no** |
| `TURN` | 40 / 40 / 40 | 1 / 1 / 1 | 0 / 0 / 0 | yes |
| `JUMP` | 40 / 40 / 40 | 1.8 / 2 / 1.5 | 2.90918 / 4.16858 / 12.2305 | yes |

kp/kd are scheduled per skill and within a skill (walk 40/40/40, running trot 13/3/2 with ~11 Nm RMS calf tau_ff). They are stored as per-sample time series and must be applied; a position-only replay under the fork's configured gains (kp 40 / kd 1) will not reproduce the running trot.

## Caveats

**Loop seam.** `WALK` closes with a 0.089 rad jump (1.16× its own largest step), `RUN` closes with a 0.346 rad jump (1.34× its own largest step). A ratio above 1 means the wrap is faster than anything inside the cycle, so a looped replay commands a step the real robot never commanded. The cut is on the reference foot's touchdown, which is a genuine discontinuity in the command stream — this is the cost of cutting there rather than an extraction bug. Blend the seam or play a single cycle if it shows up in the replay.

**Cycle rate.** The working figure carried into this task was 3.25 Hz / 308 ms. Measured on the selected session the running-trot cycle is **3.09 Hz / 324 ms** (spread 27 ms over 7 cycles), and across all 11 running-trot sessions `skill_profile.csv` gives 2.79-3.09 Hz. Nothing in the clip is snapped to 308 ms; the cut is on contact events and the clip carries whatever period that produced.

**Downsampling is nearly free for `WALK`, `TROT`, `TURN`.** The sport controller wrote new commands at ~43 Hz on those sessions, below the 50 Hz target, so the ~419 Hz copy is a zero-order-held staircase of the same information. `RUN` and `JUMP` update in the hundreds of Hz and do lose content.

**Mapping discrimination.** `sim/diagnose.py`'s leg-order/sign search separates the candidates far better on `RUN` and `JUMP` (margin ~0.18) than on `WALK` and `TROT` (~0.04, under its own 0.05 warning threshold): the slow gaits are too symmetric to tell a mirrored mapping from the right one. Verify the Isaac convention on `RUN` or `JUMP` first.

## Unverified

Joint sign convention, zero offsets and hip abduction direction are taken from the log as-is and have NOT been checked against the Isaac Lab Go2 asset. Run scripts/verify_skill_replay.py on a machine with Isaac Lab before trusting a replay.
