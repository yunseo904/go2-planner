# motion_toolkit

CPU-only analysis of the **read-only** curated Go2 log set (36 sessions).
Nothing here writes to the curated tree and nothing imports Isaac Lab / torch.
The root is resolved by `terrain_toolkit/paths.py` (`$GO2_CURATED_ROOT`, else the
sibling directory `../curated`).

```
scripts/profile_skills.py   36 sessions → outputs/skill_profile.csv
                                        → outputs/jump_profile.csv
                                        → outputs/skill_profile.md
                                        → outputs/skill_transition.md
```

## Modules

| module | what it does |
| :--- | :--- |
| `session.py` | loads `data.npz` + `meta.json` + `MANIFEST.json` + `events.jsonl`; refuses the all-zero `foot_pos_*` / `foot_vel_*` columns; maps `events.jsonl` `t_mono` onto the sample time axis via `meta["t0_monotonic"]` |
| `window.py` | motion window from `mean_j abs(dq_j)` (two-level split, gap bridging, minimum segment length) |
| `contact.py` | per-session/per-leg contact from `foot_force` (Schmitt trigger + despeckle), stride frequency, duty, inter-leg phase, gait pattern from contact correlation |
| `profile.py` | one feature row per session (~100 columns) |
| `jump.py` | take-off / flight / landing for the four-leg-sync skills: flight time, ballistic and push-off-impulse estimates of take-off speed and apex, horizontal travel, landing peaks, odometry drift rate |
| `transitions.py` | per-`skill_send` cost: call time, lag to first motion, motion duration, settle time; predecessor bigrams |
| `report.py` | the two markdown reports |

## The three decisions that matter

1. **The motion window comes from joint velocity, not from `events.jsonl`.**
   `run.sh` needs ~2.4 s to start and the sport controller adds its own
   preparation, so a `skill_send` timestamp precedes the actual motion by
   4.06 ± 0.28 s (measured, `skill_transition.md` §1). Standing still, mean
   `|dq|` sits at ~0.027 rad/s and is very quiet, which makes a two-level split
   at `p10 + {0.05, 0.25}·(p99.5 − p10)` reliable.

2. **Contact is re-derived per session, per leg.** The logged `contact_*`
   columns use a fixed 20 N threshold; the thresholds this derives run 24–51 N
   across the set, so the fixed value would have been wrong everywhere. The
   hysteresis (`enter` 0.45, `leave` 0.25 of the p5–p95 span) plus a 40 ms
   minimum run is what stops the force ripple inside a stance phase from
   double-counting touchdowns — without it stride frequency comes out up to 2×
   too high on the slow-walk sessions.

3. **Statistics use the union of active segments, not the window span.** A
   session with a quiet gap in it (`stand_down` … `recovery_stand`, or a jump's
   crouch/flight/recover) would otherwise report that gap as stance and as zero
   body velocity.

## The jump analysis (`jump.py`)

The sport state cannot see a flight phase: `base_pos_*`, `base_v*` and
`body_height` are leg-kinematic, so with no foot on the ground they stop
tracking (a 0.45 s `front_jump` flight moves `base_pos_z` by 10-30 mm). Height
and take-off speed therefore come from the flight time (`v_z0 = g·T/2`,
`apex = g·T²/8` — clock only, no accelerometer scale) cross-checked against the
push-off impulse `∫(acc_z − g)dt`. The two agree to ~3 % on `front_jump`.

An all-four-off run is **not** automatically flight: these skills also unload
every foot while crouching and while swinging both legs through a lunge, and
those runs come with a push-off impulse at or below zero. Only runs whose
push-off impulse reaches `BALLISTIC_DV` count. By that test `front_pounce` never
leaves the ground.

## Known limits

* `foot_force` clips at ~210 N: contact timing is sound, landing magnitude is a
  lower bound (`foot_force_clipped_frac`).
* `mode` / `gait_type` are always 0 on this firmware and are not used.
* `acc_z` arrives held at the sport-state publish rate, so a millisecond-scale
  landing impact is aliased away: landing peaks are lower bounds, and integrating
  a landing recovers only a fraction of the velocity that was actually arrested.
* `stride_hz` / `duty` / `gait_pattern` are only gait parameters where
  `periodic` is True (21 of 36 sessions).
* The three 2026-08-04 gamepad sessions have no `events.jsonl` and no
  `t0_monotonic`, so they carry no transition timings.

## Skill clips (`clips.py`)

`scripts/extract_skill_clips.py` freezes one replayable clip per skill to
`data/skill_clips.npz`, with `outputs/skill_clips.md` as the readable summary.

Sessions are chosen by measurement, not by name: the duty band separates
WALK/TROT/RUN (0.64 / 0.52 / 0.31, the axis `skill_profile.md` found), and among
the sessions that qualify the one with the lowest `stride_cv` wins — a clip that
is going to be looped is judged on cycle regularity, not on speed. `front_jump`
is picked as the jump with the most typical flight time rather than the best one.

Cyclic clips are cut on the reference foot's touchdown, phase-averaged over every
clean cycle (median across cycles, so a single mis-detected touchdown cannot
smear the result), and stored on a phase grid so the last sample wraps onto the
first. One-shot clips are stored whole.

Both the session's own rate and 50 Hz are written. The 50 Hz copy is
anti-aliased first — Fourier truncation for the periodic clips, zero-phase
Butterworth for the one-shot — never plain-decimated. `kp`/`kd` are exempt: they
are commanded gain *levels* and filtering them would invent gains that were never
sent.

**Three things the clips make visible that the profile tables did not:**

1. **The gain schedule is per skill and not constant.** Slow walk, turns and
   lateral moves run at `kp` 40/40/40 with `tau_ff = 0` — a position replay is
   faithful. The running trot runs at `kp` 13/3/2 with ~11 Nm RMS of calf
   `tau_ff`; with a calf gain of 2 Nm/rad the leg is nearly free and `q_des` is
   not the signal that produces the gait. Replaying RUN as position targets under
   this fork's Go2 gains (kp 40 / kd 1, `legged_robot_config.py`) will not reproduce it.
   Those gains happen to match the *walk* sessions exactly, which is why WALK and TROT
   need no special handling and RUN does. See `outputs/gain_feasibility.md`.
2. **The command stream is slower than the log.** The log samples at ~419 Hz but
   the sport controller writes a new `q_des` at ~43 Hz on the slow-walk sessions
   and ~160-165 Hz on the running trot and the jump. The high-rate copy of a walk
   clip is a zero-order-held staircase, and its 50 Hz version loses almost
   nothing; the running trot's does lose content.
3. **The measured running-trot cycle is 3.09 Hz / 324 ms**, not the 3.25 Hz /
   308 ms working figure, and it ranges 2.79-3.09 Hz across the 11 running-trot
   sessions. Clips carry the period their contact events produced; none is
   snapped to a nominal value.

Sign and zero conventions are **unverified** against the Isaac Lab Go2 asset.
`scripts/verify_skill_replay.py` is what settles them.
