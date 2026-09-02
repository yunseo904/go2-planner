# Why it rolls over: measured, not intervened on

SESSION_STATE §16.1, taken as a measurement rather than a target. No new intervention is
proposed here and none was run — that was the instruction and it is also the right order:
the roll couple was adopted on a score without its mechanism ever being established.

Instruments: a recording-only `--trace-npz` on `run_benchmark.py` (root pose and velocity,
foot world positions, the clip's stance mask, and — new this session — **foot contact
forces** from a `ContactSensor` created only when a trace is asked for). Plus the curated
real-robot logs, which need no simulator.

---

## 1. The fall is scattered; the ROLL ONSET is not

WALK, seed 1, roll couple on, 200 cells. `--no-roughness` is the clean case because the
terrain then cannot be the trigger.

| quantity | p10 | p25 | **median** | p75 | p90 | **CV** |
|---|---|---|---|---|---|---|
| time to fall, s | 4.98 | 6.22 | **8.56** | 20.0 | 20.0 | 0.54 |
| distance at fall, m | 0.66 | 0.74 | **1.07** | 1.92 | 2.85 | 0.64 |
| **distance at \|roll\|>10°** | 0.72 | 0.75 | **0.88** | 1.18 | 1.89 | **0.51** |
| **distance at \|roll\|>20°** | 0.68 | 0.72 | **0.89** | 1.15 | 1.76 | **0.47** |

**The roll onset is a tighter quantity than the fall it causes.** And 10° and 20° happen at
almost the same place (0.88 vs 0.89 m, p25 0.75 vs 0.72) — **once it passes 10° it is already
gone**. The roll is a step change, not a slow lean.

With roughness on, the onset moves earlier and scatters badly: median 0.28 m at 10°, CV 0.97.
**Roughness brings the event forward and randomises it; it does not create it.**

## 2. The trigger is time, not distance

At the moment \|roll\| first exceeds 20°, no roughness:

| quantity | p25 | median | p75 | mean | sd | **CV** |
|---|---|---|---|---|---|---|
| time, s | 4.80 | 5.78 | 7.22 | 6.58 | 2.51 | **0.38** |
| **distance, m** | 0.72 | 0.89 | 1.15 | 1.04 | 0.49 | **0.47** |
| cycles | 6.49 | 7.81 | 9.76 | 8.89 | 3.39 | **0.38** |

**Distance is the loosest of the three.** The implied speed at the onset has a CV of 0.31, so
the distance spread is mostly the speed spread — robots roll out after a similar *elapsed
gait*, and end up at different places because they walk at different speeds.

So "0.7–0.9 m" is a consequence, not the thing. **The event is an accumulation over gait
cycles, not something at a location** — which also means no terrain feature is being hit,
consistent with §1's no-roughness arm showing it at all.

**Time and cycle count are the same quantity here** (one clip, one playback rate: cycles =
time ÷ period), so this pair cannot be separated without changing the rate.

**The attempt to separate them failed, and it failed informatively.** `--rate hi` turns out
not to be a faster playback at all — `hi` and `lo` are the *same cycle* sampled at 418 Hz and
50 Hz, and every harness here plays one frame per control step at 50 Hz, so `hi` **stretches
the gait 8.3×** (WALK 0.74 s per cycle → 6.12 s). Measured: WALK `hi` scores **0.005** with
v_x **−0.0045 m/s**, alive 3/200, against `lo`'s 1.090 / +0.1070 / 74. That is not the same
gait, so it cannot serve as a controlled change of cycle rate.

One observation survives it and is worth recording, weakly: **WALK `hi` still rolls out at
5.68 s against `lo`'s 5.78 s** — nearly the same clock time — while covering 0.12 m instead of
0.89 m. If cycle count were the driver, an 8.3× longer cycle should have pushed the onset far
later in time. It did not. That points at **clock time** rather than cycles, but the gaits
differ so much that it is a hint, not a result.

## 2b. Every skill does it, at its own time

Benchmark, no roughness, roll couple on, 200 cells, at \|roll\| > 20°:

| skill | median time | CV | median distance | CV | score |
|---|---|---|---|---|---|
| **TROT** | **3.65 s** | 0.69 | 0.98 m | 0.81 | 1.035 |
| **WALK** | **5.78 s** | 0.38 | 0.89 m | 0.47 | 1.090 |
| **TURN** | **11.82 s** | **0.19** | 0.37 m | 0.30 | 0.675 |

**It is the same event in all three, with a skill-specific clock.** TURN lasts three times as
long as TROT and is by far the most consistent about it (CV 0.19) — and TURN is the skill
that translates least, which is another way of saying the onset does not need distance.
Time, not distance, in every skill.

## 3. The real robot does not do this

The curated logs, every `03_slow_walk` session — the family the WALK clip comes from. Roll is
from the IMU; `CLAUDE.md` §6 warns that `base_pos`/`base_v` are leg-kinematics derived, so
speed is shown but not leaned on.

| session | moving | \|roll\| mean | p95 | max |
|---|---|---|---|---|
| `gait_classic_walk` | 5.1 s | 1.35° | 2.60° | **3.39°** |
| `gait_cross_step` | 5.3 s | 1.12° | 2.24° | 2.55° |
| `run_06` | 3.9 s | 0.64° | 1.45° | 2.09° |
| `walk_fwd_default` | 5.3 s | 1.21° | 2.06° | 2.39° |

**The worst roll the real robot reaches in any of them is 3.39°**, over the same 4–5 s window
in which the sim passes 20° and does not come back. The sim's *mean* roll on flat ground,
before anything happens, is 2.3–2.6° — already most of the real robot's worst case.

There is no roll event in the recording to reproduce. Whatever this is, it is the sim's.

## 4. What is already excluded

From `cross_track_is_the_fall.md`, on the same traces:

- **the stance feet are planted** — 0.019 m/s against a body speed of 0.192, 0.9 mm of
  vertical range within a bout interior, net lateral slip −0.010 m;
- **the footfall pattern does not march** — +0.2 mm per footfall laterally;
- so the lateral offset that used to be called "drift" **is this roll**, at −0.93 correlation.

And from `trot_4cm.md`: TROT dies the same way on the probe rig's flat run-up, 2 of 3
surviving entry phases never reaching the 4 cm step they were supposed to fail at.

## 5. What this does not yet say

**The mechanism is still open.** What is established is where to look: a per-cycle
accumulation, present without terrain, absent from the real robot, that goes from 10° to 20°
almost instantly once it starts. What is *not* established is what accumulates.

The measurement that would say is the one this session built the instrument for and did not
finish: **the per-foot contact forces through the onset** — which foot unloads first, whether
the support polygon is lost before or after the roll starts, and where the base is relative to
it. The `ContactSensor` is wired behind `--trace-npz` and the trace format carries the forces;
what remains is the analysis.

> **A warning for whoever runs it.** The sensor makes PhysX emit a
> `getMaterialFromInternalFaceIndex` warning per contact per step against the trimesh
> terrain — about 3 GB of log per 200-cell run, which fills the disk. Pipe the run through
> `grep -aFv getMaterialFromInternalFaceIndex`. The warning is a material lookup for a face
> index and does not touch the force values.
