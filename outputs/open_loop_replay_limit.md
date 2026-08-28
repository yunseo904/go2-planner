# Open-loop joint replay: what it can and cannot reproduce

*2026-08-28, z4. Everything below is measured with the settings day 2 settled on —
hip convention `keep`, effective friction 1.3 (product of the two materials), 30 N
contact threshold, physics 200 Hz / control 50 Hz, `--start-phase level`,
`--settle-mode stand`. No stabilising feedback of any kind was added; every run is the
recorded joint trajectory played into the PD and nothing else.*

## Summary

The failure is **not** lateral slipping and **not** the initial condition. It is a roll
that grows geometrically, cycle over cycle, from a disturbance the size of the one WALK
shrugs off.

| | survives | roll growth per cycle | usable distance | verdict |
|---|---|---|---|---|
| WALK | 60 cycles / 43.9 s / 10.7 m, no fall | ×1.2, ×0.7, ×1.0 — **bounded** | unbounded | open-loop replay works |
| TROT | 2.19 s | ×3.6, ×4.6 — **divergent** | 0.72 m | open-loop replay fails |
| RUN | 2.47 s | ×1.9, ×2.5, ×2.3 — **divergent** | 0.41 m | open-loop replay fails |

The open-loop limit cycle is **attracting for WALK and repelling for TROT and RUN**. That
is the finding. It is narrower than "open-loop replay cannot reproduce fast gaits" and it
is what the data actually supports.

## The measurement that decides it

Both gaits are handed over from the standing settle with almost the same disturbance:

| | handover speed | handover ang. rate | peak roll, cycle 0 |
|---|---|---|---|
| WALK | 0.020 m/s | 3.2 °/s | **4.00°** |
| TROT | 0.068 m/s | 13.2 °/s | **3.27°** |

TROT starts from the *smaller* roll disturbance. Then:

```
peak |roll| per cycle
  WALK    4.0   4.8   3.2   3.2   4.2  ...  3.1   (cycle 59)
  TROT    3.3  11.6  53.1  -> over
  RUN     6.9  12.9  32.6  74.1
```

WALK's disturbance does not decay to zero — it settles into a bounded ±4° oscillation,
which is what an attracting limit cycle looks like. TROT's multiplies by 3.6 then 4.6.
Same simulator, same friction, same settle, same contact threshold; opposite fate.

**This refutes the "WALK just accumulates more slowly" reading.** WALK does not accumulate
at all. By cycle 40 it has converged to a fixed point and stays there: mean roll −0.20°,
body-frame lateral velocity +0.0001 m/s, forward 0.232 m/s, holding for 20 more cycles.

## Where the tipping threshold comes from

"Usable distance" above is the distance travelled before |roll| exceeds the angle at which
the centre of mass leaves the lateral support polygon: `atan(half stance width / base
height)`, both read from the robot's own kinematics in the first cycle. It is geometry,
not a threshold fitted to the runs (CLAUDE.md §2).

| | half stance width | base height | tipping angle | crossed at |
|---|---|---|---|---|
| WALK | 154.6 mm | 281.1 mm | 28.8° | never, in 43.9 s |
| TROT | 148.4 mm | 292.8 mm | 26.9° | 1.69 s = 2.62 cycles = 0.720 m |
| RUN | 166.6 mm | 278.7 mm | 30.9° | 0.93 s = 2.88 cycles = 0.414 m |

## What the collapse actually is

The "0.22 m/s sideways drift" reported yesterday is the horizontal component of a topple,
not slipping. TROT's last cycles:

| cycle | mean roll | peak roll | CoP y (base) | stance FL/FR/RL/RR | impulse L−R |
|---|---|---|---|---|---|
| 0 | +0.97° | +3.3° | **+0.9 mm** | 0.53 0.50 0.44 0.59 | −4.3% |
| 1 | +6.28° | +11.6° | −23.3 mm | 0.41 0.66 0.41 0.66 | −25.9% |
| 2 | +25.34° | +53.1° | −63.9 mm | 0.22 0.94 **0.00** 0.44 | −81.7% |

Read left to right: the left legs stop carrying load (rear-left reaches zero stance in
cycle 2), the centre of pressure migrates 64 mm to the right of the base, and the robot
rolls onto its right side — 122° of roll with only the front-right foot loaded at the end.

**Cycle 0's centre of pressure is +0.9 mm — centred.** There is no initial lateral bias to
blame. The asymmetry is produced by the divergence, not the other way round.

## Question 1: is the recording asymmetric?

Yes, and it matters — but for the curving, not for the collapse.

A left-right symmetric gait maps onto itself under mirror + half-cycle shift. Every clip's
best shift is *exactly* half a cycle (153/306, 134/268, 69/138), and the leftover is:

| clip | residual after mirror + half cycle | duty L−R | real robot lateral drift |
|---|---|---|---|
| WALK | 0.046 rad = **8.3%** of peak-to-peak | −0.021 | +0.0072 m/s = 3.9% of forward |
| TROT | 0.051 rad = **7.7%** of peak-to-peak | −0.021 | −0.0027 m/s = 0.6% of forward |
| RUN | 0.133 rad = **15.6%** of peak-to-peak | +0.014 | +0.0247 m/s = 4.8% of forward |

(The same test independently re-confirms the hip convention: mirroring with the hip column
negated leaves 7.7–8.3%, mirroring with it kept leaves 19–24%. The abduction axis reverses
under the mirror, as it must.)

Two things follow immediately:

1. **The asymmetry does not discriminate.** WALK's is *larger* than TROT's (8.3% vs 7.7%),
   and WALK is the one that survives. Whatever kills TROT is not "the clip is asymmetric".
2. **The real robot went straight anyway.** With these same commands the logged odometry
   shows 0.6% lateral drift for TROT. The asymmetry in the recording is not a bias the
   robot was suffering from — it is most likely the correction the robot's own controller
   was applying. Replayed open loop, that correction has nothing left to correct, so it
   acts as a bias instead.

### Two ablations that attribute it

Both change the clip and are **diagnostic only** — `--ablate {mirror,symmetrize}`, printed
with a loud banner, stamped into the result row, never a reported skill. The frozen archive
is untouched. Neither adds feedback; both stay open-loop joint trajectories.

**`mirror` — play the gait left-right mirrored.** If the bias lives in the robot or the
sim, the robot falls the same way; if it lives in the clip, the fall flips.

| TROT | forward | lateral | yaw | mean roll | terminated |
|---|---|---|---|---|---|
| as recorded | +0.749 m | **−0.513 m** | **+24.4°** | **+21.3°** | 2.19 s |
| mirrored | +0.750 m | **+0.513 m** | **−24.2°** | **−21.1°** | 2.19 s |

An exact mirror, to within a few parts per thousand, with the identical start frame and
handover state. So the simulator, the ground and the robot asset are left-right symmetric,
and the clip alone decides *which side* it falls to. It does not decide *whether*.

**`symmetrize` — average the clip with its own mirror half a cycle later**, which deletes
the asymmetric component and keeps the symmetric one (verified: the result is its own
mirror to 1e−4).

- **TROT still collapses** — 1.45 s, roll growing ×7.6 in one cycle — and it still picks a
  side, falling left this time. A symmetric input whose symmetric solution is unstable
  breaks symmetry anyway. *Caveat: `level` chose a different start frame for the
  symmetrised clip (1 vs 16) and its handover was worse (0.109 vs 0.068 m/s), so "falls
  sooner" is confounded. The claim supported without caveat is that removing the asymmetry
  does not prevent the collapse.*
- **WALK's curving nearly vanishes.** Measured over the last 20 of 60 cycles, after both
  runs have converged:

| WALK, converged | yaw rate | curvature | forward | lateral |
|---|---|---|---|---|
| as recorded | +2.77 °/s | **+11.8 °/m** | 0.232 m/s | +0.0001 m/s |
| symmetrised | +0.42 °/s | **+1.55 °/m** | 0.227 m/s | −0.0004 m/s |
| real robot (log odometry) | — | +2.05 °/m | 0.187 m/s | +0.0072 m/s |

So the clip's asymmetry *is* causal — for the steady turn. Open loop it curves 5.7× more
than the real robot did on the same commands, and removing the asymmetric component brings
the curvature back to roughly what the robot actually did. That is the clearest single piece
of evidence that the recorded asymmetry was a live correction rather than a property of the
gait.

## Question 2: is this a structural limit of open-loop replay?

**Confirmed for TROT and RUN. Refuted for WALK.** The precise statement:

> Replaying recorded joint trajectories open loop reproduces a gait only when that gait's
> open-loop limit cycle is attracting. For the Go2 slow walk (duty 0.64) it is: a 4° roll
> disturbance stays bounded over 60 cycles and the base converges to a fixed point. For the
> trot (duty 0.52) and the running trot (duty 0.31) it is not: a 3° disturbance multiplies
> by 3–5 per cycle and the robot tips past its geometric support angle within three cycles.
> The recording contains the joint angles the real controller commanded but not the base
> state it was closing the loop on, so the stabilising component of those commands cannot
> act. Nothing in the clip can supply it, because nothing in the clip observes the base.

Supporting facts, each measured rather than assumed:

1. A clip carries joint angles and **no base trajectory** (`channels` in
   `skill_clips.meta.json`). Foot placement in sim is therefore whatever the pose plus the
   sim's own dynamics produce.
2. The divergence is not seeded by the initial condition: TROT's cycle-0 roll (3.27°) is
   *smaller* than WALK's (4.00°), and TROT's handover is quieter than WALK's on the
   measure that failed yesterday.
3. The divergence is not seeded by clip asymmetry: symmetrising does not prevent it, and
   mirroring only flips its direction.
4. The divergence is not a simulator artefact biased to one side: the mirrored run is an
   exact mirror.
5. The real robot did not diverge on the same commands — 0.6% lateral drift, 5.6° of yaw
   over 1.38 m — because it was closing a loop that the replay does not have.

### What this does *not* say

It does not say the sim is unable to trot. It says *this* trot, played open loop from a
recording, is not self-stabilising. A policy trained to track these clips as reference
trajectories would close the loop and could well hold the gait — but that is a different
experiment with a different premise, and it is the user's call whether to run it.

## Consequence for the skill library

**Only WALK is usable as a sustained locomotion skill.** TROT's usable horizon is 0.72 m,
which is under a third of one benchmark goal segment (~2.25 m), and it ends with the base
already rolled 27° — a state nothing can be chained onto. A skill that cannot hand over to
the next skill is not a planner action. RUN is worse (0.41 m).

Judgment on "is TROT usable over short stretches": **no, not as a planner action.** It
would be usable only if a segment shorter than 0.7 m existed that ended in a fall being
acceptable, and no such segment exists in the benchmark.

WALK's usability comes with one caveat that has to be carried into the comparison: played
open loop as recorded, it curves at 11.8 °/m, against the real robot's 2.05 °/m. Over one
2.25 m goal segment that is 26° of heading error. A rule planner built on WALK either models
that curve or the comparison has to state it. This is a fidelity gap in the harness's WALK,
not a property of the robot.

This narrows the rule-based planner's locomotion library to a single primitive. That is a
substantive change to the experiment's design — the Single-skill lower bound in CLAUDE.md §2
is specified as "트롯 고정", and trot is not available under this premise — so it is flagged
here rather than resolved (CLAUDE.md §0).

## What would turn inference into measurement

One claim above is an inference, not a measurement: that the recording's asymmetry is the
controller's correction. It becomes measurable the moment `curated/` is on z4 — the
cycle-to-cycle variation of `q_des` at matched phase within a session *is* the feedback
component, and it is discarded when the cycles are averaged into one clip. That is now the
strongest reason to move the logs, ahead of the startup-segment extraction.

## The averaging question — built, blocked on the logs

*Asked 2026-08-28: the clip is a median over phase-aligned cycles, so the controller's
correction may have been averaged out. Replay a single raw cycle instead and A/B, over
several cycles, with a survival-time distribution.*

Agreed that this is an extraction change, not a clip edit. It cannot be run yet:
**`curated/` is still not on z4**, and the archive holds only the averaged cycle —
`WALK__hi__q_des` is 306 frames, which is exactly one cycle. The individual cycles exist
only in the original sessions.

### The premise is correct, and the archive says how much was discarded

`motion_toolkit/clips.py::_phase_average` takes the **median** over every clean cycle on a
shared phase grid, and keeps `spread = cube.std(axis=0)` of which only the maximum is
recorded. That maximum is already in the frozen meta:

| clip | cycles | period | period spread | q_des p2p | max cross-cycle std | ratio |
|---|---|---|---|---|---|---|
| WALK | 4 | 0.7315 s | 0.0509 s (7.0%) | 0.560 rad | 0.200 rad | **35.8%** |
| TROT | 5 | 0.6429 s | 0.0369 s (5.7%) | 0.660 rad | 0.203 rad | **30.8%** |
| RUN | 7 | 0.3241 s | 0.0271 s (8.3%) | 0.856 rad | 0.317 rad | **37.1%** |

So the median did discard something whose worst-case cross-cycle standard deviation is
about a third of the whole motion range. **Caveat: that is a maximum over all phases and
joints, and it conflates two things** — genuine cycle-to-cycle correction, and phase
alignment jitter, which makes a fast-moving joint look highly variable when a cycle is
resampled 5.7% long. The archive stores only the max, so the two cannot be separated from
it. The raw cycles separate them.

### What is built and tested, ready to run

- `motion_toolkit/clips.py::build_cyclic_clip(..., cycle_subset=None)` — selects which
  clean cycles feed the median. `None` is the existing path and its output is unchanged
  (`extract_skill_clips.py --verify` still reports `aab85a03…` OK). A one-element subset
  makes the median the identity, which is how a raw cycle is obtained: **same session, same
  contact-based cycle detection, same phase grid, same low-pass, same channels**. A single
  cycle also keeps its **own** period rather than being renormalised to the session median,
  since renormalising would be a second kind of averaging.
- `scripts/extract_raw_cycles.py --clip TROT` — writes `data/raw_cycles_TROT.npz` with
  `TROT_med` (all cycles, the control, rebuilt *by this script* so the A/B differs in the
  averaging and not in the code path) and `TROT_c00…` (each cycle alone). The frozen
  archive is untouched. `--self-test` runs with no logs and proves the part that matters:
  a single-cycle subset returns that cycle, the median is a median and not a mean, and the
  cross-cycle spread it discards is non-zero.
- `verify_skill_replay.py --clip-archive … --results-csv …` — plays any archive built to
  the same schema.
- `scripts/run_raw_cycle_ab.sh TROT` — sweeps every clip in the archive at the settled
  settings; `analyze_drift.py --ab outputs/raw_cycle_ab/TROT` prints the survival
  distribution.

The plumbing is verified end to end against a stand-in archive holding the frozen TROT clip
under the new names: it loaded, chose the same start frame 16, the same handover
0.068 m/s, the same stride 1.56 Hz and terminated at **2.1899 s** — identical to the frozen
run. So the alternate-archive path is neutral, and any difference the real A/B shows will
be the averaging.

### Prediction, and what would refute the conclusion

I expect the raw cycles **not** to survive materially longer, because the divergence
multiplies roll by 3.6–4.6 per cycle from a 3° disturbance and a single cycle is still an
open-loop trajectory: even if the correction survives the extraction, it is applied at a
base state that no longer matches the one it was computed for. Symmetrising — a much larger
change to the waveform — did not prevent the collapse either.

But there is a real mechanism on the other side, which is why the run is worth making:
the median smears touchdown timing across cycles whose periods differ by 5.7%, and crisper
foot-strike timing could genuinely change stability. Pulling the other way, a raw cycle
loops with a bigger seam discontinuity than the median clip's 0.019 rad. So the outcome is
not obvious.

**If a raw cycle survives materially longer than `TROT_med`, the conclusion in this document
is wrong** and the cause is the extraction, not open-loop replay. That is the falsification
criterion. Note the sample is small: TROT's session yields 5 clean cycles, all that session
has. The obvious extension is to repeat over the other 3 candidate TROT sessions — say the
word and I will.

## Reproducing

```
scripts/analyze_drift.py --clips                        # clip symmetry, no simulator
scripts/isaac_docker_run.sh scripts/verify_skill_replay.py --clip WALK --cycles 60 \
    --headless --device cpu --hip-sign keep --contact-threshold-n 30 \
    --start-phase level --settle-mode stand --trace-npz outputs/traces/WALK_long.npz
scripts/analyze_drift.py --trace outputs/traces/WALK_long.npz
```

Traces for all seven runs are in `outputs/traces/`. `logs/drift_runs.log` and
`logs/ablate_runs.log` are the full console output.
