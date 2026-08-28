# Open-loop joint replay: what it can and cannot reproduce

**Status:** measured and settled. The curated logs reached z4 on 2026-08-28, which closed
the two open items: the re-extraction gate passes (§7.1) and the clip's averaging is *not*
what causes the failure (§7.2). Two consequences for the experiment's design follow (§5, §6)
and are the open decisions.

**Scope.** Everything below is measured on z4 with the settings day 2 settled on: hip
convention `keep`, effective friction 1.3 (the product of the two materials), 30 N contact
threshold, physics 200 Hz / control 50 Hz, `--start-phase level`, `--settle-mode stand`.
**No stabilising feedback of any kind was added.** Every run is the recorded joint
trajectory played into the PD and nothing else. Where a number is an inference rather than
a measurement it says so.

---

## 1. Summary

Three findings, in the order they constrain the project.

**1. Open-loop replay works for the slow walk and fails for the trot — and the reason is
dynamical, not accumulated error.** The open-loop limit cycle is *attracting* for WALK and
*repelling* for TROT and RUN.

| | survives | roll growth per cycle | usable distance | verdict |
|---|---|---|---|---|
| WALK | 60 cycles / 43.9 s / 10.7 m, no fall | ×1.2, ×0.7, ×1.0 — **bounded** | unbounded | replay works |
| TROT | 2.19 s | ×3.6, ×4.6 — **divergent** | 0.72 m | replay fails |
| RUN | 2.47 s | ×1.9, ×2.5, ×2.3 — **divergent** | 0.41 m | replay fails |

**2. The surviving skill still cannot cross the benchmark.** Played open loop, WALK curves
at 11.8 °/m. The benchmark tolerates 0.071 °/m over its 18 m cell (0.565 °/m if the planner
is allowed to aim the arc optimally). WALK reaches **0 of 8 goals** — it leaves the 0.2 m
goal tolerance after 1.40 m, and leaves the 4 m-wide cell entirely after 4.58 m. The real
robot's own curvature on the same commands, 2.05 °/m, also exceeds the budget. **A discrete
heading-correction primitive is therefore structurally required, whatever the clip.**

**3. With a WALK + JUMP library the planned comparison does not stand.** Rule-Planner and
the Single-skill lower bound would be the same policy on 81.5 % of benchmark runs, and both
would score ~0 goals. §6 says what would restore it: one untested clip, already in the logs.

---

## 2. The measurement that decides finding 1

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
which is what an attracting limit cycle looks like. TROT's multiplies by 3.6 then 4.6. Same
simulator, same friction, same settle, same contact threshold; opposite fate.

**This refutes the "WALK just accumulates more slowly" reading.** WALK does not accumulate
at all. By cycle 40 it has converged to a fixed point and stays there: mean roll −0.20°,
body-frame lateral velocity +0.0001 m/s, forward 0.232 m/s, holding for 20 more cycles.

### Where the tipping threshold comes from

"Usable distance" is the distance travelled before |roll| exceeds the angle at which the
centre of mass leaves the lateral support polygon: `atan(half stance width / base height)`,
both read from the robot's own kinematics in the first cycle. It is geometry, not a
threshold fitted to the runs (CLAUDE.md §2).

| | half stance width | base height | tipping angle | crossed at |
|---|---|---|---|---|
| WALK | 154.6 mm | 281.1 mm | 28.8° | never, in 43.9 s |
| TROT | 148.4 mm | 292.8 mm | 26.9° | 1.69 s = 2.62 cycles = 0.720 m |
| RUN | 166.6 mm | 278.7 mm | 30.9° | 0.93 s = 2.88 cycles = 0.414 m |

### What the collapse actually is

The "0.22 m/s sideways drift" reported on day 1 is the horizontal component of a topple,
not slipping. TROT's last cycles:

| cycle | mean roll | peak roll | CoP y (base) | stance FL/FR/RL/RR | impulse L−R |
|---|---|---|---|---|---|
| 0 | +0.97° | +3.3° | **+0.9 mm** | 0.53 0.50 0.44 0.59 | −4.3% |
| 1 | +6.28° | +11.6° | −23.3 mm | 0.41 0.66 0.41 0.66 | −25.9% |
| 2 | +25.34° | +53.1° | −63.9 mm | 0.22 0.94 **0.00** 0.44 | −81.7% |

Read left to right: the left legs stop carrying load (rear-left reaches zero stance in cycle
2), the centre of pressure migrates 64 mm to the right of the base, and the robot rolls onto
its right side — 122° of roll with only the front-right foot loaded at the end.

**Cycle 0's centre of pressure is +0.9 mm — centred.** There is no initial lateral bias to
blame. The asymmetry is produced by the divergence, not the other way round.

---

## 3. Is the recording asymmetric?

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
   shows 0.6% lateral drift for TROT. The asymmetry in the recording is not a bias the robot
   was suffering from — it is most likely the correction the robot's own controller was
   applying. Replayed open loop, that correction has nothing left to correct, so it acts as
   a bias instead.

### Two ablations that attribute it

Both change the clip and are **diagnostic only** — `--ablate {mirror,symmetrize}`, printed
with a loud banner, stamped into the result row, never a reported skill. The frozen archive
is untouched. Neither adds feedback; both stay open-loop joint trajectories.

**`mirror` — play the gait left-right mirrored.** If the bias lives in the robot or the sim,
the robot falls the same way; if it lives in the clip, the fall flips.

| TROT | forward | lateral | yaw | mean roll | terminated |
|---|---|---|---|---|---|
| as recorded | +0.749 m | **−0.513 m** | **+24.4°** | **+21.3°** | 2.19 s |
| mirrored | +0.750 m | **+0.513 m** | **−24.2°** | **−21.1°** | 2.19 s |

An exact mirror, to within a few parts per thousand, with the identical start frame and
handover state. So the simulator, the ground and the robot asset are left-right symmetric,
and the clip alone decides *which side* it falls to. It does not decide *whether*.

**`symmetrize` — average the clip with its own mirror half a cycle later**, which deletes
the asymmetric component and keeps the symmetric one (verified: the result is its own mirror
to 1e−4).

- **TROT still collapses** — 1.45 s, roll growing ×7.6 in one cycle — and it still picks a
  side, falling left this time. A symmetric input whose symmetric solution is unstable breaks
  symmetry anyway. *Caveat: `level` chose a different start frame for the symmetrised clip
  (1 vs 16) and its handover was worse (0.109 vs 0.068 m/s), so "falls sooner" is confounded.
  The claim supported without caveat is that removing the asymmetry does not prevent the
  collapse.*
- **WALK's curving nearly vanishes.** Measured over the last 20 of 60 cycles, after both runs
  have converged:

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

---

## 4. The precise claim

**Confirmed for TROT and RUN. Refuted for WALK.**

> Replaying recorded joint trajectories open loop reproduces a gait only when that gait's
> open-loop limit cycle is attracting. For the Go2 slow walk (duty 0.64) it is: a 4° roll
> disturbance stays bounded over 60 cycles and the base converges to a fixed point. For the
> trot (duty 0.52) and the running trot (duty 0.31) it is not: a 3° disturbance multiplies by
> 3–5 per cycle and the robot tips past its geometric support angle within three cycles. The
> recording contains the joint angles the real controller commanded but not the base state it
> was closing the loop on, so the stabilising component of those commands cannot act. Nothing
> in the clip can supply it, because nothing in the clip observes the base.

Supporting facts, each measured rather than assumed:

1. A clip carries joint angles and **no base trajectory** (`channels` in
   `skill_clips.meta.json`). Foot placement in sim is therefore whatever the pose plus the
   sim's own dynamics produce.
2. The divergence is not seeded by the initial condition: TROT's cycle-0 roll (3.27°) is
   *smaller* than WALK's (4.00°), and TROT's handover is quieter on the measure that failed
   on day 1.
3. The divergence is not seeded by clip asymmetry: symmetrising does not prevent it, and
   mirroring only flips its direction.
4. The divergence is not a simulator artefact biased to one side: the mirrored run is an
   exact mirror.
5. The real robot did not diverge on the same commands — 0.6% lateral drift, 5.6° of yaw over
   1.38 m — because it was closing a loop that the replay does not have.

**What this does not say.** It does not say the sim is unable to trot. It says *this* trot,
played open loop from a recording, is not self-stabilising. A policy trained to track these
clips as reference trajectories would close the loop and could well hold the gait — but that
is a different experiment with a different premise, and it is the user's call whether to run
it.

---

## 5. Finding 2 — the heading budget: WALK alone reaches 0 of 8 goals

Reproduce with `scripts/analyze_heading_budget.py`. This section is pure geometry over
measured constants; nothing is fitted.

An open-loop gait holds a constant yaw rate, so its ground track is a circular arc of radius
`R = 1/κ`. The benchmark asks it to pass 8 goals spaced 2.25 m along an 18 m cell, within
`next_goal_threshold = 0.2 m` (upstream `legged_robot_config.py:188`), inside a cell 4 m wide.

| case | κ (°/m) | arc radius | deviation after one 2.25 m segment | leaves the 0.2 m tube | leaves the 4 m cell |
|---|---|---|---|---|---|
| **WALK open loop, as recorded** | **11.80** | 4.86 m | **0.512 m** | **1.40 m** | 4.58 m |
| WALK open loop, symmetrised | 1.55 | 36.97 m | 0.068 m | 3.85 m | 12.22 m |
| the real robot, same commands | 2.05 | 27.95 m | 0.091 m | 3.35 m | 10.64 m |

Goals actually reached by WALK as recorded — closest approach of the arc to each goal point:

| goal | at | closest approach | reached |
|---|---|---|---|
| 1 | 2.25 m | 0.496 m | **no** |
| 2 | 4.50 m | 1.765 m | no |
| 3–8 | 6.75–18.00 m | 3.46 → 13.79 m | no |

**0 of 8.** The first goal is missed by 2.5× the tolerance; after that the arc is leaving the
cell, not the corridor.

### The budget, and why it is not a WALK-clip defect

| requirement | tolerable κ | measured WALK is |
|---|---|---|
| straight from the launch heading, over 18 m | ≤ **0.071 °/m** | 167× too curved |
| best possible aim, arc centred on the line | ≤ **0.565 °/m** | 21× too curved |

The symmetrised clip (1.55 °/m) is still 2.7× over the best-aim budget, and **the real
robot's own open-loop curvature (2.05 °/m) is 3.6× over it.** So this is not a property of a
badly cut clip that a better extraction would fix. Any constant-curvature gait — including a
perfect recording of a real robot walking straight — misses the goals over 18 m. **Heading
has to be corrected during the run.**

That correction is legitimate under this project's premise: it is a *planner* decision made
on an observed heading, issuing discrete skill commands. It is not feedback injected into the
joint replay, and each skill still plays open loop.

### What correcting it would cost

Nulling the heading does not undo the cross-track offset already accumulated — under constant
curvature that offset is monotonic — so the planner has to re-aim *at the goal*, and the
binding constraint is the last re-aim before arrival: from distance `d` out, the arc still
bows `κd²/2` off the line.

| | |
|---|---|
| deviation reaches the 0.2 m tolerance after | 1.39 m of walking |
| so re-aim at least every | 1.39 m |
| corrections over the 18 m cell | **13** (one per 1.38 m) |
| heading to null at each | 16.3° |
| in-place turn rate available | 22.66 °/s (`turn_right`, duty 0.715) |
| turn time per correction | 0.72 s |
| walking time between corrections | 5.97 s |
| time overhead | **12 %** |
| extra skill switches | 26, plus 5.5 s of settle at 0.21 s each |

**Quantisation, unresolved:** the turn clip is one cycle of 0.96 s = 21.8°. A 16.3°
correction is 0.75 of a cycle, so the planner either over-turns to a whole cycle or plays a
partial cycle that ends off its seam. Which is acceptable is an open question.

---

## 6. Finding 3 — what this does to the experiment's design

### 6.1 The library, as measured today

| skill | source | open-loop verdict |
|---|---|---|
| WALK (duty 0.64) | `gait_classic_walk` | **usable**, subject to §5 |
| TROT (duty 0.52) | `run_06` | not usable — 0.72 m, ends at 27° of roll |
| RUN (duty 0.31) | `run_trot_20_lvl0` | not usable — 0.41 m |
| JUMP | `front_jump` | usable as a discrete action; 26 mm of ground covered, gap crossing ≈ 0 m |

TROT's usable horizon is under a third of one goal segment and it ends with the base already
rolled 27° — a state nothing can be chained onto. **A skill that cannot hand over to the next
skill is not a planner action.** Judgment on "usable over short stretches": no. It would be
usable only if a segment shorter than 0.7 m existed that ended in a fall being acceptable,
and no such segment exists in the benchmark.

This collides with CLAUDE.md §2 in a way that has to be resolved rather than absorbed: **the
Single-skill lower bound is specified as fixed trot, and trot is not available under this
premise.**

### 6.2 Does "rule judgment vs learned judgment" still stand with WALK + JUMP?

**No — not as specified.** Two independent reasons, either of which is enough.

**It has no dynamic range.** By §5, WALK alone reaches 0 of 8 goals. The Single-skill lower
bound and the Rule-Planner both walk, so both score approximately zero. A comparison between
a floor and a measurement that are both zero measures nothing.

**It has almost nothing to compare.** With the library reduced to {WALK, JUMP}, the only
decision left is *when to jump*, and in the offline sweep at the reference setting
(`SWITCH_DELAY` 0.21 s, `STEP_WALK_MAX` 0.10 m) that decision is rare:

| gate | runs where a WALK+JUMP planner differs at all from WALK-only | tasks that ever jump | share of ticks in JUMP |
|---|---|---|---|
| `near_edge` | **37 / 200 (18.5 %)** | 8 / 20 | 1.22 % |
| `tracking` | 81 / 200 (40.5 %) | 14 / 20 | 2.54 % |

On the other 81.5 % of runs the two arms are **the same policy by construction**, so their
scores are identical for reasons that have nothing to do with judgment. What the study would
report as "the value of planning" is the jump-trigger rate, measured twice.

(In the sweep as run, the difference between Rule-Planner and Single-skill was mostly the
*speed* choice — WALK 67 % / RUN 30 % / TROT 2 % of ticks. Removing TROT and RUN removes the
choice that was doing that work and leaves the 1–2 % one.)

**This is a statement about the library, not about rule-based planning.** Nothing here says a
rule planner is no better than a fixed skill. It says that with two actions, one of which
fires on a fifth of the runs, the experiment cannot tell the difference either way.

### 6.3 What would restore it, and it is already in the logs

The comparison needs a third action that is exercised often. §5 identifies exactly which one:
a heading correction, needed 13 times per cell.

**There is an untested candidate.** `turn_right_20260824_223951` is an in-place yaw at
−0.395 rad/s (−22.7 °/s), 101.6° of yaw over 0.069 m of translation, and its duty is
**0.715 — higher than WALK's 0.638**, with no flight phase and `periodic = True`. On the one
ordering the evidence supports so far (WALK 0.64 attracting, TROT 0.52 and RUN 0.31
repelling), it sits on the favourable side of every gait tested.

That is a prediction, not a result — duty is a correlate here, not a demonstrated mechanism,
and the sample is three clips. But it is cheap: the harness that produced this document
replays it unchanged.

If it survives, the library becomes {WALK, TURN, JUMP}, the planner faces a real decision on
~13 occasions per cell instead of 0.6, and the Single-skill floor becomes a genuine floor
(0/8 goals) rather than a tie. If it does not survive, the honest report is that the discrete
open-loop-clip skill library cannot cross this benchmark, and per CLAUDE.md §2 that "tried and
failed" is itself the result — the alternative being to change the premise (train per-skill
tracking policies, which closes the loop and is a different experiment).

**Two cautions on sequencing.** Adding a TURN clip is a change to the clip definitions, so it
sits behind the re-extraction gate in §7.1. And the left turn is not simply the right turn's
twin: `turn_left_20260824_223921` has duty 0.566, `periodic = False`, and a 0.88 Hz stride
against the right turn's 1.04 Hz. Constructing the left turn by mirroring the right one is
possible — §3 shows the sim mirrors to parts per thousand — but that is a clip transformation
and must be reported as one, not slipped in.

### 6.4 Open design questions this leaves for the user

1. Re-specify the Single-skill lower bound. Fixed trot is unavailable; fixed WALK is the
   natural substitute, and it scores 0/8 by §5.
2. Decide whether the study proceeds on {WALK, TURN, JUMP} (pending the turn test) or is
   reported as a negative result for open-loop clip replay.
3. If E2E is compared against a discrete arm that reaches 0 goals, decide what that
   comparison is claimed to show.

---

## 7. The two questions that were blocked on `curated/` — now answered

The logs arrived on z4 on 2026-08-28. Both runs below were made after that, in this order.

### 7.1 The re-extraction gate: PASS

CLAUDE.md §1 requires that, before any clip definition changes, `extract_skill_clips.py` is
re-run unmodified and `npz_sha256` reproduces (`aab85a03…`). It does:

```
[clips] WALK  cyclic  gait_classic_walk_20260824_224559   hi= 306@418.3Hz  lo= 37@50.6Hz
[clips] TROT  cyclic  run_06_20260824_225352              hi= 268@416.9Hz  lo= 32@49.8Hz
[clips] RUN   cyclic  run_trot_20_lvl0_20260824_232530    hi= 138@425.8Hz  lo= 16@49.4Hz
[clips] JUMP  oneshot front_jump_20260824_232314          hi=1805@410.5Hz  lo=221@50.0Hz
[gate] rebuilt sha256  aab85a03f5d9063568e2018aa15effce08482978756b8308a6676e34bdcde396
[gate] PASS: reproduced aab85a03… exactly. Clip definitions may now be changed.
```

Same four session picks, byte-identical archive. The frozen files were snapshotted and
restored afterwards (`data/skill_clips.npz` still hashes to `aab85a03…`), and `curated/`
was not written to.

**A defect in the gate as specified, found while running it.** Three things worth carrying
back into CLAUDE.md §1:

- The gate as written **cannot fail**. `extract_skill_clips.py` overwrites
  `data/skill_clips.sha256` with the digest of what it just wrote, so its own `--verify`
  compares the new file against the new hash and passes unconditionally after a rebuild.
  Run bare, it also overwrites the frozen archive in place.
- CLAUDE.md attributes the hash's stability to `np.savez_compressed` zeroing zip
  timestamps; the extractor actually uses uncompressed `np.savez`. The conclusion holds for
  the same underlying reason — verified directly: two runs two seconds apart are
  byte-identical and the zip entries carry `(1980,1,1,0,0,0)`, on numpy 2.2.6 matching
  `environment.yml`.
- `scripts/gate_reextract.py` closes both holes: it captures the frozen digest *before* the
  rebuild, snapshots and restores all four frozen files, keeps the rebuilt copies for
  inspection, and on a mismatch diffs the archive array by array — because a whole-file hash
  cannot distinguish "the extraction changed" from "the zip container was framed
  differently". Its failure path is tested against a deliberately unusable log root.

### 7.2 The averaging question: answered — the averaging is **not** what kills the trot

*Asked 2026-08-28: the clip is a median over phase-aligned cycles, so the controller's
correction may have been averaged out. Replay a single raw cycle instead and A/B, over
several cycles, with a survival-time distribution.*

An extraction change, not a clip edit, and no posture correction was added.
`build_cyclic_clip(..., cycle_subset=[i])` makes the median the identity — same session,
same contact-based cycle detection, same phase grid, same low-pass, same channels — and a
single cycle keeps its **own** period rather than being renormalised to the session median.
`TROT_med` is the control, rebuilt by the same script so the A/B differs in the averaging
and in nothing else. It reproduced the frozen run exactly (2.1899 s, start frame 16,
handover 0.0683 m/s), so the alternate-archive path is neutral.

`run_06_20260824_225352` yields **5 clean cycles, all the session has**:

| clip | survived | vs control | start frame | handover m/s | period | seam (% p2p) |
|---|---|---|---|---|---|---|
| **TROT_med** (control) | **2.1899 s** | — | 16 | 0.0683 | 0.6429 s | 2.8 % |
| TROT_c00 | 1.3900 s | −36.5 % | 18 | 0.0842 | 0.7050 s | 8.8 % |
| TROT_c01 | 1.4064 s | −35.8 % | 2 | 0.1303 | 0.6429 s | 20.5 % |
| TROT_c02 | 2.2367 s | **+2.1 %** | 15 | 0.0671 | 0.5991 s | 9.3 % |
| TROT_c03 | 2.1283 s | −2.8 % | 16 | 0.0678 | 0.6626 s | 4.9 % |
| TROT_c04 | 1.7307 s | −21.0 % | 2 | 0.0990 | 0.6167 s | 37.1 % |

Survival over the 5 raw cycles: min 1.39, median 1.73, max 2.24, mean **1.78 ± 0.35 s**,
against the control's 2.19 s. **All five fell. One of five outlasted the control, by 2.1 % —
0.047 s, an eighth of one cycle.**

> The falsification criterion — "if a raw cycle survives materially longer than `TROT_med`,
> §4 is wrong and the cause is the extraction" — is **not met**. §4 stands.

**Read the distribution with the handover confound in mind.** `--start-phase level` picks a
different start frame per clip, and the three short-lived arms were handed over in worse
states than the control (0.084 / 0.130 / 0.099 m/s against 0.068). Their −36 / −36 / −21 %
is therefore not a clean measurement of the averaging. This is the same confound that
qualified the `symmetrize` ablation in §3.

**The clean comparison is the two arms whose handover matched the control:**

| clip | handover m/s | start frame | survived |
|---|---|---|---|
| TROT_med | 0.0683 | 16 | 2.1899 s |
| TROT_c02 | 0.0671 | 15 | 2.2367 s (+2.1 %) |
| TROT_c03 | 0.0678 | 16 | 2.1283 s (−2.8 %) |

**Within ±3 %.** Matched on the initial condition, a raw unaveraged cycle and the median
clip are indistinguishable. That is the result: the averaging neither caused the collapse
nor was hiding a fix for it.

**The mechanism is unchanged too.** Peak |roll| per cycle, degrees:

```
TROT_med   3.27  11.63  53.06     x3.6  x4.6
TROT_c02   2.22   7.78  26.48     x3.5  x3.4
TROT_c03   3.15  14.02  78.43     x4.5  x5.6
```

Same geometric divergence, same multipliers. Whatever cycle-to-cycle correction survived the
extraction did not stabilise anything — as predicted in advance, because it is applied at a
base state that no longer matches the one it was computed for.

One secondary finding: the raw cycles' seam discontinuity is 4.9–37.1 % of peak-to-peak
against the median clip's 2.8 %, but seam size did not order the outcomes — the worst seam
(c04, 37.1 %) was not the worst survivor, and the two matched-handover arms match the
control at seams of 9.3 % and 4.9 %. In this sample the seam is not what decides it.

**Limits.** n = 5, one session. Repeating over the other 3 candidate TROT sessions would
strengthen it; the user has declined that extension, so this stands as measured.

---

## 8. Reproducing

```
scripts/analyze_drift.py --clips                        # clip symmetry, no simulator
scripts/analyze_heading_budget.py                       # §5, pure geometry, no simulator
scripts/extract_raw_cycles.py --self-test               # §7.2 extraction logic, no logs

scripts/gate_reextract.py                               # §7.1 the gate (needs curated/)
scripts/extract_raw_cycles.py --clip TROT               # §7.2 -> data/raw_cycles_TROT.npz
scripts/run_raw_cycle_ab.sh TROT                        # §7.2 the six replay runs
scripts/analyze_drift.py --ab outputs/raw_cycle_ab/TROT # §7.2 survival distribution

scripts/isaac_docker_run.sh scripts/verify_skill_replay.py --clip WALK --cycles 60 \
    --headless --device cpu --hip-sign keep --contact-threshold-n 30 \
    --start-phase level --settle-mode stand --trace-npz outputs/traces/WALK_long.npz
scripts/analyze_drift.py --trace outputs/traces/WALK_long.npz
```

Traces for all seven simulator runs are in `outputs/traces/`. `logs/drift_runs.log` and
`logs/ablate_runs.log` are the full console output. The offline-sweep numbers in §6.2 come
from `outputs/planner_offline_summary.csv` (7200 runs) — see `outputs/planner_offline.md` for
what that sweep does and does not establish.
