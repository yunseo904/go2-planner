# TURN: the entry phase decides it, and neither rule in the code picks a good one

**Asked**: whether the foot-coplanarity criterion used for WALK and TROT was also applied
to TURN, and whether it fits a turn in place, whose contact pattern is not a translating
gait's.

**Answer: it is applied, it does not fit, and the phase it picks is inside a ten-frame
band where the flat control never completes a turn.** But the diagnosis is not "the
criterion is wrong for TURN" — it is that **no pose criterion predicts the outcome at
all**, and the entry frame has to be measured.

All runs CPU, `--device cpu`, no GPU. 810 grid runs + 4 single-robot runs.

---

## 1. What the two rules pick, and what the harnesses actually do

Two different rules exist and a third path uses neither:

| rule | where it is used | TURN picks |
|---|---|---|
| `level_start` — most coplanar four feet | `run_planner_replay.py` (every clip), `verify_skill_replay.py --start-phase level` | **frame 24** |
| `quiescent_start` — most feet down, then slowest | `verify_skill_replay.py --start-phase stance` | frame 31 |
| none — `round(rep · n / reps)` | `run_calibration_grid.py`, one phase per repeat | 0, 9, 18, 27, 36 at `--reps 5` |

`turn_probes.md` §3 ran the third path at `--reps 5` and got 2 of 5 phases passing. That
sample was read as "TURN fails 3 of its 5 entry phases". The sweep below says the rate is
right and the composition was luck.

## 2. Every phase, both compensator arms — 810 runs

`scripts/run_calibration_grid.py --families step_up step_down gap --skill TURN --score
inplace --inplace-at spawn --max-probes 3 --reps 45`, i.e. all 45 frames of the 45-frame
clip, 9 identical flat cells each, once with `--foot-comp on` and once with it `off`.
Pass = 90° of yaw completed inside a 0.35 m circle without falling, the criterion
`turn_probes.md` §2 established.

| | pass 9/9 | fail 0/9 | mixed |
|---|---|---|---|
| `--foot-comp on` | **22 / 45** | 17 | 6 |
| `--foot-comp off` | **5 / 45** | 36 | 4 |

Two things fall out immediately.

**Foot placement is most of what makes TURN work**: 22 phases against 5, and the off-arm's
passing set `{5, 6, 7, 15, 36}` is a strict subset of the on-arm's. That is a larger effect
than anything foot placement has shown on the translating gaits.

**The passing phases are not one basin.** With the compensator on they are
`4–9, 11, 13, 15, 16, 30–32, 36–44` — two clean bands with a ragged region between them,
and `10, 12, 14` failing between passing neighbours. The six mixed phases (2, 21, 22, 23,
29, 33) are cells that are identical flat ground disagreeing with each other, which is
`harness_findings.md` §12's sensitivity showing up again. Part of the cycle is robust and
part of it is chaotic.

## 3. `level_start` picks frame 24, and 20–29 is a dead band

Frame 24 scores **0/9 with the compensator on and 0/9 with it off**. So the alternative
"the compensator is what breaks those phases" is excluded: they fail either way. It sits
in the middle of frames 20–29, a contiguous ten-frame region in which no phase passes in
either arm. The whole region reaches 37–47° of its 90 and drifts 0.38–0.59 m — out of the
circle at about half a turn.

`quiescent_start`'s frame 31 passes 9/9 with the compensator on and **0/9 with it off**, so
it is not robust either; it happens to work in the configured condition.

## 4. No pose property predicts pass or fail

The obvious candidates, over all 45 frames:

| feet down at entry (clip contact channel) | pass | fail | mixed |
|---|---|---|---|
| 2 | 16 | 6 | 0 |
| 3 | 3 | 8 | 4 |
| 4 | 3 | 3 | 2 |

Four feet down — the thing `quiescent_start` maximises — is 3 pass, 3 fail, 2 mixed. Joint
speed at entry does not separate them either (pass median 0.786 rad/s, fail 0.607, ranges
overlapping). And coplanarity actively misleads: frame 24 has the **best** foot spread in
the cycle, 5.4 mm, and 7 frames sit under 10 mm so the criterion is nearly tied across them
and the tie-break — joint speed — decides.

**So the entry frame is not derivable from the pose. It is a measurement.**

## 5. The frame that is chosen, and why that one

Of the five phases passing 9/9 in **both** arms, ranked by how many of `{k−1, k, k+1}` also
pass in both:

| frame | ±1 robust | yaw completed (on / off) | drift (on / off) |
|---|---|---|---|
| **6** | **3/3** | 91.0° / 93.2° | **0.19 / 0.19 m** |
| 7 | 2/3 | 93.5 / 98.8 | 0.19 / 0.18 |
| 5 | 2/3 | 91.5 / 92.0 | 0.19 / 0.17 |
| 36 | 1/3 | 91.7 / 91.7 | 0.17 / 0.19 |
| 15 | 1/3 | 96.5 / 92.8 | 0.19 / 0.18 |

**Frame 6 is the only phase whose neighbours also pass in both arms**, so it tolerates a
one-frame phase error and the compensator being on or off. It is now
`planner.config.skill.ENTRY_FRAME_TURN = 6`, provenance MEASURED, and reachable from the
harnesses as `--start-phase measured` (default remains `first`; nothing earlier changes).

## 6. The second harness disagrees on two of four phases, and that is the real finding

The same four phases in `verify_skill_replay.py`, one robot, analytic ground plane, 60
cycles, `--foot-comp raibert --foot-yaw log-cycle`:

| `--start-phase` | frame | verdict | yaw rate | cycles | grid, in place |
|---|---|---|---|---|---|
| `first` | 0 | FAIL, roll at 4.82 s | −10.95 °/s | 2 | 0/9 |
| `level` | 24 | **PASS** | −20.66 °/s | 63 | **0/9** |
| `stance` | 31 | FAIL, roll at 5.33 s | −11.34 °/s | 3 | 9/9 (on), 0/9 (off) |
| **`measured`** | **6** | **PASS** | **−21.33 °/s** | 62 | **9/9 both arms** |

The log's own yaw rate is −22.66 °/s, so frame 6 is the closest of the four.

The two harnesses disagree about frames 24 and 31, and the disagreement is not noise —
**they score different things.** `verify_skill_replay` terminates on roll, pitch and base
height. It has **no drift term at all**. Frame 24 turns steadily and indefinitely *while
translating*: the grid measures it 0.41–0.58 m from its spawn inside the first 8 s against
frame 6's 0.19 m. Over 60 cycles that early drift is diluted, and both come out at
`vx_mean` ≈ 0.021 m/s, so the column that ought to catch it does not.

**The 60-cycle harness's PASS verdict for TURN is blind to the only thing TURN is for.** A
turn in place that walks half a metre away is not a turn in place; it is a curved walk with
the right yaw rate. Frame 6 is the only one of the four that two independent instruments
both call good, which is the agreement CLAUDE.md §6.5 asks for.

## 7. What this does and does not buy

**Does**: TURN now turns on flat ground, reproducibly — 90° in place, inside 0.19 m, on 9
of 9 cells in both compensator arms, and 62 sustained cycles in the other harness. That was
the flat-ground goal.

**Does not**: change `turn_probes.md` §4 at all. Its reduction already used the flat control
as its own denominator, so the terrain result — TURN's limit is below every ladder's floor,
under 0.02 m of step — was already measured against phases that turn. A better entry phase
does not put a turn on a 2 cm edge.

**Also does not** generalise as a story about TURN alone. The same sweep run on TROT (see
`trot_capture_point.md` §4) shows entry frame driving the fall rate there too: 11/15 cells
fall from frame 0, 15/15 from frames 6, 19 and 26. Entry phase is a live variable for every
cyclic clip in this library and only TURN has been swept.
