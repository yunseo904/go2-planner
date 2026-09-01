# The Rule-Planner on the 200-cell benchmark, against its own lower bounds

> **TERRAIN: `data/benchmark_frozen.npz` — no roughness, no border walls.**
> Superseded 2026-09-01 as the evaluation terrain.  Every number in this file was measured
> on the frozen archive, which lacks the two things eurekaverse's own `terrain_gpt.py` adds
> to every patch (uniform noise of amplitude 0.02-0.04 m, and a 0.1 m rim raised 0.5 m), and
> which draws its courses from upstream's seed rather than the run seed.  It is therefore an
> **easier** benchmark than the one the E2E numbers come from, and a different draw besides:
> 180 of its 200 cells differ from legged_eval's.
>
> These numbers are kept, not withdrawn — they are the only 200-cell measurement of that
> terrain, and the ablation in `benchmark_legged_eval.md` needs them as its "neither" corner.
> **They may not be compared with, or averaged into, a legged_eval score.**  The re-run under
> the legged_eval protocol is `outputs/benchmark_legged_eval.md`.

The comparison CLAUDE.md §2 asks for, run for the first time with the planner actually
choosing. All 200 cells, one 20 s episode each, upstream's own goal rule (within 0.20 m
held for 0.10 s, eight goals in order), equal weight over cells. CPU, no GPU.

**All three arms run under identical conditions** — heading hold on, foot placement on,
same entry frames — which is the point of re-measuring the baselines rather than quoting
the earlier ones.

## 1. The result

| arm | score / 8 | median | max | zeros | alive at 20 s |
|---|---|---|---|---|---|
| **Rule-Planner** | **1.07** | 1.0 | 4 | 21/200 | 64/200 |
| **WALK fixed** (lower bound) | **1.11** | 1.0 | 4 | 21/200 | 64/200 |
| TROT fixed (lower bound) | 0.97 | 1.0 | 2 | 21/200 | **9/200** |
| *upstream `walk_pretrain`* | *1.05* | | | | |
| *upstream E2E teacher `_10_3`* | *4.48* | | | | |

**The planner scores below the single-skill lower bound it is supposed to beat.** Paired
per cell — the same task and level in both arms — it is **better in 0 cells, worse in 6,
and tied in 194**.

**Read the floor before quoting any of these.** Goal 0 sits **0.50 m from the spawn in 180
of the 200 cells**, so a score of 1.0 means the robot moved half a metre. The benchmark's
useful range runs from about 1 to about 4.5, not 0 to 8, and every number in the table
above is at the bottom of it.

## 2. Why: the planner is WALK with occasional TURN detours

Share of the time each robot spent upright, by skill:

| skill | mean share | cells that used it at all |
|---|---|---|
| WALK | **0.898** | 200 / 200 |
| TURN | 0.098 | 119 / 200 |
| **TROT** | **0.004** | **4 / 200** |

It does switch — median 1 switch per cell, mean 1.6, max 13, and 79 of 200 cells never
switch — and it refuses skills the library cannot serve (median 5 ticks per cell, max 189;
RUN and JUMP are both unresolved, `unmeasurable.md`).

So the arm is **WALK, plus a TURN detour in 119 cells**, and those detours are what the 6
worse cells are. TROT is chosen essentially never, which is consistent with everything
measured about it: it survives 4 of its 32 entry phases on flat and 9 of 200 cells here.

## 3. The sub-integer columns, which is where the arms actually differ

The goal count has about three and a half units of range, so it was asked to be reported
alongside position:

| | travelled, median | dist to next goal | dist to last goal |
|---|---|---|---|
| Rule-Planner | 1.11 m | **0.93 m** | 9.43 m |
| WALK fixed | **1.15 m** | 0.99 m | **9.31 m** |
| TROT fixed | 1.06 m | 1.14 m | 9.55 m |

**The planner travels slightly less and ends slightly closer to the goal it was chasing.**
That is the TURN detour doing exactly what it is for — reorienting toward a goal without
translating — and it is not enough to convert into a goal, because the threshold is a
0.20 m circle and the robot is a metre away.

## 4. Heading hold does not move this benchmark, and that is worth stating

WALK scored **1.11 with heading hold and 1.11 without** it (`benchmark_harness.md`); TROT
0.97 against 0.98. The heading work is real — `heading_hold.md` takes WALK's curvature from
13.26 to 0.24 °/m and `trot_yaw_moment.md` takes TROT's from 3.20 to 0.01 — and **none of
it reaches this score**, because at a median travel of 1.1 m nothing gets far enough for
curvature to matter. A 0.565 °/m budget over 2.25 m of goal spacing is a question that only
arises once a robot covers 2.25 m.

**So the concern that the old baselines would flatter the planner turns out to be moot** —
the baselines are the same either way — but the arms are now measured under one
configuration, which is what makes the 1.07 vs 1.11 comparison legitimate at all.

## 5. What this does and does not establish

**Establishes**: with the current skill library, the rule planner has nothing to add over
holding WALK on this benchmark. Not because the rules are wrong — it switches, and it
refuses correctly — but because two of its five skills are unresolved (RUN, JUMP), one is
chosen 4 times in 200 cells (TROT), and the remaining pair is WALK plus a turn.

**Does not establish** that a rule planner cannot work here. The arm's ceiling is its
library, and the library is currently one usable locomotion skill.

**Perception caveat.** The planner reads the frozen archive's own height field through
`planner.features.extract`, which applies the sensor model (near/far clip, blur by ground
resolution, confidence) but not rendering. This is an **optimistic perception arm** —
ground-truth geometry through a sensor model, not a depth image — and it still does not
beat WALK, so the perception is not what is limiting it.

**The 21 zeros are the same 21 cells in all three arms**, which says they are a property of
the terrain rather than of any skill.
