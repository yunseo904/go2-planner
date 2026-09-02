# The real robot against the replay of it

`why_it_rolls.md` §3 left the sharpest fact in the project: over the same 4–5 s the real
robot's worst \|roll\| is **3.39°** and the sim passes **20°** and does not come back, playing
the same joint angles. Asked: is that a reproduction failure (fixable) or a limit of
open-loop replay (a result)?

**Neither, and that is the finding.** On flat ground the replay reproduces the recording as
well as the real robot's own controller did, and it stays up. The 20° event needs terrain.

Comparisons are all against each clip's **own source session**
(`data/skill_clips.meta.json`: WALK ← `gait_classic_walk`, TROT ← `run_06`).

---

## 1. Per-cycle peak roll — the sim does not accumulate, it is just worse

Cycle boundaries from touchdown of the first leg. Real stance from `foot_force_*` above 25 %
of that leg's **own p95** — `CLAUDE.md` §6 forbids the log's fixed 20 N `contact_*` column.

| cycle | real | sim, flat rig |
|---|---|---|
| 1 | 2.38° | **6.66°** |
| 2 | 3.39° | 2.99° |
| 3 | 1.59° | 3.52° |
| 4 | 1.87° | 4.23° |
| 5 | 1.64° | 3.71° |
| … | | 3.69–4.17° through cycle 19 |
| **median** | **1.87°** | **3.71°** |
| **trend per cycle** | −0.300° | **−0.048°** |

**The sim's roll is bounded and flat for nineteen cycles.** It is about twice the real
robot's and it does not grow — the trend is slightly *negative*. Cycle 1 is the worst cycle
in the whole sim run (6.66°), which is the settle, not the gait.

**So on flat ground there is no roll event to explain.** The 20° roll-out measured in
`why_it_rolls.md` is on the **benchmark**, and the benchmark's mean \|roll\| is 6.28°
(WALK) / 8.28° (TROT) against the flat rig's 1.68°. The difference is the terrain, not the
replay.

## 2. Joint tracking — the sim is not the worse controller

\|achieved − commanded\|, radians:

| joint | real log | sim flat | **sim / real** |
|---|---|---|---|
| hip | 0.0466 | 0.0641 | 1.38 |
| thigh | 0.0674 | 0.0559 | **0.83** |
| calf | 0.1676 | 0.1457 | **0.87** |

**The real robot's own PD misses its own targets by more than the sim does on two joints of
three.** The calf misses by 0.17 rad (9.6°) *on the real robot*. Whatever the sim is doing
wrong, it is not failing to reproduce the commanded joint angles.

That answers the question as posed: **it is not a reproduction problem**, and it is not a
generic limit of open-loop replay either, because the same open-loop replay holds up for 19
cycles on flat ground.

## 3. Where it does differ: the front legs are under-loaded, and the clip does not ask for it

Per-leg stance duty, three sources:

| skill | source | FL | FR | RL | RR | **front/rear** |
|---|---|---|---|---|---|---|
| WALK | real log force | 0.792 | 0.733 | 0.842 | 0.790 | **0.934** |
| WALK | the clip's own contact channel | 0.703 | 0.649 | 0.595 | 0.703 | **1.042** |
| WALK | sim, benchmark | 0.706 | 0.621 | 0.871 | 0.791 | **0.798** |
| TROT | real log force | 0.706 | 0.658 | 0.692 | 0.715 | **0.970** |
| TROT | the clip's own contact channel | 0.594 | 0.594 | 0.531 | 0.594 | **1.056** |
| TROT | sim, benchmark | 0.495 | 0.533 | 0.665 | 0.631 | **0.793** |

**The clip is slightly front-biased (1.04–1.06), the real robot is near-symmetric
(0.93–0.97), and the sim is rear-biased (0.79) — by the same amount in both skills.** So the
asymmetry SESSION_STATE §16.2 found on TROT is **made by the replay**, is not a property of
the recording, and is not TROT-specific.

**Pitch is excluded as the cause.** A nose-down base would load the front *more*, and the
base is nose-down in both: real −1.42°, sim flat −2.85°, sim benchmark −3.40°. The direction
is wrong for the explanation. (An earlier draft of this file offered pitch as the mechanism;
it does not survive its own table.)

**Unexplained, and it is now the most specific open lead**, because §4 shows the front legs
are also the ones that let go first.

## 4. Which foot lets go first — and it is skill-specific

Benchmark, no roughness, contact force > 5 N, the leg most unloaded in the 0.5 s before
\|roll\| first passes 10°:

| skill | FL | FR | RL | RR | mean feet down, 1.5 s before → 0.5 s after |
|---|---|---|---|---|---|
| **WALK** | 42 % | 55 % | 1 % | 3 % | 3.12 → 2.70 |
| TROT | 48 % | 14 % | 23 % | 15 % | 2.42 → 2.19 |
| **TURN** | 1 % | 0 % | **99 %** | 0 % | 3.37 → 2.80 |

**WALK loses a FRONT foot 97 % of the time. TURN loses the rear-left 99 % of the time.**
TROT is front-biased but mixed. Support falls across the onset in all three.

So **the order is not the same across skills** — the answer to "same event in all three?" is:
same *shape* (support drops, then roll), different *leg*. And WALK's answer lines up with §3:
the pair the replay under-loads is the pair that lets go.

## 5. What this leaves

- **Not a reproduction failure** (§2) and **not an open-loop inevitability** (§1).
- **The rear-load bias is made by the replay** (§3) and is unexplained; pitch is excluded.
- **The front pair both carries too little and lets go first** on WALK (§3 + §4). That
  conjunction is the lead.
- **The 20° roll needs terrain**: flat holds 3.7° for 19 cycles, the benchmark averages
  6.3–8.3° and rolls out. Which terrain feature, and whether it is the 0.42 m spawn drop
  rather than the geometry, is not established here.
