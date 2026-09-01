# The oracle ceiling, the depth arm, and the threshold correction

Four questions, one grid.  All on legged_eval terrain, seed 1, 200 cells, 20 s, identical
settings (`--heading heading-only --start-phase first --foot-comp on`, planner arms also
`--yaw-moment hold`).  The depth arm is GPU 1; everything else CPU.

## 1. The oracle ceiling — no rule can win

Per cell, the best of the single-skill arms, then the equal-weight mean over cells.

| arm | score / 8 |
|---|---|
| WALK fixed | 0.755 |
| TROT fixed | 0.290 |
| **TURN fixed** (new) | **0.170** |
| **ORACLE — best of the three, per cell** | **0.810** |
| *(best of WALK/TROT only)* | *0.790* |
| Rule-Planner (optimistic perception) | 0.570 |

**A perfect chooser gains 0.06 over always walking.**  Some skill beats WALK in only 12 of
200 cells (TROT in 7, TURN in 5), and in the other 188 the best thing the library can do is
what WALK already does unaided.

This answers the question it was asked to answer.  The planner is not losing 0.19 to WALK
because the rules are wrong; **the rules are choosing among options that are not there.**
Even a planner with a perfect oracle and no switching cost would score 0.81 against a
teacher's 4.83.  The route is skill improvement, not planner redesign.

TURN fixed is 0.17 for the reason its own measurement predicts: `SPEED_TURN` is 0.0075 m/s,
so in 20 s it covers 0.15 m and goal 0 is 0.50 m away in 180 of 200 cells.  It is in the
table because an envelope with a missing corner is not an envelope, not because it was
expected to contribute — and it does contribute, in 5 cells, where re-aiming beats walking.

## 2. The depth arm — and it is not worse

`--perception depth` renders eurekaverse's own camera (mount, 87 deg, 106x60 crop to 90x60,
update every 5 steps, one step of latency, granular 0.02 and blackout 0.03, clip 0–2 m, all
read out of `CustomDepthCfg` by `sim/isaac_cfg.depth_cfg()`) and inverts it with
`legged_eval.adapters.depth_terrain`.  Sanity, printed by the run: buffer (200, 60, 90) in
[−0.500, +0.500], **79.6% of pixels returning inside 2 m**, map coverage **28.8%** of the
41x81 local grid, observed heights −1.00 to +0.16 m — the −1.00 is the ray-carve marking a
pit, the +0.16 is real geometry.

| Rule-Planner | score / 8 |
|---|---|
| optimistic (ground truth through a sensor model) | 0.570 |
| **depth (rendered, occluded, noisy)** | **0.600** |

Paired: depth better in 10 cells, worse in 4, tied in 186.

**The real sensor is very slightly better than perfect knowledge**, and the mechanism is
visible in the skill mix:

| | optimistic | depth |
|---|---|---|
| frac_WALK | 0.731 | 0.757 |
| frac_TURN | 0.256 | 0.243 |
| **frac_TROT** | **0.0138 (14 cells)** | **0.0000 (0 cells)** |
| refused ticks | 8.7 | 4.3 |
| switches | 1.72 | 2.22 |

**The depth arm never chooses TROT.**  The optimistic arm does, in 14 cells, and TROT fixed
is the arm that scores 0.29 and survives 1 cell of 200.  The privileged map is the terrain
at 0.05 m including every millimetre of the roughness; the observed map is 29% covered,
carries −1.0 m carve marks where the rays found no floor, and has the rest filled as flat.
Those two look different to a roughness threshold, and the one that looks *worse* is the
one that refuses the gait that kills the robot.

Read carefully, this is not "depth is better than truth".  It is **0.03 of 8**, and the
honest reading is the one both numbers agree on: **perception is not what limits this
planner.**  A planner that sees everything scores 0.57, one that sees what the robot really
sees scores 0.60, and a fixed WALK that sees *nothing at all* scores 0.76.

## 3. The threshold correction — real, and small

`planner/config.py` carries `STEP_TROT_MAX = 0.08 m` marked `CALIBRATION_NEEDED`, while
`SESSION_STATE.md` §7 measured TROT's step bracket at **0.02–0.04 m**.  The planner has
been choosing TROT under a belief about itself that is two to four times too generous.
Overridden at run time only (`--planner-set skill.STEP_TROT_MAX=…`); the config still says
0.08 and still says `CALIBRATION_NEEDED`.

| STEP_TROT_MAX | score | cells using TROT |
|---|---|---|
| 0.08 (config) | 0.570 | 14 |
| **0.03 (middle of the bracket)** | **0.590** | 5 |
| 0.02 (pessimistic end) | 0.590 | — |

Paired against the config run: better in 5 cells, worse in 1, tied in 194.  **The correction
is real and it is worth 0.02**, which is the same size as the effect §2 gets by accident.
The choice *inside* the bracket does not matter — 0.02 and 0.03 give the same score — so
the remaining calibration question is not urgent.

Note what this does and does not say.  It confirms the mechanism (a wrong self-belief was
putting the robot into a gait it cannot hold) and it does not rescue the arm: 0.59 is still
0.17 below WALK and 0.22 below the oracle.

## 4. The four numbers together

| | score / 8 |
|---|---|
| E2E teacher, same protocol, seed 1 | **4.83** |
| oracle over the whole skill library | 0.81 |
| WALK fixed, no perception at all | 0.76 |
| Rule-Planner, depth | 0.60 |
| Rule-Planner, corrected threshold | 0.59 |
| Rule-Planner, as configured | 0.57 |

Everything between 0.57 and 0.81 is planner engineering, and the whole of it is 0.24.
Everything between 0.81 and 4.83 is the skill library, and it is 4.02.
