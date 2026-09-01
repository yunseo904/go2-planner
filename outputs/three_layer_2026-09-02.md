# Three layers, and what they disagree about

Judging moved off the benchmark score.  (a) **flat**: 60 cycles, stride, v_x, yaw rate,
against the recording's own logged values.  (b) **probes**: step / gap / slope / roughness
limits.  (c) **benchmark**: last.  An intervention has to move (a) or (b) before (c) means
anything — and this session found a case where (b) and (c) point opposite ways.

---

## 1. TURN on flat — the hip-sign question, settled on the skill it matters for

60 cycles, against the log's **−22.66 °/s** and **+0.0075 m/s**:

| arm | yaw °/s | % of log | v_x | stride Hz | stride cv |
|---|---|---|---|---|---|
| keep, rate lo, entry level | −16.32 | 72 % | +0.017 | 1.12 | 0.224 |
| **flip** | **+1.09** | **5 %** | −0.008 | **1.94** | 0.250 |
| keep, entry first | −16.99 | 75 % | +0.017 | 1.12 | 0.228 |
| keep, entry measured | −17.75 | 78 % | +0.018 | 1.12 | 0.243 |
| foot-comp off | −14.70 | 65 % | +0.003 | 1.12 | 0.395 |
| foot-yaw off (v_y→0 law) | −15.36 | 68 % | +0.008 | 1.12 | 0.247 |
| **rate hi, entry first** | **−18.85** | **83 %** | +0.017 | 1.12 | **0.068** |
| rate hi, foot-comp off | −10.85 | 48 % | +0.003 | 1.41 | 0.494 |

**`flip` does not turn.**  +1.09 °/s is the wrong direction at 5 % of the rate, and stride
breaks from 1.12 to 1.94 Hz.  TURN is the clip whose hips move most (±0.23 rad), so this is
a **stronger** test than the WALK forward-walk comparison that set the default, and it
agrees with it: **`keep`.**  The worry that an in-place turn's different contact pattern
might reverse the verdict is answered — it sharpens it instead.

**`--rate hi` is free on flat and the benchmark has never used it**: TURN 72 → 83 % with
stride cv 0.224 → 0.068.  Control on the other gaits: TROT stride identical (1.56), cv
0.022 → 0.012, v_x +1 %; WALK stride identical (1.37), cv 0.031 → 0.017, but **v_x +13 %**,
which is outside the ±10 % band — so adopting `hi` globally is not free and it is not
adopted here.

**`convention_verified`**: written as a literal `False` by every extractor and computed by
nothing.  It is a placeholder for *"nobody has checked that the stored leg order and joint
signs match this robot"*.  The flat rig's identity check **is** that test; on TURN it
passes at r = 0.380 with a 0.054 margin, which the tool itself flags as thin.  It would
become `True` by running that check on every clip and recording the result — and the flip
row above is the first strong evidence for one clip.

## 2. The benchmark's TURN was running with ω_log = 0 — a real bug, not the missing half

`turn_target.md` established that the placement law must be given the log's own yaw rate,
because it otherwise drives every foot toward `v_y = 0` and an in-place turn's lateral foot
velocity **is** the motion.  The planner arm has had that since.  The **single-skill** arm
never did: the only way to run TURN is `--heading off` (HEADING_CAP has no TURN entry and a
zero cap is refused), and that skips the block that loads the log's motion, leaving
`wz_log = 0.0`.

Fixed (`--foot-yaw-turn`, default `auto`, `off` reproduces the old rows).  Worth
**34 % → 35 %** on the benchmark.  Real, and not the explanation.

## 3. Where the flat-to-benchmark gap actually goes — half found, half open

Flat says 72–83 %.  The benchmark says 34–38 %.  Candidates tested:

| candidate | result | verdict |
|---|---|---|
| roughness | smooth 42 % vs rough 34 % | **not it** |
| the courses' obstacles | `staircase_spiral`'s flat 2.6 m approach: **38 %** | **not it** |
| clip rate | hi helps flat (83 %) and **hurts** the grid (15 %) | not it, and itself unexplained |
| foot placement's ω_log | 34 % → 35 % | not it (§2) |
| hip sign | flip is catastrophic | not it |
| entry phase | 34 % → 36 % | not it |
| settle mode | flat `drop` 71 % vs `stand` 61 % at 12 cycles | not it — drop is *better* |
| **the measurement window** | see below | **half of it** |

The flat rig measures over 60 cycles ≈ 59 s.  The benchmark's TURN robots are upright a
median **3.4 s**.  Same settings, different window:

| cycles | ≈ seconds | yaw °/s | % of log |
|---|---|---|---|
| 4 | 3.8 | −12.97 | **57 %** |
| 11 | 10.6 | −13.93 | 61 % |
| 25 | 24.0 | −16.44 | 73 % |
| 61 | 58.7 | −16.99 | 75 % |

**The turn is a slow transient.**  It needs ~25 s to reach 73 % and the benchmark gives it
3.4 s, where flat itself only manages 57 %.  So roughly **half** the gap (75 → 57 %) is the
window, and it is not a fidelity loss at all — it is what the number means.

**Still open: 57 % → 35 % on the grid at a matched window.**  Everything in the table above
is excluded.  What is left: the per-cell lever measurement, the spawn drop from 0.42 m onto
a rough patch, and the interaction with the 200-robot scene.

## 4. Probes — where (b) and (c) disagree, and it matters

Highest level passed on **every** repeat, reps 3, `--foot-comp on --heading heading-only`,
v2 probe archive:

| skill / couple | step_up | step_down | gap | slope | roughness |
|---|---|---|---|---|---|
| WALK / off | 0.020 | 0.020 | 0.000 | 0.000 | **0.010** |
| WALK / **roll couple** | 0.020 | 0.020 | 0.000 | 0.000 | **0.005** |
| TROT / off | **0.000** | 0.000 | 0.000 | 0.000 | 0.000 |
| TROT / roll couple | **0.000** | 0.000 | 0.000 | 0.000 | 0.000 |

Per rung, WALK `step_up`: off 0.020 → **3/3**, 0.040 → 0/3.  With the couple: 0.020 → 3/3,
0.040 → **1/3**.  A movement in the right direction, not a threshold change.

Per rung, WALK `roughness`: off 0.005 → **3/3**, 0.010 → **3/3**, 0.015 → 1/3.  With the
couple: 0.005 → **2/3**, 0.010 → 1/3, 0.015 → 1/3.

**The roll couple LOWERS WALK's roughness limit, from 0.010 to 0.005 m.**  On the benchmark
the same term is worth **+0.07**, and the benchmark's terrain is made of roughness at
0.02–0.04 m.  The two layers point opposite ways.

This is exactly what the three-layer split was set up to catch, and it means one of two
things, both worth knowing: either the benchmark gain comes from something other than
roughness tolerance (the rim, the obstacles, the goal geometry), or the probe's roughness
family and the benchmark's noise are not the same disturbance.  **Level 2 should not be
described as "better on rough ground" until that is resolved.**  Its benchmark gain is
real and repeated across three seeds; its mechanism is not established.

**TROT clears nothing.**  0/3 at the lowest rung of every family, with and without the
couple — 0.020 m of step, 0.005 m of roughness, the shallowest slope.  That is consistent
with TROT surviving 1 cell of 200, and it means the 0.02–0.04 m bracket quoted for
`STEP_TROT_MAX` came from a different rig and criterion and **does not reproduce here**.
Nothing was found that raises TROT's limits.

## 5. Videos — `outputs/video_turn/`

Flat TURN, 12 cycles, at **half speed** (50 Hz control captured every frame, written at
25 fps): `turn_side.mp4`, `turn_top.mp4`, and `turn_top_nofoot.mp4` (the same clip with the
placement law off).  The top-down view was added for this — from the side a 90° rotation is
a silhouette change, from above it is the motion, and the floor grid gives the heading.

**Regression check passed exactly**: the same run with and without rendering is
**identical on every column**, including `yaw_rate_deg_s` to all 16 digits
(−17.30738639831543 both ways).

Not done: the in-air replay of the log's joint angles, and the WALK couple on/off pair on a
probe step.  Both are queued rather than attempted, and the in-air one is the more useful —
it is the kinematic control for §1.

## 6. RUN and JUMP — closed

* **RUN**: the roll couple cannot lift the body — peak base height **0.313 → 0.311 m**
  against WALK's 0.330 m standing.  A hip (abduction) torque on a stance leg produces a
  force perpendicular to the hip→foot vector, which on a near-vertical leg is horizontal.
  There is no vertical component to give.  Measured, not argued.
* **JUMP**: 23.70 / 45.43 N·m are Unitree's published **maxima**, no continuous rating is
  published, and the gaps on this grid are 0.10–5.15 m against 26 ± 4 mm of horizontal
  travel — 4× to 200× short.  Closed.
