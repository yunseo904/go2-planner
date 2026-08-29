# Heading: what can steer this robot, measured rather than argued

Foot placement bought the gait and did not buy a heading — the compensated trot runs
60 cycles and yaws 5.32 °/s while it does it, about 7.8 °/m against the benchmark's
0.565 °/m budget. This is the survey of what could correct that, with each candidate's
authority **measured** rather than reasoned about.

The order of work matters here. The obvious move is to close a loop on yaw rate, and
that was tried first and made things worse. So the question was turned into an
open-loop one — *how much yaw does a given edit buy, and what does it cost?* — because
an actuator's gain is a property you can measure once, whereas a controller that fails
tells you almost nothing about which part failed.

---

## 0. First, what the error is

Measured on the runs that survive 60 cycles:

| run | yaw rate | curvature | quarters of the run |
|---|---|---|---|
| TROT + foot placement | +5.32 °/s | 8.08 °/m | +6.18, +4.74, +5.26, +5.10 |
| WALK open loop | +2.68 °/s | 11.49 °/m | +3.02, +2.36, +2.55, +2.78 |
| WALK + foot placement | +3.00 °/s | 12.98 °/m | +3.02, +2.88, +3.00, +3.11 |

**It is a steady bias, not a random walk** — the quarter-by-quarter means barely move.
That is the kind of error a proportional corrector can remove, and it is why this is
worth doing at all. (The log's own robot also curves, at 2.05 °/m, itself 3.6× over
budget, so heading correction was always going to be needed; the sim is not inventing
the problem.)

---

## 1. Candidate (c) is not available: the Go2 has no hip yaw joint

Worth settling first because it removes an option. The articulation has 12 joints, three
per leg — `FL_hip_joint`, `FL_thigh_joint`, `FL_calf_joint` and the same for the other
three. The "hip" joint is the **abduction** joint: it rotates about the body x axis, which
is why a positive hip angle moves the foot toward +y on every leg (§ the lever
measurement, 0.306–0.316 m). There is no joint that rotates a leg about the vertical.

So steering has to come from *where the feet are placed*, not from twisting a leg.

---

## 2. The two mechanisms, measured open loop

Both are a **constant** offset added to the swing legs, with no feedback of any kind, so
a run with one on is still an open-loop replay plus a constant edit — the same category
as `--plant-comp`. The point is to measure the actuator's gain, in °/s of yaw per radian
of edit, before building any controller on it.

- **(a) differential lateral placement** (`--foot-yaw-bias`): +front / −rear on the
  **hips**, which puts the front feet one side of the body and the rear feet the other.
- **(b) differential step length** (`--foot-len-bias`): +left / −right on the **thighs**,
  so one side takes longer steps than the other.

### On WALK, with nothing else running (the clean measurement)

WALK survives open loop, so the bias can be measured with no other feedback at all
(`--foot-comp off`). Baseline +2.68 °/s, vx 0.233, stride 1.37.

| bias (rad) | (a) yaw °/s | Δ | vx | | (b) yaw °/s | Δ | vx |
|---|---|---|---|---|---|---|---|
| −0.04 | **−4.98** | −7.65 | 0.233 (+0.3%) | | +1.01 | −1.67 | 0.236 |
| −0.02 | −0.86 | −3.54 | 0.240 (+3.3%) | | — | | |
| 0 | +2.68 | — | 0.233 | | +2.68 | — | 0.233 |
| +0.02 | +5.86 | +3.18 | 0.224 (−3.7%) | | — | | |
| +0.04 | +9.12 | +6.44 | 0.213 (−8.6%) | | +4.92 | +2.24 | 0.238 |

**(a) is linear, symmetric, bidirectional and cheap: 174.5 °/s per rad**, every point
survives 60 cycles at the exact stride, and at −0.04 it drives the yaw rate *through zero*
to −4.98. Nulling WALK's whole 2.68 °/s drift needs about **0.015 rad**, half the
deviation budget the trot already spends, for a few percent of forward speed.

**(b) is 3.6× weaker on the same gait: 48.9 °/s per rad.** Safe, but it would need
0.077–0.086 rad to do the same job — nearly three times the whole stage-2 budget.

### On TROT, with the lateral loop also closed

Baseline +5.32 °/s, vx 0.659, stride 1.56.

| bias (rad) | (a) survived | yaw / Δ | vx | | (b) survived | yaw / Δ | vx |
|---|---|---|---|---|---|---|---|
| −0.04 | **7.94 s** | −1.78 / −7.10 | −25.5% | | 60+ cyc | +0.61 / **−4.71** | −1.2% |
| −0.02 | 60+ cyc | +2.22 / **−3.10** | +0.2% | | 60+ cyc | +1.34 / −3.98 | −0.2% |
| +0.02 | 60+ cyc | +6.67 / +1.35 | −0.1% | | **3.38 s** | −12.98 / −18.30 | −32.6% |
| +0.04 | **8.78 s** | +5.33 / 0.00 | −25.5% | | **2.67 s** | −6.92 / −12.24 | −39.0% |

Two things change on the trot:

- **The safe amplitude collapses to ±0.02 for (a)**, where it still buys 3.10 °/s for
  nothing measurable. At ±0.04 the trot falls in *both* directions and loses a quarter of
  its speed — so this is an amplitude limit belonging to the trot, not a sign question and
  not a property of the mechanism. WALK took ±0.04 without noticing.
- **(b) becomes strong but one-directional.** −0.04 removes 4.71 of the 5.32 °/s for 1.2%
  of speed, which is the largest correction anything here has produced; +0.02 destroys the
  gait, swinging the yaw rate to −12.98 °/s and the stride to 1.99 Hz on the way down.
  Its gain is also gait-dependent — ~199 °/s per rad on TROT against 49 on WALK — where
  (a)'s is consistent (155–175 on both). And its safe window closes on the other side too:
  pushed to −0.06 rad, the direction that was working, the trot falls at **2.77 s** with
  the stride up at 1.75 Hz. So (b) on TROT works only between about 0.02 and 0.04 rad, in
  one direction.

---

## 3. Why closing the loop on yaw *rate* made it worse

Implemented and measured before the open-loop probe, and it is instructive:

| clip | no yaw term | instantaneous yaw feedback | one-cycle-mean yaw feedback |
|---|---|---|---|
| TROT (cap 0.05) | 60+ cyc, 5.32 °/s | **2.71 s** | **2.35 s** |
| TROT (cap 0.10) | 60+ cyc, 5.14 °/s | 60+ cyc, **7.28** °/s | 60+ cyc, **6.43** °/s |
| WALK (cap 0.05) | 60+ cyc, 3.00 °/s | 60+ cyc, **4.42** °/s | 60+ cyc, **3.67** °/s |

Every variant is *worse than not doing it*. Two things are going on and they are worth
separating:

1. **The signal is mostly stride, not heading.** The instantaneous yaw rate on the
   working TROT is +5.32 °/s with a standard deviation of **20.70** — 3.9× the bias it
   should be correcting. Averaging over exactly one clip cycle keeps the mean and cuts
   the sd to 3.64 (a free window: the clip's own period). That fixes the *destabilising*
   (cycle-mean is always better than instantaneous above) but not the *direction*.
2. **Driving ω → ω_log is the wrong target for heading anyway.** TROT's log yaw rate is
   +1.12 °/s, so even a perfect rate regulator leaves 1.12 °/s = 1.7 °/m, still 3× the
   budget. Heading is an *angle* problem; a rate loop cannot hold a line, it can only
   hold a turn rate.

The open-loop probe in §2 says the actuator is fine. So what failed was the controller's
input, not its output — and the correct input is a heading **error against a reference**,
which the planner has and the low level does not.

---

## 4. Candidate (d): switching to the TURN clip

Now viable, because TURN survives 60 cycles with the derived target
(`outputs/turn_target.md`) — but it is coarse:

- one TURN cycle = −22.66 °/s × 0.892 s = **−20.2° of heading**
- the benchmark allows 0.565 °/m × 2.25 m = **1.27°** over a whole goal segment
- so one cycle is **16× the entire segment budget**

As a straightness corrector it is unusable at cycle granularity. As a *deliberate turn* it
is exactly right, and that is what the planner wants it for. The harness can already hold
a clip for a fraction of a cycle (`--via-clip`/`--via-s`), so a sub-cycle TURN insert is
testable — 0.06 s of TURN is about 1.4° — but that is blending two recordings mid-stride
and nothing here has measured what that costs yet.

---

## 5. Recommendation

**Primary: differential lateral foot placement (a), driven by a heading-ANGLE error,
capped per skill.** It is the actuator to build on, for three measured reasons: it is
linear and symmetric over the whole range tested on WALK (174.5 °/s per rad, bidirectional,
through zero), its gain is consistent across gaits where (b)'s varies 4× between them, and
nulling the drift costs about 0.015–0.024 rad — inside or just outside the 0.03 rad
deviation budget the trot already spends, and free on WALK.

Two conditions come out of the measurements, and neither is a preference:

1. **The input must be a heading error against a commanded heading, not a yaw rate.**
   Every yaw-rate loop tried here made heading worse (§3), and even a perfect rate
   regulator tracking the log's own +1.12 °/s would leave 1.7 °/m, 3× the budget. Heading
   is an angle problem. The conversion needs no new constant:
   `ω_target = ω_log − ψ_error / T_stance` spends one stance time returning to the
   reference, and T_stance is measured from the clip. The plumbing exists — `--foot-yaw`
   already turns a target yaw rate into per-leg lateral targets — so what changes is one
   term.
2. **The cap is per skill, and it is TROT that is tight.** ±0.04 rad is safe on WALK and
   falls the trot in both directions; ±0.02 is safe on both. A planner switching skills
   has to carry the skill's own limit with it, the same way `T_stance` and `ω_log` are
   already carried per clip.

**Not recommended as primary: step length (b).** Strong on TROT in one direction
(−4.71 °/s, the largest single correction measured) and destructive in the other at an
amplitude as small as 0.02 rad, with a gain that drops 4× on WALK. A controller that has
to correct both ways cannot be built on an actuator that destroys the gait on one side.
Worth keeping as a TROT-only booster if (a)'s ±0.02 ceiling proves insufficient, but its whole usable window on TROT is 0.02–0.04 rad in one direction: at −0.06 it falls too.

**Ruled out: hip yaw (c).** The joint does not exist on this robot (§1).

**Complementary, at the planner level: switch to TURN (d).** 20.2° per cycle, now that
TURN holds for 60 cycles and passes its gait gate (`outputs/turn_target.md`). Too coarse
for straightness — one cycle is 16× a whole goal segment's budget — and exactly right for
a deliberate turn. Sub-cycle inserts are testable with `--via-clip`/`--via-s`, but that
blends two recordings mid-stride and nothing here has measured what that costs.

### What is still not built, and what it would cost

The heading-error loop of (1) is **not implemented**. What is measured is the actuator, not
the controller — deliberately, because three closed loops were tried first and the
open-loop probe is what explained why they failed. Building it is one term plus a sweep of
the cap, at zero GPU cost.

The arithmetic says it should close: WALK's 2.68 °/s needs 0.015 rad against a 0.04 rad
ceiling, and TROT's 5.32 °/s needs 0.031 rad against a 0.02 ceiling — so WALK should reach
the benchmark's 0.565 °/m and **TROT should get to roughly 40% of its drift removed and
stop there**, at 4.8 °/m against a 0.565 budget. If that is what happens, the remaining
question is not the controller: §0 says the bias is constant and structural, which points
at the clip's own asymmetry or the stance, and that is a different investigation.
