# The in-air replay: what the clip and the PD can do before contact takes anything away

`--in-air` on `verify_skill_replay.py` writes the base back to a fixed pose 1.2 m up **every
control step**, so the feet touch nothing and the legs carry only their own weight. Root
velocity is zeroed with the pose, or the write would fight a falling body and inject an
acceleration into the legs each step.

It is the kinematic control SESSION_STATE §13 and `run_extension.md` both asked for, and it
answers two open questions with one rig.

A run with it on is **not a gait** — stride, duty, speed and yaw are meaningless and the
harness's gait verdict correctly reads FAIL. Only commanded-vs-achieved joint angles mean
anything.

Extension is `run_extension.md`'s own metric, `cos(thigh) + cos(thigh+calf)`, larger =
straighter.

---

## 1. TURN: the harness loses nothing before contact

| run | steps | cmd ext | ach ext | delta | hip err | thigh err | calf err |
|---|---|---|---|---|---|---|---|
| **TURN in air** | 900 | 1.5274 | 1.5222 | **−0.0051** | 0.008 | **0.009 rad (0.5°)** | 0.026 |
| WALK in air | 740 | 1.4170 | 1.4153 | −0.0017 | 0.008 | 0.014 rad (0.8°) | 0.021 |
| WALK on ground | 740 | 1.4170 | 1.3434 | −0.0736 | 0.064 | 0.056 | 0.146 |

**TURN's joint trajectory is delivered essentially exactly** — half a degree of mean thigh
error over 900 control steps. So the clip's kinematics and the PD are not where TURN's
**57 % → 35 %** of logged yaw rate goes. Whatever is lost is lost **at contact**, and the
remaining candidates SESSION_STATE §13 lists (the per-cell lever, the 0.42 m spawn drop onto
rough ground, the 200-robot scene) are all contact-side. WALK in air is the control and
shows the same thing from the other end: its on-ground error (0.056 thigh, 0.146 calf) is
four to seven times its in-air error, i.e. WALK's tracking error **is** load.

## 2. RUN: the leg extends with no load at all, and it is mostly the gains

`run_extension.md` could not separate "the gains cannot hold this posture" from "load pushes
it out", because the run it measured was 43 control steps of a falling robot. In air RUN
runs the full 320 steps.

| arm | gains | ff torque | calf err | calf **at its stop** | delta ext |
|---|---|---|---|---|---|
| `--mode torque` — the clip's own 13/3/2 + ff | clip | yes | **0.787 rad (45°)** | **88.8 %** | **+0.385** |
| `--mode ff-only` — stock gains + ff | stock | yes | 0.215 | 7.8 % | +0.100 |
| `--mode position` — stock gains, no ff | stock | no | **0.069 rad (4°)** | 0.0 % | +0.005 |

**With zero load, replayed the way the recording asks, RUN's calf flies to full extension and
sits pinned against its mechanical stop for 88.8 % of the run** — commanded range
[−2.131, −1.222] rad, limit −0.838, achieved max −0.838.

So the request's original hypothesis — *the leg ends up straight and no stroke is left* — is
**confirmed**, and `run_extension.md`'s reversal of it was the fall: its on-ground trace read
−0.0395 (more crouched) because most of those 43 steps were a robot toppling.

**But the mechanism is not load.** There is no load here. Splitting the two differences:

- **the gains do most of it** — clip gains → stock gains takes the calf error 0.787 → 0.215
  and un-pins it from the stop, 88.8 % → 7.8 %;
- **the feed-forward torque does the rest** — 0.215 → 0.069 when it is dropped.

Both push the same way. `sim/replay.py` already says why: RUN was recorded at kp 13/3/2 with
~12 N·m RMS of calf feed-forward, a torque that is only in equilibrium against a ground
reaction. Hanging in free air with kp_calf = 2, nothing resists it.

## 3. Delivering the posture does not rescue RUN

The obvious follow-up, since `--mode position` tracks in air to 4°: run it on the ground.

| | terminated |
|---|---|
| RUN on ground, `--mode torque` (the published arm) | **0.85 s**, roll |
| RUN on ground, `--mode position` | **0.91 s**, roll |

**0.06 s.** And on the ground position mode's own tracking degrades from 0.069 to 0.222 rad
of calf error, ending 0.136 *more crouched* than commanded — the load reasserts itself as
soon as there is any.

So there **is** a thigh/calf path that fixes the posture in isolation, and it does not make
RUN work. That is the question SESSION_STATE §15/item 3 asked, answered: **RUN stays
unresolved, and the posture is now excluded as the cause rather than suspected as it.**

`--mode position` is **not adopted for RUN** and should not be: it is not a replay of the
recording (the recording is a torque-mode gait), it buys 0.06 s, and adopting it would mean
reporting a different controller under the same clip's name.

## 4. What this does not say

The in-air rig removes contact, and with it every quantity that needs contact. It cannot
say what a leg would do under *partial* load, it cannot rank gaits, and its FAIL verdicts
are an artefact of asking a gait gate about a hanging robot. It answers exactly one kind of
question — *is this error there before the ground is involved* — and both answers above are
of that kind.
