# The planner now executes what it chooses — and every mid-run switch falls over

First run of the whole chain: rule planner → skill → clip replay → robot, on flat ground.
This is a **wiring test**. The thresholds it drives the planner with are the config's
placeholders and the terrain it sees is a script, so nothing here says the planner decides
*well*. It says whether the parts are connected, and what happens when they are.

`scripts/run_planner_replay.py`, no GPU (physics on CPU, nothing renders).

---

## 1. What was wired

| before | now |
|---|---|
| `StandStillPolicy` — accepts any skill, commands nothing | `ClipPolicy` — replays that skill's frozen clip with foot placement on |
| skill set {WALK, TROT, RUN, JUMP} | + **TURN**, and an executable subset `SUPPORTED = {WALK, TROT, TURN}` |
| RUN/JUMP silently selectable | selectable, **refused at the executor with a reason**, robot carries on |
| foot-placement law inline in the replay harness | `sim/footcomp.py`, one implementation, self-tested |

The information split is the point of the design and is enforced by the interfaces:
the planner gets `Observation` (terrain features) and the heading error; the low level
gets `BaseState` — base velocity and yaw rate — and nothing else. A low level that could
see terrain would make the planner ornamental.

TURN's constants are measured, not chosen: duty 0.591 and yaw rate **−0.3954 rad/s
(−22.66 °/s)** from the clip's own session, the same number its foot placement aims at.
Its selection threshold (`HEADING_ERR_TURN_DEG` = 12°) is a `CALIBRATION_NEEDED`
placeholder and is marked as one.

## 2. Does the planner choose, and does the choice reach the robot?

Yes to both. A scripted terrain drives roughness across RUN's (0.015 m) and TROT's
(0.030 m) limits and the heading error across TURN's, and all six ordered pairs are
elicited: the planner produced `RUN TROT WALK TROT TURN TROT WALK TURN WALK`.

Executed switches carry a real cost, measured at the seam:

| entry criterion | WALK→TROT commanded jump | TROT survived after |
|---|---|---|
| `level` (clip's most-coplanar frame) | 0.482 rad | 2.46 s |
| `nearest` (closest pose to the one held) | 0.237 rad | 1.48 s |
| `contact` (stance/swing agrees with the loaded feet) | 0.237 rad | 1.48 s |

**Halving the seam made it worse, not better.** That rules out the size of the commanded
step as the cause, which is why three criteria were tried before concluding anything.

## 3. The result: single skills execute correctly, switches do not survive

Controls first, because the finding depends on them:

| run | segment | cycles | stride | vx | verdict |
|---|---|---|---|---|---|
| `hold`, initial TROT, **no switch** | TROT 0–40 s | **62.2** | 1.56 / 1.56 (+0%) | 0.664 / 0.444 | **ok** |
| `pairs`, initial WALK | WALK 0–6.2 s | 8.5 | 1.35 / 1.37 (+1%) | 0.232 / 0.187 | **ok** |
| `pairs`, initial TROT | TROT 0–12.2 s | 19.0 | 1.56 / 1.56 (+0%) | 0.647 / 0.444 | **ok** |

So the runner, the clip policy, the shared foot-placement module and the per-skill
settings all reproduce the single-clip results (stage 2 measured vx 0.659 for TROT; this
runner gets 0.647–0.664, the difference being the lever measured at a different settle
pose and a 50.0 vs 49.77 Hz control rate).

Then every switch:

| switch | at | after | outcome |
|---|---|---|---|
| WALK → TROT | 6.20 s | 1.48 s | fell, roll. stride reads 3.12 vs 1.56 — double, i.e. not trotting |
| WALK → TROT | 0.20 s | 1.36 s | fell, roll |
| TROT → WALK | 12.20 s | 1.82 s | fell, roll. vx went **negative** (−0.037) |

**Both directions fail, at any entry phase, early or late.** The gait that runs for 62
cycles when started cold does not survive being switched into after 6 seconds of walking,
and the gait that survives everything open-loop does not survive being switched into after
12 seconds of trotting.

This does not contradict the earlier handover result (`04a16cb`: "a diverging skill handed
back to WALK inside one cycle recovers completely"). That switch happened one cycle in,
while the robot was still near its settle state. These happen after an established gait
has built up 0.23–0.65 m/s and a body attitude the incoming clip's first frame knows
nothing about.

## 4. Chattering: not observed, and the hysteresis is why

A schedule that parks roughness 0.1 mm over TROT's limit for 24 s produced **3 switches**,
not a stream: the downgrade fires immediately, and the upgrade back is refused by the
hysteresis band (0.25) and the minimum hold (0.50 s). In the `pairs` schedule the planner
made 2–3 decisions and executed 1 before terminating.

The count that matters is not yet measurable, though — a run that falls after the first
switch cannot chatter. Chattering has to be re-checked once transitions survive.

## 5. RUN and JUMP: refused, not removed

60 planner ticks requested RUN on flat ground (it is the fastest fit, and the planner does
not know what the executor can do). Each was refused once with its reason and the robot
kept walking:

    0.20s REFUSED RUN: no flight phase in replay: the base never leaves the ground,
          so there is no footfall for placement to choose -- holding WALK

They stay in `SkillId`, in the library, and in the planner's search. Only
`skills.SUPPORTED` excludes them, so putting them back is one tuple.

## 6. What has to happen next

The blocker is the transition, and it is a mechanism that does not exist yet rather than a
parameter that needs tuning. What the evidence points at:

1. **The real robot's own procedure was to route skill changes through `balance_stand`** —
   8 of 8 recordings before a jump (CLAUDE.md §3). The archive has no such clip, so it
   cannot be tried today. Extracting one is the cheapest experiment available.
2. **Speed matching.** WALK runs at 0.23 m/s and TROT at 0.65. The incoming clip's first
   frame is the pose of a robot at *its* speed, and the body arrives at the other one's.
   Nothing in the executor bridges that.
3. **Blending** the two commanded trajectories over some window, which the harness
   deliberately never did — the seam was left sharp to measure it. Now that it has been
   measured (0.24–0.48 rad), blending is a defensible next step, with the window as the
   swept parameter.

Until one of those works, the planner can choose and the executor can play, but the
sequence the whole experiment depends on cannot be run end to end.
