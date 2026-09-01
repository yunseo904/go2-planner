# TROT's heading: the authority was never in the feet

`outputs/trot_straight.md` closed with heading authority "spent" and named cross-track
control — a position error against a path, which only the planner has — as the one thing
left. That conclusion was right about the *routes it had tried* and wrong about the
premise underneath all three of them.

All three were **foot placement**:

- **(a)** differential lateral placement, capped at ±0.02 rad because TROT falls in both
  directions at ±0.04 (`heading_candidates.md` §2). At 155–175 °/s per rad that ceiling
  is worth about 3.1 °/s against a +5.32 °/s drift.
- **(b)** differential step length, strong one way and destructive the other.
- **(c)** either of them fed forward as a constant — a coin flip over fifteen paired
  cells (`trot_straight.md` §4b).

They share a bound, and it is not the actuator's: it is **how far a trot's footfall can
be displaced before the trot stops being one**. Widening it was already measured to make
heading worse (`lip_failure.md` §3). Three routes into the same wall is one route.

This is a fourth thing, and it is on the other side of that wall. It **moves no foot**.

---

## 1. The mechanism, in the robot's own numbers

The hip is the abduction joint — `heading_candidates.md` §1 settled that this robot has
no yaw joint at all — and `lever_i`, the hip-to-foot vertical drop, is exactly the moment
arm that turns a hip torque into a lateral force at the foot. On a **stance** leg the
foot is on the ground, so the load path is through the contact and the torque buys a
ground reaction force:

    f_y,i = -tau_i / lever_i          on the body
    M_z   = sum_i x_i * f_y,i         x_i = the hip's fore-aft offset, body frame

Take `tau_i = c * sign(x_i)` and every stance leg contributes to `M_z` with the **same
sign**:

    M_z = -c * sum_over_stance |x_i| / lever_i

That last property is not cosmetic. A trot has only a **diagonal pair** on the ground at
any moment and the pair alternates twice a cycle; a rule needing all four feet would
produce a moment that switched off twice a cycle. This one does not care which legs are
down — measured at **1.241 N·m of yaw couple per N·m of hip torque on a diagonal pair**,
2.483 with all four, identical for both diagonals.

Two side effects, both bounded in the self-test rather than asserted away:

- **net roll moment is exactly zero** (`sum_i lever_i * f_y,i = -c * sum sign(x_i)`), for
  any stance set with as many front legs as rear.
- **net lateral force is not zero** and the first draft of `sim/yawmoment.py` claimed it
  was. It cancels only if the front and rear levers are equal, and they are 0.306 and
  0.316 m. What is left is that **3.2% mismatch and nothing else** — 0.207 N against
  6.433 N of force in play on a diagonal pair, a couple-to-residual ratio of 12 N·m per N.
  The self-test now bounds it against the lever asymmetry, which is the honest test.

### Nothing new is asked of the simulator

The channel was probed a week ago and written down. `outputs/isaac_actuator_probe.json`:
the actuator is an explicit `IdealPDActuator`, `set_joint_effort_target` exists,
`q3_effort_is_additive` is **true** (5 N·m commanded produced exactly 5.0 N·m of extra
torque), and `q3_effort_replaces_pd` is **false**. The feed-forward adds to the PD that
plays the clip, and the actuator clips their **sum** at the hip's 23.70 N·m.

### What it does not see

The stance mask is the **clip's own contact channel**, phase-locked — the same gate
`ClipPolicy` already uses for foot placement, and the same information class. The sim's
contact sensor is deliberately not an input: a term that changed with what the feet
actually hit would be a contact reflex, which the skill layer is forbidden. Heading
error, hip geometry, clip phase. No terrain, no depth, no goal.

---

## 2. The actuator's gain, measured open loop before any loop was closed

The methodology is `heading_candidates.md` §2's, for the reason given there: a controller
that fails tells you almost nothing about which part failed, so measure the actuator
first. Rig: `run_planner_replay.py --schedule hold --initial TROT`, 40 s, flat, CPU —
the rig `heading_hold.md`'s 3.20 °/m came from. **Heading hold off**, so nothing fights
the constant. Baseline +5.17 °/s.

| c (N·m) | −2.0 | −1.0 | −0.5 | −0.25 | 0 | +0.25 | +0.4 | +0.5 | +0.6 | +0.75 | +1.0 | +2.0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| yaw °/s | +24.19 | +7.50 | +8.82 | +6.86 | **+5.17** | +2.98 | +1.34 | **−0.12** | −0.87 | −2.31 | −4.13 | −15.54 |
| v_x | 0.323 | 0.442 | 0.662 | 0.663 | 0.664 | 0.663 | 0.662 | 0.661 | 0.660 | 0.658 | 0.653 | 0.639 |
| stride Hz | 1.47 | 1.52 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 |
| survived | **2.28 s** | **3.58 s** | 40 s | 40 s | 40 s | 40 s | 40 s | 40 s | 40 s | 40 s | 40 s | 40 s |

- **The sign came out as derived**, which is worth saying because it is the first time in
  this project it has (`heading_hold.md` had to flip its heading term after a run, and
  `sim/footcomp.py` carries the note). Positive c → negative yaw moment.
- **It crosses zero, at c ≈ 0.49 N·m**, interpolating 0.4 → +1.34 and 0.5 → −0.12.
- **Roughly 9–10 °/s per N·m**, safe to at least ±2 N·m in the working direction: **20 °/s
  of demonstrated authority against foot placement's 3.1**, six times as much, and the
  cap could go to 11.85 N·m before the actuator guard refuses it.
- **It costs nothing measurable.** Across the entire working range v_x moves 0.664 → 0.661
  and stride does not move at all. Compare (a), whose ±0.04 rad costs a quarter of the
  forward speed *and* falls.

### The one asymmetry, stated rather than buried

The two large **negative** amplitudes fall, and −1.0 (+7.50 °/s, fell at 3.58 s) falls
while +2.0 (−15.54 °/s, survived 40 s) does not — so it is **not** simply "too much yaw".
Something about driving the drift further in its own direction ends the run, and this is
not yet explained. It is also not on the path: TROT's drift is one-signed at +5.17 °/s and
the correcting direction is the surviving one. But a bidirectional controller does have
to enter the other half occasionally, which is what §3's cap is for, and −0.25 and −0.5
both run the full 40 s — so the safe window is at least [−0.5, +2.0] and the closed loop
below never leaves it.

---

## 3. Closed loop: `c = gain × heading error`

On top of heading hold, same rig, cap 2.0 N·m.

| gain N·m/rad | off | 5 | 10 | 20 | 40 |
|---|---|---|---|---|---|
| **curvature °/m** | **3.20** | **0.01** | **0.02** | **0.03** | **0.13** |
| yaw °/s | +2.12 | +0.01 | +0.01 | +0.02 | −0.08 |
| v_x | 0.663 | 0.664 | 0.664 | 0.653 | 0.645 |
| stride Hz | 1.56 | 1.56 | 1.56 | 1.56 | 1.56 |
| commanded \|c\| max | — | 1.32 | 2.00 | 2.00 | 2.00 |
| at its cap | — | 0.0% | 0.5% | 1.1% | 2.9% |
| peak hip torque | — | 11.95 | 11.97 | 12.01 | 12.07 N·m |
| actuator clipped | — | 0/2000 | 0/2000 | 0/2000 | 0/2000 steps |

**The budget is 0.565 °/m and every arm is inside it by a factor of four to fifty.**

`gain = 5` is the operating point: curvature 0.01 °/m, forward speed and stride
**unchanged** (0.663 → 0.664, 1.56 → 1.56), and the term never reaches its own cap at
all, so the cap is not what is producing the result.

### Why a P law works here when `trot_straight.md` §3 says it cannot

§3's argument is exactly right and this does not contradict it. A proportional law must
hold a standing error to emit an output, and this disturbance is DC — so the question is
only whether the standing error it needs is small enough to *be* small. For the placement
term it is not: it needs 3.7° to reach a cap worth 3.1 °/s against a 5.3 °/s drift, so it
saturates permanently and the heading error grows without bound. For this term, nulling
the drift costs **0.49 N·m against a 2.0 N·m cap**, so at gain 5 the equilibrium sits at
0.49/5 = 0.098 rad and the loop has somewhere to sit. **The law did not change. The
authority did.**

Peak hip torque is 11.95–12.07 N·m against 23.70 and the actuator clipped on **zero** of
2000 control steps in every arm, so none of this is being paid for out of the position
tracking that produces the gait.

---

## 4. Regressions and the control group

- **`--yaw-moment off` reproduces the baseline exactly**: 3.20 °/m, yaw +2.12 °/s,
  v_x 0.663, stride 1.56 — every column of `heading_hold.md`'s TROT row.
- **`sim/yawmoment.py` defaults to a cap of 0**, and the self-test asserts a large heading
  error produces zero torque through the default path.
- The term is **TROT-only** by table (`YAW_MOMENT_CAP_NM`), not by argument: WALK already
  reaches 0.24 °/m without it and TURN is trying to *change* a heading, not hold one.

---

## 5. Flags

`--yaw-moment off|probe|hold`, `--yaw-moment-nm`, `--yaw-moment-gain`,
`--yaw-moment-cap-nm`, `--yaw-moment-skill`, on `run_planner_replay.py` and
`run_calibration_grid.py`. Default off, banner when on, every setting stamped into the
results row, and `tau_hip_max_nm` / `hip_saturated` recorded per row so a run that paid
for its couple out of the gait cannot be reported as one that did not.

The cap is refused above **half the hip's effort limit**, and that limit is read off the
**actuator**, not off `robot.data`. The first version read `robot.data.joint_effort_limits`
and got 1e9 — PhysX's limit, not the one an explicit `IdealPDActuator` enforces in Python
— which left the headroom guard permanently open and the saturation counter unable to
fire. It was caught by the banner printing "against a 1000000000.00 N·m limit" next to a
peak of 11.79.
