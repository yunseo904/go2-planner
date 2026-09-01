# Level 2 — a design, not an implementation

Asked for after the ablation: the roughness costs 1.09 → 0.76 and the working theory is
that it costs it by knocking the robot over, so a balance controller is what to aim at.
**Below is the design and the evidence for and against that theory.  Nothing is built.**

## 1. What the roughness actually does — the theory is two-thirds right

WALK, all 200 cells, roughness on vs off:

| | roughness off | on |
|---|---|---|
| alive at 20 s | 64/200 | **29/200** |
| travelled, median | 1.13 m | 0.79 m |
| goals | 1.09 | 0.76 |

Split by whether the cell survives *without* roughness:

| | cells | goals off → on | travelled off → on |
|---|---|---|---|
| survives without it | 64 | 1.05 → **0.67** | 1.83 → **0.72** m |
| falls either way | 136 | 1.12 → **0.79** | 1.06 → **0.83** m |

**Falling is the larger half but not the whole of it.**  44 of the 64 survivors stop
surviving — that is the balance failure, and it is what a balance controller would
address.  But in the 136 cells that fall either way, the roughness still costs 0.23 m of
travel and 0.33 of a goal, and there is no survival left to lose there.  So a balance
controller that made the robot unfallable would recover roughly the first row and leave
the second, i.e. **about two thirds of the 0.33, not all of it.**

Where the survival goes, by course: `agility_poles` 9 of 10 cells, `staircase_spiral` 8,
`staircase_climbing` 6, `jump_on_and_off_box` 5, `sphere_bump_lips` 5, `squeeze` 5.  Those
are the courses whose obstacles the robot was already negotiating; the noise is the extra
that takes it over.

## 2. The scale the controller has to work at

The noise is not a slope, it is a texture: amplitude drawn from 0.02–0.04 m, drawn on a
0.075 m grid and interpolated.  WALK travels 0.79 m before it dies, which is **eleven
correlation lengths**.  So the disturbance arrives at roughly 0.075 m / 0.20 m·s⁻¹ ≈
**2.7 Hz**, continuously, in both roll and pitch, and never repeats.

That rules out two things immediately:

* **It cannot be planned around.**  At 10 Hz the planner sees a new frame every 0.02 m of
  travel; the texture changes faster than the skill-switch machinery can respond, and there
  is no skill in the library that is "walk over 3 cm noise".
* **It cannot be fixed by raising the step limit.**  0.02–0.04 m is *inside* WALK's own
  measured 0.04–0.06 m step bracket, so no threshold change reclassifies it.  The terrain
  is at the limit everywhere at once, which is exactly why the loss is spread evenly over
  all 20 courses rather than concentrated.

## 3. The design

**A stance-phase attitude regulator on the hips and thighs, feed-forward in torque, sharing
the channel `sim/yawmoment.py` already proved.**

The precedent matters and is why this is the proposal rather than a foot-placement one.
`trot_yaw_moment.md` established, on this robot and in this harness, that:

* adding a feed-forward torque to the legs the *recording* has on the ground buys a real
  ground reaction, because the load path is through the contact;
* the authority available that way is about **6×** what moving a swing foot buys
  (20 °/s per N·m against 3.1), and it does not fight the gait's footfall pattern;
* `IdealPDActuator`'s effort target is **additive** to the PD, 5 N·m in gives 5.0 N·m out,
  measured (`isaac_actuator_probe.json`), so nothing new is asked of the simulator;
* at gain 5 the peak hip torque was 11.95 against the 23.70 clip, with **0 of 2000 control
  steps clipped** — there is headroom for a second term on the same channel.

So, per control step, for each leg the clip has in stance:

    tau_i  =  -( Kp_roll * roll  + Kd_roll * roll_rate ) * sign(y_i)
              -( Kp_pitch * pitch + Kd_pitch * pitch_rate ) * sign(x_i)

`roll`/`pitch` from `projected_gravity_b` (an IMU quantity — no privileged state), rates
from `root_ang_vel_b`.  `sign(y_i)` makes the left/right stance legs push in opposition,
which is a roll couple; `sign(x_i)` does the same fore/aft for pitch.  Net vertical force
is zero to the front/rear lever mismatch already measured at 3.2%.

Four things this must carry, each because something has already gone wrong without it:

1. **Stance only, from the clip's own contact channel.**  Torque into a swing leg buys
   nothing and disturbs the trajectory.  This is the rule `YawMoment` uses.
2. **A cap, and a saturation counter that is read off the actuator, not PhysX.**
   `SESSION_STATE.md` §6: `joint_effort_limits` returns 1e9 and a headroom check against it
   can never fire.
3. **Off by default, per skill, in the config table.**  The yaw couple is TROT-only by
   table and this should be WALK-only until measured otherwise.
4. **Measured against the recording, not against a score.**  CLAUDE.md §2 forbids tuning a
   threshold on benchmark performance; the gains come from a flat-with-noise calibration
   rig, not from `run_benchmark.py`.

## 4. How it would be measured, and what would falsify it

The prediction is specific and cheap to test, and it is **not** the benchmark score:

* **Rig**: flat ground plus legged_eval's own roughness at the protocol amplitude, no
  obstacles, `run_calibration_grid.py`'s shape.  WALK, 60 cycles.
* **Primary**: time-to-fall, and roll/pitch excursion (a bound, not a mean — the same
  reading `analyze_yaw_moment.py` uses, because a regulator drives the mean to zero by
  construction).
* **Sweep**: amplitude 0.00 / 0.02 / 0.03 / 0.04 m against gain 0 / 2 / 5 / 10, the same
  ladder shape as the yaw couple's.
* **Control arm**: gain 0 on the identical rig, in the same run.

**It is falsified if** the excursion does not fall monotonically with gain, or if the
surviving arm's stride or forward speed moves — the yaw couple's claim to be free was that
v_x went 0.664 → 0.661 and stride did not move at all, and the same has to hold here or
this is buying balance with locomotion.

**The honest ceiling.**  §1 says a perfect version of this recovers about two thirds of the
0.33, so roughly 1.09 − 0.11 ≈ **0.98 against WALK's current 0.76**, and the oracle over
the whole skill library is 0.81.  That is worth having and it is not a route to the
teacher's 4.83.  The thing between 0.98 and 4.83 is still the library.

## 5. Why not the two alternatives

* **Raise the swing foot.**  Already tested and already null: `swing_lift_ladder.md` re-ran
  the ladder after the symmetry fix and the untouched recording is the best row in both
  gaits, with TROT dying in 1.1–2.0 s at 60 and 70 mm.  More clearance costs more than the
  obstacle does.
* **Terrain-following foot placement.**  This is the one that would also fix the 136-cell
  travel loss, and it is the better idea — but it needs a height estimate under each foot,
  which is 0–0.25 m in front of a camera whose lowest ray reaches the ground at **0.48 m**
  (`legged_eval/adapters/depth_terrain.py`).  Under the feet is exactly where the robot
  cannot see.  It would have to come from proprioception (contact timing and joint
  deflection), which is a bigger piece of work than §3 and should follow it, not precede it.
