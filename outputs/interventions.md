# Every intervention, in one place

Two kinds of change stand between the recording and a number in this project, and they are
not the same kind of claim:

- **Plant corrections** make the *simulator* match the machine the clips were recorded on.
  A plant correction that is wrong makes every measurement wrong; one that is missing does
  the same. None of them touches the controller.
- **Added control** is signal the recording does not contain. Every one of these is an
  admission that open-loop replay of a clip is not a controller. They are the thing a
  reviewer should be most suspicious of, so each is off by default, banner-printed at
  runtime, stamped into every CSV row it influences, and exactly null at its zero setting.

Nothing here edits `data/skill_clips.npz`. Every edit is applied at replay time.

Numbers are from the files named in each row; nothing is quoted from memory.

---

## A · Plant corrections — the simulator, matched to the robot

| # | What | Value used | Why it was needed | Without it | Cost / risk |
|---|---|---|---|---|---|
| A1 | **Physics step decoupled from clip rate** | `phys_dt = ctrl_dt / decimation` → 202 Hz physics, 50 Hz control | The harness integrated PhysX at the clip's ~50 Hz. `legged_robot_config.py` — the config the whole comparison is measured against — is 200 Hz physics × decimation 4. | Belly-down on flat ground in every clip: base height 0.169 m vs nominal 0.32, **falls at 1.40 s**. And it *lied about its cause* — reported "stride +517 %", "walks backward", "wrong DOF mapping". | None. Both numbers are read from the upstream config, not chosen. `harness_findings.md` §5 |
| A2 | **Ground friction fixed at μ = 1.3** | static = dynamic = 1.3 | The env randomises friction per episode over [0.6, 2.0]. A step-clearance threshold measured on a randomised floor folds friction variance into a geometric answer. | Isaac's own default is 0.5 — below the whole training range, so a different robot. | The midpoint is the least arbitrary fixed point *inside* the distribution the policy trains on, and it comes from the env, not from anything we measured (CLAUDE.md §2 forbids picking it to suit a score). |
| A3 | **Solver / actuator from upstream, not Isaac stock** | pos-iter 4, vel-iter 0, `IdealPDActuator` kp 40 / kd 1, effort 23.7 (hip, thigh) / 45.43 Nm (calf) | Isaac's stock Go2 is not this robot: stock is 255/255 iterations and different gains. | Quoting stock numbers would be quoting the wrong machine. | None. `sim/isaac_cfg.py` parses the upstream file with `ast` — no execution, and it notices if upstream changes a gain. `sim_settings_audit.json` lists all 8 fields that differ from Isaac stock. |
| A4 | **Contact from measured force, not the clip's channel** | `--contact-threshold-n 30`, Schmitt + despeckle | The clip's own contact flags and the simulator's ground force **disagree on 51 % of leg-frames**. Only one of them is what the foot is doing. | Every duty factor, swing gate and clearance measured against the clip channel is measured against a boolean that is wrong half the time. | The threshold is a choice. It is stamped in every row (`contact_threshold_n`, `contact_rule`) and the bare-threshold duty is reported next to it so the sensitivity is visible. |
| A5 | **Probe terrain as an imported mesh, gutters void** | `TerrainImporter.import_mesh`, no ground plane | `terrain_type="plane"` silently lays a 2000 km plane at z = 0 **under** the probe mesh. Every pit was floored; every gutter between cells was solid. | The gap family read **60/60 reached, 0 % fell** — over its own trench, on an invisible floor. That is where `FOOT_SPAN_X = 0.550 m` came from. With the plane removed: **0/60, first failure at 0.05 m.** | Walking off a cell is now a fall, which is the correct verdict. `FOOT_SPAN_X` has no value and 0.550 must not be used. `step_and_gap.md` §1 |
| A6 | **Base metrics recomputed per step** | — | Every base-derived metric was the final step's value, repeated. | Invalidated a full day of results. | None. `harness_findings.md` §6 |
| A7 | **Initial condition measured, not assumed** | `--start-phase level --settle-mode stand` | The spawn pose decided the verdict and had never been measured. | A verdict that is a property of the drop, not the gait. | Stamped per row. `harness_findings.md` §7 |

**A1–A7 are not optional and are not flags.** They are the difference between a simulator
and a wrong simulator. Nothing in section B is meaningful without them.

---

## B · Added control — signal the recording does not contain

All off by default. Ordered by how much of the final result rests on them.

| # | What | Joint | Authority | Why | Without it | Deviation from the untouched clip |
|---|---|---|---|---|---|---|
| B1 | **Foot placement (Raibert)** `--foot-comp raibert` | hip only | cap **0.05 rad** | The clip is a fixed trajectory; lateral velocity has nothing to correct it. | **TROT falls at 1.95 s** (1 cycle), **TURN at 8.82 s** (7), **RUN at 1.13 s** (1). WALK alone survives without it. | hip swing RMS **0.034–0.049 rad**, thigh 0.046–0.067, calf 0.042–0.070; overwrites **78–91 %** of time, **34–42 %** of leg-steps; cap hit 2–43 % of swing samples |
| B2 | **Heading hold** `--heading heading-only` | hip only (added to B1, capped separately) | WALK **±0.04**, TROT **±0.02** rad | Both gaits curve. The benchmark budget is 0.565 °/m. | WALK **13.26 °/m** — 23× the budget. TROT 7.79. | WALK **13.26 → 0.24 °/m** (inside the budget, the first time anything here has met it). TROT 7.79 → **3.20**, still 5.7× — the ±0.02 ceiling binds and TROT falls at ±0.04 in both directions. Stride and vx untouched: WALK +1 %, 0.233 → 0.235 m/s. |
| B3 | **TURN yaw feed-forward** `--foot-yaw log-cycle` | hip | within B1's cap | TURN's foot-placement target has to come out of the rotation, not out of a straight-line assumption. | TURN reaches 3 cycles and rolls. | With it, 68 cycles. `turn_target.md` |
| B4 | **Swing lift** `--swing-lift` | thigh + calf (never the hip) | 0 mm default = byte-exact null | Supervisor's decision: raise the clip's swing arc rather than wait for a re-recording. Literature says swing height is terrain-adaptive; the clips are flat-ground recordings. | — | at 80 mm: thigh RMS 0.127, calf **0.240 rad** (peak 0.47) — **5× the whole B1 budget**. Stance exactly 0 by construction, checked. |
| B5 | **Stance-height compensation** `--plant-comp height` | thigh + calf | α default 0 | Proposed to fix a 40 mm body-height deficit. | — | **Rejected.** The deficit was measured against *commanded* geometry, which is not a pose the robot holds. Against the real robot the sim already stands **at or above** it, and the correction pushes WALK **38 mm above** the machine. `stance_height.md` |
| B6 | **Attitude PD** `--balance-*` | hip | default 0 | Proposed to fix roll. | — | **Rejected.** `balance_comp.csv` |

### What B4 actually bought

**Nothing, on the quantity it was built for.** The step-height curve is flat: WALK's best
step score in the whole sweep belongs to the **untouched recording** (4/5 at 0.02 m, 0 mm
lift). Below ~40 mm the commanded lift does not break contact and the edit does not reach
the ground; above it, the edit destabilises TROT (breaks between 60 and 80 mm) and TURN
(breaks below 20 mm). `swing_lift.md`, `swing_lift_symmetry.md`.

It is kept because the *measurement* it enabled is the result: **foot height is not what
caps these gaits on a step.**

---

## C · What is still not calibrated

| Parameter | State | Why |
|---|---|---|
| `robot.FOOT_SPAN_X` | **CALIBRATION_NEEDED** | 0.550 m was measured over an invisible floor (A5). WALK cannot cross 0.05 m. |
| `skill.STEP_TROT_MAX` | **not measurable yet** | TROT holds 3.20 °/m against a 0.565 budget, so it leaves the 4 m lane before the answer is about the step. Its 0/N at every level is a lane-departure score. |
| `skill.STEP_WALK_MAX` | measurable; 0.02 passes, 0.04 fails | The mechanism is known: the rear feet swing 4–5 mm and the foot's own radius (23 mm) is what lets a 20 mm riser through. `lip_failure.md` |
| `skill.STEP_RUN_MAX`, `skill.STEP_JUMP_MAX` | open | RUN has no flight phase in the clip; JUMP is torque-short. `run_collapse.md`, `jump_torque.md` |
| slope / roughness families | generated, never run | — |
