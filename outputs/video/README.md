# Side-view recordings — 2026-09-01

Nine mp4s, 960×540, side view, one condition each. Recorded on GPU1 with the WMP training
paused, stopped and resumed by `~/WMP_중단_재개.md`. GPU0 never touched.

Failure cases are written at **25 fps against a 50 Hz control rate = half speed**. Passing
cases are real time. `--video-fps` forces the writer's rate; it does not touch the
simulation.

| file | what it shows | speed | outcome |
|---|---|---|---|
| `flat_WALK_60cycles_pass.mp4` | WALK, flat, 60 cycles | real time | ran to the end, 2220 steps |
| `flat_TROT_60cycles_pass.mp4` | TROT, flat, 60 cycles | real time | ran to the end, 1920 steps |
| `flat_TURN_entry06_measured_pass.mp4` | TURN from the **measured** entry frame 6 | real time | ran to the end, 2700 steps, yaw −21.33 °/s |
| `flat_TURN_entry00_fail_halfspeed.mp4` | TURN from frame 0, the recorded start | **½** | rolls out at 4.82 s |
| `flat_RUN_collapse_1.13s_halfspeed.mp4` | RUN, flat | **½** | rolls out at 1.13 s |
| `flat_JUMP_no_takeoff_halfspeed.mp4` | JUMP, one shot | **½** | no take-off; ends on height at 0.44 s |
| `terrain_WALK_step0.02_pass.mp4` | WALK over a 0.02 m step-up | real time | **reached the goal** |
| `terrain_WALK_step0.04_fail_halfspeed.mp4` | WALK over a 0.04 m step-up | **½** | fell, stopped 1.28 m short |
| `terrain_TROT_lane_departure_halfspeed.mp4` | TROT on the 0.02 m step-up cell | **½** | fell, 1.59 m from the goal — the lane departure |

## The regression check (CLAUDE.md §8: rendering must not change physics)

Every recorded run was preceded by the identical run with no camera, on CPU with `GPU=none`.

**Flat harness — recorded vs un-recorded:**

| recording | terminated (recorded) | terminated (baseline) | steps | match |
|---|---|---|---|---|
| WALK 60 cycles | — | — | 2220 / 2220 | ✔ |
| TROT 60 cycles | — | — | 1920 / 1920 | ✔ |
| TURN entry 6 | — | — | 2700 / 2700 | ✔ |
| TURN entry 0 | 4.818515234399546 s | 4.818515234399546 s | 244 / 244 | ✔ |
| RUN | 1.134285974500017 s | 1.134285974500017 s | 57 / 57 | ✔ |
| JUMP | 0.44 s | 0.44 s | 23 / 23 | ✔ |

**Terrain harness:** all **45 of 45** recorded cells reproduce their un-recorded row exactly
— `fell`, `reached` and `final_dist_m` to the last digit (e.g. 17.634678 m, 1.280613 m,
1.591748 m).

So the render path adds a camera and `sim.render()` and changes nothing about the physics or
the control rate. That is the property CLAUDE.md §8 asks a render run to demonstrate, and it
now has a table rather than an assurance.

**Do not compare `steps` across cells within one grid recording** — it is the repeat's shared
loop length, not the robot's (`harness_findings.md` §14). The per-cell columns above are the
ones that carry a verdict.
