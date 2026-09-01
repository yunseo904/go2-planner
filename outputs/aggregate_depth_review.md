# Benchmark aggregate

Sources: outputs/bench_le/depth_review_s1.csv (200 rows)

## all rows

> **Conditions.** `terrain`=legged_eval-seed1, `terrain_seed`=1, `measurement_seed`=deterministic, `skill`=PLANNER, `perception`=depth, `steering`=dead-reckoned-heading-hold, `heading`=heading-only, `heading_cap_rad`=0.0, `episode_length_s`=20.0, `episodes`=1, `num_envs`=200, `rate`=lo, `start_phase`=first, `foot_comp`=on, `foot_clip_rad`=0.05, `spawn_z`=0.42, `settle_s`=0.5, `gutter`=1.0, `roll_couple`=hold, `roll_gain`=8.0, `roll_damp`=0.8, `roll_cap_nm`=2.0, `roll_sign`=1.0, `yaw_moment`=off, `yaw_moment_gain`=5.0, `swing_lift_mm`=0.0, `turn_target`=settle, `foot_yaw_turn`=auto, `cmd_mean_feet_down`=mixed(2.649/2.65/2.651/2.652), `cmd_frac_below_3_feet`=mixed(0.5005/0.5015/0.5046/0.507), `partial`=0

### 1. Overall

| arm | goals /8 | falls/min | mean episode s | episodes | finished % |
|---|---:|---:|---:|---:|---:|
| all cells | 0.685 | 8.28 | 6.88 | 200 | 5.0 |

### 2. By difficulty

| difficulty | goals /8 | falls/min | mean episode s | episodes | finished % |
|---|---:|---:|---:|---:|---:|
| level 0 | 0.600 | 7.43 | 7.27 | 20 | 10.0 |
| level 1 | 0.650 | 9.75 | 6.16 | 20 | 0.0 |
| level 2 | 0.800 | 8.41 | 6.78 | 20 | 5.0 |
| level 3 | 0.850 | 8.00 | 7.50 | 20 | 0.0 |
| level 4 | 0.700 | 10.27 | 5.84 | 20 | 0.0 |
| level 5 | 0.700 | 7.56 | 7.54 | 20 | 5.0 |
| level 6 | 0.750 | 8.81 | 6.47 | 20 | 5.0 |
| level 7 | 0.600 | 9.53 | 5.98 | 20 | 5.0 |
| level 8 | 0.550 | 6.54 | 7.80 | 20 | 15.0 |
| level 9 | 0.650 | 7.63 | 7.47 | 20 | 5.0 |

### 3. By course

| course | goals /8 | falls/min | mean episode s | episodes | finished % |
|---|---:|---:|---:|---:|---:|
| stepping_stones_randomly_arranged | 1.100 | 9.92 | 6.05 | 10 | 0.0 |
| forward_ramp_lips | 0.900 | 8.47 | 7.08 | 10 | 0.0 |
| sideways_ramp | 0.900 | 7.54 | 7.16 | 10 | 10.0 |
| balance_beam | 0.900 | 9.96 | 6.02 | 10 | 0.0 |
| jump_on_and_off_box | 0.800 | 6.24 | 7.69 | 10 | 20.0 |
| forward_ramp_no_lips | 0.800 | 9.68 | 6.20 | 10 | 0.0 |
| flush_a_frame | 0.800 | 8.79 | 6.82 | 10 | 0.0 |
| sphere_bump | 0.800 | 5.01 | 9.58 | 10 | 20.0 |
| box_jump_uneven | 0.800 | 7.97 | 7.53 | 10 | 0.0 |
| stepping_stones_cylinder | 0.800 | 5.54 | 8.66 | 10 | 20.0 |
| staircase_climbing | 0.800 | 6.50 | 8.31 | 10 | 10.0 |
| box_jump_even | 0.700 | 11.34 | 5.29 | 10 | 0.0 |
| flat_circle_jump | 0.700 | 7.81 | 6.92 | 10 | 10.0 |
| bump_jump | 0.700 | 11.06 | 5.42 | 10 | 0.0 |
| squeeze | 0.700 | 7.68 | 7.03 | 10 | 10.0 |
| sphere_bump_lips | 0.600 | 9.19 | 6.53 | 10 | 0.0 |
| staircase_walking | 0.500 | 10.16 | 5.91 | 10 | 0.0 |
| agility_poles | 0.400 | 8.93 | 6.72 | 10 | 0.0 |
| staircase_walking_full_width | 0.000 | 10.65 | 5.64 | 10 | 0.0 |
| staircase_spiral | 0.000 | 8.51 | 7.05 | 10 | 0.0 |
