# Benchmark aggregate

Sources: outputs/bench_le/h2r_rough_on.csv (200 rows)

## all rows

> **Conditions.** `terrain`=legged_eval-seed1, `terrain_seed`=1, `measurement_seed`=deterministic, `skill`=WALK, `perception`=n/a, `steering`=dead-reckoned-heading-hold, `heading`=heading-only, `heading_cap_rad`=0.04, `episode_length_s`=20.0, `episodes`=1, `num_envs`=200, `rate`=lo, `start_phase`=first, `foot_comp`=on, `foot_clip_rad`=0.05, `spawn_z`=0.42, `settle_s`=0.5, `gutter`=1.0, `roll_couple`=hold, `roll_gain`=8.0, `roll_damp`=0.8, `roll_cap_nm`=2.0, `roll_sign`=1.0, `yaw_moment`=off, `yaw_moment_gain`=5.0, `swing_lift_mm`=0.0, `turn_target`=settle, `foot_yaw_turn`=auto, `cmd_mean_feet_down`=2.649, `cmd_frac_below_3_feet`=0.5676, `partial`=0

### 1. Overall

| arm | goals /8 | falls/min | mean episode s | episodes | finished % |
|---|---:|---:|---:|---:|---:|
| all cells | 0.835 | 5.42 | 9.36 | 200 | 15.5 |

### 2. By difficulty

| difficulty | goals /8 | falls/min | mean episode s | episodes | finished % |
|---|---:|---:|---:|---:|---:|
| level 0 | 0.850 | 5.15 | 9.33 | 20 | 20.0 |
| level 1 | 0.600 | 7.99 | 7.13 | 20 | 5.0 |
| level 2 | 0.950 | 4.60 | 10.44 | 20 | 20.0 |
| level 3 | 0.900 | 5.78 | 9.35 | 20 | 10.0 |
| level 4 | 0.800 | 8.35 | 7.19 | 20 | 0.0 |
| level 5 | 0.900 | 5.52 | 9.24 | 20 | 15.0 |
| level 6 | 0.900 | 3.66 | 11.48 | 20 | 30.0 |
| level 7 | 0.800 | 5.75 | 8.87 | 20 | 15.0 |
| level 8 | 0.750 | 3.67 | 10.64 | 20 | 35.0 |
| level 9 | 0.900 | 5.76 | 9.90 | 20 | 5.0 |

### 3. By course

| course | goals /8 | falls/min | mean episode s | episodes | finished % |
|---|---:|---:|---:|---:|---:|
| stepping_stones_randomly_arranged | 1.500 | 8.19 | 7.33 | 10 | 0.0 |
| balance_beam | 1.300 | 7.25 | 8.28 | 10 | 0.0 |
| box_jump_even | 1.100 | 7.54 | 7.96 | 10 | 0.0 |
| squeeze | 1.100 | 3.31 | 10.87 | 10 | 40.0 |
| forward_ramp_lips | 1.000 | 2.28 | 15.80 | 10 | 40.0 |
| forward_ramp_no_lips | 1.000 | 7.65 | 7.85 | 10 | 0.0 |
| flush_a_frame | 1.000 | 8.38 | 7.16 | 10 | 0.0 |
| sphere_bump | 0.900 | 5.17 | 9.29 | 10 | 20.0 |
| box_jump_uneven | 0.900 | 7.60 | 7.89 | 10 | 0.0 |
| jump_on_and_off_box | 0.800 | 7.14 | 7.56 | 10 | 10.0 |
| sphere_bump_lips | 0.800 | 2.93 | 12.28 | 10 | 40.0 |
| flat_circle_jump | 0.800 | 5.52 | 8.69 | 10 | 20.0 |
| sideways_ramp | 0.800 | 7.12 | 7.59 | 10 | 10.0 |
| stepping_stones_cylinder | 0.800 | 5.67 | 8.46 | 10 | 20.0 |
| staircase_climbing | 0.800 | 3.08 | 13.64 | 10 | 30.0 |
| bump_jump | 0.700 | 6.93 | 7.79 | 10 | 10.0 |
| staircase_walking | 0.700 | 9.39 | 6.39 | 10 | 0.0 |
| agility_poles | 0.700 | 2.75 | 13.11 | 10 | 40.0 |
| staircase_walking_full_width | 0.000 | 8.44 | 7.11 | 10 | 0.0 |
| staircase_spiral | 0.000 | 3.49 | 12.05 | 10 | 30.0 |
