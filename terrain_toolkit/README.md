# terrain_toolkit

CPU-only tooling for the eurekaverse Go2 parkour **benchmark** terrains
(20 tasks × 10 difficulty levels). No Isaac Lab / Isaac Sim / torch imports.

The only upstream code that is executed is
`extreme-parkour/legged_gym/legged_gym/utils/set_terrain_benchmark.py`
(numpy only). It is loaded from the read-only upstream checkout resolved by
`terrain_toolkit/paths.py` (`$EUREKAVERSE_ROOT`, else the sibling directory
`../eurekaverse-go2-parkour`).

```
scripts/freeze_benchmark.py   generate → fix_terrain → data/benchmark_frozen.npz (+ sha256, upstream commit)
scripts/profile_terrains.py   per-goal-segment features → outputs/terrain_profile.csv (+ summary, plots)
scripts/render_terrains.py    height-field PNGs → outputs/terrains/*.png, outputs/terrains_overview_level*.png
```

## How the terrain object is faked (`stub.py`)

Upstream `Terrain.make_terrain` (in `terrain_gpt.py`) does, for the benchmark type:

```python
set_seed(int(variation * 1e3 + difficulty * 1e6))          # np.random / random / torch
terrain = SubTerrain("terrain",
                     width=length_per_env_pixels,          # 18 m / 0.05 = 360 cells along x  (NB: swapped names)
                     length=width_per_env_pixels,          #  4 m / 0.05 =  80 cells along y
                     vertical_scale=0.005, horizontal_scale=0.05)
terrain.goals = np.zeros((8, 2))
set_idx = set_terrain_benchmark(terrain, variation, difficulty)   # fills height_field_raw (int16) + goals (m)
```

`SubTerrainStub` reproduces exactly the attributes touched by
`set_terrain_benchmark.set_terrain` and by `fix_terrain`:
`width, length, horizontal_scale, vertical_scale, height_field_raw, goals`.
`difficulty = row / 9`, `variation = col / 20`, dispatcher index `= int(variation * 20)`.

Not reproduced (on purpose – they are added *after* `set_terrain` in the sim):

* `random_uniform_terrain` roughness (±0.02–0.06 m noise; uses `scipy.interpolate.interp2d`,
  removed in scipy ≥ 1.14 – it would not even run in this env),
* the 0.1 m-wide, 0.5 m-high border pad `Terrain.__init__` writes around every cell.

## Frozen archive (`data/benchmark_frozen.npz`)

All arrays indexed `[task, level, ...]`, tasks in dispatcher order.

| key | dtype/shape | meaning |
|---|---|---|
| `height_fields` | int16 (20,10,360,80) | **after** `fix_terrain`, units of `vertical_scale` (×0.005 → m). `[x, y]`, x = forward |
| `height_fields_before_fix` | int16 (20,10,360,80) | straight out of `set_terrain` |
| `goals`, `goals_before_fix` | float64 (20,10,8,2) | metres (x, y) in the sub-terrain frame |
| `set_idx`, `seeds` | int64 (20,10) | dispatcher index / RNG seed used |
| `difficulties` (10,), `variations` (20,) | float64 | `row/9`, `col/20` |
| `difficulty_scaling` (20,2), `scaled_difficulties` (20,10) | float64 | per-task `(min_d, max_d)` and the resulting scaled difficulty |
| `task_names` (20,), `fix_descs` (20,10) | str | |
| scalars | | `horizontal_scale, vertical_scale, terrain_length_m, terrain_width_m, x_cells, y_cells, num_goals, num_rows, num_cols, spawn_x, spawn_y, upstream_commit, upstream_module_sha256, content_sha256` |

The robot spawns at `(spawn_x, spawn_y) = (1.0, 2.0)` m (upstream `env_origin`).
`data/benchmark_frozen.sha256` is in `sha256sum -c` format; `content_sha256`
hashes the array contents independent of the zip container. The npz is
byte-reproducible (`python scripts/freeze_benchmark.py --verify`).

`fix_terrain` is a verbatim numpy port (`fix_terrain.py`) of
`terrain_utils.fix_terrain`, whose module cannot be imported here (pydelatin / pyfqmr / torch).
**Note:** at the frozen commit the upstream `benchmark` branch does *not* call `fix_terrain`
(only `default`/custom types do). Both states are stored; which one the sim
actually used is `height_fields_before_fix`.

## Profile (`outputs/terrain_profile.csv`)

One row per (task, level, segment); 8 segments = `spawn → goal0 → … → goal7`.
Full definitions in the `profile.py` docstring. In short, every segment is
sampled every cell along the straight line; at each position a lateral (y)
cross-section is taken and the **nearest walkable surface within ±0.5 m** of
the line is followed (needed because stepping-stone goals sit in the pit).

| column | meaning |
|---|---|
| `max_step_up_m / max_step_down_m / max_step_m` | largest single-cell height change of the followed surface (pits removed, so a jump contributes take-off→landing) |
| `max_gap_width_m`, `n_gaps`, `gap_landing_dz_m` | longest run of positions with no walkable surface within the corridor (pit < −0.5 m) |
| `mean_slope_deg / max_slope_deg` | slope over a 0.2 m baseline of *continuous* surface (no pit, no single-cell change > 0.08 m, rise distributed over the window → stairs are steps, ramps/domes are slope) |
| `min_width_m / median_width_m` | lateral span around the followed surface bounded by pit / terrain edge / lateral cliff (> 0.08 m per cell) |
| `lateral_offset_max_m` | how far the followed surface strays from the straight line |
| `center_pit_fraction`, `rise_m`, `max_height_m`, `min_height_m`, `path_len_m` | |

Thresholds live in `ProfileParams`.

## Findings about the upstream file

* 21 `set_terrain_*` functions are defined, the dispatcher lists 20:
  `set_terrain_stepping_stones_flat` is unused **and** would raise `KeyError`
  (`difficulty_scaling` has no `"stepping_stones_flat"` entry).
* `sphere_bump`, `sphere_bump_lips`, `flat_circle_jump`, `bump_jump` build
  their masks with `np.ogrid[:m_to_idx(length), :m_to_idx(width)]`, where
  `m_to_idx` returns `np.int16`, so the grids are int16 and `(x - mid_x)**2`
  overflows for cells > 181 away from the centre. Result: `sqrt` of negative
  numbers (NaN → not masked, harmless) and wrapped-around small positives that
  paint **spurious blobs** far from the bump (visible at x > 15 m in the
  `sphere_bump*` renders, 26–37 stray cells inside the spawn area). The sim sees
  the same artefacts. `fix_terrain` erases the small ones and resets the spawn
  area, which is why those two tasks report a fix on all 10 levels.
* `staircase_spiral` ignores `difficulty_scaling` (uses raw difficulty).
* `agility_poles` and stepping-stone tasks put some goals off the walkable
  surface (in the pit / at the pole); `fix_terrain` only clamps goals to the
  terrain bounds.
