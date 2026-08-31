# v2 probes: slope and roughness, and the first thresholds the harness has produced

The slope and roughness generators have been in `terrain_toolkit/calibrate.py` since it was
written and were **never in the frozen archive** — `data/calibration_probes.npz` holds 42
probes (`step_up` 15, `step_down` 15, `gap` 12) and the plan document has been describing
62 the whole time. Two things had to be fixed before they could be run, and both are in
§0. All CPU, no GPU.

`data/calibration_probes_v2.npz`, 62 probes, frozen **beside** the pinned archive rather
than over it. Probe indices 0–41 are byte-identical terrain with byte-identical names and
goals (checked, not assumed: the generator appends), so every result measured against the
pinned archive stays comparable.

---

## 0. Two bugs the new families exposed

**Probe names were not unique, and the reduction is keyed by name.** `Probe.name` formats
the level to two decimals. The roughness ladder steps by 0.005 m, so 0.005, 0.010 and 0.015
all render as `roughness_0p01`. The per-repeat rows carried the right `level_m` throughout —
only the name-keyed per-level summary was wrong, which is exactly why it survived a reading
of the CSV: it reported `0/15` for a family with 5 repeats and nothing about that row said
three terrains had been merged into one. Fixed by giving the roughness family three
decimals (`Probe.NAME_DECIMALS`) and by making `build_probes()` **raise** on a duplicate
name rather than return one. Step, gap and slope names are unchanged, so v1 is untouched.

**A robot that cannot settle took the whole grid down with it.** Standing a robot on the
obstacle (`--score inplace`, §`turn_probes.md`) puts it astride a 0.60 m pit with nothing
under its feet, or on a 25° ramp it slides off. `FootPlacement` is built after the settle
from the settled pose and correctly refuses a hip-to-foot lever of 0.034 m — its law is
linearised about a standing pose — but the refusal aborted the run for all 42 robots,
including the 40 that had settled fine. Such a robot is now **scored a failure at settle**,
which is the true answer, the grid-median lever is substituted only so the object can be
constructed, and every one of its rows is stamped `settle_ok = 0`.

---

## 1. WALK on slope: fails at 10°, against a 35° placeholder

`--params skill.SLOPE_WALK_MAX`, 5 repeats, `--foot-comp on --heading heading-only`.

| angle | passed | |
|---|---|---|
| 5° | **4/5** | one entry phase falls |
| 10° | 0/5 | |
| 15°–40° | 0/5 | |

Monotone. Under the protocol's own rule — top of the unbroken run that passed **every**
repeat, then back off one level — 5° is 4/5, so there is **no all-pass level and no value to
propose**. What the sweep does establish is a bracket: **WALK's slope limit is between 0°
and 10°, and 5° is marginal.**

`planner/config.py` carries `SLOPE_WALK_MAX = 35.0` as a `CALIBRATION_NEEDED` placeholder.
The measurement says that is **at least 3.5× too optimistic**, and it should not be moved to
MEASURED on this evidence — it should stay flagged, now with a bracket instead of nothing.

## 2. WALK on roughness: a clean, monotone curve, and the first all-pass level

`--families roughness --skill WALK`, 5 repeats, same control.

| amplitude | passed | |
|---|---|---|
| 0.005 m | **5/5** | |
| 0.010 m | **3/5** | |
| 0.015 m | 0/5 | |
| 0.020–0.060 m | 0/5 | |

Monotone, and the only place in this project where the protocol has been able to apply its
own reduction end to end: top all-pass **0.005 m**, first failure 0.010 m, back off one
level → **0.000**. The margin rule eats the entire result because the ladder's floor *is*
the limit.

`ROUGHNESS_TROT_MAX = 0.03` is the nearest placeholder. WALK — the more tolerant gait —
falls at half that, and TROT (below) does not survive the smallest level at all.

**Read this against what the planner measures, not against the amplitude.** The generator's
`A` is a triangle-wave amplitude; `planner.features._detrended_rms` measures the residual to
a local straight line, which for this wave is about **0.7·A**. So the bracket in the
planner's own units is roughly **0.004–0.010 m of detrended RMS**, and the ladder step is
0.0035 m in those units. Quoting the amplitude as if it were the feature would be the
CLAUDE.md §6.5 error again.

## 3. TROT on roughness: 0/5 at every level

Expected and uninformative for the same reason `STEP_TROT_MAX` is: TROT does not reach the
patch on a line (`trot_straight.md`). Recorded so the zero is on the record with its cause
attached, not read as a roughness limit of zero.

## 4. TURN on slope and roughness

In `turn_probes.md` §4, since it needs the in-place criterion and the flat control to be
read at all. In short: roughness below the 0.005 m floor; slope non-monotone and therefore
not a limit, with the separate and solid finding that **the robot cannot establish a stance
at all on 20° or steeper**.

## 5. What this changes

Nothing is applied to `planner/config.py`. Three of the ten `CALIBRATION_NEEDED` fields now
have **brackets** rather than nothing, which is the first time any of them have had
anything:

| field | placeholder | measured |
|---|---|---|
| `skill.SLOPE_WALK_MAX` | 35° | between 0° and 10°; 5° is 4/5 |
| `skill.ROUGHNESS_TROT_MAX` | 0.03 m | WALK falls at 0.015 m; TROT at 0.005 m |
| `skill.SLOPE_TROT_MAX` / `SLOPE_RUN_MAX` | 20° / 10° | not run: TROT cannot hold a line, RUN cannot stand |

**The ladders' floors are too high.** Both families were sized against the placeholders —
slope from 5° to 40°, roughness from one `vertical_scale` unit to twice
`ROUGHNESS_TROT_MAX` — and the placeholders are where the error is. A ladder that starts
below the limit is what a threshold needs, and neither of these does. Re-cutting them
(slope 1°–10° in 1°, roughness 0.001–0.010 m in 0.001) is a regeneration of the v2 archive
and costs nothing but the run.
