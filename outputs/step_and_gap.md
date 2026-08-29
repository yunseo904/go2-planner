# Step capability, and the ground plane that was under everything

Four questions, answered from traces and the frozen archive. Flat CPU, no GPU.

---

## 1. The gap family was measuring nothing — an infinite plane was under the terrain

`TerrainImporterCfg(terrain_type="plane")` is the only terrain type that needs no extra
config, and constructing it calls `import_ground_plane()`, which lays a **2000 km × 2000 km
plane at z = 0** *under* the imported probe mesh.

Every pit in the archive was floored by it, and every gutter between cells was solid
ground. The gap family's geometry is correct — checked directly in the archive: `gap_0p60`
is **0.60 m wide, 1.00 m deep, spanning all 80 lateral cells** (the full 4 m width, no way
round, goal 2 at x = 5.0 m on the far side). The robot was walking over its own trench on
an invisible floor.

That is where `FOOT_SPAN_X = 0.550 m` came from: 60/60 reached, 0% fell, `first fail` none.

With the plane removed (and the removal asserted, not assumed):

| | reached | fell | first failing level |
|---|---|---|---|
| gap WALK, plane present | 60/60 | 0% | none |
| **gap WALK, plane removed** | **0/60** | 80% | **0.05 m** |

**WALK cannot cross a 5 cm gap.** `FOOT_SPAN_X` has no value and 0.550 must not be used.
Removing the plane also makes walking off a cell a fall, which is the correct verdict for
a probe that leaves its own terrain.

---

## 2. TROT on a step: it is not clearance, and it does climb

The question was whether TROT's swing feet reach the lip at all. They do, comfortably.

Swing-bout apex clearance measured on the flat approach of each probe (mm):

| skill | FL | FR | RL | RR | vs a 20 mm step |
|---|---|---|---|---|---|
| TROT | 51.9 | 68.4 | 64.4 | 54.2 | **all four clear it by 2.5×** |
| WALK | 41.4 | 48.4 | **3.8** | **5.5** | front only |

And the trace shows TROT **climbing the 2 cm step successfully** — it crosses the lip at
x = 3.95 m with the body level (roll ±1°, height 0.35 m) and keeps trotting. Then:

    t=5.46  x=4.13  roll  -0.9    on the platform, level
    t=5.94  x=4.33  roll  -8.6
    t=6.26  x=4.45  roll -19.7
    t=6.58  x=4.44  roll -36.1
    t=6.90  x=4.31  roll -118.4   over

**It rolls over 0.4 m past the step**, the roll growing monotonically over ~1.4 s. That is
the same divergence signature as the untreated flat-ground TROT, seeded by the step
instead of by a handover: the step injects a lateral disturbance the capped correction
cannot absorb, and it grows.

So the answer to "can TROT climb steps" is: **it can climb a 2 cm step and cannot survive
having done so.** Not a kinematic limit — a stability one, and the same one everything
else in this project keeps running into.

## 3. WALK is a different failure, and it is the rear feet

WALK's rear swing apex is **3.8–5.5 mm**. It cannot lift its rear feet over anything.

- 2 cm step: front clears (41–48 mm), and it reaches the goal 5/5.
- 4 cm step: the front apex of ~40 mm is now marginal against the step itself, the rear
  cannot lift at all, and it fails 4/5.
- The trace shows the mechanism: pitch goes nose-up to −10°, x stalls at 3.85 → 4.11 m
  astride the lip, roll builds +6.8° → +18.2°, then over.

So WALK's 84% falls are **not** a harness artifact — they are the rear legs. (There *was*
a harness artifact and it was §1, on a different family.)

**`STEP_WALK_MAX`: 2 cm reliable, 4 cm not.** Confirmed with the plane removed: 0.02 m
reached 5/5, 0.04 m reached 1/5, 0.06 m 0/5. The protocol's one-level margin makes the
config value 0.000 because the ladder's floor *is* 0.02 and there is nothing below to back
off to. Nothing below 0.02 has been cut — the archive stays as it is.

`STEP_TROT_MAX` still has no value: TROT fails at the floor, 75/75.

---

## 4. TROT → TURN goes through WALK

The one switch still failing after the heading work. WALK sits between them at 0.23 m/s.

| | TROT | WALK | TURN | outcome |
|---|---|---|---|---|
| **direct** TROT→TURN | 15.9 cyc, ok | — | 1.3 cyc, stride 1.52/1.12, yaw −66.9 °/s | **FAIL, fell 1.1 s in** |
| **bridged** TROT→WALK→TURN | 15.9 cyc, ok | 8.2 cyc, ok | **8.0 cyc, stride 1.03/1.12 (+8%), vx −0.014, yaw −15.7** | **ok** |

The bridge turns a 1.1 s collapse into 8 clean TURN cycles inside the ±10% stride gate and
in place. It confirms the speed gap as the cause — 0.64 m/s into a gait whose measured
speed is 0.008 — and gives the planner a rule it can hold: **route TROT→TURN through
WALK.**

TURN still ends at 23.3 s, 7.1 s after entry, against 60 cycles from its own settle. So
the bridge makes the transition survivable, not permanent.

---

## 5. Slope and roughness probes now exist

The five parameters with no measuring terrain (`SLOPE_WALK/TROT/RUN_MAX`,
`ROUGHNESS_TROT/RUN_MAX`) have families, generated under the same rules: one condition per
terrain, no upstream import, **no RNG**.

- **slope**: 8 levels, 5°–40°, a 2.0 m constant incline from x = 4.0 m with flat lane
  before and after. Reads back as 5.71 / 20.56 / 40.36° against nominal 5 / 20 / 40 —
  quantisation bias, and roughness stays at 0.001 m.
- **roughness**: 12 levels, 5–60 mm, a **four-cell triangle wave** (0, +A, 0, −A). Reads
  back as 0.69 A of `roughness_m`, which is what a triangle wave should give.

The wave shape is the one design decision worth stating. A two-cell square wave was tried
first and rejected: it makes the cell-to-cell jump **2A**, so at the top of the ladder the
planner reads a 0.12 m *step* alongside 0.057 m of roughness and a failure could be
attributed to either — which breaks the one-condition-per-terrain rule the toolkit exists
to enforce. The triangle halves the jump to exactly A. It cannot be removed: roughness at
this scale *is* local steps, and the coupling is now stated rather than hidden.

Frozen as **`data/calibration_probes_v2.npz`** (62 probes, sha256 `8301f16c…`),
**beside** the pinned 42-probe archive, which is untouched and still hashes to
`de9d7863…`. `freeze_calibration.py --out` exists for exactly this. The v2 set has not
been run yet.
