# The symmetric rear lift on the step ladder — a null, and a corrected step bracket

`swing_lift.md`'s ladder table was committed at `9f225b4`; the symmetry fix landed at
`c0d77bf`, **after** it. So that table was measured with the per-leg (asymmetric) lift that
`swing_lift_symmetry.md` then showed is a roll input — +2° → +55° of roll and 1.5 m of
sideways crab over a 3 m approach. Re-running it with the symmetric lift was therefore
open, and this is that re-run.

## 1. Rear-only lift is automatic, and the harness prints it

The amplitude is `max(0, target − existing apex)` per **mirror pair**, so a target between
the front and rear apexes lifts the rear pair and leaves the front alone. Measured, from
the harness's own report:

| clip | target | FL | FR | RL | RR |
|---|---|---|---|---|---|
| WALK | 40 mm | +24.5 | +24.5 | +39.9 | +39.9 |
| WALK | 60 mm | +44.5 | +44.5 | +59.9 | +59.9 |
| WALK | 80 mm | +64.5 | +64.5 | +79.9 | +79.9 |
| TROT | 60 mm | **+7.0** | **+7.0** | **+55.0** | **+55.0** |
| TROT | 70 mm | +17.0 | +17.0 | +65.0 | +65.0 |

Left and right are identical in every row, which is what the symmetry fix is for. **TROT's
rear apex is ~5 mm, not the 50–57 mm `swing_lift.md` reports** — that figure is the front
pair's. Both gaits drag their rear feet, not just WALK.

## 2. It does not raise the passable step, and on TROT it ends the run

TROT, at its three surviving clip frames, yaw couple on:

| lift | survived |
|---|---|
| 0 | the full budget |
| 60 mm | **1.1–2.0 s** at all three frames |
| 70 mm | **1.1–2.0 s** at all three frames |

WALK, `--params skill.STEP_WALK_MAX`, 5 entry phases, symmetric, by the strict criterion of
§3 — the highest level the base actually **climbed**:

| swing lift | 0 | 40 mm | 60 mm | 80 mm |
|---|---|---|---|---|
| highest level climbed | **0.040** | 0.020 | none | 0.020 |
| goal 2 reached at 0.020 | **4/5** | 2/5 | 0/5 | 0/5 |

**The best result in the table is the untouched recording, in both gaits.** That is
`swing_lift.md`'s conclusion, and it survives the symmetry fix. `--swing-lift` stays off.

## 3. The criterion mattered more than the intervention, and it corrects §6 of `trot_yaw_moment.md`

The bracket reported earlier — **TROT 0.04–0.06 m** — used "did the base's x get past the
riser". That admits a base leaning over the lip while pitched down, which is what
`lip_failure.md` §1 describes WALK doing at 0.04 ("advances to x = 4.096, holds at 4.09 for
1.5 s pitched −9°, and rolls over"). Taken to its logical end that metric calls TROT's
highest crossed level **0.120 m**, which is plainly wrong.

Replaced with: **base ≥ 0.30 m past the riser AND risen ≥ 60% of the step height, held
0.5 s.** Sensitivity of the answer to those three numbers:

| margin | rise fraction | TROT (6 runs) | WALK (lift 0) |
|---|---|---|---|
| 0.10 m | 0.5 / 0.6 | 0.040 | 0.040 |
| 0.10 m | 0.8 | 0.020 | 0.040 |
| **0.20 / 0.30 m** | **0.5 / 0.6 / 0.8** | **0.020** | **0.040** |
| 0.50 m | any | 0.020 | 0.020 |

**Corrected brackets:**

| | climbs | does not climb | bracket |
|---|---|---|---|
| **TROT** | 0.02 m | 0.04 m | **0.02–0.04 m** |
| **WALK** | 0.04 m | 0.06 m | **0.04–0.06 m** |

TROT's is exactly `lip_failure.md` §1's foot-radius prediction — the loaded foot centre
rides at 23 mm, so a 20 mm riser is an edge a foot rolls over and a 40 mm riser is a wall.
WALK sits one level higher, at a duty of 0.64 against TROT's 0.52.

**Neither is written to `planner/config.py`.** `STEP_TROT_MAX` stays 0.08 and
`STEP_WALK_MAX` 0.10, both `CALIBRATION_NEEDED`, by the user's decision.

**This is the third time in this work that the reported number was a neighbouring
quantity** (`harness_findings.md` §16 has the other two). The pattern is the same each
time: a plausible number, a plausible mechanism, and no second frame of reference until
one was demanded.
