# Server day 1 — results

z4, 2026-08-27. Isaac Lab 3.0 container (`scripts/isaac_docker_run.sh`), physics on CPU.
Defects found and fixed along the way are in `outputs/harness_findings.md`.

## Reached: step 3 complete, step 2 settled

`server_day1.md` stop point 3 ("WALK and TROT transfer, the harness is proven"), with the
step 2 convention question settled rather than left open.

---

## 0 · Environment — PASS

| check | result |
|---|---|
| `benchmark_frozen.npz` | OK `01e3bc17…`, content `c117de08…`, 20 tasks × 10 levels |
| `calibration_probes.npz` | OK `de9d7863…`, content `ae467d86…` deterministic over two rebuilds |
| `skill_clips.npz` | OK `aab85a03…`, WALK/TROT/RUN/JUMP all present |
| `import isaaclab, isaacsim` | isaaclab 6.1.16, Isaac Sim 6.0.1-rc.7, torch 2.10.0+cu128 |
| config read back from upstream | 50 Hz control, `IdealPDActuator` explicit, kp 40 / kd 1, effort 23.70 / 45.43 |

The calibration archive was checked **read-only**: `freeze_calibration.py --verify` calls
`save()` unconditionally and would have overwritten it. See `harness_findings.md`.

## 1 · Actuator probe — PASS, day-1 table row 1

`outputs/isaac_actuator_probe.json` (committed).

| | result |
|---|---|
| Q1 | `IdealPDActuator`, explicit — Python-side PD |
| Q2 | doubling stiffness scaled \|τ\| by **2.000** → runtime gain write works |
| Q3 | +5 Nm effort target moved applied torque by **+5.000 Nm** → additive |
| Q4 | `write_joint_stiffness_to_sim` accepted, but writes PhysX gains the Python PD never reads |
| Q5 | saturation at **23.70 / 45.43 Nm**, exactly the configured `effort_limit` |

→ full fidelity available; the torque-headroom analysis in `gain_feasibility.md` stands.

## 2 · Convention — **settled: the hip sign is flipped**

The offline `--convention` prediction is confirmed in simulation.

| clip | mapping result |
|---|---|
| WALK, `--hip-sign flip` | `identity`, r = 0.553, **margin 0.060** — above the 0.05 the doc asks for |
| WALK, `--hip-sign keep` | identity wins by only 0.020 — indecisive, as the doc predicts for WALK |
| TROT, `--hip-sign keep` | collapses at 0.944 s |
| TROT, `--hip-sign flip` | survives all 256 steps, min base 0.247 m |

The decisive evidence is not the correlation margin but the **collapse**: TROT replayed with
the hip columns as logged goes down at 0.94 s and with them negated it does not. Use
`--hip-sign flip` everywhere.

Two things the doc expected that did not hold:

* It routes a both-sides-collapse to *"thigh/calf sign or zero offset … suspect
  `default_joint_pos` handling in the settle phase."* Instrumenting the settle phase ruled
  that out: WALK / TROT / RUN settle to 0.317 / 0.334 / 0.309 m against a 0.32 m nominal,
  all four feet loaded. Settle is sound. The height is lost **during replay**, cycle by cycle.
* It expects RUN and JUMP to be the discriminating clips. RUN could not discriminate here —
  once the robot is on its belly the measured joint angles stop tracking the command and the
  mapping correlations degenerate (r = 0.071 / 0.107). WALK with the collapse removed did the
  job instead.

## 3 · WALK and TROT baseline — PASS

`--mode position --hip-sign flip --contact-threshold-n 30`

| clip | stride | criterion | duty | verdict |
|---|---|---|---|---|
| WALK | **1.40 Hz** vs 1.37 expected (+2.2 %) | within 15 % ✅ | within 0.10 ✅ | **WARN** ✅ |
| TROT | **1.56 Hz** vs 1.56 expected (0 %) | within 15 % ✅ | within 0.10 ✅ | **WARN** ✅ |

### The contact threshold was wrong, and the doc said to check it first

Default `--contact-threshold-n 1.0` counts grazing contact as stance. `CLAUDE.md` §6 already
records that the real logs needed **24–51 N** per-leg thresholds; 1 N is a different
measurement being compared against the log's.

| threshold | WALK stride | TROT stride |
|---|---|---|
| 1 N | 1.40 Hz, duty 0.88 vs 0.66 | 3.32 Hz vs 1.56 (**+113 %**) |
| 10 N | 1.40 Hz, duty 0.81 vs 0.66 | 2.69 Hz (+73 %) |
| 20 N | 1.37 Hz, duty ok | 1.61 Hz (+3 %) |
| 30 N | 1.40 Hz, duty ok | **1.56 Hz (0 %)** |

TROT's *"stride +113 %"* was entirely a threshold artefact. It is the same failure shape as
the physics-rate defect: a measurement fault that reads as a gait fault.

## Open, carried into day 2

* **Forward speed is short on both clips** — WALK 0.024 vs 0.187 m/s, TROT 0.122 vs 0.444 m/s.
  The gait runs at the right frequency and does not fall, but does not travel. Ground friction
  and the `--gains log` / `--apply-tau-ff` path are the doc's suggested next reads.
* **Yaw drift** — WALK 20 °/s, TROT 56 °/s while commanded straight. Identity still wins the
  mapping, so this is not leg order.
* **Step 4 (RUN) not started.** Under the old harness RUN failed both ways; it has not been
  re-judged since the physics-rate fix and the threshold finding.
* **Step 5 (calibration) not started.**

## Cost, measured

Isaac Sim startup ≈ 28 s per container run. Stepping runs at **3.96× real time on CPU**
(WALK, 8 vs 32 cycles: 29.73 s / 34.16 s wall for 5.85 s / 23.41 s simulated).

The step 5 sweep is **360 sequential episodes**, not parallel ones — `run_calibration.py`
runs a single `SimulationContext` in a loop. At 20 s budget each that is 2.0 h simulated,
so **≈ 30 min wall clock on CPU**, less with early termination. No GPU is needed, and none
would help much: one robot per episode uses no parallelism. Both GPUs on this box are
occupied by another user's training (17 GB each, ~7.4 GB free).
