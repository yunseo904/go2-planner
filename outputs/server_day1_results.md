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

## 2 · Convention — **REOPENED, see the addendum at the end**

The offline `--convention` prediction is confirmed in simulation.

| clip | mapping result |
|---|---|
| WALK, `--hip-sign flip` | `identity`, r = 0.553, **margin 0.060** — above the 0.05 the doc asks for |
| WALK, `--hip-sign keep` | identity wins by only 0.020 — indecisive, as the doc predicts for WALK |
| TROT, `--hip-sign keep` | collapses at 0.944 s |
| TROT, `--hip-sign flip` | survives all 256 steps, min base 0.247 m |

The decisive evidence here was the **collapse**: TROT replayed with the hip columns as
logged goes down at 0.94 s and with them negated it does not.

**This conclusion did not survive the traction measurement. Read the addendum before
acting on it.**

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


---

# Addendum — the convention question is reopened

> **Superseded by the Day 2 section at the end of this file.** The measurements below were
> taken through the aliasing defect (`harness_findings.md` §6) and are withdrawn. The
> convention is `keep`; the friction proposal in this addendum was right but incomplete.

Written at the end of day 1, after the forward-speed investigation. **It contradicts the
step 2 conclusion above.** Do not act on `--hip-sign flip` until this is resolved.

## What was measured

A traction probe (foot slip, foot position in the base frame, base travel) on WALK,
8 cycles = 5.85 s simulated, Isaac Lab's default ground friction (0.5):

| | `--hip-sign keep` | `--hip-sign flip` |
|---|---|---|
| base travel, x | **+1.589 m** | −0.130 m |
| vx mean | **+0.270 m/s** (log: 0.187) | −0.013 m/s |
| yaw drift | **−0.08 °/s** | 8.59 °/s |

`keep` walks forward at close to the logged speed and holds a straight line. `flip` does
not travel at all and yaws. The same split shows in the step 3 verdicts: WALK `keep` read
vx 0.338 m/s, WALK `flip` read 0.024 m/s.

## Why the day-1 evidence pointed the other way

Both readings that favoured `flip` are weaker than they looked:

* **The mapping margin** (identity at 0.060 with `flip`, 0.020 with `keep`) is computed from
  the correlation between commanded and measured joint angles. It says the joints track
  their commands; it says nothing about whether the resulting gait moves the robot.
* **The TROT collapse** under `keep` is real and unexplained. It is the one piece of
  evidence still standing for `flip`.

So the two clips disagree: WALK wants `keep`, TROT survives only under `flip`. That is not
a convention — a joint-frame sign difference cannot be per-clip. Something else is being
attributed to the hip sign.

## What the foot geometry says — nothing decisive

Both conventions put the feet on the correct side of the body. Foot y in the base frame,
against a nominal stance of FL +0.172 / FR −0.172 / RL +0.176 / RR −0.176:

* `keep`: +0.130 / −0.130 / +0.155 / −0.155
* `flip`: +0.158 / −0.157 / +0.151 / −0.152

Neither crosses the midline, so this test does not separate them. An earlier reading of
this table as evidence *for* `flip` was wrong: it only checked `flip` against nominal and
never ran `keep`.

## Ground friction is measured, and it is not the forward-speed cause

* Isaac Lab's `RigidBodyMaterialCfg` defaults to **static 0.5 / dynamic 0.5**, and the
  harness spawns `GroundPlaneCfg()` with no override, so 0.5 is what both replay scripts
  have been running on.
* Upstream sets `randomize_friction = True`, `friction_range = [0.6, 2.0]`
  (`legged_robot_config.py: CustomDomainRandCfg`). **The env never uses 0.5** — the harness
  ground is below the entire range training and evaluation sample from.

Friction sweep, WALK, `--hip-sign flip`:

| mu | base travel x | vx mean | yaw °/s | foot slip total, FL/FR/RL/RR (m) |
|---|---|---|---|---|
| 0.5 (Isaac default) | −0.130 | −0.013 | 8.59 | 0.67 / 0.23 / 0.73 / 0.85 |
| 0.6 (range min) | −0.156 | −0.019 | 5.74 | 0.36 / 0.25 / 0.38 / 0.65 |
| 1.0 | −0.243 | −0.063 | 3.83 | 0.17 / 0.18 / 0.27 / 0.34 |
| 1.3 (range mean) | −0.017 | +0.009 | 3.75 | 0.15 / 0.15 / 0.21 / 0.18 |
| 2.0 (range max) | −0.023 | −0.001 | 2.31 | 0.14 / 0.17 / 0.17 / 0.15 |

Raising friction does what it should to **slip** — the feet slide 0.85 m less per run at the
top of the range — and yaw drift falls monotonically with it. But **forward travel stays at
zero** across the whole range. Under `flip` the gait does not generate forward motion at
all, and no friction value rescues it. Friction is a real defect in the harness setup, but
it is not what is holding the robot still.

## Recommended ground friction — proposed, not applied

Nothing has been changed. The proposal is **static = dynamic = 1.3**, and the reason is that
it is the **midpoint of the env's own `friction_range = [0.6, 2.0]`**, not a value chosen to
make a number come out. Rationale:

* It is inside the distribution the E2E policy trains and is evaluated on, so both arms of
  the comparison stand on ground the E2E arm has seen. 0.5 is outside it.
* A single fixed value, not a per-episode sample: the calibration sweep measures "how tall a
  step does this skill clear", and randomising the ground would fold friction variance into
  a geometric threshold. The midpoint is the least arbitrary fixed choice.
* It must be applied identically to the rule-planner and E2E evaluation runs, and recorded
  next to the results. CLAUDE.md §2 forbids tuning it to performance data — this value comes
  from the env config, and it should not be revisited because a score improves.

The alternative worth considering is **0.6**, the range minimum, as the conservative choice:
any skill threshold measured there is a lower bound that holds across the whole range.

## Where to pick up

1. **Resolve the convention conflict first.** WALK travels only under `keep`; TROT stands
   only under `flip`. Both cannot be true. Suspect the TROT collapse is a different fault
   being masked by the hip sign — re-run TROT `keep` with friction 1.3 and threshold 30 N,
   which is a combination that has never been tried (the collapse was measured at friction
   0.5). If TROT survives `keep` there, the convention is `keep` and the day-1 step 2
   conclusion is simply wrong.
2. **Then decide friction**, apply it in both `verify_skill_replay.py` and
   `run_calibration.py`, and record the value in this file.
3. **Then step 4 (RUN)** — not re-judged since the physics-rate fix and the 30 N threshold.
4. **Then step 5 (calibration)** — ~30 min wall clock on CPU, no GPU needed.

What is settled and should not be re-litigated: the physics rate (`harness_findings.md` §5),
the contact threshold (30 N, from CLAUDE.md §6's measured 24-51 N), the actuator probe
answers, and the archive hashes.


---

# Day 2 — the addendum's conflict resolved, and why it existed

The addendum asked which of `keep` and `flip` was the joint-frame convention, and noted
the evidence pointed both ways. It pointed both ways because **the evidence was fabricated
by a recording defect**. That is the day's finding; the convention falls out of it.

## The defect: base metrics were the final step, repeated

`verify_skill_replay.py` appended `robot.data.root_pos_w[0].cpu().numpy()` each control
step. `[0]` is basic indexing, so it returns a *view* into a buffer PhysX overwrites in
place; on `--device cpu` `.cpu()` copies nothing. Every appended "sample" aliased one
buffer. Measured on a 42-step TROT replay: `root_pos_w`, `root_quat_w`, `root_lin_vel_b`
and `root_ang_vel_b` each had **1 distinct row out of 42**, while `q` and `tau` had 42/42
— they were read with `[0, idx_t]`, which is advanced indexing and allocates.

So `base_height_mean_m` was never a mean. It was the base height at the *final* step,
which on a run that ends by falling is the height of a fallen robot **by construction**.
Every clip reported "the robot is on its belly", whether or not it ever fell. `vx_mean`,
`vy_mean` and `yaw_rate_deg_s` were likewise single instantaneous values.

Full write-up in `harness_findings.md` §6. Fixed by copying out of the sim, plus a guard
that refuses to report numbers when a base array is bit-identical across every step.
**The defect is invisible on GPU** — `.cpu()` copies from CUDA. Every run so far was CPU.

Both halves of the addendum's contradiction came from this channel, so both are withdrawn:
the day-1 "hip sign is flipped" conclusion and the addendum's traction table alike.

## The convention is `keep`

Re-measured after the fix, at the corrected friction (below), 8 cycles:

| | WALK `keep` | WALK `flip` | TROT `keep` | TROT `flip` |
|---|---|---|---|---|
| stride | **1.37 Hz vs 1.37 expected** | 1.39 Hz | 4.52 Hz (+191%) | 1.84 Hz (+19%) |
| forward speed | **0.223 m/s vs log 0.187** | −0.017 m/s | 0.029 m/s | −0.021 m/s |
| terminated | no | no | **0.84 s** | no |
| verdict | WARN | WARN | FAIL | WARN |

WALK under `keep` is the only configuration that reproduces the logged gait: stride exact
to two decimals, forward speed within 20% of the log. Under `flip` the same clip does not
travel at all. Independently, the offline mapping search over the clips' own `q` vs `q_des`
— which needs no simulator and cannot be affected by any of this — ranks **identity first
for all four clips**, with RUN's margin at +0.177.

The one piece of evidence that had favoured `flip`, TROT's collapse under `keep`, is
explained in the next section and is not about the hip sign.

## Why TROT collapses under `keep`: the initial condition, not the clip

TROT keep + friction 1.3 + 30 N still terminates, at 0.84 s. Per instruction, the data
before the interpretation. State at the **first replay step** — i.e. what the settle handed
over, before a single clip frame was played:

| clip | \|v\| | \|w\| | contact FL / FR / RL / RR (N) | roll | outcome |
|---|---|---|---|---|---|
| WALK `keep` | 0.121 m/s | 6.2 °/s | 63 / 27 / 40 / 49 | −2.3° | survives |
| TROT `keep` | **0.365 m/s** | **51.6 °/s** | 92 / **0** / 34 / 32 | −13.9° | collapses 0.84 s |
| RUN position | 0.194 m/s | 12.6 °/s | **0** / 27 / 44 / 40 | +4.3° | collapses 1.11 s |

TROT begins its first control step already travelling at 0.365 m/s and rotating at 52 °/s,
with the front-right foot carrying nothing. Every clip that fails hands over with a foot
unloaded and the base already tumbling; the one that survives does not.

The collapse itself, step by step: roll runs −14° → −120° while RL holds 71% stance duty
and never swings, FR and RR sit at 0 N from 0.26 s and 0.44 s onward. The robot tips onto
its left side. It is not walking badly — it is falling over from the pose it was handed.

**Cause.** The settle wrote the base to the env's spawn height of 0.42 m — a height chosen
for the env's *default standing pose*, all four feet down — then overrode the joints to the
clip's **first frame**, which for a trot is a mid-gait pose with a foot in swing, and let
the robot fall into it. It held for a fixed 0.5 s of wall clock with no convergence check
and handed over whatever state it was in. This ranks the clips exactly as their outcomes
rank: WALK frame 0 has 3 feet down and settles quietly; TROT frame 0 has 3 feet down but
does not settle; RUN's clip has **no frame at all with 4 feet down**.

**A/B, `--settle-mode stand`** — settle on the default standing pose first, then drive to
the clip pose with the PD while the robot is already up (same `settle_s` for each half, so
no new tuned constant):

| | handover \|v\| | handover \|w\| | terminated |
|---|---|---|---|
| TROT drop | 0.365 m/s | 51.6 °/s | 0.82 s |
| TROT stand | **0.072 m/s** | **23.2 °/s** | 1.57 s |
| RUN torque drop | 0.174 m/s | 9.7 °/s | 1.11 s |
| RUN torque stand | 0.092 m/s | 60.5 °/s | **none** |

It cuts the handover disturbance 3–5× and roughly doubles survival — **but no clip passes
under it**, and it flips WALK's mapping verdict to a failure. So it is added but left
**non-default**, and the question of what the right initial condition is stays open. What
is now permanent is that handover speed, angular rate and loaded-foot count are printed and
warned on for every run, so no verdict can be read without the state it started from.

## Ground friction — 1.3 confirmed, but it is a product of two materials

The value is confirmed as proposed: **1.3, the midpoint of the env's own
`friction_range = [0.6, 2.0]`**, chosen because it is the least arbitrary fixed point
inside the distribution the E2E policy trains and is evaluated on, so both arms of the
comparison stand on the same ground; because a step-clearance threshold measured on a
randomised floor would fold friction variance into a geometric answer; and because it comes
from the env config rather than from anything measured here. It is read from the config at
runtime (`Go2Config.ground_friction`), not hardcoded — so the commented-out alternative
`friction_range = [0.2, 2.]` in that file cannot be picked up by accident.

The §4 audit then found that setting it was not enough. The env's terrain material is
static 1.0 / dynamic 1.0 / restitution 0.0 with **both combine modes set to `multiply`**
(`legged_robot.py::_create_trimesh`); Isaac Lab's default is `average`. The env's
randomisation writes its sampled coefficient to the **robot's** shapes, so the effective
friction is a *product of two materials* and the harness was only ever setting one:

| | ground | robot | combine | effective mu |
|---|---|---|---|---|
| before | 0.5 (Isaac default) | 0.50 (from `go2.usd`) | average | **0.5** |
| first patch | 1.3 | 0.50 | average | **0.9** — still wrong |
| now | 1.0 (env's terrain) | 1.3 (range midpoint) | multiply | **1.3** ✓ |

`go2.usd` shipping 0.50 was measured at runtime, not assumed. Applied in
`verify_skill_replay.py` and `run_calibration.py` through shared helpers
(`sim/replay.py: ground_material_cfg`, `set_robot_friction`) so the two cannot drift.

## §4 audit — the remaining settings

`scripts/audit_sim_settings.py` (new) reads Isaac Lab's defaults from the live dataclasses
and the env's values from the upstream AST, so neither side is a recollection. Full table
in `outputs/sim_settings_audit.json`. Eight settings where the env departs from the Isaac
default:

| setting | env | Isaac default | status |
|---|---|---|---|
| `dt` | 0.005 | 0.0167 | fixed day 1 (§5) |
| `static_friction` / `dynamic_friction` | 1.0 | 0.5 | **fixed today** |
| `friction_combine_mode` | multiply | average | **fixed today** |
| `restitution_combine_mode` | multiply | average | **fixed today** |
| `max_position_iteration_count` | 4 | 255 | see below |
| `max_velocity_iteration_count` | 0 | 255 | see below |
| `render_interval` | 4 | 1 | no effect headless |

The two iteration counts are a **global clamp**, not the solver setting itself. The
per-articulation counts come from `UNITREE_GO2_CFG` (`solver_position_iteration_count=4`,
`solver_velocity_iteration_count=0`), which the harness inherits by importing that config,
and the clamp is not binding at either value. Recorded as a difference, not a defect — but
it should be set explicitly if the harness ever builds its own `PhysxCfg`.
`bounce_threshold_velocity`, `gravity`, `restitution` and `solver_type` already match.

## Step 4 (RUN) — re-judged, FAIL in both modes

Run at friction 1.3, 30 N, `--hip-sign keep`, both the torque mode and the `--mode position`
negative control:

| | `--mode torque` | `--mode position` (control) |
|---|---|---|
| terminated | 1.11 s | 1.11 s |
| stride | 5.81 Hz vs 3.09 expected | 4.49 Hz vs 3.09 |
| forward speed | −0.130 m/s vs log 0.514 | −0.157 m/s |
| mapping | best under `diagonal_swapped` (r=0.078) | identity, margin 0.087 |

Both fail, and they fail the same way at the same time — which is the negative control
doing its job: it says the torque path is not what is breaking RUN. RUN also has the worst
initial condition of the three clips (no frame with four feet down), and terminates 0.27 s
after the handover disturbance. **Step 4 is blocked on the settle question, not on gains.**

## Step 5 (calibration) — not run, and it could not have been

The gate was "proceed if 1 and 2 pass". Neither passed, so the sweep was not run: a
360-episode step-clearance sweep now would measure the settle, not the skills.

The `--max-probes 3` smoke was still run as a code-path check on the files edited today,
and it turned up something worth knowing before the gate ever opens: **the calibration loop
cannot complete a second episode.** Three failures in sequence (`harness_findings.md` §8) —
`TerrainImporter` missing `env_spacing`, so episode 1 never ran at all; then a prim-path
collision on episode 2; then, once the stale prims are cleared,
`Failed to create articulation at /World/Robot/base`, because Isaac Lab 3.0 keeps the old
`Articulation` registered with the physics manager after its prim is deleted.

The third is not a patch, it is a redesign: build terrain and robot once and per episode
rewrite only the root state and the probe mesh, or run one process per probe. The first two
are fixed; the script now prints the diagnosis and refuses to pretend a multi-episode sweep
is under way. **Whichever way the settle question is decided, this has to be rebuilt before
step 5 can produce a number.**

## Where to pick up

1. **The initial condition is the blocker.** `stand` improves it 3–5× without fixing
   anything, so the question is what the right protocol *is* — and it must be decided
   before calibration, because the E2E arm has to be evaluated from the same one. This is
   a protocol choice with comparison consequences, so it is left for you rather than
   settled here.
2. **Then re-judge TROT and RUN** under whatever that protocol is.
3. **Rebuild the calibration episode loop** (§8) — it has never run more than one episode.
   Then step 5, smoke first, then the sweep. The earlier "~30 min on CPU" estimate came
   from a loop that was never executing, so treat it as unmeasured.
4. **Re-run everything base-derived on GPU at least once.** Defect §6 cannot occur there,
   so a GPU run is an independent check on every number in this file.

Settled, not to be re-litigated: the physics rate; the contact threshold (30 N); the
actuator probe answers; the archive hashes; **the convention is `keep`**; **effective
friction is 1.3, as a product of a 1.0 ground and 1.3 robot shapes under `multiply`**.
