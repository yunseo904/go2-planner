# Harness defects — what breaks when code written without a simulator meets one

Server day 1, 2026-08-27, z4. Every script in `scripts/` that touches Isaac Lab was
written on a machine that had no Isaac Lab. This records what failed at first contact,
because the failure modes have a shape worth recognising: **four of the five were silent.**
The scripts exited 0. Two of them printed a full, confident, wrong diagnosis.

Environment: `nvcr.io/nvidia/isaac-lab:3.0.0-beta2-post1` — Isaac Sim 6.0.1-rc.7,
isaaclab 6.1.16, torch 2.10.0+cu128. Run via `scripts/isaac_docker_run.sh`; Isaac Lab is
not installed natively on this box.

---

## 1 · `--device` declared twice — *loud in one script, silent in two*

`probe_isaac_actuator.py`, `verify_skill_replay.py`, `run_calibration.py` each did:

```python
ap.add_argument("--device", default="cpu")
...
AppLauncher.add_app_launcher_args(ap)      # also adds --device
```

Isaac Lab 6.x raises `ValueError: The passed ArgParser object already has the field
'device'`. `probe_isaac_actuator.py` had no guard and died visibly. The other two wrapped
the call in `except Exception:` and fell through to a branch that adds only `--headless` —
so they ran on an argparse that had **silently lost every AppLauncher flag**, and would
have handed `AppLauncher(args)` an args object missing the fields it reads.

Fix: `AppLauncher` owns `--device`; re-assert the CPU default with `ap.set_defaults`.

> A bare `except Exception` around third-party setup converts a version incompatibility
> into a wrong run. The fallback branch was written for "Isaac Lab is not installed",
> and it also caught "Isaac Lab is installed and disagrees with you".

---

## 2 · `probe_isaac_actuator.py` wrote its output after `app.close()`

```python
    app.close()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(R, ...))     # never runs
    print("\n== what this means for the replay mode ==")   # never runs
```

`SimulationApp.close()` tears the process down. The probe printed all five answers to the
terminal, exited **0**, and produced no `outputs/isaac_actuator_probe.json` and no
recommendation. `server_day1.md` describes both as things the script does — reading the
doc rather than the run would not have caught it.

Fix: everything that produces output happens before `app.close()`.

---

## 3 · `verify_skill_replay.py` closed the app before computing its metrics

`simulation_app.close()` sat in the middle of `run_isaac`, immediately after the stepping
loop and **before** the gait metrics were computed and returned. So: no diagnosis, no
verdict, no `outputs/replay_verify.csv`, exit 0. The run looked like it had worked.

## 4 · `run_calibration.py` closed the app one line before `return rows`

Same defect, same consequence: `main()` never receives the rows, so no results CSV and no
`calibration_report.md` — after a sweep that `server_day1.md` budgets 2–3 hours for.

Fix for both: the app is held in a module global and closed once in `main()`, in a
`finally`, after the results are written.

> 2, 3 and 4 are one mistake made three times: *treating `close()` as cleanup*. It is not
> cleanup, it is termination. Written against the docs it reads as a teardown call; the
> first run on a real machine is the only thing that says otherwise.

---

## 5 · PhysX integrated at the clip rate instead of the config rate — **the expensive one**

```python
dt = 1.0 / clip["fs"]                                  # ~0.0198 s
sim = SimulationContext(SimulationCfg(dt=dt, ...))     # PhysX at ~50 Hz
... one sim.step() per clip sample
```

`legged_robot_config.py` — the config this entire comparison is measured against — is
`sim_dt 0.005` × `decimation 4`: **200 Hz physics, 50 Hz control**. The harness ran contact
integration four times coarser than the robot it claimed to replay. The quadruped went
down on its belly on flat ground regardless of clip, replay mode or joint sign.

A/B, WALK, position mode, everything else held fixed:

| | 1 step per sample (~50 Hz physics) | 4 steps per sample (~202 Hz) |
|---|---|---|
| settle, start → end | 0.416 → **0.204 m** | 0.418 → **0.317 m** |
| base height, mean over replay | 0.169 m | **0.316 m** |
| outcome | **falls at 1.40 s**, 72 / 296 steps | **completes**, 296 / 296 |

Nominal base height is 0.32 m. The 1.40 s matches the full WALK run's fall time exactly, so
the probe reproduced the harness and not something adjacent.

**This one lied about its cause.** The diagnosis printed under the broken dt was:

* `WALK` → *"stride 8.43 Hz vs the clip's 1.37 Hz (+517%)"*, *"walks backward"*
* `RUN`  → *"stride 24.69 Hz vs 3.09 Hz (+700%)"*
* `TROT` → *"matches best under `left_right_swapped + thigh/calf sign flipped` (r=0.329),
  not under identity"* → *"the clip is being written into the wrong DOF indices, or with
  the wrong sign"*

All of it downstream of contact chatter from an unstable integrator. After the fix, WALK's
stride reads 1.37 Hz against 1.37 Hz expected and TROT's best mapping is `identity`. Acting
on the first diagnosis would have meant a day spent auditing DOF indices and joint signs
that were correct all along.

Fix: control period from the clip, physics step from the config — `phys_dt = ctrl_dt /
decimation`, holding the command across the substeps, which is what decimation means in the
env. Both numbers read from `legged_robot_config.py`, neither hard-coded.

`run_calibration.py` cannot use the clip's period: a `SimulationContext` takes its `dt`
once and the clips do not share a rate. It uses the config's `sim_dt` as the physics step
and resolves the substep count per clip, leaving under 1.3 % control-rate error.

---

## Related: a "verification" flag that overwrites the thing it verifies

Not a harness defect, but the same family — `server_day1.md` step 0 prescribes:

```bash
python scripts/freeze_calibration.py --verify
```

`--verify` rebuilds the archive twice, asserts the two agree, and then calls `save()`
**unconditionally**. There is no `--check` and no `--force` guard. It overwrites
`data/calibration_probes.npz` and rewrites `data/calibration_probes.sha256` from the file
it just wrote, so the verification it appears to perform becomes vacuous. This is exactly
the 2026-08-26 `freeze_benchmark.py --verify` incident recorded in
`data/upstream_commit.txt`, in a script that never received that incident's guard.

Day 1 was run with a read-only equivalent instead: rebuild twice in memory, compare
`content_sha256` to each other and to the archive on disk, write nothing.
`ae467d86…` three ways — deterministic, and matching what is committed.

---

## 6 · Every base-derived metric was the final step, repeated — **the one that invalidated day 1**

`verify_skill_replay.py` recorded the base state like this:

```python
rec["root_pos_w"].append(robot.data.root_pos_w[0].cpu().numpy())
```

`robot.data.root_pos_w[0]` is **basic** indexing, so it returns a *view* into a buffer
PhysX overwrites in place every step. On `--device cpu`, `.cpu()` is a no-op and
`.numpy()` shares memory. So all N appended "samples" alias one buffer, and the array
reads back as **the last control step repeated N times**.

Measured on a 42-step TROT replay — distinct rows out of 42:

| array | how it was indexed | distinct rows |
|---|---|---|
| `q` | `[0, idx_t]` — advanced | 42 / 42 |
| `tau` | `[0, idx_t]` — advanced | 42 / 42 |
| `contact` | `(f > thr)` — new tensor | 7 / 42 (correct) |
| `root_pos_w` | `[0]` — **basic** | **1 / 42** |
| `root_quat_w` | `[0]` — **basic** | **1 / 42** |
| `root_lin_vel_b` | `[0]` — **basic** | **1 / 42** |
| `root_ang_vel_b` | `[0]` — **basic** | **1 / 42** |

Advanced indexing allocates, which is the only reason `q` and `tau` escaped — the
distinction between `[0, idx]` and `[0]` decided which half of the harness was real.

What this corrupted:

* `base_height_mean_m` was not a mean. It was the height at the final step — which, on a
  run that terminates by falling, is *by construction* the height of a fallen robot. Every
  clip therefore reported "the robot is on its belly", including clips that never fell.
* `vx_mean`, `vy_mean`, `yaw_rate_deg_s` were single instantaneous values.
* The mapping ranking (identity vs hip-flipped vs diagonal-swapped) correlates commanded
  against measured joint angles. `q` was fine — but the verdicts it fed were read next to
  base metrics that were not.

This is what produced the day-1 hip-sign conclusion and then the day-1 *contradiction* of
that conclusion. Both readings were of the same corrupted channel. Fixed by copying out
of the sim (`snap()`), plus a guard that refuses to report numbers when a base array is
bit-identical across every step, because a robot whose base never moves by a micron over
hundreds of steps is a recording bug, not a stiff robot.

**This defect is invisible on GPU.** `.cpu()` copies when the source is CUDA. Every
harness run so far has been `--device cpu`.

---

## 7 · The initial condition was never a measurement, and it decided the verdict

Not a coding defect — a design one, found only because 6 made the base readable.

The settle wrote the base to the env's spawn height (0.42 m), overrode the joints to the
clip's **first frame** — a mid-gait pose with one or two feet in swing — and let the robot
fall into it, holding for a fixed 0.5 s wall clock with no convergence check. State handed
to the replay's first step:

| clip | \|v\| at handover | \|w\| at handover | feet loaded | outcome |
|---|---|---|---|---|
| WALK | 0.121 m/s | 6.2 °/s | 4 / 4 | survives, stride exact |
| TROT | 0.365 m/s | 51.6 °/s | **3 / 4** (FR at 0 N) | collapses at 0.82 s |
| RUN  | 0.19 m/s | 12.6 °/s | **3 / 4** (FL at 0 N) | collapses at 1.11 s |

Every clip that failed began its first control step with a foot already unloaded and the
base already tumbling. The failure was in the initial condition, not in the clip.

`--settle-mode stand` was added as an A/B: settle on the default standing pose the 0.42 m
spawn height is actually designed for, then drive to the clip pose with the PD while the
robot is already up. It cuts the handover disturbance 3–5× (TROT 0.365 → 0.072 m/s,
51.6 → 23.2 °/s) and roughly doubles survival (TROT 0.82 → 1.57 s, RUN torque no longer
terminates early) — **but it does not make any clip pass**, and it flips WALK's mapping
verdict to a failure. It is left non-default and unresolved.

The durable part is that handover speed, angular rate, and loaded-foot count are now
reported on every run and warned on, so no future verdict can be read without seeing the
state it started from.

---

## 8 · `run_calibration.py` cannot complete a second episode — the sweep never ran

Found by the `--max-probes 3` smoke, which was the first time this script's Isaac path was
executed at all. Three failures, in sequence, each hiding the next:

1. `TerrainImporter(TerrainImporterCfg(prim_path=..., terrain_type="plane"))` raises
   `ValueError: Environment spacing must be specified` — it configures grid-like env
   origins in `__init__`. Needs `num_envs` and `env_spacing`. **Episode 1 never ran.**
2. With that fixed, episode 1 completes (`STEP_WALK_MAX WALK step_up_0p02 fail x=0.74
   t=1.7s`) and episode 2 dies: `A prim already exists at path: '/World/ground/terrain'`.
   The loop rebuilds terrain and robot at fixed prim paths every episode.
3. Clearing the stale prims from the USD stage gets past that and straight into
   `RuntimeError: Failed to create articulation at: /World/Robot/base`. The previous
   `Articulation` is still registered with the physics manager after its prim is gone, and
   re-initialises against a dead path on the next `sim.reset()`.

(3) is not a bug to patch — Isaac Lab 3.0 does not support building and tearing down a
scene per episode inside one `SimulationContext`. The loop needs redesigning: either build
terrain and robot once and per episode rewrite only the root state and the probe mesh, or
run one process per probe from a shell loop.

The script has been left failing fast with that diagnosis printed up front, because the
alternative is a sweep that looks like it is working for hours. Note the interaction with
defect 4: had `close()` still been swallowing the traceback, all three of these would have
presented as *exit 0 with no CSV*.

---

## Self-inflicted recurrences, same session

Three of the five original defects reappeared in code written *during* the diagnosis:

* `contacts.find_bodies(".*_foot")` returns indices into the **sensor's** body list
  (0..3), not the articulation's. Used against `robot.data.body_pos_w`, it silently
  recorded the base and the hips as "feet" — caught because foot z was bit-identical to
  base z.
* `root_quat_w` was read as `(w,x,y,z)`; it is `(x,y,z,w)`. The wrong order turned a
  near-identity attitude into a "robot facing backwards at −166° yaw" that would have been
  a compelling and entirely fictional finding.
* `audit_sim_settings.py` put `app.close()` in a bare `finally`, so an `ImportError`
  exited 0 with no traceback — defect 2, verbatim, in a script written to catch defects.

The lesson is not that these are avoidable with more care. It is that every one was caught
by the same move: checking that the numbers were *possible* before interpreting them.

---

## The pattern

| # | defect | exit code | visible? |
|---|---|---|---|
| 1 | duplicate `--device` | 1, then 0 | loud once, then swallowed by `except Exception` |
| 2 | write after `app.close()` | 0 | **silent** — no file, no recommendation |
| 3 | `close()` before metrics | 0 | **silent** — no verdict, no CSV |
| 4 | `close()` before `return rows` | 0 | **silent** — no report after a 2–3 h sweep |
| 5 | physics at 50 Hz not 200 Hz | 0 | **worse than silent** — a confident wrong diagnosis |
| 6 | base metrics aliased one buffer | 0 | **worse than silent** — every clip "on its belly", and a convention conclusion built on it |
| 7 | settle handed over a tumbling robot | 0 | **worse than silent** — the initial condition read as a gait result |
| 8 | calibration cannot run 2 episodes | 1 (after 4 was fixed) | loud only because 4 was fixed first |

Offline code review found none of these, and could not have: every one is about the
behaviour of an API at runtime, not the shape of the source. What found them was running
the thing and asking whether the output that was supposed to appear had appeared.

Defects 5, 6 and 7 share a shape the first four do not: they do not withhold an answer,
they supply a wrong one that is internally consistent. 6 is the worst of them, because the
number it corrupted — base height — is the one the harness uses to decide whether a run is
worth interpreting at all. Two days of conclusions about a joint-sign convention were
drawn from it, in both directions.

The generalisation for the rest of this project: **an Isaac Lab buffer read is not a
sample until it is copied**, and on CPU nothing in the type system says so. The check that
costs nothing is asking whether a recorded array actually varies before believing what its
mean says.

Defect 5 also shows the cost asymmetry. 1–4 waste minutes. 5 hands you a plausible,
specific, internally consistent explanation pointing at the wrong subsystem — and
`server_day1.md`'s own branch table would have routed the reader straight into it.
