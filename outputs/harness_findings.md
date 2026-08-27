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

## The pattern

| # | defect | exit code | visible? |
|---|---|---|---|
| 1 | duplicate `--device` | 1, then 0 | loud once, then swallowed by `except Exception` |
| 2 | write after `app.close()` | 0 | **silent** — no file, no recommendation |
| 3 | `close()` before metrics | 0 | **silent** — no verdict, no CSV |
| 4 | `close()` before `return rows` | 0 | **silent** — no report after a 2–3 h sweep |
| 5 | physics at 50 Hz not 200 Hz | 0 | **worse than silent** — a confident wrong diagnosis |

Offline code review found none of these, and could not have: every one is about the
behaviour of an API at runtime, not the shape of the source. What found them was running
the thing and asking whether the output that was supposed to appear had appeared.

Defect 5 also shows the cost asymmetry. 1–4 waste minutes. 5 hands you a plausible,
specific, internally consistent explanation pointing at the wrong subsystem — and
`server_day1.md`'s own branch table would have routed the reader straight into it.
