# Server, day 1

Everything on this page can be done in one sitting. It is ordered so that each step's
failure is *diagnosable* with what the previous steps established — the point is never to
get to the end, it is to know exactly where you stopped and why.

Three things were already settled without a simulator, and day 1 should not re-litigate
them:

| settled | where |
|---|---|
| control rate is **50 Hz** (`dt 0.005 × decimation 4`), matching the clips' `lo` rate | `outputs/gain_feasibility.md` |
| the Go2 actuators are **explicit** `IdealPDActuator`, kp 40 / kd 1 — and the repo already writes gains at runtime | same |
| the **hip sign is probably flipped** between the log and the articulation; thigh and calf agree | `verify_skill_replay.py --convention` |

Two things are expected to bite and are not bugs:

* **RUN and JUMP exceed the sim's effort clip** by 10–20 % (`--headroom`). The sim robot is
  weaker than the one that produced the clips. Do not raise `effort_limit`.
* **WALK and TROT are the easy case.** Their recorded gains *are* the config's gains, so a
  position replay is faithful. If they fail, the cause is convention, not gains.

---

## 0 · Environment (15 min)

```bash
conda env create -f environment_eureka_lab6.yml     # in the eurekaverse repo
python -c "import isaaclab, isaacsim; print(isaaclab.__version__)"
cd ~/projects/go2-planner
python scripts/freeze_benchmark.py --check          # READ-ONLY. never --verify
python scripts/freeze_calibration.py --verify
python scripts/extract_skill_clips.py --verify
```

**Success:** three archives report OK, `isaaclab` imports.
**If the benchmark hash mismatches:** stop. `data/upstream_commit.txt` records a known
container rewrite on 2026-08-26 whose `content_sha256` was unchanged; compare
`content_sha256`, not the container hash, before concluding anything is wrong.

---

## 1 · Actuator probe (5 min) — *do this before anything else*

```bash
python scripts/probe_isaac_actuator.py --headless
```

Answers the five API questions `gain_feasibility.md` could not settle offline and writes
`outputs/isaac_actuator_probe.json`. Every later branch depends on it.

| result | means | go to |
|---|---|---|
| Q2 gain write **works**, Q3 effort **additive** | full fidelity available | step 2, use `--mode torque` for RUN |
| Q2 fails, Q3 additive | fallback (a) | step 2, use `--mode fixed-gain` |
| Q3 not additive / absent | no feed-forward path at all | step 2 for WALK/TROT only, then **jump to step 6** |
| Q5 plateau ≠ 23.7 / 45.43 Nm | the effort clip is not what the config says | re-read `--headroom`; the torque analysis is invalidated |

The script prints its own recommendation at the end. Trust that over this table if they
disagree — it saw the machine.

---

## 2 · Convention, on RUN first (20 min)

Not WALK. The mapping search separates candidates by **0.18** on RUN and JUMP and only
**0.04** on WALK and TROT — the slow gaits are too symmetric to tell a mirrored mapping
from the right one, and `diagnose.py` will say so with a "too symmetric" warning.

```bash
python scripts/verify_skill_replay.py --convention          # offline, re-read the prediction
python scripts/verify_skill_replay.py --clip RUN --mode torque --headless
python scripts/verify_skill_replay.py --clip RUN --mode torque --hip-sign flip --headless
```

**Success:** the run with the better verdict identifies the convention. The mapping search
should report `identity` as best with a margin above 0.05.

| symptom | branch |
|---|---|
| `--hip-sign flip` is clearly better | the offline prediction was right. Use `flip` everywhere below and record it. |
| both equally bad, mapping search picks a *swapped* permutation | DOF indices are wrong. The script resolves them by name — check the printed `clip joint -> DOF index` map against `robot.joint_names`. |
| robot collapses immediately in both | thigh/calf sign or zero offset, not hip. The offline check says thigh and calf agree, so suspect `default_joint_pos` handling in the settle phase. |
| mapping search warns "too symmetric" | you are on WALK or TROT. Go back to RUN. |

Then confirm on JUMP, which is the other high-margin clip.

---

## 3 · WALK and TROT baseline (20 min)

```bash
python scripts/verify_skill_replay.py --clip WALK --mode position --headless
python scripts/verify_skill_replay.py --clip TROT --mode position --headless
```

These carry no gain question at all: kp 40 / kd 1, `tau_ff = 0`, which is the config
exactly. A clean `PASS` here means the pipeline — DOF mapping, rate, contact measurement,
gait metrics — is sound, and every later failure is about torque rather than plumbing.

**Success:** stride within 15 % of 1.37 Hz (WALK) / 1.56 Hz (TROT), duty within 0.10,
verdict `PASS` or `WARN`.
**If these fail after step 2 settled the convention:** the problem is the replay harness,
not the clips. Check the contact threshold (`--contact-threshold-n`) before anything else —
a duty mismatch alone is usually a threshold, not a gait.

---

## 4 · RUN, the torque-driven case (30 min)

```bash
python scripts/verify_skill_replay.py --clip RUN --mode torque --headless
python scripts/verify_skill_replay.py --clip RUN --mode fixed-gain --headless
python scripts/verify_skill_replay.py --clip RUN --mode position --headless   # expected to fail
```

The third is run *deliberately* as a negative control. If `position` reproduces RUN as well
as `torque` does, the gain schedule was not the issue and the analysis in
`gain_feasibility.md` needs revisiting.

**Expect it to fall short on forward speed regardless.** RUN's thigh torque peaks at 1.16×
the sim's clip; watch `torque_saturated_frac` in `outputs/replay_verify.csv`. Above ~2 % the
sim robot is torque-limited and the shortfall is physics, not the clip.

| result | branch |
|---|---|
| `torque` clearly beats `position` | as designed. Record the mode with the numbers. |
| `torque` and `fixed-gain` are equal | expected — the gains are near-constant within a skill. Use `fixed-gain`, it is simpler. |
| both fail, `torque_saturated_frac` high | effort clip. Not fixable without changing the robot. Record it and go to step 6 for RUN specifically. |
| both fail, saturation low | the trajectory does not transfer. Go to step 6. |

---

## 5 · Calibration (2–3 h, mostly unattended)

```bash
python scripts/run_calibration.py --plan                     # 360 episodes; sanity-check first
python scripts/run_calibration.py --headless --max-probes 3  # smoke: 3 levels per family
python scripts/run_calibration.py --headless                 # the real sweep
```

The smoke run exists so a typo costs three minutes rather than three hours. Do not skip it.

**Success:** `outputs/calibration_report.md` with a value for each of the five covered
parameters and `monotone = yes`.

| result | branch |
|---|---|
| a parameter is **non-monotone** | the script already refuses to promote it to `MEASURED`. Read the raw pass matrix; something other than the level is deciding. |
| `STEP_JUMP_MAX` comes out below 0.10 m | probably the effort clip, not the skill — JUMP is over the limit on two joint groups. Cross-check `torque_saturated_frac`. Report it as a sim-robot limit, which is the correct thing for the planner anyway. |
| everything fails at the smallest level | the replay is not walking. You should not be here; go back to step 3. |

Five of the ten `CALIBRATION_NEEDED` parameters are **not** covered — the slope and
roughness ones need ramp and roughness probes that are deliberately not in the frozen
archive. They stay `CALIBRATION_NEEDED` after a successful day 1. That is expected.

---

## 6 · If the replay approach is dead

Reached when step 1 says there is no feed-forward path, or step 4 fails with low
saturation. It kills RUN and JUMP; **it does not kill WALK and TROT**, which never needed
anything beyond position control. Keep them.

Fallback (c) from `gain_feasibility.md`: a parameterised gait generator — contact schedule
(period, duty, phase offsets) plus a swing trajectory, with parameters read from
`outputs/skill_profile.csv` rather than invented. The clips already carry the measured
contact signal, so the *timing* is in hand; only the foot trajectory needs building.
`extreme-parkour/legged_gym/legged_gym/scripts/record_gait.py` does the same thing from a
trained policy and is the template.

It has no convention problem, no gain problem and no torque problem, because it never
replays a recorded trajectory. It costs the most and it is the robust answer.

**Do not** work around a dead replay by raising `effort_limit`, by tuning gains until the
gait looks right, or by excepting RUN out of the skill set. The first two calibrate a robot
the evaluation never runs; the third is ruled out by `CLAUDE.md` §2 — a skill that cannot
be reproduced should score zero, and "tried and failed" is a result.

---

## Stop points

Honest places to stop, in order of how much day 1 still bought:

1. **Through step 5.** Calibration done, five placeholders promoted.
2. **Through step 4.** Replay verified; calibration is unattended and can run overnight.
3. **Through step 3.** WALK and TROT transfer. Half the skill library is usable and the
   harness is proven; RUN and JUMP are open.
4. **Through step 2.** The joint convention is settled — the single biggest unknown, and
   it unblocks everything else whenever it is picked up again.
5. **Through step 1.** The five API questions are answered. Small, but it is the branch
   point for every plan above and it cannot be answered off the server.

Stopping at 4 or 5 is a good day. Stopping at 1 with the probe JSON committed is still
strictly better than the state before, because the guesswork is gone.
