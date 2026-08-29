# Why RUN collapses in 1.13 s, and why foot placement does not move it

**Status: unresolved, to be retried. Not closed.** RUN is out of the executable skill
set (`planner.skills.SUPPORTED`) and the planner refuses it with this reason at
runtime. §6 lists what would have to be true for it to come back.

Stage 2's foot placement held TROT for 60 cycles and did not change RUN by a single
control step: 1.13 s either way. That is not a null result to shrug at — it says the
thing RUN is missing is not foot placement. This is what it is instead.

Everything here is read from traces already on disk (`scripts/diagnose_run_collapse.py`).
No simulator, no GPU.

---

## 1. How it falls: roll, from the first cycle, with one leg never loaded

| | first half | second half | at termination |
|---|---|---|---|
| \|roll\| mean | 6.45° | 27.85° | **+61.0°** (the 60° gate) |
| \|pitch\| mean | 11.24° | 22.71° | +4.1° |
| base height | 0.302 m (start) | — | **0.163 m** |

Both axes grow, roll faster, and roll is what terminates it. The base **sinks
monotonically** from 0.302 m to 0.163 m — it is not a robot that trips, it is a robot
that never gets its weight up.

Per-foot load over the 1.13 s, with the 30 N threshold the gait numbers use:

| leg | first sustained unload | peak | mean | loaded |
|---|---|---|---|---|
| FL | 0.51 s | 93.0 N | 34.1 N | 45.6% |
| FR | 0.30 s | 78.0 N | 31.1 N | 50.9% |
| **RL** | 0.30 s | 73.6 N | **12.6 N** | **19.3%** |
| RR | 0.22 s | 90.5 N | 29.9 N | 40.4% |

The rear-left leg carries a third of what the others carry and is loaded a fifth of the
time. The support polygon is missing a corner for most of the run, which is a roll
divergence waiting to happen.

---

## 2. The flight phase does not exist in the sim, and that is the mechanism

RUN is duty 0.31 with `flight_frac` 0.28 in the log — a running trot that leaves the
ground. Swing-bout apex clearance, measured per bout as the foot's height above the
floor (5th percentile of all foot heights):

| leg | RUN apex (median) | TROT at cap 0.05, for contrast |
|---|---|---|
| FL | **2.5 mm** | 58.9 mm |
| FR | **1.7 mm** | 60.2 mm |
| RL | 9.2 mm | 58.9 mm |
| RR | 9.0 mm | 58.5 mm |

**RUN's front feet never leave the floor.** They scuff along at 1.7–2.5 mm while the
clip's schedule says they are in the air. The working TROT lifts all four by ~59 mm,
consistently (p10 54 mm, p90 65 mm).

This is the "a clip stores joint angles and no base trajectory" problem in its sharpest
form. On the real robot the flight phase is a fact about the BASE: the body is ballistic,
the hips are high, and a leg that is only modestly retracted still clears the ground. In
the replay the base never becomes ballistic — it sinks instead — so the same joint
angles put the foot on the floor. **The flight phase is not in the clip to be replayed.**

(Care is needed reading foot heights: sampling the trace at fixed intervals aliases
against the gait and made TROT's feet look like they were dragging at 0.8–1.3 mm too.
The numbers above are per swing bout, which is the honest statistic. TROT is fine.)

---

## 3. So the swing gate is pointing at loaded feet

Foot placement is defined on a foot that is in the air. For RUN it is not:

- the clip's swing schedule and the sim's own contacts **agree on only 59.6%** of
  leg-steps (the clip says swing 66.2% of the time, the sim's contacts say 61.0%)
- the correction is therefore applied, for a large fraction of its samples, to a foot
  that is bearing load, where moving the hip target does not move a footfall — it
  scrubs a loaded foot sideways

That is why the compensator cannot help RUN and why it changes the collapse time by
nothing. It is not that foot placement is too weak here; it is that its precondition is
absent.

---

## 4. Torque mode was not skipped — it is what already ran

RUN's clip is not position-controlled (kp 13/3/2, kd 1.8/2.0/1.5, calf feed-forward
11.79 Nm RMS), so `default_mode_for` requests TORQUE for it automatically. Both RUN rows
in `outputs/foot_comp.csv` say `mode_requested=torque, mode_used=torque` with 57 gain
writes — one per control step — and the trace confirms the torque actually arrived:

| joint | applied in sim, RMS | peak | effort limit | log tau_ff RMS |
|---|---|---|---|---|
| hip | 3.60 Nm | 10.24 | 23.7 | 3.67 |
| thigh | 5.27 Nm | 13.55 | 23.7 | 2.59 |
| calf | **14.82 Nm** | 29.13 | 45.43 | 11.79 |

`torque_saturated_frac` is 0.000 and the peaks are well inside the limits. The log's own
gains and its feed-forward torque are being applied, nothing is being clipped, and the
robot still collapses in 1.13 s. **The replay mode is not the explanation.**

---

## 5. Two further defects of the RUN clip itself

Neither is about the compensator, and both are measured in the archive meta:

- **Loop seam.** `loop_seam_over_max_step` is **1.34** for RUN against 0.15 for TROT: the
  discontinuity where the one-cycle clip loops is 1.34x larger than the biggest step
  inside the cycle. At 3.09 Hz that is a jolt three times a second, every second.
- **Sampling.** 16 samples per cycle at the `lo` rate, with `alias_energy_frac_above_25hz`
  0.0126 — three times TROT's. The clip's own `q_des_update_hz` is 165 Hz, so the
  50 Hz replay is resampling a signal that was updating three times faster.

**The stride numbers for RUN carry no weight and should not be quoted.** A 1.13 s run at
3.09 Hz contains three touchdowns, and the stride comes from the *median touchdown
interval*, so every RUN row here is built on `n_cycles = 1` — a single interval — or on
none at all: `run_off` reports 2.47 Hz from one gap, `c_run_yawlog` reports 3.09 Hz from
one gap, and `run_cap005` reports nothing. The spread between those two is sampling
noise, not a difference between the runs. RUN is not producing enough cycles to be
measured as a gait, which is itself the finding.

(Separately: the table script had RUN's expected stride hardcoded at 2.47 — a *measured*
value copied into the expectations column — which would have made a slow replay read as
exact. The harness's own verdict was never affected; it reads `expected_from_meta`. The
table script now reads the meta as well.)

---

## 6. What would have to be true for RUN to replay

In order, cheapest first:

1. **The base has to leave the ground.** Nothing in the joint trajectory makes that
   happen on its own; it needs the push-off impulse to land while the legs are loaded,
   which is a timing question between the clip's phase and the sim's contact state —
   not a foot-placement question. A vertical-impulse compensator (stage 3?) is the
   shape of the answer, and it is a different mechanism from this one.
2. **The seam has to stop jolting.** A one-cycle clip with a seam 1.34x its own largest
   step is being asked to loop 60 times. Either the cut moves or the clip is played from
   a longer, uncut source (`extract_full_sessions.py` exists for this).
3. **The swing gate has to come from the sim, not the schedule** — `--foot-swing-source
   sim` — because for a gait this far from reproducing, the recording's labels describe
   a different robot's feet.

Until (1), foot placement has nothing to place: there is no flight, so there is no
footfall to choose.
