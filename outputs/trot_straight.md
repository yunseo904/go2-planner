# TROT and the line it will not hold: what more authority buys, and what actually helps

Three questions were asked, all of them prerequisites for `STEP_TROT_MAX`:

1. is there a route to more heading authority other than raising the ±0.02 cap;
2. can the 84–87% cap saturation be reduced *itself*, rather than by widening the cap;
3. can mechanisms (a) and (b) be split by the sign of the heading error.

Short answers: **(3) yes, and it is now built; (1) yes, and as a proportional term it does
nothing; (2) no, and the reason is that saturation is set by the disturbance, not by the
actuator.** A fourth thing — feeding the bias forward as a constant — looked like a large
win on one cell and **does not survive a paired test across fifteen**; §4 is the measurement
and §4b is why the first reading of it was wrong.

All CPU, `--device cpu`, no GPU.

---

## 0. First, two measurement corrections, because they change what the numbers mean

**The whole-run curvature was reading the fall.** TROT's terminal tumble adds 30–50° of
yaw in half a second. Measured whole-run, condition A came out at 4.71 °/m and two
conditions had opposite-signed final yaw (+19.3 vs −16.1) and looked like different
mechanisms. Trimmed to one second before the fall they are 3.44 and 3.67 and were tracking
each other the whole way. Every number below is over the **controlled window** — up to one
second before the base drops through 0.15 m. This is the CLAUDE.md §6.5 trap again: total
yaw over the run is *almost* the controller's heading error.

**Trimmed, the grid and the 60-cycle rig agree.** `heading_hold.md` measured TROT at
**3.20 °/m** on flat with `verify_skill_replay.py`; the probe grid measures **3.44 °/m** on
the flat approach with the same controller. Two harnesses, two terrain representations
(analytic plane vs triangulated mesh with the infinite plane deleted), one number. **The
grid is not degrading TROT**, which had to be established before anything here could be
attributed to control.

---

## 1. Question 3 first, because it decides the shape of the answer

**Yes, and it is not optional — the split is forced by the measurement.**

`heading_candidates.md` §2 measured mechanism (b), differential step length on the thighs,
as **strong in one direction and destructive in the other** on TROT: −0.04 rad removes 4.71
of the 5.32 °/s for 1.2% of forward speed, −0.06 falls at 2.77 s, **+0.02 destroys the gait
outright** (yaw swings to −12.98 °/s, stride to 1.99 Hz) and +0.04 falls at 2.67 s. Its
survivable window is roughly **[−0.05, +0.01] rad**. Mechanism (a), differential lateral
placement on the hips, is linear and bidirectional over the whole range tested.

So a controller may use (a) for either sign and (b) for **one sign only**. That is what is
built: the term is clipped by *side* rather than by magnitude, to `[-0.04, 0]` in its
equivalent coefficient, so a heading error of the wrong sign switches it off rather than
reversing it. Both bounds are measured endpoints of the survivable set, not tuned values.
`sim/footcomp.py` carries the reasoning and `_self_test` asserts it (`negative heading
error: the len half is OFF, not reversed`).

**The consequence has to be stated out loud: this half can correct only one sign of
heading error.** It is usable here only because the drift it removes is a steady
one-signed bias — `heading_candidates.md` §0 measures the quarter means at +6.18, +4.74,
+5.26, +5.10 °/s. On a clip whose bias runs the other way it contributes nothing.

## 2. Question 1: the authority exists, is parameter-free, and as a P term does nothing

The heading law's lateral half comes from substituting `ω_target = ω_log − ψ_err/T_stance`
into the per-leg lateral error `(ω − ω_target)·x_i`. **Read in the other axis it gives (b)
for free.** A base with yaw error ψ carries the hip at `(x_i, y_i)` fore-aft at
`−(ω − ω_target)·y_i`, so the same half-stance-time Raibert step is

    Δx_i     = (T_st/2) · −(ψ_err / T_st) · y_i  =  −ψ_err · y_i / 2
    Δq_thigh = −Δx_i / lever_i                   =  −ψ_err · y_i / (2 · lever_i)

**`T_stance` cancels here too.** So the step-length half is as parameter-free as the
lateral one: the heading error, the hip's own *lateral* offset, the same lever. The sign is
flipped for the same reason the lateral half's is, and confirmed against the same probe.

Measured, one cell, five entry phases, everything else identical:

| | alive | x covered | \|ψ\| max | curvature | y drift | heading cap saturated |
|---|---|---|---|---|---|---|
| **A** lateral half only | 6.86 s | 3.36 m | 12.1° | **3.44 °/m** | 0.88 m | 62.5% |
| **B** + step-length half (P) | 7.08 s | 3.44 m | 13.3° | **3.67 °/m** | 0.96 m | 68.0% |

**A null result, and slightly the wrong way.** +3% survival, −2% curvature into the noise,
worse lateral drift. Per-repeat survival is 344/69/67/81/57 steps against 355/69/67/82/56 —
the same run.

Why: the term needs ψ = 0.226 rad (13°) to reach its cap, against the lateral half's 0.064
rad (3.7°), because `y_i/(2·lever) = 0.177` against `x_i/(2·lever) = 0.311`. TROT's error
stays under 13°, so the term never reaches its own cap (**0.0% of swing frames**) and is
switched off for whichever half of the time the error has the wrong sign. It is a
proportional term added to a proportional term.

## 3. Question 2: no, and this is why

**Saturation went up, not down: 62.5% → 68.0%.** Adding a second actuator did not unload
the first.

The heading term saturates whenever `ψ·x_i/(2·lever) > 0.02`, i.e. whenever the heading
error exceeds **3.7°**. TROT's error sits at 10–13° for the whole run. So the cap is
saturated because *the error is large*, and the error is large because the disturbance is
larger than what any proportional law can null at a safe amplitude — **not** because the
actuator is undersized. Widening the cap was already measured to make it worse
(`lip_failure.md` §3: 87% → 5.0% saturation at ±0.08 and every robot still falls, with peak
error growing to 47–67°). Narrowing the error is the only thing that reduces saturation,
and a P law cannot narrow it, because:

**A proportional law must hold a standing error to produce an output, and this disturbance
is DC.** The drift is a constant bias. To emit the −0.02 rad that nulls it, the lateral
term needs a permanent 3.7° of error; to emit −0.04 the step-length term needs a permanent
13°. The controller is not failing — it is doing exactly what a P law does with a constant
disturbance.

## 4. The fourth thing: feed the bias forward (and read §4b before believing it)

`heading_candidates.md` §2 had already measured the constants, open loop, on the 60-cycle
rig: (a) −0.02 and (b) −0.04, each surviving 60 cycles, together taking TROT's yaw rate
from +5.32 to −2.01 °/s for 0.5% of forward speed and no change in stride. What was never
tried is those constants **underneath the heading loop** — the DC fed forward, the loop
left to handle what is left.

Same cell, same five entry phases, everything else identical:

| | alive | x covered | v_x | \|ψ\| max | curvature | y drift | head sat |
|---|---|---|---|---|---|---|---|
| A lateral P only | 6.86 s | 3.36 m | 0.57 | 12.1° | 3.44 °/m | 0.88 m | 62.5% |
| B + step-length P | 7.08 s | 3.44 m | 0.57 | 13.3° | 3.67 °/m | 0.96 m | 68.0% |
| C + **lateral FF** −0.02 | 7.12 s | 3.35 m | 0.55 | 10.0° | 2.80 °/m | 0.95 m | 50.9% |
| D + **both FF** | 6.80 s | 2.75 m | 0.47 | 29.4° | 9.10 °/m | 1.58 m | 85.9% |
| **E + step-length FF −0.04** | **11.82 s** | **6.45 m** | **0.60** | 13.1° | **1.95 °/m** | 1.64 m | 71.8% |

On this cell E is a large win: **+72% alive, +92% ground covered, −43% curvature**, with
forward speed going *up* (0.57 → 0.60 m/s). Per-repeat survival 592/75/133/82/54 steps
against A's 344/69/67/81/57 — better or equal at every entry phase. It crosses the obstacle
line and runs to 7.4 m.

And D says the constants are not additive under feedback: both together is much *worse*
than either alone (9.10 °/m), which is the overshoot `heading_candidates.md` measured open
loop (+5.32 → −2.01 °/s, through zero) now being fought by a P loop that sees the reversed
error. Open loop the pair was the recommendation; closed loop at most one of them can be
carried.

## 4b. And the win does not replicate — the cell it was measured on is the outlier

The comparison above is *controlled* (same cell, same coordinates, five entry phases) but it
is one realisation, and `harness_findings.md` §12 measures this harness turning 0.3 mm into
a metre. So the same two conditions were run over the whole 15-cell `step_up` ladder, which
is a **paired** design — cell *k* sits at identical world coordinates in both arms (checked,
not assumed).

| | A, lateral P only | E, + step-length FF | E better in |
|---|---|---|---|
| alive, median | 9.06 s | 7.50 s | **7 / 15** cells |
| curvature, median | 5.34 °/m | 7.08 °/m | **6 / 15** cells |
| x reached, median | 3.73 m | 3.62 m | **5 / 15** cells |

**A coin flip, with the mean going slightly the wrong way** (−1.01 s alive, +0.66 °/m). And
the single cell §4 was run on is cell 0 of this table — 7.10 → 11.64 s, 2.83 → 1.47 °/m,
4.43 → 7.44 m — **the largest gain of the fifteen, on every column.**

So the honest statement is that **the feed-forward's effect on TROT is not established.** The
mechanism is real and open-loop measured (`heading_candidates.md` §2, −4.71 of 5.32 °/s), the
reasoning about DC and P control in §3 stands on its own, and one paired realisation out of
fifteen shows exactly what that reasoning predicts. Fourteen do not.

**How this was nearly reported wrongly, and the defence that caught it.** A controlled A/B
at n = 1 realisation is not a small version of a controlled A/B at n = 15; on a chaotic
plant it is a draw from the distribution of *differences*, and that distribution here is
centred near zero and wide. The single-cell design was chosen precisely *because* §12 says
cross-cell comparison is unreliable — and it fixed the wrong problem, trading an
uncontrolled comparison for a controlled one with no power. The check that caught it is the
one CLAUDE.md §6.5 asks for in another form: **measure it a second way and require
agreement.** They disagreed.

## 5. What nothing here fixes, and it is a different quantity

On the cell where E works it reaches x = 7.4 m and **still scores 0/5**, because goal 2 is a
*point* and it arrives 1.64 m to the side of it. Across the full ladder E scores 0/5 at
every level, as does A. At 1.95 °/m over 6.45 m the accumulated *heading* is 12.6° — but
the *path* has departed 1.64 m, so most of that offset is not heading error at all. It is
lateral offset: the robot is going nearly the right way while no longer being in the right
place.

**A heading-hold controller has no term for that, by construction.** Its reference is the
heading the robot was handed over at; no position, no goal and no terrain reaches it
(deliberately — that is what keeps it inside the depth/terrain prohibition). Closing the
remaining gap needs **cross-track error**, which is a position error against a path, and
the planner is the only thing that has one. That is a different controller with a different
input, and it is not built.

## 6. Status of `STEP_TROT_MAX`

Still not measurable, and **none of the four conditions moved it**: 0/5 at all fifteen
levels for every one of them. What has moved is the diagnosis. "TROT leaves the 4 m lane
before the lip" was read off single runs of a multi-cell grid; paired across cells, TROT
reaches x = 3.5–3.9 m on the median and past the obstacle on its better cells, and what
stops it is a mixture of falling at ~7 s and, when it does not fall, **arriving past the
obstacle but beside the goal** against a 0.35 m radius.

The next thing that would move it is cross-track control — a position error against a path —
and not more heading authority, which §2–4b measure three different ways as spent.

## 7. Flags

`sim/footcomp.py` gained `heading_len` / `heading_len_cap_rad` (default **off**; the
self-test asserts the hips are bit-identical to a run without it). `run_calibration_grid.py`
gained `--heading-len`, `--heading-len-cap`, `--foot-yaw-bias`, `--foot-len-bias`, and the
trace now carries the heading and step-length cap-hit counts separately from the lateral
one — the lateral cap was the only one in the trace, and the 84–87% figure the question was
about is a different cap on a different term.
