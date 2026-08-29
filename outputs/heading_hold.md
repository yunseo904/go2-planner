# Heading hold: WALK comes inside the benchmark budget, TROT does not

The heading corrector proposed in `heading_candidates.md` §5, built and measured. It is
the mechanism the switch failure, the cap saturation and the empty calibration all traced
back to.

## The law

    omega_target = omega_log - psi_err / T_stance

substituted into the per-leg lateral term `(omega - omega_target)·x_i` gives, for the
heading half,

    u_head,i = -psi_err · x_i / (2 · lever_i)

**`T_stance` cancels.** The heading correction carries no constant at all — not even a
measured one. Its inputs are the heading error, the hip's own fore-aft offset and the
hip-to-foot lever, all of which the robot knows about itself. No terrain, no depth, no
goal reaches it; the reference heading is the one the robot was handed over at.

Caps are per skill, taken from the open-loop steering probe: **WALK ±0.04, TROT ±0.02**
(TROT falls in both directions at ±0.04). The heading term is capped separately from the
lateral term and added to it — they answer different questions and clipping their sum
would let one starve the other.

### The sign is negative, and that was settled by measurement

The substitution as written yields `+psi·x/(2·lever)`, and **that sign is positive
feedback.** The steering probe measured `+bias → +yaw rate`, so a positive heading error
needs a *negative* bias to come back. Built with the derived sign, the controller drove
WALK's curvature from 13.26 to **19.99 °/m** and made the full-formula TROT fall at
2.12 s. Flipped, it works. Same lesson as the stage-1 attitude PD: the joint frame is not
a reliable guide to the sign, and the run is.

## What it does

40 s per run, one skill held, flat ground, CPU.

| | curvature | yaw rate | stride | vx | lateral cap saturation |
|---|---|---|---|---|---|
| WALK, off | 13.26 °/m | +3.09 °/s | 1.35/1.37 (+1%) | 0.233 | 46.8% |
| **WALK, heading** | **0.24 °/m** | **+0.06 °/s** | 1.35/1.37 (+1%) | **0.235** | **30.7%** |
| TROT, off | 7.79 °/m | +5.17 °/s | 1.56/1.56 (+0%) | 0.664 | 2.7% |
| **TROT, heading** | **3.20 °/m** | **+2.12 °/s** | 1.56/1.56 (+0%) | **0.663** | 2.7% |

Against the four things asked of it:

- **WALK curvature 11.5–13.3 → 0.24 °/m.** The benchmark budget is 0.565 °/m, so WALK is
  now **inside it**, by a factor of two. This is the first time anything in this project
  has met that number.
- **TROT −59%, and stops there.** 7.79 → 3.20 °/m at the ±0.02 ceiling — still 5.7× the
  budget. The ceiling is the limit, not the mechanism: TROT falls at ±0.04 in both
  directions, so the authority that WALK is allowed is not available to it.
- **Stride and forward speed are untouched.** WALK +1% stride and 0.233 → 0.235 m/s;
  TROT +0% and 0.664 → 0.663. This is not the failure pattern named at the start of all
  this — nothing was traded away for the heading.
- **Saturation falls where it was high.** WALK 46.8% → 30.7% of swing samples at the
  lateral cap. TROT was never saturated in steady state (2.7%) and still is not.

The full formula (keeping the yaw-**rate** term as well) is worse in both gaits, exactly
as `heading_candidates.md` §3 measured: it is a stride-frequency signal, not a heading
one. `--heading heading-only` — the heading half alone — is what these numbers are.

## What it does not do

TROT's 3.20 °/m is a third of the way to the budget and stuck behind an amplitude limit
that is a property of the trot's own stability margin. Two things could move it, neither
attempted: the combined mechanism (`heading_candidates.md` §2b measured lateral placement
and step-length differential adding to cross zero on TROT, at 0.5% of forward speed), or
finding what makes TROT intolerant of ±0.04 in the first place.
