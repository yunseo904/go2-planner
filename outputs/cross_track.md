# The robot cannot go straight, and foot placement cannot make it

## 1. The drift is real, large, and one-sided

Heading hold closes a loop on the yaw ANGLE and measures 13.26 -> 0.24 deg/m.  Nothing
closes one on lateral POSITION.  Measured on WALK, seed 1, 200 cells, `--no-roughness` so
the terrain is not doing it (`h2r_norough_off.csv`):

| | |
|---|---|
| forward travel from the spawn, median | **1.006 m** |
| \|cross-track\| at the end, median | **0.562 m** |
| **drift per metre travelled** | **0.531 m/m** |
| cells drifting the SAME way (+y) | **149 / 200** |
| median v_y / v_x | 0.0305 / 0.1195 = **0.66** |

**It moves sideways more than half as far as it moves forward, and three quarters of the
grid goes the same way.**  This is not noise about a straight path; it is a bias.  It is
also consistent with the two courses `zero_courses.md` found at 0.00 -- `staircase_spiral`
puts goal 0 2.6 m ahead and 0.30 m to the right over flat ground, and a robot with this
drift does not arrive at a point 0.30 m to one side.

With roughness on the same measure is 0.598 m of \|cross-track\| (`xt_off.csv`).

## 2. The term, and why it changes nothing

`--cross-track hold` (default off, banner, per-row stamp, paired arms) asks the lateral
placement law for a velocity that returns to the y each robot SETTLED at:
`v_y,target = -gain * cross-track error`, bounded by `--cross-track-cap`.  The datum is the
robot's own start line -- no goal, no terrain -- so the term has the same standing as
heading hold and the planner never sees it.  Implemented as a `vy_bias` on the law rather
than by overwriting `vy_target`, because the yaw modes do not read `vy_target` at all.

WALK, seed 1, 200 cells, `--heading heading-only`, paired against `--cross-track off`:

| arm | gain | score | \|ct\| median | peak ct median | v_x | paired |
|---|---|---|---|---|---|---|
| **off (control)** | — | **0.755** | 0.598 | 0.408 | 0.0840 | — |
| hold | 0.5 | 0.765 | 0.576 | 0.419 | 0.0833 | 15 b / 13 w |
| hold | 1.0 | 0.720 | 0.586 | 0.421 | 0.0784 | 7 b / 15 w |
| hold | 2.0 | 0.755 | 0.590 | 0.407 | 0.0831 | 14 b / 14 w |
| hold | **-1.0** | 0.750 | 0.596 | 0.398 | 0.0761 | 12 b / 13 w |

**The sign test is the one that settles it.**  A term with authority puts +gain and -gain on
opposite sides of the control -- that is exactly how the roll couple's sign was fixed
(+1 -> 0.83, -1 -> 0.66, off 0.76).  Here **-1.0 is indistinguishable from +2.0 and from
off**: every arm sits inside 0.720-0.765 with peak cross-track inside 0.398-0.421.  The
term is not being applied in the wrong direction.  It is not being applied.

## 3. Why: it saturates the placement law's cap immediately

Checked offline against `sim/footcomp.py` rather than inferred from the sweep:

    vy_bias = -0.10 m/s  ->  hip command 0.0500 rad on every swing leg
    --foot-clip-rad cap  =  0.05 rad

**The smallest cap tried already asks for exactly the maximum the law may give.**  Through
the 0.306 m hip-to-foot lever, 0.05 rad is **0.0155 m** of foot offset.  Every gain from 0.5
to 2.0 requests 0.25-1.0 m/s against a 0.10 m/s cap, so the command is the cap at all times
and the gain ladder was never a ladder -- which is why it reads flat, and why the sign flip
reads flat too.

So the finding is not "cross-track hold does not work".  It is:

> **0.0155 m of foot offset per step cannot retire 0.5 m of lateral error inside the ~6 s
> the robot stays up.**

## 4. It is not an authority limit either — the term simply does not act

The saturation above is real, so the obvious next question is whether more authority would
let the term work.  `--foot-clip-rad 0.15` -- three times the cap -- run as a pair so the
wider clip is separated from the term that uses it:

| arm | clip | score | \|ct\| median | peak ct median | v_x | alive |
|---|---|---|---|---|---|---|
| off, clip 0.05 (control) | 0.05 | 0.755 | 0.598 | 0.408 | 0.0840 | 29 |
| **off, clip 0.15** | 0.15 | **0.780** | 0.546 | 0.428 | 0.0781 | **47** |
| on (gain 1.0, cap 0.30), clip 0.15 | 0.15 | 0.750 | 0.553 | 0.429 | 0.0779 | 28 |

**At three times the authority the term still does nothing to the thing it targets**: peak
cross-track +0.1 %, final |cross-track| +1.3 %, against its own paired off-arm.  The score
falls 0.780 -> 0.750 and survival 47 -> 28.

So the saturation is not the explanation.  **The lateral placement channel does not move
lateral position on this terrain at all** -- not at 0.05 rad, not at 0.15, not at either
sign.  Whatever produces 0.53 m/m of drift is not the body-frame lateral velocity that this
law regulates.  Candidates not yet separated: foot slip against the ground, yaw-coupled
sliding that the heading term corrects in angle while leaving the translation, and drift
accumulated during the fall itself rather than during walking (`cross_track_m` is read at
the end of the episode and therefore includes post-mortem sliding, while
`cross_track_abs_max_m` is capped at the last upright step -- the two disagree, 0.598
against 0.408, and that gap is itself worth chasing).

**An earlier draft of this file blamed the 0.05 rad cap and connected it to TURN's 31-36 %.
That is withdrawn: the wider-cap arm refutes it.**

## 5. The by-product, which is the most promising thing found this session

`--foot-clip-rad` has been 0.05 since the placement law was written and **has never been
swept**.  Widening it to 0.15 **with no cross-track term at all** is worth **0.755 -> 0.780**
and takes WALK from **29 to 47 alive** at 20 s, at a v_x cost of -7 % (inside the +-10 %
band).  That is a larger survival move than the roll couple's, from a default nobody had
looked at.

It is NOT adopted on this evidence: one seed, one skill, and the support gate has not been
run on it.  It is the first thing to sweep properly next session.

## 6. Status

**Cross-track hold: not adopted.**  The flag stays, default off, because the drift it targets
is real and large and the next attempt should not have to rebuild the wiring, rediscover the
saturation, or re-learn that widening the cap is not the answer.  The representative number
is unchanged.

What the session establishes: the drift is large, systematic and one-sided; heading hold does
not address it; and the lateral foot-placement channel does not address it either, at any
gain, either sign, or three times the authority.  **Where the drift comes from is now the
open question, and it is a better-posed one than it was.**
