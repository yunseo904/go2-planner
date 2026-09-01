# RUN's posture: the tracking failure is real, the direction is the opposite

Section 4 of the request: *RUN cannot push because its legs are already straight -- the
torque-mode gains (kp 13/3/2) are too low to hold a crouch, so load pushes the leg into
extension and no stroke is left at the start of stance.*

Measured on the flat rig, 20 cycles, traces in `outputs/trace_ext_run.npz` /
`trace_ext_walk.npz`.  Extension is the request's own metric, `cos(thigh) + cos(thigh+calf)`,
larger = straighter, computed on the COMMANDED angles and on the ACHIEVED ones.

| | commanded | achieved | delta | in stance | in swing |
|---|---|---|---|---|---|
| **RUN** | 1.3253 | 1.2858 | **-0.0395** | **-0.1921** | +0.0402 |
| **WALK** | 1.4170 | 1.3434 | **-0.0736** | **-0.1052** | -0.0117 |

**Half the hypothesis is confirmed and the other half is reversed.**

## Confirmed: RUN does not track

| \|q - q_des\| | RUN | WALK | ratio |
|---|---|---|---|
| hip | 0.0865 rad | 0.0641 rad | 1.3x |
| **thigh** | **0.3683 rad (21 deg)** | 0.0559 rad | **6.6x** |
| **calf** | **0.3326 rad (19 deg)** | 0.1457 rad | **2.3x** |
| worst thigh sample | **0.9488 rad (54 deg)** | 0.2001 rad | 4.7x |

RUN's thigh misses its commanded angle by 21 degrees on average and 54 at worst.  That is
not a replay of the recording; it is a different trajectory.  The low-gain account of *why*
stands.

## Reversed: the leg ends up MORE crouched than commanded, not less

The delta is **negative in both clips and most negative in stance** -- exactly where load
acts.  Under load the leg **collapses toward the body**; it is not pushed into extension.
So "no stroke left because the leg is already straight" is not what the trace shows.  If
anything the achieved posture has more room than the commanded one, and the problem is that
the joint is nowhere near where it was told to be.

## The limitation, which is large

**RUN terminated at 0.851 s -- 43 control steps, one cycle.**  Most of that trace is a robot
falling, and its per-leg deltas say so: `[+0.14, +0.07, +0.01, -0.38]`, one leg doing
something completely different from the other three.  WALK's are even
(`[-0.09, -0.08, -0.06, -0.06]`) because WALK stays up for the full 740 steps.

**So this does not settle the question, and it should not be quoted as if it did.**  What it
establishes is that the tracking error is real and enormous, and that the extension error's
sign is not the one the hypothesis needs.

## What would settle it

The **in-air replay** -- hold the robot off the ground, pass `q_des` through, and read the
achieved angles with no contact and no load.  That separates "the gains cannot hold this
posture even unloaded" from "load pushes it out of the posture", and it is the same
instrument SESSION_STATE 13 lists as not done for TURN.  One rig answers two open questions,
which is why it should be the next thing built.
