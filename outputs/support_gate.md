# The support-polygon gate, and what it says about the interventions already adopted

CLAUDE.md 2.5 was added this session.  This is its evidence, including the part that made
the rule have to be written differently from the way it was proposed.

## 1. The proposed rule rejects WALK

The proposal was: *an intervention that takes WALK below 3 simultaneous feet is rejected,
because WALK is a 4-beat statically stable gait and TROT is a 2-beat dynamically stable one,
and this harness has no feedback with which to supply dynamic stability.*

The second half is right and it explains a great deal (see §4).  The first half does not
survive contact with our own recording.

`scripts/support_polygon.py`, on `data/skill_clips.npz`, the COMMANDED pattern:

| clip | duty | mean feet | min | time below 3 feet | benchmark |
|---|---|---|---|---|---|
| WALK | 0.662 | 2.65 | **2** | **56.8 %** | 0.76 |
| TROT | 0.578 | 2.31 | 2 | 78.1 % | 0.29 |
| **TURN** | 0.672 | **2.69** | 2 | 48.9 % | **0.17** |
| RUN | 0.344 | 1.38 | 0 | 100 % | 0.00 |
| JUMP | 0.251 | 1.00 | 0 | 86.9 % | — |

**Our WALK clip is below 3 feet for 57 % of its cycle, and it cannot be otherwise.**  Mean
feet down = 4 x duty, so "3 feet at all times" requires duty >= 0.75.  A textbook crawl walk
has that; the `03_slow_walk` recording has 0.66.  Read as an absolute gate, the rule
disqualifies the only skill that works.

**And support does not order the scores.**  TURN has the widest support of any clip (2.69)
and the worst score (0.17).  Support belongs to the necessary conditions, not to the
ranking.

So the rule is written as a RELATIVE one: an intervention that moves foot trajectories must
not reduce support **below the unedited clip's own measurement**, measured paired.

## 2. Measured support is lower than commanded, and that is the number that counts

Flat rig, WALK, 60 cycles, contact from force (`sim/diagnose.py`, schmitt+despeckle):

| arm | mean feet | min | time below 3 | duty | v_x | stride |
|---|---|---|---|---|---|---|
| reference (lift 0) | **2.374** | 2 | 70.8 % | 0.593 | 0.2332 | 1.367 |

The clip COMMANDS 2.65 feet down and the simulator LOADS 2.37.  The gate therefore reads
the measured channel, not the commanded one -- which is also the only channel that can see
`--swing-lift` at all, because that flag never touches the clip's `contact` array.

## 3. Swing lift: rejected, but not for the proposed reason

| lift | mean feet | min | time below 3 | duty | v_x | vs ref |
|---|---|---|---|---|---|---|
| 0 mm | 2.374 | 2 | 70.8 % | 0.593 | 0.2332 | — |
| 20 mm | 2.406 | 2 | 64.1 % | 0.601 | 0.2715 | **+16 %** |
| 40 mm | 2.421 | **1** | 61.7 % | 0.605 | 0.2787 | **+20 %** |
| 60 mm | 2.512 | **1** | 54.5 % | 0.628 | 0.2770 | **+19 %** |

**The proposed mechanism is not what happens.**  Support does not fall -- mean feet down
rises monotonically, 2.374 -> 2.512, and the time below 3 feet FALLS from 71 % to 55 %.
The rear feet are not being taken off the ground; the gait is spending longer in stance.

What does appear is a state the reference never has: **single-foot support**, 11 frames of
2220 at 40 mm and 1 at 60 mm, where the reference's minimum is 2 all the way through.

So swing lift is **rejected** on two criteria, neither of them the proposed one:

* **v_x moves +16 to +20 %**, outside the +-10 % band.  It is a speed edit, which is what
  the benchmark already said (+10/+19/+32 %, score flat at 0.83/0.76/0.82/0.83 while
  survival collapsed 31 -> 12).  A `sin^2` arc with zero slope at both endpoints was chosen
  precisely so it would not be one, and it is one anyway.
* **minimum support drops 2 -> 1**, which is a new failure state rather than a shift of an
  existing distribution.

Recorded as closed.  The correction to keep: *it was never removing support; it was adding
speed.*

## 4. The part of the hypothesis that stands

The static/dynamic distinction still explains the three things it was offered to explain,
and the duty numbers back it: WALK 0.59-0.66, TROT 0.52-0.58, RUN 0.33-0.34.  This harness
replays clips open loop and has no balance feedback, so a gait that needs the dynamic half
does not get it.  That remains the best available account of why WALK survives, TROT clears
nothing on any probe rung, and RUN scores 0.00.

What the measurement removes is only the *quantitative* form -- "3 feet" is not our WALK's
property and is not the discriminator between our clips.

## 5. The already-adopted interventions, re-examined

| intervention | writes a swing leg's `q_des`? | in scope | verdict |
|---|---|---|---|
| foot placement (`--foot-comp`) | **yes**, the hip | yes | **passes** |
| heading hold (`--heading`) | **yes**, hip + thigh | yes | **passes** |
| swing lift (`--swing-lift`) | yes | yes | **rejected**, §3 |
| roll couple (`--roll-couple`) | **no** -- stance-leg torque only | **exempt** | see `level2_verdict.md` |
| yaw couple (`--yaw-moment`) | **no** -- stance-leg torque only | **exempt** | — |

Measured, WALK, 60 cycles on flat, against the same reference:

| arm | mean feet | min | time below 3 | duty | v_x | vs ref |
|---|---|---|---|---|---|---|
| reference (both off) | 2.374 | **2** | 70.8 % | 0.593 | 0.2332 | — |
| **foot placement on** | **2.385** | **2** | 69.4 % | 0.596 | 0.2315 | **-0.7 %** |
| **+ heading hold** | **2.438** | **2** | 66.0 % | 0.610 | 0.2201 | **-5.7 %** |
| swing lift 40 mm | 2.421 | **1** | 61.7 % | 0.605 | 0.2787 | +20 % |

**Both adopted foot-moving terms pass, and neither is marginal.**  Support rises rather than
falls, the minimum stays at the reference's 2, and forward speed moves -0.7 % and -5.7 %,
inside the +-10 % band.  Swing lift sits in the same table with a minimum of 1 and +20 %.

That is the useful shape of the result: the gate separates the three interventions that edit
feet into two that leave the support pattern alone and one that does not, and it does so on
the same rig with the same reference.

The two couples are exempt *by construction*, not by assumption: `sim/attitude.py` and
`sim/yawmoment.py` write `set_joint_effort_target` on the legs the recording has DOWN and
never touch `q_des`.  They have no mechanism by which to move a contact pattern.  They are
still stamped in the results rows, so "exempt" never becomes "unmeasured".
