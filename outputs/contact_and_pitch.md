# The contact channel is fine, and the pitch is not what costs the clearance

Two flags from the previous round, chased. Both turn out to be the same failure shape as
the clearance error itself, and one of them is mine twice over.

---

## 1. The 51% contact-channel disagreement was my proxy, not the channel

**Withdrawn.** The 51% came from calling a leg "in swing" whenever its hip-to-foot drop
was more than 3 mm above *that leg's own lowest value over the clip*. On a planted foot
the hip-to-foot distance changes continuously as the body passes over it, so that rule
flags most of stance as swing. It was never a test of the contact channel.

Redone per frame across legs — a foot is in stance when its drop is within 3 mm of the
deepest foot at that instant — the agreement is 60.1% (WALK), 69.5% (TROT), 56.1% (TURN).
**That proxy is confounded too**: it calls 73.6% of WALK leg-frames swing against a duty
that allows 36%, because the rear legs stand 17 mm more crouched than the front (332–334
vs 349–351 mm of extension) and are therefore almost always "higher". The same front/rear
asymmetry that produced the original error.

The test that is not a proxy is the replay's **measured contact force** during the frames
the clip declares swing:

| leg | force in clip-swing | in clip-stance | loaded >30 N during swing |
|---|---|---|---|
| FL | 1.5 N | 58.6 N | 1.7% |
| FR | 1.1 N | 60.9 N | 0.8% |
| RL | 15.5 N | 45.8 N | 7.6% |
| RR | 8.0 N | 45.8 N | 11.5% |

The channel is substantially right: when it says swing, the foot is carrying 1–15 N
against 46–61 N in stance.

And operationally it does not matter which gate is used. WALK, 60 cycles, foot placement
on, clip gate vs the sim's own contacts:

| gate | stride | vx | vy | yaw | overwritten |
|---|---|---|---|---|---|
| clip | 1.37 / 1.37 (+0%) | 0.231 | −0.0034 | +3.00 °/s | 78.4% of steps |
| sim | 1.37 / 1.37 (+0%) | 0.230 | −0.0068 | +3.27 °/s | 97.1% |

Identical outcomes, as TROT already showed (0.657 vs 0.659). **No change to the swing
gate is warranted**, and the concern is closed.

---

## 2. The pitch is a real plant mismatch — and it is not the cause

Real robot, steady window of each clip's own session, against the sim replaying it:

| clip | real pitch | sim pitch | mismatch |
|---|---|---|---|
| WALK | **−0.08° ± 2.66** | −3.25° ± 0.84 | 3.2° |
| TROT | +0.19° ± 0.32 | +1.69° ± 0.74 | 1.5° |
| TURN | +0.67° ± 0.48 | +2.24° ± 1.44 | 1.6° |

So the real robot is level and the sim is not — a plant mismatch of the same kind as the
stance width and the ground friction, and worth recording as one.

**But it does not explain the lost rear clearance, and the sign is why.** −3.25° over the
0.386 m hip span puts the rear hips **11 mm higher**, not lower. It works against the
observation. My earlier explanation had the sign backwards.

What the measurement actually says, per leg, in world coordinates:

| leg | stance level | apex | rise |
|---|---|---|---|
| FL | 23.2 mm | 71.2 | **48.0** |
| FR | 23.2 | 82.2 | **59.0** |
| RL | 22.8 | 26.3 | **3.5** |
| RR | 22.9 | 29.6 | **6.7** |

All four stand on the same floor. The rear feet rise 3.5–6.7 mm while **unloaded** (§1),
so nothing is pinning them — they are simply not getting high enough relative to where the
body is riding.

And that is the quantity that fits: the recording's deepest hip-to-foot extension is
**351 mm**, and the replayed base sits at **316 mm**. The body rides about 35 mm lower
than the clip's own stance geometry. That deficit consumes the rear legs' 53–63 mm of
retraction almost entirely and leaves most of the front's 75–81 mm intact — which is
exactly the 48/59 vs 3.5/6.7 split observed.

**The plant mismatch that matters here is body height, not pitch.** It is distinct from
the compensation rejected earlier: that one widened the stance and cost a third of the
forward speed. This is a diagnosis, and no fix is proposed here.

---

## 3. Re-measuring WALK's 2 cm step limit is blocked on the above

Not run. The limit is set by rear-foot ground clearance, and that clearance is currently
3.5–6.7 mm because the body rides 35 mm low — not because of anything about the WALK gait,
which commands 72–83 mm. Re-running the probes before the height deficit is addressed
would measure the same artefact again to five repeats.

`STEP_WALK_MAX` stays `CALIBRATION_NEEDED`.

---

## Method note, third instance in one session

Both flags in this round were the same error as the one they were chasing: a quantity
*almost* the one being reasoned about.

- "51% disagreement" — a swing test that measured each leg against its own history rather
  than against the other legs.
- "the pitch lowers the rear hips" — a sign asserted from a plausible mental picture
  instead of computed.

The second one was caught by computing the 11 mm rather than describing it. CLAUDE.md §6.5
now carries the rule that produced both catches.
