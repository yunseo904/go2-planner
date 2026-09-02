# The cross-track drift is the fall

`outputs/cross_track.md` measures **0.53 m of lateral offset per metre forward, 149 of 200
cells the same way**, calls it a bias rather than scatter, and concludes that the robot
cannot walk straight and that the lateral foot-placement channel cannot make it. The second
half is right and the first half is an artefact of where the offset is read.

**Instrument**: a new recording-only `--trace-npz` on `run_benchmark.py` (root pose, foot
world positions, the alive mask, the clip's own stance mask). Both arms below reproduce
their published scores exactly — **0.835 with roughness, 1.090 without** — so the trace does
not perturb the run.

---

## 1. Three layers, and only the third is walking

`alive` admits **|roll| and |pitch| up to 60°**, and a 0.30 m base at 55° of roll has already
carried its centre a quarter of a metre sideways without a foot going anywhere. So the
offset was re-accumulated **increment by increment over steps that satisfy a roll cut**,
rather than by truncating at the first excursion — truncating shortens the window every time
the gait leans transiently, and understates forward travel for the same reason it
understates lateral.

WALK, seed 1, roll couple on, `--no-roughness` (the configuration `cross_track.md` §1 used):

| cut | forward | lateral (signed) | \|lateral\| | **lat/fwd** | cells drifting +lat |
|---|---|---|---|---|---|
| alive only (\|roll\|<60°) | 0.974 m | +0.079 | 0.253 | **0.259** | 110/198 |
| \|roll\| < 30° | 0.912 | +0.050 | 0.138 | 0.152 | 109/198 |
| \|roll\| < 20° | 0.903 | +0.028 | 0.089 | 0.099 | 109/198 |
| \|roll\| < 15° | 0.894 | +0.028 | 0.066 | 0.074 | 112/198 |
| **\|roll\| < 10°** | **0.888** | **+0.003** | **0.034** | **0.039** | **102/198** |

**Forward travel does not move across the cuts (0.974 → 0.888) while the lateral offset
collapses by a factor of seven.** The forward metre is walked; the sideways metre is the
fall.

And the one-sidedness goes with it: **110/198 → 102/198, which is 51.5 % — chance.** The
"149 of 200 cells the same way" that made this look like a bias rather than scatter is a
property of how robots topple, not of how they walk.

With roughness on, the same table runs 0.450 → **0.101** m/m and 111/184 → 103/177.

### The three layers, as read off the same no-roughness run

| where the offset is read | median | implied drift |
|---|---|---|
| `cross_track_m` — end of episode, includes post-mortem sliding | 0.517 m | 0.53 m/m *(published)* |
| `cross_track_abs_max_m` — stops at the last upright step | 0.398 m | |
| **accumulated over \|roll\| < 10° steps** | **0.034 m** | **0.039 m/m** |

**92 % of the "drift" happens while the robot is rolled past 10°.**

## 2. The topple signature, measured directly

| | roughness | no roughness |
|---|---|---|
| final lateral sign agrees with final roll sign | **9 / 176** | **3 / 133** |
| correlation(final roll°, final lateral m) | **−0.926** | **−0.930** |
| median \|roll\| at the last upright step | 88.0° | 87.8° |

A correlation of −0.93 with an almost deterministic sign is a rigid body rotating about a
contact line, not a gait with a steering bias. (The last recorded upright step sits at 88°
because the cutoff is tested after the step.)

## 3. What the feet were doing, since that was the other candidate

Stance defined by the clip's own contact channel; **interiors only**, first and last 20 % of
each bout dropped, because those are touchdown and liftoff and are supposed to move:

| | |
|---|---|
| stance-foot world speed, median | **0.019 m/s** against a body speed of 0.192 |
| ratio foot / body | **0.10** |
| stance steps with foot speed < 2 cm/s | 51.7 % |
| foot vertical range within a bout interior | **0.9 mm** |
| net lateral slip per cell | **−0.010 m** against +0.287 m of body offset |

**The feet are planted.** Foot slip and yaw-coupled sliding — two of the three candidates in
SESSION_STATE §15.3 — are both excluded; the third, post-mortem drift, is real but is the
smaller half of the artefact. The footfall pattern was checked too: the mid-stance position
of each foot marches **+0.2 mm per footfall** laterally, +0.037 m summed over a cell, which
is not 0.287 m either.

> **A first pass of this measurement used whole stance bouts and reported a foot/body
> ratio of 0.54, i.e. "the feet slide".** That was touchdown and liftoff motion inside the
> window. The interior-only number is 0.10 and it reverses the conclusion. Recorded because
> it is the same failure the rest of this file is about: a quantity that is *almost* the one
> intended.

## 4. What this changes

- **`cross_track.md` §1's headline is withdrawn.** The drift over level walking is
  0.039 m/m without roughness and 0.101 with it, against a benchmark budget question that
  only arises past 2.25 m. Not "the robot cannot go straight".
- **`--cross-track hold`'s null is explained, and the term is exonerated.** Every gain,
  either sign, and three times the authority reproduced the control because there was
  nothing there to correct. A lateral-position loop cannot stop a robot from swinging
  sideways as it topples. The flag should stay off, and the reason in §6 of that file should
  be replaced with this one.
- **The `staircase_spiral` conjecture is weakened.** Goal 0 sits 0.30 m to one side there,
  and a 0.039 m/m walking drift over 2.6 m is 0.10 m, not 0.30 m. Whatever scores 0.00 on
  that course, arriving beside the goal because of a steering bias is not established.
- **SESSION_STATE §15.1 and §15.3 are the same question after all, and it is neither of the
  ones asked.** Not "why can't a robot that stays up go anywhere" and not "where does the
  drift come from", but **why does it roll over at about 0.7–0.9 m**. Everything downstream
  — the goal count, the cross-track column, the survival/score decoupling — follows from
  that one event.
