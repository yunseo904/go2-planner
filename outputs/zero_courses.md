# The two courses nothing scores on

`staircase_walking_full_width` and `staircase_spiral` are 0.00 in **every** arm — WALK,
TROT, TURN, the Rule-Planner on depth, the Rule-Planner on ground truth, and the oracle
over all three single skills.  Two of twenty.  Asked: is it the goal, the terrain, or the
library, and what does the teacher do there?

## 1. The teacher walks both of them, and one of them better than its own average

E2E teacher, **same protocol, same seed**, per course (`~/eval_out/teacher_lab_seed1.json`):

| course | teacher | our best arm |
|---|---|---|
| `staircase_walking_full_width` | **4.49** | 0.00 |
| `staircase_spiral` | **4.87** | 0.00 |
| *(teacher's overall)* | *4.83* | |

`staircase_spiral` is **above** the teacher's own mean.  So the terrain is not impassable
and this is not a property of the courses.  It is ours.

## 2. Neither is a hard course.  Both put goal 0 somewhere else.

18 of the 20 courses put goal 0 exactly **0.50 m** from the spawn.  These two do not:

| course | goal 0 | what is between |
|---|---|---|
| `staircase_walking_full_width` | **1.00 m** ahead, on the lane centre | flat for 1.0 m, then a riser **exactly at x = 2.00 m, which is where goal 0 is** |
| `staircase_spiral` | **2.52–2.69 m** ahead and **0.30 m to the right** | **perfectly flat, 0.00 m the whole way, at every difficulty** |

That is the whole finding.  Our arms travel a median of 0.7–1.2 m.  On the other 18 courses
that clears a 0.50 m goal and scores 1.  Here it does not clear the goal, and scores 0.

**`staircase_spiral`'s approach has no staircase in it at all** — the first 2.6 m is flat
ground.  The 0.00 is not a failure on a spiral staircase; it is a failure to walk 2.6 m in
a straight line.

## 3. Each fails for a different one of two reasons, and both are already-known limits

### `staircase_walking_full_width` — the goal is on top of the first step

Clean height along the lane centre:

| x (m) | 1.0 … 1.9 | 2.00 | 2.20 | 2.30 |
|---|---|---|---|---|
| L0 | 0.00 | **0.06** | 0.06 | 0.14 |
| L5 | 0.00 | **0.11** | 0.26 | 0.26 |
| L9 | 0.00 | **0.16** | 0.32 | 0.50 |

Goal 0 is at x = 2.00 m, which is the top of the first riser.  That riser is **0.06 m at
difficulty 0** and grows to 0.16 m — at or above WALK's measured 0.04–0.06 m step bracket
**at every difficulty, including the easiest one**.  So even a robot that arrived would not
score.

But that is the *second* barrier.  The first is that the robots travel 0.75 m (median, max
0.84), so they stop 0.25 m short of the riser and never test it.  **Both barriers are real
and the near one is binding.**

### `staircase_spiral` — 2.6 m of travel, plus 0.3 m of lateral accuracy

Goal 0 is 2.5–2.7 m ahead and 0.30 m to the **right** of the spawn line.  WALK, per level:

| level | travelled | ended at | distance to goal 0 |
|---|---|---|---|
| 0 | 2.50 m | (3.44, 2.54) | 0.84 m |
| 5 | 2.80 m | (3.67, 2.83) | 1.14 m |
| 7 | **3.22 m** | (4.22, 2.22) | 0.78 m |
| others (7 of 10) | 0.75–1.55 m | — | 1.0–2.3 m |

Three of ten get far enough **in x**.  All three miss anyway, and the miss is entirely
**lateral**: the goal is at y = 1.70 and they end at y = 2.2–2.8, drifting left while the
goal is to the right.  So this course needs 2.6 m of travel *and* holding inside 0.20 m of
a line for that distance, and the arms fail the first in 7 cells and the second in the 3
that pass it.

## 4. Is "2 of 20 untouchable" a structural limit?

**No — it is the same two limits everywhere else has, measured against a goal that is
placed differently.**  Nothing new is needed to score on these courses:

* `staircase_spiral` needs travel distance (2.6 m against a current 0.79 m median) and
  cross-track accuracy on flat ground.  Both are already the top items on the list —
  the roughness costs 0.33 m of travel per §2 of `benchmark_legged_eval.md`, and the
  lateral drift is what `trot_yaw_moment.md`'s couple exists to fix.  **No new skill.**
* `staircase_walking_full_width` needs the same travel plus a ≥0.06 m step, which is the
  step limit already recorded, at the level where it first bites.  **A better gait, not a
  new one.**

What the pair does say is that **the 0.50 m goal-0 placement on the other 18 courses is
doing a lot of work for our scores.**  A score of 1.0 there means the robot moved half a
metre.  These two courses are the only ones in the benchmark that ask for more than that
before awarding anything, and the answer to what our arms do when asked is 0.00.

That is worth stating plainly next to every table: **eighteen of our twenty course scores
are "moved 0.5 m", and the two courses that ask for 1.0 m and 2.6 m both return zero.**
