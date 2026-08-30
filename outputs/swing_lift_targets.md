# What foot height to ask each skill for — and why the answer is "the one it has"

The supervisor's target is the documented default, 0.08 m, capped by each skill's own
safety limit. Setting that per skill independently gives WALK 80 / TROT 60 / TURN < 20,
which **inverts the order the real robot walks in**: the slow gait would lift higher than
the fast one. This checks whether any assignment avoids that.

---

## 1. The recording's own order, measured

Real robot, achieved angles `q` (`q_des` is not a pose it holds), articulation held in the
air and driven to those angles — no hand-written kinematics, `body_pos_w` read directly.
Two references, as CLAUDE.md §6.5 requires:

- **chord** — apex above the straight line joining that bout's liftoff and touchdown feet.
  Independent of trunk attitude. This is the quantity `--swing-lift` targets.
- **plane** — height above the plane the stance feet define at the same frame.

| skill | chord apex, per leg (FL / FR / RL / RR) | chord median | plane median | swing frac |
|---|---|---|---|---|
| TURN | 9.3 / 2.5 / 11.7 / 22.0 | **10.5** | 5.3 | 0.33 |
| JUMP | 43.1 / 2.3 / 25.2 / 3.6 | **14.4** | 1.6 | 0.75 |
| WALK | 22.0 / 31.3 / 1.2 / 8.1 | **15.1** | 12.7 | 0.34 |
| RUN | 47.4 / 56.9 / 50.3 / 56.4 | **53.3** | — | 0.66 |
| TROT | 75.1 / 59.8 / 21.2 / 55.2 | **57.5** | 19.3 | 0.42 |

**Order: TURN < JUMP < WALK ≪ RUN ≈ TROT.**

The expectation was WALK < TROT < RUN. Measured it is WALK < **RUN < TROT**, but RUN and
TROT are 53.3 vs 57.5 — inside the leg-to-leg spread of either clip, so the defensible
statement is that **RUN and TROT are together at ~55 mm and WALK is at a third of that.**
The gross expectation (slow walk lowest, fast gaits ~4× higher) holds.

The plane column is thin for the fast gaits on purpose: RUN has a flight phase and TROT
often has fewer than three feet down, so no stance plane exists to measure against. Where
both references exist they agree on the ordering, and the chord is the one that carries it.

`outputs/skill_foot_heights.json`, `scripts/check_skill_foot_heights.py`.

## 2. Each skill's demonstrated ceiling

Flat, 60 cycles, symmetric lift, foot placement on (`swing_lift_sym.csv`):

| skill | passes | fails | demonstrated ceiling |
|---|---|---|---|
| WALK | 40, 60, 80 | — | **≥ 80 mm** |
| TROT | 60 | 80 (roll at 2.97 s) | **60 mm** |
| TURN | 0 (63 cyc) | 20 (roll at 3.67 s) | **< 20 mm**, i.e. its own 10.5 |
| RUN | — | falls at 1.13 s even at lift 0 | not measurable |

## 3. The only order-preserving assignment is ~+4 %

Scaling all skills by one factor `k` keeps the order by construction. The binding
constraints:

- TROT: `k · 57.5 ≤ 60` → **k ≤ 1.04**
- TURN: `k · 10.5 < 20` → k < 1.90
- WALK: `k · 15.1 ≤ 80` → k ≤ 5.3

**TROT binds at k = 1.04.** At that factor: TURN 10.9, WALK 15.7, RUN 55.5, TROT 60.0 mm —
a 4 % change, inside the run-to-run spread of every one of them.

The reason is structural, not a tuning failure: the gait that is already **closest** to the
documented 80 mm is the one with the **least headroom left**, and the gait with all the
headroom (WALK, ≥80) is the one that must stay lowest to keep the order. Raising WALK to 80
would put it above TROT's own recorded 57.5 and above TROT's ceiling of 60 — a robot whose
amble lifts higher than its trot.

## 4. Recommendation: 0 mm for all three, and record why

1. **The curve is flat.** WALK's best step score in the whole sweep is the untouched
   recording (4/5 at 0.02 m). No lift level improves any skill's step height
   (`swing_lift.md`, and §5 below for the symmetric re-run).
2. **No order-preserving assignment gets meaningfully closer to 80 mm** — +4 % is the
   ceiling, and it is noise.
3. **The cost is real.** 80 mm is 0.24 rad calf RMS, 0.47 peak — 5× the entire
   foot-placement budget — and it costs an order of magnitude of heading authority
   (0.23–1.01 → 8.4–13.1 °/m).
4. **The 0.08 m default may not be the same quantity.** It is carried in this repo as a
   docstring assertion with no upstream source checked in. Unitree's `footRaiseHeight` is
   a parameter *commanded to* their own controller; ours is an *achieved* apex above the
   liftoff/touchdown chord. Commanded and achieved differ by ~40 mm of PD sag on this
   robot in every other place we have compared them (`commanded_angles.md`,
   `stance_height.md`). **Matching our achieved 80 to their commanded 80 is exactly the
   "almost the same quantity" trap of CLAUDE.md §6.5**, and it should be checked against
   the SDK before any target is set from it.

If a nonzero value is required for the record, the self-consistent one is the uniform
k = 1.04 set above. It is defensible and it changes nothing measurable — which is the
honest description of what the whole edit does.
