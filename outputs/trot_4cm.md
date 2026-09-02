# TROT at 4 cm: two of three surviving phases never reach the riser

Asked: frame-level per-leg contact and swing timing, the diagonal partner's state when a
front foot meets the wall, whether the observed mode (front pair caught at the lip, the
diagonal rear leg collapsing with it, no support left) is dominant or phase-specific, and
whether it is a different mechanism from WALK's rear-foot block — **run at TROT's surviving
entry frames**, because the default is not one of them.

Rig: `run_calibration_grid.py --params skill.STEP_TROT_MAX --reps 1 --trace-full`, clip
frames **0, 1 and 16**. Contact from measured force (>5 N), resolved to the clip's leg order
**by name**. Riser 3.00 m ahead of the spawn.

## 1. The premise mostly does not apply: they die before the wall

| clip frame | died | furthest x | riser reached? |
|---|---|---|---|
| **0** (the grid's default) | 18.02 s | **2.89 m** | **no** |
| 1 | 12.54 s | 3.12 m | yes |
| **16** | 6.20 s | **2.96 m** | **no** |

**Two of the three die on the flat run-up, 0.11 m and 0.04 m short of the step they were
supposed to fail at.** There is no front-foot-meets-wall event to look at in those two.

## 2. The one that reaches it climbs it and dies 7.3 s later

Frame 1, 0.040 m. Crossing at t = 5.24 s, death at 12.54 s.

| | 1 s before the crossing | 1 s after |
|---|---|---|
| \|roll\| | 2.12° | **2.73°** |
| mean feet down | 2.38 | **2.52** |

Frame by frame across the crossing the diagonal alternation is intact — `XX.X`, `X..X`,
`.XX.` — and **the front feet are loaded at 54–85 N right through it**. Support *improves*
slightly. Roll barely moves.

What does move is **pitch: +2° → −11 to −14° nose-down**, and forward progress stops dead at
0.11 m past the riser while the robot stays up for another seven seconds.

**So at these phases the observed mode is not what happens.** No front-foot block, no
diagonal partner unloading, no collapse of the support polygon at the lip. It climbs, pitches
nose-down, stalls, and dies later somewhere else.

The observation itself is not disputed — it may well be what the videos show, and the videos
were not made at these phases. What this measures is that **it is not the dominant mode at
the phases TROT can actually walk at**, which is what was asked.

## 3. The real asymmetry is front/rear load, not the diagonal

Per-leg duty from measured force, whole run:

| clip frame | FL | FR | RL | RR |
|---|---|---|---|---|
| 0 | 0.48 | 0.48 | **0.76** | **0.79** |
| 1 | 0.35 | 0.30 | **0.78** | **0.73** |
| 16 | 0.69 | 0.56 | 0.57 | 0.59 |

The TROT clip's own duty is **0.578**. At frames 0 and 1 the **front pair carries far less
than that and the rear pair far more** — the robot is sitting on its back legs and skipping
on its front ones. That is a front/rear imbalance, and it is not the diagonal-lock the
question proposed. Frame 16, the one that dies soonest, is the one whose duties are even.

Frame 1 also spends **8.8 % of its frames on one foot or none** against ~0 % for the other
two, which is the only genuine support-loss signal in the set — and it is the phase that
survives second-longest, so support alone does not order these either (the same thing
`support_gate.md` found across skills).

## 4. Is it the same event as the flat roll-out? Yes

Asked directly, and the answer is the one the question anticipated. Two of three never reach
the riser; the third survives it by 7.3 s. **The 4 cm step is not what ends TROT here.** What
ends it is the same thing that ends WALK on this rig's flat approach at 14–16 s and ends both
skills on the benchmark — see `why_it_rolls.md`.

The distances differ (2.9 m here against 0.8 m on the benchmark) because this rig's run-up is
smooth and the benchmark's carries 0.02–0.04 m of roughness, which `why_it_rolls.md` measures
as bringing the same event forward and scattering it.

**Different from WALK's mechanism, though.** `lip_failure.md` §1's WALK failure is a rear foot
against a riser taller than the foot's 23 mm radius — a real contact event at the step. TROT
at these phases does not get that far.
