# The roll couple: keep it, and the answer is (b)

SESSION_STATE 6 left level 2 with two measurements pointing opposite ways and told nobody to
call it "better on rough ground" until that was resolved.  This resolves it.

## The 2x2

WALK, seed 1, all 200 cells, `--heading heading-only` (the arm every published WALK number
was measured on), roll couple at its recommended point (gain 8, damp 0.8, cap 2, sign +1 --
the argparse defaults).

| | couple off | couple on | delta | paired |
|---|---|---|---|---|
| **roughness ON** | **0.755** | **0.835** | **+0.080** | 24 better / 12 worse / 164 tied |
| **roughness OFF** | **1.095** | **1.090** | **-0.005** | 3 better / 3 worse / 194 tied |

**Two regression checks passed exactly.**  0.755 is the published 0.76 arm (`rc_off.csv`) and
1.095 is the published no-roughness arm (`walk_norough.csv`), both reproduced to three
decimals by code five sessions newer.  The paired counts reproduce too: `level2_results.md`
recorded 24 / 12 for the roughness arm and this run gives 24 / 12.

**The gain is entirely roughness-specific.**  Without roughness the couple is worth nothing
-- 3 cells better, 3 worse, 194 unchanged, which is not a small effect but the absence of
one.

Repeated on the `--heading off` arm as a control, where the same 2x2 gives **+0.080** with
roughness and **+0.000** (9 better / 8 worse) without.  Two different steering arms, same
answer.

## The verdict: (b)

* **(a) is refuted.**  The gain does not survive `--no-roughness`.
* **(b) holds.**  The benchmark is right and the probe rig's roughness family is the wrong
  criterion for this intervention.
* **(c) does not apply.**  `sim/attitude.py` writes `set_joint_effort_target` on the legs
  the recording already has DOWN and never writes `q_des`.  It has no mechanism by which to
  change a contact pattern, so the support gate exempts it by construction
  (`support_gate.md` 5).  **0.83 is not cancelled and stands.**

**So level 2 *may* be described as better on rough ground.**  That is the opposite of the
caution SESSION_STATE 6 imposed, and it is the benchmark's 200 paired cells that overturn it.

## Why the probe disagreed, as far as this goes

Two candidate reasons, neither yet decisive, both worth writing down:

1. **The disturbances are not the same size.**  The probe's roughness family runs
   0.005-0.015 m.  The benchmark's is 0.02-0.04 m -- above the top of the probe's ladder
   everywhere.  The couple may simply do nothing useful at 5 mm and something useful at
   30 mm, in which case both rigs are right about their own amplitude.
2. **The probe rung is n = 3.**  Its headline was WALK roughness 0.010 m at 3/3 without the
   couple and 1/3 with, and 0.005 m at 3/3 against 2/3.  A 3/3-vs-2/3 difference is one
   episode.  Against that, the benchmark's reading is 200 paired cells repeated over two
   steering arms and three seeds.  The project's own rule is "n=1 금지"; three binary trials
   per rung is closer to that floor than it looks.

**What is now established is the direction and its dependence, not the mechanism.**  The
couple needs roughness present to be worth anything, which is consistent with a balance term
doing balance work -- but "consistent with" is not the same as shown, and the probe should be
re-run at the benchmark's own amplitudes (0.02-0.04 m) with more repeats before the
mechanism is claimed.

## One more thing the 2x2 says

Without roughness the couple still buys **survival** -- 64 robots alive at 20 s becomes 74,
and median upright 7.71 s becomes 8.56 s -- while the score does not move at all.  On this
grid the median robot dies at 0.79 m and goal 0 sits at 0.50 m, so staying up longer without
going further collects nothing.  It is a reminder that survival and score are separate
columns and that the couple's roughness-arm gain is the one that pays.

Of the 64 cells WALK survives without roughness, **44 stop surviving when roughness is added
and the couple recovers 12** -- inside the 6-18 that `level2_design.md`'s estimate was
revised to, and far short of the 44 the original design assumed.
