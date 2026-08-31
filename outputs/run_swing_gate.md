# RUN: the swing gate was pointing at loaded feet, pointing it correctly changes nothing

`run_collapse.md` §6 listed three things that would have to be true for RUN to replay,
and item 3 was **"the swing gate has to come from the sim, not the schedule"**
(`--foot-swing-source sim`). The flag existed and had never been run. It has now.

All CPU, no GPU. `scripts/verify_skill_replay.py --clip RUN --cycles 60 --foot-comp
raibert --foot-clip-rad 0.05`, paired on `--foot-swing-source {clip, sim}` across three
entry conditions.

## The pair

| `--start-phase` | gate = clip | gate = sim | change |
|---|---|---|---|
| `level` | 1.1343 s | 1.1343 s | **0.0000 s** |
| `first` | 0.9317 s | 0.9520 s | +0.0203 s |
| `stance` | 1.5394 s | 1.6407 s | +0.1013 s |

Every run ends the same way, `term_reason=roll`, and none reaches a second cycle of the
60 asked for. The mean change is **+2.9%** on a run that has to last about 19 s.

The gate change is doing what it says — it moves the correction. On the `level` pair the
cap-hit fraction goes 26.5% → 36.8% and the fraction of leg-steps the correction
overwrites goes 66.2% → 59.6%. The correction is being applied to different feet. The
collapse time is **identical to fourteen decimal places** anyway.

## What that excludes

`run_collapse.md` §3 measured the clip's swing schedule and the sim's contacts agreeing on
59.6% of leg-steps and concluded the correction was being applied to feet bearing load.
That measurement stands. What is now excluded is that **it is the reason RUN collapses**.
Correcting the gate corrects the pointing and the robot falls at the same instant, which is
what §6's ordering predicted: item 1 — the base never becomes ballistic, so there is no
footfall to place — is upstream of items 2 and 3, and until it is true nothing downstream
of it can matter.

## One thing the sweep did surface

Collapse time moves **0.93 → 1.64 s across three entry phases**, a 76% spread, with the
gate held fixed. Every earlier RUN number in this repo is a single entry phase
(`--start-phase level`, 1.13 s), and `run_collapse.md` §5 already warns that RUN's stride
figures rest on `n_cycles = 1`. The 1.13 s should be read as one draw, not as RUN's
collapse time. This is the same variable that decides TURN outright
(`turn_entry_phase.md`) and that moves TROT's fall rate from 11/15 cells to 15/15
(`trot_capture_point.md` §4).

## Status

RUN stays out of `planner.skills.SUPPORTED`. The remaining candidate is unchanged and is
`run_collapse.md` §6 item 1: something has to put a vertical impulse under the base,
because the flight phase is a fact about the body that the joint trajectory does not
contain. That is a different mechanism from foot placement and it is not built.

## Item 2 (the loop seam) — what is already on disk, and why it does not settle anything

`run_collapse.md` §6 item 2 is the seam: RUN's one-cycle clip has
`loop_seam_over_max_step` 1.34, so looping it 60 times is a jolt three times a second.
`outputs/full_session_replay_RUN.csv` already plays the **uncut** session instead of the
loop, and it was on disk before this session:

| | terminated | duty | flight_frac | v_x | log's value |
|---|---|---|---|---|---|
| RUN, uncut session | 10.28 s | **0.96** | **0.006** | 0.035 m/s | duty 0.31, flight 0.28, 0.48 m/s |
| TROT, uncut session | 6.50 s | 0.48 | — | 0.038 | duty 0.52 |
| WALK, uncut session | did not terminate | 0.645 | — | 0.065 | duty 0.64 |

Read carelessly this says the seam is worth 9× the survival (1.13 s → 10.28 s). **It does
not.** At duty 0.96 the robot has all four feet on the ground 96% of the time and is
covering 0.035 m/s: it survives by standing, not by running. And all three uncut replays
come out an order of magnitude slower than their clip replays, so the uncut path is a
different regime and the two survival numbers are not the same measurement.

What it does support is that **the uncut source does not reproduce RUN's gait either** —
duty 0.96 against 0.31, flight 0.6% against 28%. So item 2 is not obviously the way in, and
item 1 stands: the base has to leave the ground, and no joint trajectory in this archive
makes that happen on its own.

