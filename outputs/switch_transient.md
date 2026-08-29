# Why a mid-run switch falls, measured: the correction is already saturated when it lands

The integration run left one blocker: single skills execute correctly and no switch
survives. Three questions were asked of it — is the foot placement working across the
seam, does speed matching help, does blending help — and the answers are, in order:
**it is working and it is clipped**, **no**, and **no**. A fourth thing does help, and it
came out of the first answer.

All of this is flat ground, CPU, no GPU. `scripts/diagnose_switch.py` reads the traces.

---

## 1. What the foot placement does across the seam

WALK → TROT at 6.20 s, 1.5 s either side:

| window | mean \|v_y\| | max \|v_y\| | yaw mean | yaw sd | max \|roll\| | hip corr RMS | at the cap |
|---|---|---|---|---|---|---|---|
| before | 0.053 m/s | 0.148 | 2.6 °/s | 20.2 | 4.6° | 0.0405 rad | **45%** |
| after | **0.248 m/s** | **1.255** | 26.6 °/s | 47.0 | **61.4°** | 0.0447 rad | **71%** |

Three answers to the question as asked:

- **Is foot-comp seeing it?** Yes. `v_y` is its only input and it responds monotonically
  right up to the clip. It is not blind to the transient; it is **clipped by it**. After
  the seam the correction sits pinned at the 0.05 rad cap continuously for 0.28 s. At the
  peak `v_y` of 1.25 m/s the law asks for 0.186 × 1.25 / 0.29 = **0.80 rad**, sixteen
  times what it is allowed.
- **Is the correction invalidated by a reset?** No, and there is nothing to reset. For
  WALK and TROT `yaw_mode` is off, so the law is **memoryless** — a static function of the
  instantaneous `v_y`, with no integrator and no history. Only TURN carries state (the
  one-cycle yaw ring) and switching into TURN does clear it, which costs it a partial
  averaging window for one cycle.
- **Does the new clip's `T_stance` matter?** Yes, and in the wrong direction. The gain is
  `T_stance/2` and `T_stance` is a property of the clip, so it steps discontinuously:
  WALK 0.484 s → TROT 0.372 s is a **23% gain drop at the instant the body is furthest
  from either gait's steady state**.

### The finding that reframes it

Step by step through the seam, the correction was **already pinned at the cap three steps
before the switch**:

    step   t      vx      vy     yaw     roll    hip correction, 4 legs
      -4  6.12  +0.252  +0.068   +8.0   -3.11   0.0000 0.0429 0.0443 0.0000
      -3  6.14  +0.271  +0.090   +0.7   -3.71   0.0000 0.0500 0.0500 0.0000  <- at cap
      -2  6.16  +0.288  +0.117   -3.4   -4.21   0.0000 0.0500 0.0500 0.0000
      -1  6.18  +0.287  +0.132   +4.8   -4.47   0.0000 0.0500 0.0500 0.0000
       0  6.20  +0.260  +0.118  +13.6   -4.76   0.0000 0.0500 0.0500 0.0000  <- SEAM

WALK was mid lateral excursion — `v_y` climbing 0.044 → 0.132 m/s, roll drifting to
−4.8°, correction saturated — and the switch landed on **that**. The switch is not what
creates the transient. It arrives during one the current gait was already failing to
answer, and hands the body to a clip whose first frame assumes it is not happening.

(WALK sits at its cap 43–45% of the time even in the runs that survive 60 cycles. Living
at the cap is normal for WALK; switching while at it is not.)

## 2. Speed matching — implemented, does not fix it

`SPEED_MATCH_MAX` is now a transition safety condition in the rule engine (the fourth
mechanism, alongside hysteresis, minimum hold and the switch delay): a switch is refused
while the body's own speed is further than the band from the incoming skill's measured
speed.

At the placeholder 0.25 m/s it **refused 19 switches and admitted one**, which still fell
(8.02 s → 9.00 s, a 1.3 s reprieve). The one it admitted is the problem: the library's
measured speeds are WALK 0.19, TROT 0.44, TURN 0.008 m/s, so a band under 0.43 forbids
WALK↔TROT outright — and the tick that got through was the tick where WALK happened to be
moving *fastest*, at 0.30 m/s, which is a peak of an excursion rather than a settled
moment.

So the speed condition is either a prohibition or an invitation to switch at the worst
instant. **That is a fact about the library's speeds, not about the threshold**: nothing
in {WALK, TROT, TURN} is within 0.25 m/s of anything else.

## 3. Blending — swept, uniformly null

Ramping the commanded pose from the one held at the switch to the new clip's stream, with
the clip's phase still advancing:

| blend window | 0 | 0.05 s | 0.10 s | 0.20 s | 0.40 s |
|---|---|---|---|---|---|
| terminated at | 7.66 s | 7.54 | 7.52 | 7.62 | 7.58 |

Every window lands within 0.14 s of the sharp seam. Blending the *pose* cannot help
because the pose was never the problem — the same conclusion the three entry criteria
reached from the other side, and consistent with the smaller seam having done worse.

## 4. What does help: don't switch while the correction is saturated

`--switch-gate settled` defers a switch while the foot-placement correction is pinned at
its cap. It adds **no new constant** — the test is the cap the law already uses — and it
defers rather than forbids, because unsaturated moments recur every stride.

| | switch at | post-switch segment | verdict |
|---|---|---|---|
| no gate | 6.20 s | stride **3.12** / 1.56 Hz, vx 0.219 | FAIL |
| settled gate (2 ticks deferred) | 6.40 s | stride **1.56 / 1.56 (+0%)**, vx 0.408 / 0.444 | **ok** |

**The first post-switch segment that reproduces its gait.** The commanded jump was
slightly *larger* (0.313 vs 0.237 rad) and it did not matter, which is the same evidence
again.

It is not a fix yet: the run still terminates, at 8.02 s against 7.66 — 1.64 s of correct
trotting instead of 1.48 s of something else. The roll still diverges afterwards. And
adding blending on top made it worse again (7.46 s, back to FAIL), so the two do not
compose.

## 5. Where this leaves the transition

The mechanism is now named: **a gait change is survivable only from inside the current
gait's linear range, and the range is bounded by a cap that stage 2 sized for steady
state.** Two directions follow, and they are different experiments:

1. **Widen the budget during the transient.** The law asks for 0.80 rad and gets 0.05.
   Stage 2 measured 0.03 rad as the threshold and 0.0718 as where the law saturates *in
   steady state*; a transient cap is a different quantity and has never been measured.
   This is the direct answer to the mechanism, and it is a new parameter, so it needs its
   own sweep and its own control.
2. **Make the excursions smaller before switching**, rather than tolerating them. WALK
   living at its cap 45% of the time is the underlying condition; the heading work
   (`outputs/heading_candidates.md`) found the same gaits are carrying a constant,
   structural lateral bias. If that bias is what fills the budget, removing it frees the
   budget for the transient.

Candidate ① (route through `balance_stand`) is still not attempted and is still second on
merit: 04a16cb measured direct handover at 7.68° of heading cost against 16.17° through
BALANCE, and a stop-and-restart raises the planner's switch cost for every decision.
