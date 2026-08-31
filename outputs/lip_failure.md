# Why WALK stalls at the lip and TROT never reaches it

> **Two corrections, both later than this document.**
> §2's "yaw drift" column was decoded from the quaternion as scalar-FIRST; every trace here
> is scalar-last, so those angles are wrong and the "+47°" in it is not a measurement
> (`swing_lift_symmetry.md` §0, `harness_findings.md` §11). §1's positional facts — foot and
> base x/y/z, contact forces, torques — are unaffected.
> §3's reading that TROT is stopped by the lane is superseded: with the step-length bias fed
> forward TROT crosses the obstacle line and runs to 7.4 m, and what stops it is lateral
> offset against a point goal (`trot_straight.md` §5). Every single-run row in §3 is also one
> cell of a multi-cell grid, and `harness_findings.md` §12 measures those cells diverging
> from 0.3 mm to 1 m over a run — so they are samples, not settings compared like for like.


Instrument: `run_calibration_grid.py --trace-full`, which adds contact-force vectors on
**every** link, joint torque, per-link position and the per-step foot-placement
correction. All runs CPU, no GPU. Traces: `outputs/trace_lip.npz`,
`trace_trot_0.04.npz`, `trace_trot_0.08.npz`, `trace_walk_lift60.npz`.

## 0. What the contact channel can and cannot be asked

Validated before use: total vertical contact force on flat ground is **150 N** against a
weight of 147 N (implied mass 15.25 kg), non-foot links carry **exactly 0 N**, swing feet
carry ~0 while stance feet carry the load. The vertical channel is sound.

**The horizontal channel is not.** On flat ground `sd(Fx) = 0.00 N` while the base's own
acceleration requires `m·sd(a_x) = 8.43 N`, and Fx is exactly zero in 48.5% of frames.
`ContactSensor.net_forces_w` is reporting the normal component only. So the requested
**horizontal/vertical decomposition of the contact force cannot be made from this channel**
and nothing below rests on it. Recorded in `harness_findings.md` §10.

## 1. WALK: the rear feet never leave the ground

Achieved swing apex above the stance floor, flat approach, from the contact channel
(airborne = foot force < 5 N) and confirmed by foot height — two frames of reference:

| leg | apex mean | apex max | airborne |
|---|---|---|---|
| FL | 37.6 mm | 46.1 mm | 29.7% |
| FR | 52.9 mm | 68.1 mm | 37.8% |
| **RL** | **4.0 mm** | **11.0 mm** | 15.9% |
| **RR** | **5.4 mm** | **7.9 mm** | 23.2% |

The clip's own contact channel says those rear legs are swinging; the ground says they are
loaded **59% (RL) and 33% (RR) of the frames the clip calls swing**, at 9–15 N. The rear
feet scuff along the ground through their whole swing. This is on **flat ground**, before
any obstacle, and it is the same fact the achieved-angle clearance work found (rear apex
1.2 mm) arrived at from a different instrument.

The foot centre rides **23 mm** above the ground when loaded, i.e. the foot's own radius.
That sets the threshold:

- **0.02 m step: passes.** A 20 mm riser is inside the foot's radius, so a dragging foot
  rolls over the edge instead of hitting a wall.
- **0.04 m step: fails.** The riser is a wall the rear feet cannot get over.

The stall, frame by frame (`trace_lip.npz`, WALK 0.04): the front feet mount the step
normally at t=12.5 (foot z 0.023 → 0.064, i.e. on top). The rear feet close on the riser
from −0.71 m to **−0.05 m and stop there**, never rising above 0.035 m. The base advances
to x = 4.096, holds at 4.09 for 1.5 s pitched −9°, and rolls over at t=16.5.

Ruling out the other candidates that were asked about:

- **Leg or body contact with the lip: no.** Across the whole stall, contact force appears
  on `FL/FR/RL/RR_foot` and on nothing else. Calf, thigh, hip, base and head are at
  exactly 0 N in every frame. Nothing but the feet ever touches the step.
- **Torque limit: no.** Peak torque at the lip is 53% of `effort_limit` (12.5/23.7 Nm
  thigh, 20.2/45.43 Nm calf) and **0.0% of frames are saturated**, on every one of the 12
  joints. There is torque to spare.
- **Tracking: the plant is following.** PD error at the lip is within 1.5× of its
  flat-ground value on 10 of 12 joints. The robot is doing what the clip asks; what the
  clip asks does not clear a 40 mm step.

So it is neither propulsion shortfall nor propulsion misdirected. It is a **kinematic dead
end**: front axle up, rear axle jammed against a wall its swing arc cannot cross.

## 2. Lifting the feet clears the riser and moves the failure

`--swing-lift 60` on the same two probes lifts every foot well past the riser — rear apex
**51–76 mm against a 40 mm riser**, front 92–98 mm. The clearance problem is genuinely
solved. Both robots still fail, and the trace says why they now fail:

| probe | lift | reaches lip? | ends at | yaw drift |
|---|---|---|---|---|
| step_up_0.02 | 0 | yes | reaches goal 2 | within ±10° |
| step_up_0.04 | 0 | yes, jams | x 4.09, rolls in place at y 1.54 | +4 → −28° |
| step_up_0.02 | 60 mm | yes | **y = 4.16, outside the 0–4 m lane** | +2 → −12° |
| step_up_0.04 | 60 mm | **no, x = 3.92** | **y = 4.03, outside the lane** | +2 → **+47°** |

The lift destabilises heading, and the robot leaves the lane before the step matters. The
mechanism is in the lift itself: it resolves a per-leg "need", and the needs are not
left-right symmetric — FL +39.7 mm vs FR +28.4 mm, RL +60.0 mm vs RR +51.4 mm. An
asymmetric lift is a steering input. **This is why the foot-height → step-height curve came
out flat**: below 40 mm the edit does not reach the ground, and above it the edit steers.

## 3. TROT does not reach the step at all

Every TROT step-up run ends **short of the lip at x = 4.0**:

| probe | heading cap | upright until | ends at x | ends at y | \|ψ\| max | cap saturated |
|---|---|---|---|---|---|---|
| 0.02 | 0.02 (spec) | 7.16 s | 4.29 | **3.98** | 30.6° | **87.3%** before the lip |
| 0.04 | 0.02 (spec) | 5.98 s | **3.58** | 3.39 | 29.1° | **83.9%** |
| 0.02 | 0.04 | 18.00 s | 3.69 | 3.38 | 66.8° | 66.2% |
| 0.04 | 0.04 | 6.64 s | 3.93 | 3.41 | 14.4° | 46.6% |
| 0.02 | 0.08 | 8.88 s | 3.45 | 3.39 | 47.8° | 21.5% |
| 0.06 | 0.08 | 6.16 s | 3.62 | 3.30 | 17.4° | 5.0% |

The lane is 4 m wide and TROT spawns at its centre, y = 2.0. It arrives at the lip having
drifted to **y = 3.4–4.0** — at or over the edge. With the gutters correctly void
(`harness_findings.md` §9) stepping over the edge is a fall, and that is what these rows
are scoring.

Answering the three sub-questions directly:

- **Saturation before vs after the lip: the same, because the problem is before it.** The
  heading term sits at its ±0.02 cap **84–87% of leg-frames on the flat approach**, and
  87% → 82% across the lip. Nothing is invalidated at the lip; it was already pinned.
- **Contact detection on the raised surface is fine.** The vertical channel weighs the
  robot correctly on the step top as on the flat, and foot z on the step reads
  0.064 = riser 0.040 + foot radius 0.023, which is right.
- **T_stance, the swing gate and the heading reference do not drift at the lip.** The gate
  is the clip's own contact channel and is clip-time, not terrain-dependent; the heading
  reference is the per-cell settled yaw and is fixed at reset.

Raising the cap removes the saturation (87% → 5.0% at ±0.08) and **makes it worse**: peak
heading error grows to 47–67° and every robot still falls. The cap is not the shortfall.
This confirms `heading_hold.md` from the other side — TROT holds 3.20 °/m, 5.7× the
budget, and over the 3 m approach that predicts 9.6° of yaw error; the trace measures
9.8–10.3°. Two independent measurements, one number.

## 4. Consequence for the calibration

**`STEP_TROT_MAX` is not measurable until TROT holds a line.** Its 0/N at every level is a
lane-departure score, not a step limit, and no amount of repeats will turn it into one.
`STEP_WALK_MAX` is measurable — WALK holds heading at 0.24 °/m — and the mechanism sets it
at the foot radius: passes 0.02, fails 0.04, because the rear feet drag.

The open item that matters is therefore not clearance. It is that **WALK's rear legs do not
swing** and TROT does not go straight.
