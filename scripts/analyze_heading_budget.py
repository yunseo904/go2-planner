#!/usr/bin/env python3
"""Can an open-loop gait that curves at a constant rate pass the benchmark's 8 goals?

    python scripts/analyze_heading_budget.py

Pure geometry.  No simulator, no fitted constant: every input is measured elsewhere
and cited on the line that uses it.  A gait replayed open loop holds a constant yaw
rate, so its ground track is a circular arc of radius ``R = v / yaw_rate = 1 / kappa``.
The question is whether that arc stays inside ``next_goal_threshold`` of eight goal
points spaced along the cell's centre line.

Two ways to aim the arc, because they give very different answers:

* **as launched** -- the robot starts on the line, heading along it.  Deviation after
  arc length s is ``R (1 - cos(s/R))``, one-sided and growing.
* **best aim** -- the planner is allowed to pick the initial heading (and offset) that
  keeps the whole 18 m arc as close to the line as possible; the arc then straddles it
  and the worst deviation is half the sagitta.  This is the most generous reading, and
  it is still not enough.
"""

from __future__ import annotations

import argparse
import math

# --- measured inputs -------------------------------------------------------------
SEG_M = 2.25              # goal spacing, outputs/terrain_profile.csv (CLAUDE.md 3)
N_GOALS = 8               # goals per cell, CLAUDE.md 3
GOAL_TOL_M = 0.2          # next_goal_threshold, upstream legged_robot_config.py:188
CELL_HALF_WIDTH_M = 2.0   # cell is 18 m x 4 m, CLAUDE.md 3

# curvature, deg/m -- outputs/open_loop_replay_limit.md, WALK over its last 20 of 60 cycles
CASES = {
    "WALK open loop, as recorded": 11.8,
    "WALK open loop, symmetrised clip": 1.55,
    "real robot on the same commands": 2.05,
}


def track_deviation(kappa_deg_per_m: float, s: float) -> float:
    """Lateral deviation from the launch line after arc length ``s``."""
    k = math.radians(kappa_deg_per_m)
    if k == 0:
        return 0.0
    R = 1.0 / k
    return R * (1.0 - math.cos(s / R))


def closest_approach(kappa_deg_per_m: float, goal_s: float) -> tuple[float, float]:
    """Min distance from the arc to the goal point at (goal_s, 0), and where it happens."""
    k = math.radians(kappa_deg_per_m)
    if k == 0:
        return 0.0, goal_s
    R = 1.0 / k
    best, best_s = float("inf"), 0.0
    s = 0.0
    while s <= 4.0 * goal_s + 4.0:          # far enough past the goal to pass the minimum
        th = s / R
        x, y = R * math.sin(th), R * (1.0 - math.cos(th))
        d = math.hypot(x - goal_s, y)
        if d < best:
            best, best_s = d, s
        s += 0.001
    return best, best_s


def max_kappa_as_launched(total_m: float, tol: float) -> float:
    """Largest curvature whose one-sided deviation stays within tol over total_m."""
    lo, hi = 0.0, 90.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if track_deviation(mid, total_m) <= tol:
            lo = mid
        else:
            hi = mid
    return lo


def max_kappa_best_aim(total_m: float, tol: float) -> float:
    """Largest curvature that fits in a +/-tol corridor when the arc is centred on it.

    Worst deviation of a chord-centred arc is half the sagitta.
    """
    lo, hi = 0.0, 90.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        k = math.radians(mid)
        R = 1.0 / k
        half = total_m / 2.0
        if R <= half:
            hi = mid
            continue
        sagitta = R - math.sqrt(R * R - half * half)
        if sagitta / 2.0 <= tol:
            lo = mid
        else:
            hi = mid
    return lo


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--turn-rate-deg-s", type=float, default=22.66,
                    help="in-place yaw rate of the turn candidate (default: the measured "
                         "0.3954 rad/s of turn_right_20260824_223951)")
    ap.add_argument("--walk-speed", type=float, default=0.232,
                    help="open-loop WALK forward speed, m/s (converged, 60-cycle run)")
    ap.add_argument("--turn-stride-hz", type=float, default=1.0404,
                    help="stride rate of the turn candidate, Hz (outputs/skill_profile.csv)")
    args = ap.parse_args()

    total = SEG_M * N_GOALS
    print(f"benchmark cell: {N_GOALS} goals x {SEG_M} m = {total:.1f} m, "
          f"goal tolerance {GOAL_TOL_M} m, cell half-width {CELL_HALF_WIDTH_M} m\n")

    print("1. How far off the line does each curvature put the robot?\n")
    print(f"  {'case':36s} {'deg/m':>7s} {'R m':>8s} {'dev @1 seg':>11s} "
          f"{'leaves 0.2 m':>13s} {'leaves cell':>12s}")
    for name, kap in CASES.items():
        R = 1.0 / math.radians(kap)
        dev1 = track_deviation(kap, SEG_M)
        s_tol = next((s / 100 for s in range(1, 4000)
                      if track_deviation(kap, s / 100) > GOAL_TOL_M), float("nan"))
        s_cell = next((s / 100 for s in range(1, 4000)
                       if track_deviation(kap, s / 100) > CELL_HALF_WIDTH_M), float("nan"))
        print(f"  {name:36s} {kap:7.2f} {R:8.2f} {dev1:9.3f} m "
              f"{s_tol:10.2f} m {s_cell:10.2f} m")

    print(f"\n2. Which goals does WALK reach, as launched? "
          f"(closest approach vs the {GOAL_TOL_M} m threshold)\n")
    kap = CASES["WALK open loop, as recorded"]
    print(f"  {'goal':>4s} {'at m':>6s} {'closest approach':>17s} {'reached':>8s}")
    first_missed = None
    for i in range(1, N_GOALS + 1):
        gs = SEG_M * i
        d, _ = closest_approach(kap, gs)
        ok = d <= GOAL_TOL_M
        if not ok and first_missed is None:
            first_missed = i
        print(f"  {i:4d} {gs:6.2f} {d:15.3f} m {'yes' if ok else 'NO':>8s}")
    print(f"\n  first goal missed: {first_missed}  "
          f"-> {first_missed - 1 if first_missed else N_GOALS} of {N_GOALS} goals")

    print("\n3. What curvature would the benchmark actually tolerate?\n")
    k_launch = max_kappa_as_launched(total, GOAL_TOL_M)
    k_aim = max_kappa_best_aim(total, GOAL_TOL_M)
    meas = CASES["WALK open loop, as recorded"]
    print(f"  as launched, over the full {total:.0f} m : <= {k_launch:.3f} deg/m "
          f"-> measured WALK is {meas / k_launch:.0f}x too curved")
    print(f"  best aim, arc centred on the line  : <= {k_aim:.3f} deg/m "
          f"-> measured WALK is {meas / k_aim:.0f}x too curved")
    for name, kv in CASES.items():
        if name.startswith("WALK open loop, as"):
            continue
        print(f"  ({name} at {kv} deg/m is {kv / k_aim:.1f}x the best-aim budget)")

    print("\n4. If heading is corrected discretely instead, what does it cost?\n")
    # Nulling the heading does NOT remove the cross-track offset already accumulated --
    # the offset is monotonic under constant curvature.  The planner has to re-aim at the
    # goal, and the binding constraint is the LAST re-aim before arrival: from distance d
    # out, the arc still bows kappa*d^2/2 away from the straight line to the goal.
    k = math.radians(meas)
    d_max = math.sqrt(2.0 * GOAL_TOL_M / k)
    n_corr = math.ceil(total / d_max)
    d_used = total / n_corr
    turn_deg = meas * d_used
    turn_s = turn_deg / args.turn_rate_deg_s
    walk_s = d_used / args.walk_speed
    cycle_s = 1.0 / args.turn_stride_hz
    per_cycle_deg = args.turn_rate_deg_s * cycle_s
    print(f"  deviation reaches the {GOAL_TOL_M} m tolerance after       : {d_max:.2f} m of walking")
    print(f"  so the planner must re-aim at the goal every    : <= {d_max:.2f} m")
    print(f"  corrections needed over the {total:.0f} m cell          : {n_corr} "
          f"(one per {d_used:.2f} m)")
    print(f"  heading to null at each correction              : {turn_deg:.1f} deg")
    print(f"  in-place turn rate available                    : {args.turn_rate_deg_s:.2f} deg/s "
          f"(turn_right, duty 0.715)")
    print(f"  turn time per correction                        : {turn_s:.2f} s")
    print(f"  walking time between corrections                : {walk_s:.2f} s "
          f"(at {args.walk_speed} m/s)")
    print(f"  time overhead                                   : {100 * turn_s / walk_s:.0f} %")
    print(f"  extra skill switches over the cell              : {2 * n_corr} "
          f"(out of WALK and back)")
    print(f"  settle cost at 0.21 s per switch                : {0.21 * 2 * n_corr:.1f} s")
    print(f"\n  Quantisation: the turn clip is one cycle of {cycle_s:.2f} s = "
          f"{per_cycle_deg:.1f} deg.")
    print(f"  A {turn_deg:.1f} deg correction is {turn_deg / per_cycle_deg:.2f} cycles, so either the "
          f"planner\n  over-turns to a whole cycle or a partial cycle is played and the clip ends "
          f"off\n  its seam. Which of those is acceptable is an open question, not a settled one.")
    print("\n  Note: the correction is a PLANNER decision on an observed heading, not "
          "feedback\n  injected into the joint replay. Each skill still plays open loop.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
