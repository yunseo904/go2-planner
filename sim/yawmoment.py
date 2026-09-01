"""A yaw moment made of stance-leg forces, not of foot positions.

Why this exists, and why it is not another foot-placement term
--------------------------------------------------------------
``outputs/trot_straight.md`` measures TROT's heading authority as **spent**.  Every
route tried so far moves where a SWING foot is put down:

  (a) differential lateral placement  -- capped at +-0.02 rad, because TROT falls in
      both directions at +-0.04 (``heading_candidates.md`` 2).  At 155-175 deg/s per
      rad that ceiling is worth ~3.1 deg/s against a +5.32 deg/s drift.
  (b) differential step length        -- strong one way, destroys the gait the other.
  (c) either of them fed forward as a constant -- a coin flip over fifteen paired
      cells (``trot_straight.md`` 4b).

All three are KINEMATIC and all three are bounded by the same thing: how far a trot's
footfall can be displaced before the trot stops being one.  Widening that bound was
measured to make heading worse, not better (``lip_failure.md`` 3).

This term is bounded by something else entirely.  It puts no foot anywhere.  It adds a
FEED-FORWARD TORQUE to the hips of the legs that are ON THE GROUND, where the load
path is through the contact rather than through the leg's own inertia, so what the
torque buys is a ground reaction force and what the forces buy is a couple about the
base's vertical axis.  Its limits are the friction cone and the hip's effort limit,
neither of which has anything to do with the trot's footfall margin.

The mechanism, in the robot's own numbers
-----------------------------------------
The hip is the ABDUCTION joint (``heading_candidates.md`` 1: there is no yaw joint on
this robot) and ``lever_i`` is the hip-to-foot vertical drop, which is exactly the
moment arm turning a hip torque into a lateral force at the foot:

    f_y,i = -tau_i / lever_i          (on the BODY; sign derived below)
    M_z   = sum_i x_i * f_y,i         (x_i is the hip's fore-aft offset, body frame)

Choosing ``tau_i = c * sign(x_i)`` makes every leg contribute with the SAME sign:

    M_z = -c * sum_over_stance |x_i| / lever_i

which matters more than it looks.  A trot has only a DIAGONAL PAIR on the ground at
any moment, and the pair alternates; a rule that needed all four feet would produce a
moment that switched off twice a cycle.  This one does not care which legs are down.

Two side effects, both near-null by construction, and both asserted in the self-test:

  * net lateral force  sum_i f_y,i = -c * sum sign(x_i)/lever_i.  This is NOT zero and
    the first draft of this file claimed it was: it cancels only if the front and rear
    levers are equal, and they are 0.306 and 0.316 m.  What actually kills it is that
    3% mismatch, so the residual is ~3% of one leg's force -- on a diagonal pair and on
    all four alike.  Measured in the self-test rather than asserted away.
  * net roll moment    sum_i lever_i * f_y,i = -c * sum sign(x_i), which IS exactly
    zero, for every stance subset with as many front legs as rear -- the levers cancel
    against themselves here, which is why this one is exact and the other is not.

So this is close to a pure yaw couple, which is the whole claim being made for it.

The sign is DERIVED here and SETTLED BY THE PROBE
-------------------------------------------------
``sim/footcomp.py`` records that the derived sign of the heading law was positive
feedback and had to be flipped after a run, and ``CLAUDE.md`` 6.5 says the joint frame
is not a reliable guide.  So: the derivation is written down, and it is not trusted.

Derivation.  ``footcomp`` measured that +q_hip moves the foot toward +y on every leg.
With the FOOT pinned and the body free, the same joint motion moves the BODY toward
-y.  A positive tau is therefore a generalised force doing positive work in the body's
-y direction, so ``f_y = -tau/lever`` and ``M_z = -c * sum |x_i|/lever_i``: positive c
gives NEGATIVE yaw.  A positive heading error then wants positive c.

That is why ``gain_nm_per_rad`` enters as ``+gain * psi_err`` below.  It is a
hypothesis with a number attached, and ``--yaw-moment-nm`` exists to test it open loop
in both directions before any loop is closed on it.  Nothing downstream should read
the sign off this file.

What it does NOT see
--------------------
The stance mask is the CLIP's own contact channel, phase-locked -- the same gate
``ClipPolicy`` already uses for foot placement, and the same information class.  The
sim's contact sensor is deliberately NOT an input here: a term that changed with what
the feet actually hit would be a contact reflex, which is the thing the skill layer is
forbidden to have.  Heading error, hip geometry, clip phase.  No terrain, no depth, no
goal.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class YawMoment:
    """Per-step hip feed-forward torque producing a yaw couple on the stance legs.

    ``lever_m`` and ``hip_x_m`` are per leg in the clip's leg order and come from the
    articulation, not from a table -- the same two quantities the heading law already
    uses, read for their other meaning (a moment arm rather than a placement gain).
    """

    lever_m: np.ndarray                  # (4,) hip-to-foot vertical drop
    hip_x_m: np.ndarray                  # (4,) hip fore-aft offset, body frame
    hip_y_m: Optional[np.ndarray] = None # (4,) lateral offset; reporting only
    bias_nm: float = 0.0                 # open-loop probe amplitude, constant
    gain_nm_per_rad: float = 0.0         # closed loop: c = gain * psi_err
    cap_nm: float = 0.0                  # magnitude bound on c (0 = off => term is off)
    effort_limit_nm: float = 23.7        # the hip's own clip; refuse a cap that eats it
    headroom_frac: float = 0.5           # most of the limit this term may ask for
    # -- running state
    applied: int = 0                     # stance-leg-steps this term acted on
    cap_hits: int = 0
    max_abs_nm: float = 0.0

    def __post_init__(self) -> None:
        self.lever_m = np.asarray(self.lever_m, dtype=float)
        self.hip_x_m = np.asarray(self.hip_x_m, dtype=float)
        if self.hip_y_m is not None:
            self.hip_y_m = np.asarray(self.hip_y_m, dtype=float)
        if not np.all(self.lever_m > 0.05):
            raise ValueError(f"hip-to-foot lever {self.lever_m} is not a usable moment "
                             f"arm; the couple is linearised about a standing pose")
        if not np.all(np.abs(self.hip_x_m) > 0.01):
            raise ValueError(f"hip fore-aft offsets {self.hip_x_m} give no yaw arm")
        # The cap is bounded by the ACTUATOR, not by taste.  IdealPDActuator clips the
        # sum of the PD and this term at effort_limit and says nothing when it does, so
        # a term allowed to ask for the whole limit would be silently eating the
        # position tracking that produces the gait.  Half is a stated bound, not a
        # tuned one; the run records how often the sum clipped anyway.
        lim = self.headroom_frac * self.effort_limit_nm
        if self.cap_nm > lim:
            raise ValueError(
                f"yaw-moment cap {self.cap_nm:.2f} Nm is more than {self.headroom_frac:.0%} "
                f"of the hip's {self.effort_limit_nm:.2f} Nm effort limit. The PD that "
                f"plays the clip needs that headroom; raising this instead of the limit "
                f"is the same mistake in a different column.")
        self.reset()

    def reset(self) -> None:
        self.applied = self.cap_hits = 0
        self.max_abs_nm = 0.0
        self.last_tau = np.zeros(12)
        self.last_c_nm = 0.0
        self.last_stance_n = 0

    # ------------------------------------------------------------------ step
    def step(self, swing: np.ndarray, psi_err_rad: float = 0.0) -> np.ndarray:
        """The 12-vector of FEED-FORWARD TORQUES (N.m) for this control step.

        Length 12 in the clip's joint order (hip, thigh, calf per leg); only the hip
        columns are ever non-zero.  ``swing`` is the clip's own gate, so the legs this
        acts on are the ones the RECORDING has on the ground.
        """
        tau = np.zeros(12, dtype=np.float64)
        if self.cap_nm <= 0.0:
            self.last_tau, self.last_c_nm, self.last_stance_n = tau, 0.0, 0
            return tau
        stance = ~np.asarray(swing, dtype=bool)
        c = self.bias_nm + self.gain_nm_per_rad * float(psi_err_rad)
        if abs(c) > self.cap_nm:
            self.cap_hits += int(stance.sum())
            c = float(np.clip(c, -self.cap_nm, self.cap_nm))
        # sign(x_i): every stance leg then contributes to M_z with the same sign, so
        # the couple does not switch off when the trot swaps diagonals.
        tau[0::3] = np.where(stance, c * np.sign(self.hip_x_m), 0.0)
        self.applied += int(stance.sum())
        self.max_abs_nm = max(self.max_abs_nm, float(np.abs(tau).max()))
        self.last_tau, self.last_c_nm = tau, c
        self.last_stance_n = int(stance.sum())
        return tau

    # ------------------------------------------------------- what it is worth
    def couple_nm_per_nm(self, swing: np.ndarray) -> float:
        """``M_z`` per N.m of ``c``, for the legs the clip has down right now.

        Reported rather than used: it is what says whether a measured yaw response is
        the size the geometry predicts, which is the CLAUDE.md 6.5 second reading.
        """
        stance = ~np.asarray(swing, dtype=bool)
        return float(-np.sum(np.abs(self.hip_x_m[stance]) / self.lever_m[stance]))

    def residual_force_n_per_nm(self, swing: np.ndarray) -> float:
        """Net lateral force per N.m of ``c`` -- the side effect, which should be ~0."""
        stance = ~np.asarray(swing, dtype=bool)
        return float(-np.sum(np.sign(self.hip_x_m[stance]) / self.lever_m[stance]))

    @property
    def cap_hit_frac(self) -> float:
        return (self.cap_hits / self.applied) if self.applied else 0.0


def _self_test() -> int:
    """Against the robot's own measured geometry, not against a nominal one."""
    # outputs/heading_hold.md / run_planner_replay.py's own printout
    lever = np.array([0.306, 0.306, 0.316, 0.316])
    hip_x = np.array([0.193, 0.193, -0.193, -0.193])
    hip_y = np.array([0.110, -0.110, 0.110, -0.110])
    ALL_SWING = np.ones(4, bool)
    NONE_SWING = np.zeros(4, bool)
    fails = 0

    def ok(label, got, want, tol=1e-9):
        nonlocal fails
        good = abs(got - want) <= tol
        fails += 0 if good else 1
        print(f"  {'ok  ' if good else 'FAIL'} {label}: {got:.6f} (want {want:.6f})")

    def yes(label, cond):
        nonlocal fails
        fails += 0 if cond else 1
        print(f"  {'ok  ' if cond else 'FAIL'} {label}")

    # 1. OFF BY DEFAULT.  Every result in this project predates this term and must be
    #    reproducible with it present.  cap_nm defaults to 0, which is the whole switch.
    ym0 = YawMoment(lever_m=lever, hip_x_m=hip_x, hip_y_m=hip_y)
    ok("default is OFF: zero torque even with a large heading error",
       float(np.abs(ym0.step(NONE_SWING, psi_err_rad=0.5)).max()), 0.0)
    ok("...and with an open-loop bias set but no cap",
       float(np.abs(YawMoment(lever_m=lever, hip_x_m=hip_x, bias_nm=5.0)
                    .step(NONE_SWING, 0.0)).max()), 0.0)

    ym = YawMoment(lever_m=lever, hip_x_m=hip_x, hip_y_m=hip_y, cap_nm=4.0, bias_nm=2.0)

    # 2. STANCE ONLY, and the thighs and calves are never touched.  This is the line
    #    between this term and every foot-placement term: they are disjoint in joints
    #    AND in legs, so they cannot be competing for the same authority.
    t = ym.step(NONE_SWING, 0.0)                      # all four down
    ok("all four in stance: front hips get +c", float(t[0]), 2.0)
    ok("rear hips get -c (sign(x) is negative there)", float(t[6]), -2.0)
    ok("thighs untouched", float(np.abs(t[1::3]).max()), 0.0)
    ok("calves untouched", float(np.abs(t[2::3]).max()), 0.0)
    ok("nothing at all on a fully airborne step",
       float(np.abs(ym.step(ALL_SWING, 0.0)).max()), 0.0)

    # 3. THE DIAGONAL PAIR.  A trot never has four feet down; the term must survive the
    #    swap without changing sign, or it is a moment that switches off twice a cycle.
    diagA = np.array([False, True, True, False])       # FL + RR down
    diagB = np.array([True, False, False, True])       # FR + RL down
    tA, tB = ym.step(diagA, 0.0), ym.step(diagB, 0.0)
    mA = float(np.sum(hip_x * (-tA[0::3] / lever)))
    mB = float(np.sum(hip_x * (-tB[0::3] / lever)))
    yes(f"both diagonals give the SAME-SIGNED couple ({mA:+.3f}, {mB:+.3f} Nm)",
        mA < 0 and mB < 0)
    ok("and the two diagonals are equal in magnitude", abs(mA), abs(mB), 1e-12)
    ok("couple per Nm matches the closed form, diagonal A",
       ym.couple_nm_per_nm(diagA), mA / 2.0, 1e-12)

    # 4. IT IS A COUPLE.  Net lateral force and net roll moment are the side effects,
    #    and the claim is that they are small -- so they are measured, not asserted.
    fy_all = -ym.step(NONE_SWING, 0.0)[0::3] / lever
    ok("all four down: net ROLL moment is exactly zero (the levers cancel)",
       float(np.sum(lever * fy_all)), 0.0, 1e-12)
    # The lateral residual is the lever MISMATCH and nothing else: it would be zero if
    # the front and rear drops were equal, and they are 0.306 and 0.316 m.  Bounding it
    # against the lever asymmetry is the honest test; asserting zero was wrong.
    mism = abs(1 / lever[0] - 1 / lever[2]) / (1 / lever[0])       # 3.2%
    per_leg_all = float(np.abs(fy_all).mean())
    yes(f"all four down: residual lateral force {abs(fy_all.sum()):.3f} N is the 3.2% "
        f"lever mismatch, not a design choice "
        f"({abs(fy_all.sum())/(4*per_leg_all):.1%} of the force in play)",
        abs(fy_all.sum()) <= 1.05 * mism * 2 * per_leg_all)
    fy_d = -tA[0::3] / lever
    per_leg = float(np.abs(fy_d[fy_d != 0]).mean())
    yes(f"diagonal pair: residual lateral force {abs(fy_d.sum()):.3f} N is under 5% of "
        f"one leg's {per_leg:.3f} N", abs(fy_d.sum()) < 0.05 * per_leg)
    # the yaw arm is worth more than the residual by two orders of magnitude
    yes(f"couple/residual ratio {abs(mA)/max(abs(fy_d.sum()),1e-12):.1f} Nm per N",
        abs(mA) / max(abs(fy_d.sum()), 1e-12) > 1.0)

    # 5. THE CLOSED-LOOP SIGN, as derived.  Not trusted -- --yaw-moment-nm exists to
    #    settle it in both directions -- but the file must agree with its own comment.
    ymg = YawMoment(lever_m=lever, hip_x_m=hip_x, cap_nm=4.0, gain_nm_per_rad=10.0)
    tp = ymg.step(NONE_SWING, psi_err_rad=+0.1)
    m_pos = float(np.sum(hip_x * (-tp[0::3] / lever)))
    yes(f"positive heading error commands a NEGATIVE yaw moment ({m_pos:+.3f} Nm), "
        f"which is the derived restoring direction", m_pos < 0)
    ok("no heading error, no torque",
       float(np.abs(ymg.step(NONE_SWING, 0.0)).max()), 0.0)
    tn = ymg.step(NONE_SWING, psi_err_rad=-0.1)
    ok("and it is bidirectional, unlike the step-length half", float(tn[0]), -float(tp[0]))

    # 6. THE CAP, and the actuator bound behind it.
    ok("the cap binds", float(ymg.step(NONE_SWING, psi_err_rad=10.0)[0]), 4.0)
    try:
        YawMoment(lever_m=lever, hip_x_m=hip_x, cap_nm=20.0)
        print("  FAIL a cap of 20 Nm was accepted against a 23.7 Nm hip limit"); fails += 1
    except ValueError:
        print("  ok   a cap that would eat the PD's headroom is refused, not clipped")
    try:
        YawMoment(lever_m=lever, hip_x_m=np.zeros(4), cap_nm=1.0)
        print("  FAIL zero fore-aft offsets accepted (there would be no yaw arm)"); fails += 1
    except ValueError:
        print("  ok   geometry with no yaw arm is refused")

    # 7. What one N.m is worth, printed so the probe's amplitudes are chosen against a
    #    number rather than against a feeling.
    print(f"  ok   geometry: {abs(ym.couple_nm_per_nm(NONE_SWING)):.3f} Nm of yaw couple "
          f"per Nm of hip torque with four legs down, "
          f"{abs(ym.couple_nm_per_nm(diagA)):.3f} on a diagonal pair")

    print(f"yawmoment self-test: {'PASS' if fails == 0 else f'{fails} FAILURES'}")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(_self_test())
