"""A roll couple made of stance-leg forces -- the dual of ``sim/yawmoment.py``.

WHY ROLL, AND ONLY ROLL
-----------------------
Measured before this was written, WALK on all 200 legged_eval cells, both roughness arms
(``outputs/bench_le/axis_rough.csv`` / ``axis_norough.csv``):

    roughness ON    ended on roll 165,  on pitch   0,  below floor  6,  timeout 29
    roughness OFF   ended on roll 125,  on pitch   0,  below floor 11,  timeout 64

    peak |roll|   median 88 deg  (the cutoff is 1.5 rad = 85.9)
    peak |pitch|  median 15 deg, p90 30-37 -- never within 45 deg of the cutoff

**Every fall is a roll fall.  There has been no pitch termination, in either arm, in 400
episodes.**  So this file has no pitch term.  A pitch regulator would be a term with
nothing to do, and it would have had to live on the THIGH channel, which is the one the
gait itself is driving -- the risk was real and the measurement removed it rather than
managing it.  If a pitch failure ever appears, that measurement is the trigger to add one,
and it is cheap to re-run.

THE MECHANISM, AND WHY IT IS THE SAME CHANNEL AS THE YAW COUPLE
---------------------------------------------------------------
``yawmoment.py`` derives what a hip (abduction) torque does on a leg that is on the
ground.  The load path is through the contact, so the torque buys a lateral force at the
foot:

    f_y,i = -tau_i / lever_i          lever_i = hip-to-foot vertical drop

and that force, applied at the foot, gives about the base

    roll moment  M_x = sum_i lever_i * f_y,i = -sum_i tau_i
    yaw  moment  M_z = sum_i x_i     * f_y,i = -sum_i x_i * tau_i / lever_i
    lateral force F_y = sum_i f_y,i          = -sum_i tau_i / lever_i

**The two terms are exact duals and that is why they can share one channel.**

    yawmoment   tau_i = c * sign(x_i)   ->  M_x is EXACTLY zero (the levers cancel
                                            against themselves), M_z is the signal
    attitude    tau_i = c               ->  M_z is zero to the front/rear mismatch in
                                            x_i / lever_i, M_x = -4c is the signal

Neither is a foot placement, so neither is bounded by how far a gait's footfall may be
displaced -- the bound is the friction cone and the hip's effort limit.  ``yawmoment.py``
measured 11.95 N.m of peak hip torque against the 23.70 clip with **0 of 2000 control
steps clipped**, which is the headroom this term is spending.

F_y IS THE MECHANISM, NOT A SIDE EFFECT
---------------------------------------
Unlike the yaw couple, this one has a NET LATERAL FORCE, and it is not small:
F_y = M_x / lever, about 3.2 N per N.m of summed hip torque.  That is not a defect to be
cancelled.  A quadruped rights a roll by pushing the ground sideways; the reaction both
rights the body and translates it, and the two are the same event seen twice.  A term that
produced a roll moment with no lateral force would not be correcting a lean, it would be
twisting about it.

What this does mean is that the term moves the robot sideways while it acts, so the
lateral drift is a thing to measure and not to assume away -- ``run_benchmark.py`` records
``final_y_m`` and the calibration rig records cross-track, and both are read in the
falsification test.

THE SIGN IS DERIVED HERE AND NOT TRUSTED
----------------------------------------
``sim/footcomp.py`` records that the derived sign of the heading law was positive feedback
and had to be flipped after a run; CLAUDE.md 6.5 says the joint frame is not a reliable
guide.  The derivation says a positive roll (right side down, by ``quat_to_rpy_deg``)
wants a negative ``c``, hence the minus in ``step()``.  ``sign`` flips it, and the
OPEN-LOOP probe settles it -- run at a fixed ``bias_nm`` with the loop off and see which
way the robot rolls.  A closed loop with the wrong sign does not error, it falls faster.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class RollCouple:
    """Per-step hip feed-forward torque regulating base roll on the stance legs.

    ``lever_m`` and ``hip_x_m`` come from the articulation, not from a table -- the same
    two quantities ``YawMoment`` reads, for a third meaning.
    """

    lever_m: np.ndarray                   # (4,) hip-to-foot vertical drop
    hip_x_m: np.ndarray                   # (4,) hip fore-aft offset, body frame
    hip_y_m: Optional[np.ndarray] = None  # (4,) lateral offset; reporting only
    bias_nm: float = 0.0                  # open-loop probe amplitude, constant
    kp_nm_per_rad: float = 0.0            # closed loop: c = -(kp*roll + kd*roll_rate)
    kd_nm_per_rad_s: float = 0.0
    cap_nm: float = 0.0                   # magnitude bound on c (0 = off => term is off)
    sign: float = 1.0                     # flip if the open-loop probe says so
    effort_limit_nm: float = 23.7         # the hip's own clip
    headroom_frac: float = 0.5            # most of the limit this term may ask for
    shared_with_nm: float = 0.0           # cap another term already holds on this joint
    # -- running state
    applied: int = 0
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
        if self.sign not in (1.0, -1.0):
            raise ValueError(f"sign must be +1 or -1, got {self.sign}")
        # The cap is bounded by the ACTUATOR and by whatever else is already on this
        # joint.  IdealPDActuator clips the SUM of the PD and both feed-forward terms at
        # effort_limit and says nothing when it does, so two terms each sized against the
        # whole budget would silently eat the position tracking that produces the gait.
        # This is the check yawmoment.py has, plus the term it did not have to share with.
        lim = self.headroom_frac * self.effort_limit_nm
        if self.cap_nm + self.shared_with_nm > lim:
            raise ValueError(
                f"roll-couple cap {self.cap_nm:.2f} Nm plus the {self.shared_with_nm:.2f} "
                f"Nm already committed on the hip is more than {self.headroom_frac:.0%} of "
                f"the {self.effort_limit_nm:.2f} Nm effort limit. The PD that plays the "
                f"clip needs that headroom; raising this instead of the limit is the same "
                f"mistake in a different column.")
        self.reset()

    def reset(self) -> None:
        self.applied = self.cap_hits = 0
        self.max_abs_nm = 0.0
        self.last_tau = np.zeros(12)
        self.last_c_nm = 0.0
        self.last_stance_n = 0

    # ------------------------------------------------------------------ step
    def step(self, swing: np.ndarray, roll_rad: float = 0.0,
             roll_rate_rad_s: float = 0.0) -> np.ndarray:
        """The 12-vector of FEED-FORWARD TORQUES (N.m) for this control step.

        Only the hip columns are ever non-zero, and only for legs the RECORDING has on
        the ground -- torque into a swing leg buys nothing (no contact, no load path) and
        disturbs the trajectory that is the gait.
        """
        tau = np.zeros(12, dtype=np.float64)
        if self.cap_nm <= 0.0:
            self.last_tau, self.last_c_nm, self.last_stance_n = tau, 0.0, 0
            return tau
        stance = ~np.asarray(swing, dtype=bool)
        c = self.bias_nm - self.sign * (self.kp_nm_per_rad * float(roll_rad)
                                        + self.kd_nm_per_rad_s * float(roll_rate_rad_s))
        if abs(c) > self.cap_nm:
            self.cap_hits += int(stance.sum())
            c = float(np.clip(c, -self.cap_nm, self.cap_nm))
        # UNIFORM over the stance legs: that is what makes M_x add and M_z cancel.  The
        # yaw couple's sign(x_i) is the other choice and gives the other moment.
        tau[0::3] = np.where(stance, c, 0.0)
        self.applied += int(stance.sum())
        self.max_abs_nm = max(self.max_abs_nm, float(np.abs(tau).max()))
        self.last_tau, self.last_c_nm = tau, c
        self.last_stance_n = int(stance.sum())
        return tau

    # ------------------------------------------------------- what it is worth
    def roll_nm_per_nm(self, swing: np.ndarray) -> float:
        """``M_x`` per N.m of ``c``, for the legs the clip has down right now.

        Exactly -(number of stance legs): the levers cancel, which is why this one is
        clean and the yaw residual is not.
        """
        stance = ~np.asarray(swing, dtype=bool)
        return float(-stance.sum())

    def yaw_residual_nm_per_nm(self, swing: np.ndarray) -> float:
        """``M_z`` per N.m of ``c`` -- the side effect, which should be ~0."""
        stance = ~np.asarray(swing, dtype=bool)
        return float(-np.sum(self.hip_x_m[stance] / self.lever_m[stance]))

    def lateral_force_n_per_nm(self, swing: np.ndarray) -> float:
        """``F_y`` per N.m of ``c``.  Reported because it is the MECHANISM, not noise."""
        stance = ~np.asarray(swing, dtype=bool)
        return float(-np.sum(1.0 / self.lever_m[stance]))

    @property
    def cap_hit_frac(self) -> float:
        return (self.cap_hits / self.applied) if self.applied else 0.0


def _self_test() -> int:
    """Against the robot's own measured geometry, not a nominal one."""
    lever = np.array([0.306, 0.306, 0.316, 0.316])
    hip_x = np.array([0.193, 0.193, -0.193, -0.193])
    hip_y = np.array([0.110, -0.110, 0.110, -0.110])
    ALL_SWING = np.ones(4, bool)
    NONE_SWING = np.zeros(4, bool)
    DIAG = np.array([False, True, True, False])       # FL + RR down, a trot's pair
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

    # 1. OFF BY DEFAULT.  Every result in this project predates this term and has to be
    #    reproducible with it present.  cap_nm defaults to 0, which is the whole switch.
    rc0 = RollCouple(lever_m=lever, hip_x_m=hip_x, hip_y_m=hip_y,
                     kp_nm_per_rad=10.0, kd_nm_per_rad_s=1.0)
    ok("default is OFF: zero torque even with a large roll error",
       float(np.abs(rc0.step(NONE_SWING, roll_rad=0.5, roll_rate_rad_s=2.0)).max()), 0.0)
    ok("...and with an open-loop bias set but no cap",
       float(np.abs(RollCouple(lever_m=lever, hip_x_m=hip_x, bias_nm=5.0)
                    .step(NONE_SWING)).max()), 0.0)

    rc = RollCouple(lever_m=lever, hip_x_m=hip_x, hip_y_m=hip_y, cap_nm=4.0, bias_nm=2.0)

    # 2. STANCE ONLY, HIPS ONLY.  This is the line between this term and every foot
    #    placement term: disjoint in joints, so they cannot compete for the authority.
    t = rc.step(NONE_SWING)
    ok("all four in stance: every hip gets the SAME c", float(t[0]), 2.0)
    ok("...including the rear ones (no sign(x) here -- that is the yaw couple)",
       float(t[6]), 2.0)
    ok("thighs untouched", float(np.abs(t[1::3]).max()), 0.0)
    ok("calves untouched", float(np.abs(t[2::3]).max()), 0.0)
    ok("nothing at all on a fully airborne step",
       float(np.abs(rc.step(ALL_SWING)).max()), 0.0)
    t = rc.step(DIAG)
    yes("a trot's diagonal pair is driven and the swinging pair is not",
        t[0] == 2.0 and t[9] == 2.0 and t[3] == 0.0 and t[6] == 0.0)

    # 3. THE DUAL PROPERTY.  This is the claim that lets it share the hip with the yaw
    #    couple: uniform c gives roll and (almost) no yaw, sign(x) gives yaw and no roll.
    ok("roll per N.m, all four down, is exactly -4", rc.roll_nm_per_nm(NONE_SWING), -4.0)
    ok("roll per N.m on a diagonal pair is exactly -2", rc.roll_nm_per_nm(DIAG), -2.0)
    yes("yaw residual is under 2% of the roll it is buying, all four down",
        abs(rc.yaw_residual_nm_per_nm(NONE_SWING)) < 0.02 * 4.0)
    from sim.yawmoment import YawMoment
    ym = YawMoment(lever_m=lever, hip_x_m=hip_x, hip_y_m=hip_y, cap_nm=2.0, bias_nm=1.0)
    ty = ym.step(NONE_SWING, 0.0)
    yes("the yaw couple's own roll is exactly zero -- the two are duals, not rivals",
        abs(float(np.sum(ty[0::3]))) < 1e-12)

    # 4. F_y IS REPORTED, NOT HIDDEN.  It is about 3.2 N per N.m and it is the mechanism.
    fy = rc.lateral_force_n_per_nm(NONE_SWING)
    yes(f"lateral force is reported and is the size the geometry says ({fy:.2f} N per N.m)",
        -13.5 < fy < -12.5)

    # 5. THE SIGN IS A SWITCH, because the derivation is not trusted (footcomp.py).
    a = RollCouple(lever_m=lever, hip_x_m=hip_x, cap_nm=4.0, kp_nm_per_rad=10.0)
    b = RollCouple(lever_m=lever, hip_x_m=hip_x, cap_nm=4.0, kp_nm_per_rad=10.0, sign=-1.0)
    ok("a positive roll asks for a negative c (derived)",
       float(a.step(NONE_SWING, roll_rad=0.1)[0]), -1.0)
    ok("...and the flip is exactly the negation",
       float(b.step(NONE_SWING, roll_rad=0.1)[0]), 1.0)

    # 6. THE CAP IS BOUNDED BY THE ACTUATOR, AND BY THE TERM ALREADY ON THE JOINT.
    try:
        RollCouple(lever_m=lever, hip_x_m=hip_x, cap_nm=20.0)
        yes("a cap over half the effort limit is refused", False)
    except ValueError:
        yes("a cap over half the effort limit is refused", True)
    try:
        RollCouple(lever_m=lever, hip_x_m=hip_x, cap_nm=10.0, shared_with_nm=2.0)
        yes("a cap that fits alone but not beside the yaw couple is refused", False)
    except ValueError:
        yes("a cap that fits alone but not beside the yaw couple is refused", True)
    yes("2.0 beside the yaw couple's 2.0 is allowed (4.0 of the 11.85 budget)",
        RollCouple(lever_m=lever, hip_x_m=hip_x, cap_nm=2.0,
                   shared_with_nm=2.0).cap_nm == 2.0)

    # 7. THE CAP COUNTER, which is what says the loop had somewhere to sit.
    r = RollCouple(lever_m=lever, hip_x_m=hip_x, cap_nm=1.0, kp_nm_per_rad=100.0)
    r.step(NONE_SWING, roll_rad=0.5)                   # asks for 50, gets 1
    yes("asking past the cap is counted", r.cap_hit_frac == 1.0)
    r.reset(); r.step(NONE_SWING, roll_rad=0.001)      # asks for 0.1
    yes("asking inside the cap is not", r.cap_hit_frac == 0.0)

    print(f"\nroll-couple self-test: {'PASS' if fails == 0 else f'{fails} FAILURES'}")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(_self_test())
