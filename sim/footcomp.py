"""The Raibert foot-placement law, as a stepper that owns no simulator.

Extracted so the planner's clip policy and the replay harness compute the same
correction rather than two copies that drift.  Like ``sim/diagnose.py`` this is
numpy only -- no Isaac Lab, no torch -- so it imports on a laptop.

The law, per leg, in the swing phase only:

    foot_y_i = (T_stance / 2 + k) * dv_y_i
    dv_y_i   = (v_y - v_y_log) + (omega_z - omega_log) * x_i     [yaw mode on]
             =  v_y - v_y_target                                 [yaw mode off]
    dq_hip_i = sign * foot_y_i / lever_i,   clipped to +-cap

``T_stance`` is duty x period measured off the clip's own contact channel, so the
leading term has no free parameter.  ``x_i`` is the hip's fore-aft offset in the
body frame, measured from the articulation.  ``lever_i`` is the hip-to-foot
vertical drop, which is exactly dy/dq for the abduction joint.  ``v_y_log`` and
``omega_log`` are the recording's own steady values.  Nothing here is chosen to
make a number come out.

Why the yaw term is per leg: a base turning at omega carries the hip at
(x_i, y_i) sideways at omega * x_i, so front and rear hips need opposite lateral
placement to support the same turn.  Driving all four to v_y = 0 is a command to
stop turning -- measured, it cut TURN's yaw rate from -14.7 to -8.0 deg/s
(outputs/turn_target.md).

Why the yaw rate is averaged over a cycle: the instantaneous signal on a working
trot is +5.32 deg/s with sd 20.70 -- 3.9x the bias it should correct -- and
feeding it back raw turns a 60-cycle run into a 2.71 s one.  The one-cycle mean
keeps the mean at +5.32 and cuts the sd to 3.64.  The window is the clip's own
period, so it is not a tuned filter.

This CLOSES A LOOP on base velocity.  A run using it is not an open-loop replay,
and callers are expected to say so out loud.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


def stance_time_s(contact: np.ndarray, fs_hz: float) -> float:
    """Mean stance duration per cycle from a clip's own ``(n, 4)`` contact channel."""
    c = np.asarray(contact, dtype=float)
    return float(c.mean() * (c.shape[0] / float(fs_hz)))


@dataclass
class FootPlacement:
    """Per-step foot-placement correction for one clip being played.

    ``lever_m`` and ``hip_x_m`` are per leg in the clip's leg order and come from
    the robot, not from a table.  Everything else is per clip.
    """

    t_stance_s: float
    lever_m: np.ndarray                 # (4,) hip-to-foot vertical drop
    hip_x_m: np.ndarray                 # (4,) hip fore-aft offset, body frame
    hip_y_m: Optional[np.ndarray] = None    # (4,) hip lateral offset, body frame
    vy_log: float = 0.0
    wz_log: float = 0.0
    vx_log: float = float("nan")
    k_s: float = 0.0
    cap_rad: float = 0.05
    sign: float = 1.0
    yaw_mode: str = "off"               # off | log | log-cycle | heading | heading-only
    heading_cap_rad: float = 0.0        # separate cap for the heading term (0 = same as cap)
    axis: str = "y"                     # y | xy
    vy_target: float = 0.0
    cycle_len: int = 0                  # control steps in one clip cycle
    yaw_bias: float = 0.0               # open-loop steering probe, hips
    len_bias: float = 0.0               # open-loop steering probe, thighs
    # -- running state
    _wz_buf: Optional[np.ndarray] = field(default=None, repr=False)
    _wz_sum: float = 0.0
    _wz_i: int = 0
    _wz_n: int = 0
    _wz: float = 0.0
    applied: int = 0
    cap_hits: int = 0
    max_abs: float = 0.0

    def __post_init__(self) -> None:
        self.lever_m = np.asarray(self.lever_m, dtype=float)
        self.hip_x_m = np.asarray(self.hip_x_m, dtype=float)
        if self.hip_y_m is not None:
            self.hip_y_m = np.asarray(self.hip_y_m, dtype=float)
        if not np.all(self.lever_m > 0.05):
            raise ValueError(f"hip-to-foot lever {self.lever_m} is not a usable moment arm; "
                             f"the correction is linearised about a standing pose")
        if self.yaw_mode == "log-cycle" and self.cycle_len < 1:
            raise ValueError("--foot-yaw log-cycle needs the clip's cycle length in steps")
        if self.yaw_mode != "off" and not (np.isfinite(self.vy_log) and np.isfinite(self.wz_log)):
            raise ValueError("yaw mode needs the log's own v_y and yaw rate; refusing to "
                             "fall back to zero silently -- that is the assumption being tested")
        self.reset()

    def reset(self) -> None:
        self._wz_buf = np.zeros(max(self.cycle_len, 1))
        self._wz_sum, self._wz_i, self._wz_n, self._wz = 0.0, 0, 0, 0.0
        self.applied = self.cap_hits = 0
        self.max_abs = 0.0

    # ------------------------------------------------------------------ step
    def step(self, vy: float, wz: float, swing: np.ndarray, vx: float = 0.0,
             psi_err_rad: float = 0.0) -> np.ndarray:
        """The 12-vector to ADD to the commanded joint angles this control step.

        ``psi_err_rad`` is the heading error against the commanded heading, used only
        by the heading modes.  It is an angle the body knows about itself; no terrain
        and no depth reach this function.
        """
        swing = np.asarray(swing, dtype=bool)
        u = np.zeros(12, dtype=np.float32)
        u_head = np.zeros(4)

        if self.yaw_mode in ("heading", "heading-only"):
            # omega_target = omega_log - psi_err / T_stance, i.e. spend one stance time
            # returning to the reference.  Substituted into the per-leg error term
            # (omega - omega_target) * x_i, the heading half is
            #
            #     (T_stance/2) * (psi_err / T_stance) * x_i / lever = psi_err * x_i / (2 lever)
            #
            # -- T_stance CANCELS.  The heading correction carries no constant at all,
            # not even a measured one: it is the heading error, the hip's own fore-aft
            # offset, and the hip-to-foot lever.  Front and rear hips get opposite signs
            # from the same error, which is the differential lateral placement the
            # open-loop probe measured at 155-175 deg/s per rad.
            # NEGATIVE. The substitution omega_target = omega_log - psi_err/T_stance
            # yields +psi*x/(2*lever), and that sign is positive feedback: the open-loop
            # steering probe measured +bias (front outboard one way, rear the other) ->
            # +yaw rate, so a positive heading error needs a NEGATIVE bias to come back.
            # Implemented as written it drove WALK's curvature from 13.26 to 19.99 deg/m
            # and made the full-formula TROT fall at 2.12 s. Settled by measurement, the
            # same way the stage-1 attitude PD's sign was.
            u_head = -psi_err_rad * self.hip_x_m / (2.0 * self.lever_m)
            hcap = self.heading_cap_rad or self.cap_rad
            u_head = np.clip(u_head, -hcap, hcap)
        if self.yaw_mode == "heading-only":
            # The rate half dropped.  Every yaw-RATE loop measured made heading worse
            # (outputs/heading_candidates.md 3), so it is separable here rather than
            # carried on faith: `heading` is the full substitution, `heading-only` is
            # the half the evidence supports.
            dvy = np.full(4, vy - self.vy_log)
            dvx = np.zeros(4)
        elif self.yaw_mode != "off":
            if self.yaw_mode in ("log-cycle", "heading"):
                self._wz_sum += wz - self._wz_buf[self._wz_i]
                self._wz_buf[self._wz_i] = wz
                self._wz_i = (self._wz_i + 1) % self._wz_buf.size
                self._wz_n = min(self._wz_n + 1, self._wz_buf.size)
                self._wz = self._wz_sum / self._wz_n        # warms up over one cycle
            else:
                self._wz = wz
            dvy = (vy - self.vy_log) + (self._wz - self.wz_log) * self.hip_x_m
            dvx = ((vx - self.vx_log) - (self._wz - self.wz_log) * (self.hip_y_m
                   if self.hip_y_m is not None else 0.0)
                   if self.axis == "xy" and np.isfinite(self.vx_log) else np.zeros(4))
        else:
            dvy = np.full(4, vy - self.vy_target)
            dvx = (np.full(4, vx - self.vx_log)
                   if self.axis == "xy" and np.isfinite(self.vx_log) else np.zeros(4))

        gain = 0.5 * self.t_stance_s + self.k_s
        raw_hip = self.sign * (gain * dvy) / self.lever_m
        raw_thigh = -self.sign * (gain * np.asarray(dvx, dtype=float)) / self.lever_m
        cap = self.cap_rad
        # The two terms are capped SEPARATELY and then added: the lateral cap is the
        # stage-2 budget for answering v_y, and the heading cap is what the steering
        # probe measured each gait tolerates.  Clipping their sum would let one starve
        # the other, and they answer different questions.
        u[0::3] = np.where(swing, np.clip(raw_hip, -cap, cap) + u_head, 0.0)
        if self.axis == "xy":
            u[1::3] = np.where(swing, np.clip(raw_thigh, -cap, cap), 0.0)

        self.applied += int(swing.sum())
        self.cap_hits += int((swing & (np.abs(raw_hip) > cap)).sum())

        # The open-loop steering probes go on AFTER the feedback term so the two add
        # rather than one overwriting the other, and they are NOT clipped -- the bias
        # is the amplitude being set, so clipping it would measure the clip.
        if self.yaw_bias or self.len_bias:
            u[0::3] += np.where(swing, self.yaw_bias * np.sign(self.hip_x_m), 0.0)
            if self.hip_y_m is not None:
                u[1::3] += np.where(swing, self.len_bias * np.sign(self.hip_y_m), 0.0)
        self.max_abs = max(self.max_abs, float(np.abs(u).max()))
        return u

    @property
    def cap_hit_frac(self) -> float:
        return (self.cap_hits / self.applied) if self.applied else 0.0


def _self_test() -> int:
    """Check the law against the numbers the stage-2 sweep was measured with."""
    lever = np.array([0.306, 0.306, 0.316, 0.316])
    hip_x = np.array([0.193, 0.193, -0.193, -0.193])
    fails = 0

    def ok(label, got, want, tol=1e-9):
        nonlocal fails
        good = abs(got - want) <= tol
        fails += 0 if good else 1
        print(f"  {'ok  ' if good else 'FAIL'} {label}: {got:.6f} (want {want:.6f})")

    # 1. straight clip, yaw off: every leg gets the same correction
    fp = FootPlacement(t_stance_s=0.372, lever_m=lever, hip_x_m=hip_x, cap_rad=1.0)
    u = fp.step(vy=0.2, wz=0.0, swing=np.ones(4, bool))
    ok("hip correction, v_y 0.2, T_st 0.372", float(u[0]), 0.5 * 0.372 * 0.2 / 0.306)
    ok("thigh untouched", float(np.abs(u[1::3]).max()), 0.0)

    # 2. the cap binds and only the swing legs move
    fp2 = FootPlacement(t_stance_s=0.372, lever_m=lever, hip_x_m=hip_x, cap_rad=0.02)
    u2 = fp2.step(vy=0.5, wz=0.0, swing=np.array([True, False, True, False]))
    ok("clipped to the cap", float(u2[0]), 0.02)
    ok("stance leg untouched", float(u2[3]), 0.0)
    ok("cap-hit fraction", fp2.cap_hit_frac, 1.0)

    # 3. TURN: the per-leg target is the rotation's, front and rear opposite
    fp3 = FootPlacement(t_stance_s=0.600, lever_m=lever, hip_x_m=hip_x,
                        vy_log=0.0143, wz_log=-0.3954, yaw_mode="log", cap_rad=1.0)
    u3 = fp3.step(vy=0.0143, wz=-0.3954, swing=np.ones(4, bool))
    ok("on target, front", float(u3[0]), 0.0, 1e-12)
    ok("on target, rear", float(u3[6]), 0.0, 1e-12)
    u4 = fp3.step(vy=0.0143, wz=0.0, swing=np.ones(4, bool))    # not turning at all
    front, rear = float(u4[0]), float(u4[6])
    print(f"  {'ok  ' if front * rear < 0 else 'FAIL'} front and rear corrections oppose "
          f"({front:+.4f} vs {rear:+.4f})")
    fails += 0 if front * rear < 0 else 1

    # 4. the one-cycle mean keeps the mean and kills the ripple
    n = 32
    fp5 = FootPlacement(t_stance_s=0.372, lever_m=lever, hip_x_m=hip_x, yaw_mode="log-cycle",
                        cycle_len=n, cap_rad=1.0, wz_log=0.0)
    rng = np.random.default_rng(0)
    bias, ripple = 0.09, 0.36
    for i in range(4 * n):
        fp5.step(vy=0.0, wz=bias + ripple * rng.standard_normal(), swing=np.ones(4, bool))
    ok("cycle mean tracks the bias, not the ripple", fp5._wz, bias, 4 * ripple / np.sqrt(n))

    # 5. the heading term: parameter-free, opposite on front and rear, and it is the
    #    heading error that drives it rather than any rate
    fp6 = FootPlacement(t_stance_s=0.484, lever_m=lever, hip_x_m=hip_x,
                        yaw_mode="heading-only", cycle_len=37, cap_rad=0.05,
                        heading_cap_rad=0.04, vy_log=0.0, wz_log=0.0)
    u6 = fp6.step(vy=0.0, wz=0.0, swing=np.ones(4, bool), psi_err_rad=0.1)
    # u is float32, so the tolerance is float32 epsilon rather than the default 1e-9
    ok("heading term = -psi*x/(2*lever), front", float(u6[0]), -0.1 * 0.193 / (2 * 0.306), 1e-7)
    ok("heading term, rear is opposite", float(u6[6]), -0.1 * -0.193 / (2 * 0.316), 1e-7)
    ok("no heading error, no heading term",
       float(np.abs(fp6.step(0.0, 0.0, np.ones(4, bool), psi_err_rad=0.0)).max()), 0.0)
    u7 = fp6.step(vy=0.0, wz=0.0, swing=np.ones(4, bool), psi_err_rad=2.0)
    ok("heading term respects its own cap", float(u7[0]), -0.04)
    # T_stance must not appear: the same error on a clip with a different stance time
    fp7 = FootPlacement(t_stance_s=0.372, lever_m=lever, hip_x_m=hip_x,
                        yaw_mode="heading-only", cycle_len=32, cap_rad=0.05,
                        heading_cap_rad=0.04)
    ok("T_stance cancels out of the heading term",
       float(fp7.step(0.0, 0.0, np.ones(4, bool), psi_err_rad=0.1)[0]), float(u6[0]), 1e-7)

    print(f"footcomp self-test: {'PASS' if fails == 0 else f'{fails} FAILURES'}")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(_self_test())
