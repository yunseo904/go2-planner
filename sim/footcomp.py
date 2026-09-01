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
    heading_len: bool = False           # add the fore-aft (step-length) half of the heading law
    heading_len_cap_rad: float = 0.04   # magnitude bound on it; the window below is one-sided
    axis: str = "y"                     # y | xy
    vy_target: float = 0.0
    #: Lateral velocity the CALLER wants on top of whatever this mode's target already is,
    #: m/s, rewritten every control step.  It exists for cross-track hold: heading hold
    #: pulls the yaw ANGLE back to the settle heading and nothing pulls the lateral
    #: POSITION back, so a robot holds its heading and leaves the lane sideways anyway
    #: (measured: WALK drifts 0.53 m sideways per metre forward, 149 of 200 cells to the
    #: same side).  Added as a BIAS rather than by overwriting ``vy_target`` because the
    #: yaw modes do not read ``vy_target`` at all -- they track the recording's own
    #: ``vy_log`` -- so a target-only knob would have been silently dead in exactly the
    #: arm that needs it.  0.0 reproduces every earlier run bit for bit.
    vy_bias: float = 0.0
    cycle_len: int = 0                  # control steps in one clip cycle
    yaw_bias: float = 0.0               # open-loop steering probe, hips
    len_bias: float = 0.0               # open-loop steering probe, thighs
    # -- capture-point variant (quadruped_pympc's form of the same rear term).  All
    #    three default to the values that reproduce the half-stance law exactly.
    gain_mode: str = "half-stance"      # half-stance | capture
    com_height_m: float = float("nan")  # measured base height; capture mode only
    vy_avg_n: int = 0                   # moving average on v_y, in control steps (0 = off)
    offset_clip_m: float = 0.0          # clip on the FOOT OFFSET in metres (0 = off)
    # -- running state
    _wz_buf: Optional[np.ndarray] = field(default=None, repr=False)
    _wz_sum: float = 0.0
    _wz_i: int = 0
    _wz_n: int = 0
    _wz: float = 0.0
    _vy_buf: Optional[np.ndarray] = field(default=None, repr=False)
    _vy_sum: float = 0.0
    _vy_i: int = 0
    _vy_n: int = 0
    off_clip_hits: int = 0
    applied: int = 0
    cap_hits: int = 0
    max_abs: float = 0.0
    head_cap_hits: int = 0
    len_cap_hits: int = 0
    len_engaged: int = 0

    def __post_init__(self) -> None:
        self.lever_m = np.asarray(self.lever_m, dtype=float)
        self.hip_x_m = np.asarray(self.hip_x_m, dtype=float)
        if self.hip_y_m is not None:
            self.hip_y_m = np.asarray(self.hip_y_m, dtype=float)
        if not np.all(self.lever_m > 0.05):
            raise ValueError(f"hip-to-foot lever {self.lever_m} is not a usable moment arm; "
                             f"the correction is linearised about a standing pose")
        if self.gain_mode not in ("half-stance", "capture"):
            raise ValueError(f"unknown gain_mode {self.gain_mode!r}")
        if self.gain_mode == "capture" and not (np.isfinite(self.com_height_m)
                                                and self.com_height_m > 0.05):
            raise ValueError("gain_mode 'capture' needs a measured com_height_m; refusing "
                             "to substitute a nominal one -- the gain IS sqrt(h/g)")
        if self.yaw_mode == "log-cycle" and self.cycle_len < 1:
            raise ValueError("--foot-yaw log-cycle needs the clip's cycle length in steps")
        if self.yaw_mode != "off" and not (np.isfinite(self.vy_log) and np.isfinite(self.wz_log)):
            raise ValueError("yaw mode needs the log's own v_y and yaw rate; refusing to "
                             "fall back to zero silently -- that is the assumption being tested")
        self.reset()

    def reset(self) -> None:
        self._wz_buf = np.zeros(max(self.cycle_len, 1))
        self._wz_sum, self._wz_i, self._wz_n, self._wz = 0.0, 0, 0, 0.0
        self._vy_buf = np.zeros(max(self.vy_avg_n, 1))
        self._vy_sum, self._vy_i, self._vy_n = 0.0, 0, 0
        self.off_clip_hits = 0
        self.applied = self.cap_hits = 0
        self.head_cap_hits = self.len_cap_hits = self.len_engaged = 0
        self.max_abs = 0.0
        self.last_raw_hip = np.zeros(4)
        self.last_u = np.zeros(12)
        self.last_cap_hits = self.last_swing_n = 0
        self.last_u_head = np.zeros(4)
        self.last_u_len = np.zeros(4)

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

        # Moving average on the lateral velocity, off by default.  The yaw RATE has been
        # averaged over a cycle since stage 2 for a measured reason (the instantaneous
        # signal is 3.9x the bias it should correct); the velocity never was.
        # quadruped_pympc averages v over 20 samples before differencing it against
        # v_ref, and this is that, expressed in control steps so the window is stated
        # rather than inherited from someone else's control rate.
        if self.vy_avg_n > 0:
            self._vy_sum += vy - self._vy_buf[self._vy_i]
            self._vy_buf[self._vy_i] = vy
            self._vy_i = (self._vy_i + 1) % self._vy_buf.size
            self._vy_n = min(self._vy_n + 1, self._vy_buf.size)
            vy = self._vy_sum / self._vy_n

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
            self.head_cap_hits += int((swing & (np.abs(u_head) > hcap)).sum())
            u_head = np.clip(u_head, -hcap, hcap)
            if self.heading_len and self.hip_y_m is not None:
                # The SAME substitution, read in the other axis.  A base whose yaw error
                # is psi carries the hip at (x_i, y_i) fore-aft at -(omega - omega_tgt)*y_i,
                # so the Raibert half-stance-time step-length correction is
                #
                #   dx_i     = (T_st/2) * -(psi_err / T_st) * y_i  = -psi_err * y_i / 2
                #   dq_thigh = -dx_i / lever_i                     = +psi_err * y_i / (2 lever)
                #
                # T_stance cancels here too, so this half is as parameter-free as the
                # lateral one: heading error, the hip's own LATERAL offset, the same
                # lever.  Left and right get opposite signs -- one side takes longer
                # steps -- which is mechanism (b) of outputs/heading_candidates.md 2,
                # measured there at ~199 deg/s per rad on TROT against (a)'s 155-175.
                #
                # Sign: flipped, for the same reason the lateral half is flipped, and
                # confirmed against the same probe -- --foot-len-bias -0.04 (which is
                # -0.04 on the LEFT thighs) removed 4.71 of TROT's +5.32 deg/s, so a
                # positive heading error needs a negative coefficient.
                #
                # ONE-SIDED, and this is the whole reason it is a separate cap.  The
                # open-loop probe measured this mechanism's survivable window on TROT as
                # roughly [-0.05, +0.01] rad: -0.06 falls at 2.77 s, -0.04 and -0.02 run
                # 60 cycles, +0.02 destroys the gait (yaw swings to -12.98 deg/s, stride
                # to 1.99 Hz) and +0.04 falls at 2.67 s.  Both bounds below are measured
                # endpoints of that survivable set, not tuned values.  The consequence is
                # deliberate and has to be stated: this half can only correct ONE SIGN of
                # heading error.  It is usable because the drift it is being asked to
                # remove is a steady one-signed bias (+5.32 deg/s, quarter means +6.18,
                # +4.74, +5.26, +5.10 -- heading_candidates.md 0); on a clip whose bias
                # runs the other way it contributes nothing and (a) is alone again.
                u_len = -psi_err_rad * self.hip_y_m / (2.0 * self.lever_m)
                lcap = self.heading_len_cap_rad
                # sign(y) is +1 on the left pair, so the coefficient this is equivalent
                # to is c = -psi*|y|/(2*lever); the window is c in [-lcap, 0], i.e. the
                # term is clipped by SIDE rather than by magnitude.
                lo = np.where(self.hip_y_m >= 0.0, -lcap, 0.0)
                hi = np.where(self.hip_y_m >= 0.0, 0.0, lcap)
                self.len_cap_hits += int((swing & ((u_len < lo) | (u_len > hi))).sum())
                u_len = np.clip(u_len, lo, hi)
                self.len_engaged += int((swing & (np.abs(u_len) > 0)).sum())
                self.last_u_len = np.where(swing, u_len, 0.0)
        if self.yaw_mode == "heading-only":
            # The rate half dropped.  Every yaw-RATE loop measured made heading worse
            # (outputs/heading_candidates.md 3), so it is separable here rather than
            # carried on faith: `heading` is the full substitution, `heading-only` is
            # the half the evidence supports.
            dvy = np.full(4, vy - self.vy_log - self.vy_bias)
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
            dvy = (vy - self.vy_log - self.vy_bias) + (self._wz - self.wz_log) * self.hip_x_m
            dvx = ((vx - self.vx_log) - (self._wz - self.wz_log) * (self.hip_y_m
                   if self.hip_y_m is not None else 0.0)
                   if self.axis == "xy" and np.isfinite(self.vx_log) else np.zeros(4))
        else:
            dvy = np.full(4, vy - self.vy_target - self.vy_bias)
            dvx = (np.full(4, vx - self.vx_log)
                   if self.axis == "xy" and np.isfinite(self.vx_log) else np.zeros(4))

        # The Raibert leading term, two forms of the same quantity:
        #
        #   half-stance  T_stance/2      -- the neutral point of the stance this gait
        #                                   actually has, measured off its own contact
        #                                   channel.  The default; every result before
        #                                   this flag existed was run with it.
        #   capture      sqrt(h/g)       -- the linear-inverted-pendulum time constant,
        #                                   which is what quadruped_pympc uses.  It is a
        #                                   property of the BODY, not of the gait, so it
        #                                   is the same number for every clip.
        #
        # For TROT these are 0.186 s and 0.175 s -- within 6% -- so the gain is NOT
        # where the two laws differ.  What differs is the averaging above and the clip
        # below, and separating the three is the whole point of having three flags.
        if self.gain_mode == "capture":
            gain = float(np.sqrt(self.com_height_m / 9.81))
        else:
            gain = 0.5 * self.t_stance_s + self.k_s
        off_y = gain * dvy                              # the foot offset, in METRES
        if self.offset_clip_m > 0.0:
            # quadruped_pympc bounds the foot OFFSET at +-0.05 m.  Ours bounds the hip
            # ANGLE at +-0.05 rad, which through a 0.31 m lever is +-0.0155 m -- a third
            # as much authority for the same-looking number.  When this is set it
            # REPLACES the radian cap on this term; the heading term keeps its own.
            self.off_clip_hits += int((swing & (np.abs(off_y) > self.offset_clip_m)).sum())
            off_y = np.clip(off_y, -self.offset_clip_m, self.offset_clip_m)
        raw_hip = self.sign * off_y / self.lever_m
        raw_thigh = -self.sign * (gain * np.asarray(dvx, dtype=float)) / self.lever_m
        cap = float("inf") if self.offset_clip_m > 0.0 else self.cap_rad
        # The two terms are capped SEPARATELY and then added: the lateral cap is the
        # stage-2 budget for answering v_y, and the heading cap is what the steering
        # probe measured each gait tolerates.  Clipping their sum would let one starve
        # the other, and they answer different questions.
        u[0::3] = np.where(swing, np.clip(raw_hip, -cap, cap) + u_head, 0.0)
        if self.axis == "xy":
            u[1::3] = np.where(swing, np.clip(raw_thigh, -cap, cap), 0.0)
        # The step-length heading half goes on the THIGHS, and it ADDS to whatever the
        # lateral law put there (nothing, at the default axis="y").  Separate cap, added
        # after -- same rule as u_head on the hips, and for the same reason: the two
        # answer different questions and clipping their sum would let one starve the
        # other.  They also act through different joints, which is why the open-loop
        # probe found them close to independent (7.33 deg/s combined against 3.10 + 4.71
        # separately).
        u[1::3] += np.where(swing, self.last_u_len, 0.0)

        self.applied += int(swing.sum())
        # With the metre clip active the radian cap is not the binding one, so the
        # reported cap-hit count is the metre clip's -- one column, one meaning.
        self.cap_hits = (self.off_clip_hits if self.offset_clip_m > 0.0
                         else self.cap_hits + int((swing & (np.abs(raw_hip) > cap)).sum()))
        # Per-step, not cumulative: cap_hit_frac is a whole-run average and cannot say
        # whether saturation changed at a particular moment (a lip, a switch).  These
        # two are read by traces and by nothing else -- they do not feed the law.
        self.last_raw_hip = np.where(swing, raw_hip, 0.0)
        self.last_u_head = np.where(swing, u_head, 0.0)
        self.last_cap_hits = int((swing & (np.abs(raw_hip) > cap)).sum())
        self.last_swing_n = int(swing.sum())

        # The open-loop steering probes go on AFTER the feedback term so the two add
        # rather than one overwriting the other, and they are NOT clipped -- the bias
        # is the amplitude being set, so clipping it would measure the clip.
        if self.yaw_bias or self.len_bias:
            u[0::3] += np.where(swing, self.yaw_bias * np.sign(self.hip_x_m), 0.0)
            if self.hip_y_m is not None:
                u[1::3] += np.where(swing, self.len_bias * np.sign(self.hip_y_m), 0.0)
        self.max_abs = max(self.max_abs, float(np.abs(u).max()))
        self.last_u = u.copy()
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

    # 6. the step-length heading half: same substitution in the other axis, one-sided
    hip_y = np.array([0.110, -0.110, 0.110, -0.110])
    fp8 = FootPlacement(t_stance_s=0.484, lever_m=lever, hip_x_m=hip_x, hip_y_m=hip_y,
                        yaw_mode="heading-only", cycle_len=37, cap_rad=0.05,
                        heading_cap_rad=0.02, heading_len=True, heading_len_cap_rad=0.04,
                        vy_log=0.0, wz_log=0.0)
    u8 = fp8.step(vy=0.0, wz=0.0, swing=np.ones(4, bool), psi_err_rad=0.1)
    ok("len term = -psi*y/(2*lever), left thigh", float(u8[1]),
       -0.1 * 0.110 / (2 * 0.306), 1e-7)
    ok("len term, right thigh is opposite", float(u8[4]), +0.1 * 0.110 / (2 * 0.306), 1e-7)
    print(f"  {'ok  ' if float(u8[1]) < 0 < float(u8[4]) else 'FAIL'} positive heading error "
          f"gives the direction the open-loop probe survived (left thighs negative)")
    fails += 0 if float(u8[1]) < 0 < float(u8[4]) else 1
    # the destructive sign is refused outright, not merely capped
    u9 = fp8.step(vy=0.0, wz=0.0, swing=np.ones(4, bool), psi_err_rad=-0.1)
    ok("negative heading error: the len half is OFF, not reversed",
       float(np.abs(u9[1::3]).max()), 0.0)
    # and the working direction is bounded by the measured survivable amplitude
    u10 = fp8.step(vy=0.0, wz=0.0, swing=np.ones(4, bool), psi_err_rad=2.0)
    ok("len half respects its own cap", float(u10[1]), -0.04)
    ok("the lateral half is capped separately and still binds", float(u10[0]), -0.02)
    # T_stance must not appear in this half either
    fp9 = FootPlacement(t_stance_s=0.372, lever_m=lever, hip_x_m=hip_x, hip_y_m=hip_y,
                        yaw_mode="heading-only", cycle_len=32, cap_rad=0.05,
                        heading_cap_rad=0.02, heading_len=True)
    ok("T_stance cancels out of the step-length half too",
       float(fp9.step(0.0, 0.0, np.ones(4, bool), psi_err_rad=0.1)[1]), float(u8[1]), 1e-7)
    # off by default: every run before this one must be unchanged
    fp10 = FootPlacement(t_stance_s=0.484, lever_m=lever, hip_x_m=hip_x, hip_y_m=hip_y,
                         yaw_mode="heading-only", cycle_len=37, cap_rad=0.05,
                         heading_cap_rad=0.02, vy_log=0.0, wz_log=0.0)
    ok("heading_len defaults OFF (the thighs stay untouched)",
       float(np.abs(fp10.step(0.0, 0.0, np.ones(4, bool), psi_err_rad=0.1)[1::3]).max()), 0.0)
    ok("and the hips are bit-identical to the run without it",
       float(fp10.step(0.0, 0.0, np.ones(4, bool), psi_err_rad=0.1)[0]), float(u8[0]), 0.0)

    # 7. the capture-point variant: gain, averaging and the metre clip, each separable
    fp11 = FootPlacement(t_stance_s=0.372, lever_m=lever, hip_x_m=hip_x, cap_rad=1.0,
                         gain_mode="capture", com_height_m=0.31)
    u11 = fp11.step(vy=0.2, wz=0.0, swing=np.ones(4, bool))
    ok("capture gain = sqrt(h/g)", float(u11[0]),
       float(np.sqrt(0.31 / 9.81)) * 0.2 / 0.306, 1e-7)
    try:
        FootPlacement(t_stance_s=0.372, lever_m=lever, hip_x_m=hip_x, gain_mode="capture")
        print("  FAIL capture mode accepted a missing com_height_m"); fails += 1
    except ValueError:
        print("  ok   capture mode refuses to run without a measured com_height_m")
    # the two gains are close on TROT, which is the point: the gain is not the difference
    print(f"  ok   TROT gains: half-stance {0.5*0.372:.4f} s vs capture "
          f"{float(np.sqrt(0.31/9.81)):.4f} s "
          f"({100*abs(0.5*0.372-np.sqrt(0.31/9.81))/(0.5*0.372):.1f}% apart)")
    # moving average on v_y: same shape of test as the yaw-rate one above
    fp12 = FootPlacement(t_stance_s=0.372, lever_m=lever, hip_x_m=hip_x, cap_rad=1.0,
                         vy_avg_n=20)
    rng2 = np.random.default_rng(1)
    bias2, ripple2 = 0.05, 0.25
    for _ in range(80):
        u12 = fp12.step(vy=bias2 + ripple2 * rng2.standard_normal(), wz=0.0,
                        swing=np.ones(4, bool))
    ok("v_y average tracks the bias, not the ripple",
       float(u12[0]) * 0.306 / (0.5 * 0.372), bias2, 4 * ripple2 / np.sqrt(20))
    # the metre clip: 0.05 m through a 0.306 m lever is 0.163 rad, not 0.05
    fp13 = FootPlacement(t_stance_s=0.372, lever_m=lever, hip_x_m=hip_x, cap_rad=0.05,
                         offset_clip_m=0.05)
    u13 = fp13.step(vy=5.0, wz=0.0, swing=np.ones(4, bool))
    ok("metre clip binds in metres and replaces the radian cap", float(u13[0]),
       0.05 / 0.306, 1e-7)
    ok("...and our 0.05 rad cap is this much foot offset", 0.05 * 0.306, 0.0153, 1e-4)
    # off by default, and bit-identical to the law before any of this existed
    fp14 = FootPlacement(t_stance_s=0.372, lever_m=lever, hip_x_m=hip_x, cap_rad=1.0)
    # Bit-identity is asserted against the value the law produced at the TOP of this
    # self-test, not against a float64 expression: both are float32 and the claim is
    # that the new fields changed nothing, not that float32 is exact.
    ok("all three default OFF: bit-identical to the law before they existed",
       float(fp14.step(vy=0.2, wz=0.0, swing=np.ones(4, bool))[0]), float(u[0]), 0.0)

    print(f"footcomp self-test: {'PASS' if fails == 0 else f'{fails} FAILURES'}")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(_self_test())
