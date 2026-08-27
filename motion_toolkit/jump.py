"""Take-off / flight / landing analysis for the four-leg-sync skills.

Why this needs its own module: **the sport state cannot see a flight phase.**
`base_pos_*`, `base_v*` and `body_height` are leg-kinematic estimates, so while
all four feet are off the ground they simply stop tracking -- during a 0.47 s
`front_jump` flight `base_pos_z` moves 10-30 mm and `base_vz` stays at
0.001 m/s.  Jump height and take-off speed therefore have to come from
quantities that are still valid in the air:

``T`` (flight time)
    Longest all-four-off run inside the motion window, from the per-session
    contact threshold.  Corroborated by the IMU: during the run the specific
    force magnitude drops to ~1 m/s^2 (free fall) from ~9.8 m/s^2 standing.
``v_z0 = g*T/2``, ``apex = g*T^2/8``
    Ballistic inversion of the flight time.  Valid because take-off and
    touch-down happen at nearly the same body height (within ~20 mm), and it
    needs no accelerometer scale at all -- only the clock.
``push-off impulse = int (acc_z - g) dt`` over the 0.30 s before take-off
    An independent estimate of the same ``v_z0``.  The push-off is a slow
    ~0.25 s ramp, so the ~100 Hz sport-state accelerometer resolves it.

The two agree within ~10 % for `front_jump`, which is what makes the ballistic
numbers trustworthy.

What is *not* trustworthy: the **landing** impulse.  ``acc_z`` arrives held at
the sport-state publish rate (repeated sample values are visible in the log), so
an impact lasting a few milliseconds is aliased away -- integrating the landing
recovers only ~0.1 m/s of the ~2.3 m/s that has to be arrested.  Landing peak
acceleration and peak foot force are therefore **lower bounds**, the same way
`foot_force`'s ~210 N clip is.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np
import pandas as pd

from .contact import detect_contact
from .session import LEGS, Session
from .window import detect_motion, runs_of

G = 9.81

#: |specific force| below this counts as free fall (standing reads ~9.8).
FREEFALL_A = 2.5
#: Window before take-off over which the push-off impulse is integrated.
PUSH_S = 0.30
#: A run only counts as a real jump if the push-off impulse reaches this (m/s).
#: The four-leg-sync skills also lose all four contacts while *crouching* and
#: while swinging both legs forward in a lunge; those runs come with a push-off
#: impulse at or below zero (the body is falling, not launching) and must not be
#: read as flight.
BALLISTIC_DV = 0.5
#: Shortest all-four-off run worth looking at.
MIN_RUN_S = 0.03
#: Window after touch-down for the landing peaks.
LAND_S = 0.30
#: Pre/post windows for the horizontal displacement of the jump.
PRE_A, PRE_B = 0.25, 0.05      # before take-off
POST_A, POST_B = 0.30, 0.50    # after touch-down
#: A leg force at or above this is clipped by the sensor.
CLIP_N = 209.0


def _median_window(x: np.ndarray, t: np.ndarray, lo: float, hi: float) -> float:
    m = (t >= lo) & (t <= hi)
    return float(np.nanmedian(x[m])) if m.any() and np.isfinite(x[m]).any() else np.nan


def odom_drift_rate(sess: Session, mask: np.ndarray) -> float:
    """Odometry drift in m/s, measured while the robot is provably standing still.

    Uses the longest quiet stretch before the first motion segment.
    """
    t = sess.t
    first = int(np.flatnonzero(mask)[0]) if mask.any() else sess.n
    lo, hi = 1.0, max(1.5, t[first] - 1.0)
    m = (t >= lo) & (t <= hi)
    if m.sum() < 10:
        return np.nan
    px, py = sess.col("base_pos_x")[m], sess.col("base_pos_y")[m]
    d = float(np.hypot(px[-1] - px[0], py[-1] - py[0]))
    return d / float(t[m][-1] - t[m][0])


def analyse_jump(sess: Session) -> Optional[Dict[str, object]]:
    """Take-off / flight / landing metrics for one four-leg-sync session."""
    t, fs = sess.t, sess.fs
    win = detect_motion(sess)
    if not win.ok:
        return None
    force = sess.foot_force()
    cr = detect_contact(force, win.mask, fs)
    airborne = (~cr.contact).all(axis=1) & win.mask
    az = sess.col("acc_z")
    amag = np.sqrt(
        np.nan_to_num(sess.col("acc_x")) ** 2
        + np.nan_to_num(sess.col("acc_y")) ** 2
        + np.nan_to_num(az) ** 2
    )
    dt = np.gradient(t)

    def push_dv_of(start: int) -> float:
        w0 = (t >= t[start] - PUSH_S) & (t < t[start])
        return float(np.nansum((az[w0] - G) * dt[w0])) if w0.any() else np.nan

    runs = [(a, b) for a, b in runs_of(airborne) if (b - a) / fs >= MIN_RUN_S]
    if not runs:
        return None
    scored = [(a, b, push_dv_of(a)) for a, b in runs]
    ballistic = [r for r in scored if np.isfinite(r[2]) and r[2] >= BALLISTIC_DV]
    a, b, push_dv = max(ballistic or scored, key=lambda r: r[1] - r[0])
    is_ballistic = bool(ballistic)
    t_to, t_td = float(t[a]), float(t[b - 1])
    T = float((b - a) / fs)
    pre = (t >= t_to - PUSH_S) & (t < t_to)

    px, py, pz = sess.col("base_pos_x"), sess.col("base_pos_y"), sess.col("base_pos_z")
    h = sess.col("body_height")
    x0 = _median_window(px, t, t_to - PRE_A, t_to - PRE_B)
    y0 = _median_window(py, t, t_to - PRE_A, t_to - PRE_B)
    x1 = _median_window(px, t, t_td + POST_A, t_td + POST_B)
    y1 = _median_window(py, t, t_td + POST_A, t_td + POST_B)

    land = (t > t_td) & (t <= t_td + LAND_S)
    fl_slice = slice(a, b)

    row: Dict[str, object] = {
        "group": sess.group,
        "session": sess.path.name,
        "skill": (sess.skill_sequence() or ["?"])[-1],
        # -- flight
        "takeoff_s": t_to,
        "touchdown_s": t_td,
        "flight_s": T,
        "ballistic": is_ballistic,
        "n_airborne_runs": len(runs),
        "n_ballistic_runs": len(ballistic),
        "acc_mag_flight_med": float(np.nanmedian(amag[fl_slice])),
        "freefall_frac": float(np.mean(amag[fl_slice] < FREEFALL_A)),
        # -- vertical, ballistic inversion of the flight time
        "vz_takeoff_ballistic": G * T / 2.0,
        "apex_rise_ballistic_m": G * T * T / 8.0,
        # -- vertical, independent IMU push-off impulse
        "push_impulse_dv": push_dv,
        "apex_rise_impulse_m": (push_dv ** 2) / (2 * G) if np.isfinite(push_dv) and push_dv > 0 else np.nan,
        # -- what the odometry claims over the same flight (blind: see module docstring)
        "dz_odom_flight_m": float(pz[b - 1] - pz[a]),
        "dh_flight_m": float(h[b - 1] - h[a]),
        "vz_odom_takeoff": _median_window(sess.col("base_vz"), t, t_to - 0.05, t_to),
        # -- horizontal
        "vx_takeoff": _median_window(sess.col("base_vx"), t, t_to - 0.05, t_to),
        "dx_jump_m": float(x1 - x0),
        "dy_jump_m": float(y1 - y0),
        "dxy_jump_m": float(np.hypot(x1 - x0, y1 - y0)),
        "dx_flight_odom_m": float(px[b - 1] - px[a]),
        "range_from_vx_m": float(_median_window(sess.col("base_vx"), t, t_to - 0.05, t_to) * T),
        "dx_skill_m": float(
            _median_window(px, t, t[win.segments[-1][1] - 1] - 0.4, t[win.segments[-1][1] - 1])
            - _median_window(px, t, t[win.segments[0][0]] - 0.4, t[win.segments[0][0]])
        ),
        "odom_drift_m_s": odom_drift_rate(sess, win.mask),
        # -- posture
        "crouch_height_m": float(np.nanmin(h[win.mask])),
        "takeoff_height_m": _median_window(h, t, t_to - 0.05, t_to),
        "touchdown_height_m": _median_window(h, t, t_td, t_td + 0.05),
        "stand_height_after_m": _median_window(h, t, t[-1] - 1.0, t[-1]),
        "pitch_flight_min": float(np.nanmin(sess.col("pitch")[fl_slice])),
        "pitch_flight_max": float(np.nanmax(sess.col("pitch")[fl_slice])),
        # -- landing (lower bounds, see module docstring)
        "land_acc_z_peak": float(np.nanmax(az[land])) if land.any() else np.nan,
        "land_acc_mag_peak": float(np.nanmax(amag[land])) if land.any() else np.nan,
        "land_dv_measured": float(np.nansum((az[land] - G) * dt[land])) if land.any() else np.nan,
        "land_force_leg_peak_N": float(np.nanmax(force[land])) if land.any() else np.nan,
        "land_force_total_peak_N": float(np.nanmax(np.nansum(force, axis=1)[land])) if land.any() else np.nan,
        "land_force_clipped_samples": int((force[land] >= CLIP_N).sum()) if land.any() else 0,
        "push_force_total_peak_N": float(np.nanmax(np.nansum(force, axis=1)[pre])) if pre.any() else np.nan,
    }
    return row


def jump_table(sessions: List[Session]) -> pd.DataFrame:
    rows = [r for r in (analyse_jump(s) for s in sessions) if r is not None]
    df = pd.DataFrame(rows)
    return df.sort_values(["skill", "session"]).reset_index(drop=True)


def repeatability(df: pd.DataFrame, columns: List[str]) -> pd.DataFrame:
    """mean / std / CV / min / max of each metric, per skill."""
    out = []
    for skill, sub in df.groupby("skill"):
        for c in columns:
            x = pd.to_numeric(sub[c], errors="coerce").to_numpy(dtype=float)
            x = x[np.isfinite(x)]
            if x.size == 0:
                continue
            mean = float(x.mean())
            sd = float(x.std(ddof=1)) if x.size > 1 else np.nan
            out.append({
                "skill": skill, "metric": c, "n": int(x.size),
                "mean": mean, "std": sd,
                "cv": abs(sd / mean) if (np.isfinite(sd) and abs(mean) > 1e-9) else np.nan,
                "min": float(x.min()), "max": float(x.max()),
            })
    return pd.DataFrame(out)
