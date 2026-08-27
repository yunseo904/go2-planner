"""Per-session skill features for the curated Go2 log set.

One row per session.  Everything is nan-safe (``base_v*``, ``body_height`` and
``*_q_des`` all carry occasional dropouts) and every statistic is taken over the
**active mask** from :mod:`motion_toolkit.window`, not over the whole recording
and not over ``events.jsonl`` timestamps.

Column groups
-------------
``identity``  group / session / skill names, commanded parameters
``window``    motion window, active duration, lag from skill_send to first motion
``velocity``  base_vx / base_vy / yaw_speed statistics plus an odometry cross-check
``gait``      per-session contact threshold, stride frequency, duty, flight
``body``      body_height level and excursion
``torque``    joint torque RMS / peak
``command``   PosStopF sentinel rate in ``*_q_des``, kp levels
``quality``   NaN rates of the columns actually used
"""

from __future__ import annotations

from typing import Dict, List

import numpy as np
import pandas as pd

from .contact import classify_pattern, detect_contact, relative_phases, stride_frequency
from .session import JOINTS, LEGS, POS_STOP_F, Session
from .window import MotionWindow, detect_motion, runs_of

#: Skills that only set a parameter / a gait mode and never move the robot by
#: themselves.  ``primary_skill`` is the last skill_send that is not one of these
#: parameter calls; ``gait_mode`` is the last gait-selecting call.
PARAM_SKILLS = {"speed_level"}
GAIT_SKILLS = {"trot_run", "static_walk", "economic_gait", "classic_walk", "cross_step", "free_walk"}

#: Settling thresholds after a segment ends (body at rest).
SETTLE_V = 0.05      # m/s on |base_v|
SETTLE_W = 0.10      # rad/s on yaw_speed
SETTLE_H = 0.005     # m on body_height std
SETTLE_HOLD_S = 0.5


def _cmd_params(sess: Session) -> Dict[str, float]:
    """The ``move`` parameters and ``speed_level`` of the session, if any."""
    out: Dict[str, float] = {
        "cmd_vx": np.nan, "cmd_vy": np.nan, "cmd_wz": np.nan,
        "cmd_duration_s": np.nan, "speed_level": np.nan,
    }
    for ev in sess.skill_sends():
        p = ev.param or {}
        if ev.skill == "move":
            out["cmd_vx"] = float(p.get("x", np.nan))
            out["cmd_vy"] = float(p.get("y", np.nan))
            out["cmd_wz"] = float(p.get("z", np.nan))
            out["cmd_duration_s"] = float(p.get("duration", np.nan))
        elif ev.skill == "speed_level":
            out["speed_level"] = float(p.get("data", np.nan))
    return out


def _skill_names(sess: Session) -> Dict[str, str]:
    seq = sess.skill_sequence()
    if not seq:
        return {
            "skill_seq": "(none: gamepad teleop)",
            "primary_skill": "gamepad",
            "gait_mode": "default",
            "prereq_skills": "",
        }
    motion_sends = [s for s in seq if s not in PARAM_SKILLS]
    primary = motion_sends[-1] if motion_sends else seq[-1]
    gait = [s for s in seq if s in GAIT_SKILLS]
    return {
        "skill_seq": "→".join(seq),
        "primary_skill": primary,
        "gait_mode": gait[-1] if gait else "default",
        "prereq_skills": "→".join(seq[:-1]),
    }


def _motion_type(cmd: Dict[str, float], primary: str) -> str:
    """Direction class from the commanded ``move`` parameters."""
    vx, vy, wz = cmd["cmd_vx"], cmd["cmd_vy"], cmd["cmd_wz"]
    if primary not in ("move", "gamepad"):
        return "discrete"
    if not np.isfinite(vx):
        return ""  # gamepad session -- filled in from the measurement
    if abs(wz) > 1e-6 and abs(vx) < 1e-6 and abs(vy) < 1e-6:
        return "yaw"
    if abs(vy) > 1e-6 and abs(vx) < 1e-6:
        return "lateral"
    if vx < -1e-6:
        return "backward"
    if vx > 1e-6:
        return "forward"
    return "none"


def _measured_motion_type(vx: float, vy: float, wz: float) -> str:
    """Direction class from the measured steady-state body velocity (gamepad runs)."""
    turning = abs(wz) > 0.2
    if turning and abs(vx) < 0.2 and abs(vy) < 0.2:
        return "yaw"
    if abs(vy) > 0.2 and abs(vx) < 0.2:
        return "lateral+yaw" if turning else "lateral"
    if vx < -0.2:
        return "backward+yaw" if turning else "backward"
    if vx > 0.2:
        return "forward+yaw" if turning else "forward"
    return "none"


def _skill_key(row: Dict[str, object]) -> str:
    """Aggregation key for the per-skill table.

    A locomotion session is characterised by *gait mode + direction*, not by the
    ``move`` call alone: ``trot_run`` and ``classic_walk`` both end in ``move``.
    Discrete skills key on their own name.
    """
    primary = str(row["primary_skill"])
    mt = str(row["motion_type"])
    if primary == "gamepad":
        return f"gamepad:{mt}"
    if primary != "move":
        # a discrete session is named by its motion skills; balance_stand is a
        # precondition, not the skill under test
        core = [s for s in str(row["skill_seq"]).split("\u2192") if s not in PARAM_SKILLS and s != "balance_stand"]
        return "\u2192".join(dict.fromkeys(core)) or primary
    return f"{row['gait_mode']}+move:{mt}"


def _settle_time(sess: Session, stop_idx: int, speed: np.ndarray, quiet_level: float) -> float:
    """Seconds from ``stop_idx`` until the robot is settled, held ``SETTLE_HOLD_S``.

    ``stop_idx`` is the last *vigorous* sample (smoothed joint speed above the
    detector's enter level), not the segment end -- the segment already ends when
    joint velocity has decayed, so measuring from there would return ~0 by
    construction.  "Settled" needs all four of:

    * ``|base_v| < SETTLE_V`` and ``|yaw_speed| < SETTLE_W``,
    * joint speed back below the detector's exit level,
    * ``body_height`` flat -- standard deviation below ``SETTLE_H`` over the hold
      window.  A *level* criterion cannot be used: ``stand_up_down`` settles at
      0.07 m in the middle of the recording and at 0.32 m at the end.
    """
    t = sess.t
    if stop_idx >= sess.n - 2:
        return np.nan
    vx, vy = sess.col("base_vx"), sess.col("base_vy")
    wz, h = sess.col("yaw_speed"), sess.col("body_height")
    fs = sess.fs
    hold = max(2, int(round(SETTLE_HOLD_S * fs)))
    sl = slice(stop_idx, sess.n)
    v = np.hypot(np.nan_to_num(vx[sl], nan=0.0), np.nan_to_num(vy[sl], nan=0.0))
    yaw = np.abs(np.nan_to_num(wz[sl], nan=0.0))
    hh = h[sl]
    hh = np.nan_to_num(hh, nan=float(np.nanmedian(hh)) if np.isfinite(hh).any() else 0.0)
    calm = (v < SETTLE_V) & (yaw < SETTLE_W) & (speed[sl] < quiet_level)
    ok = np.convolve(calm.astype(int), np.ones(hold, dtype=int), mode="valid") == hold
    # rolling std of body_height over the same hold window
    c1 = np.convolve(hh, np.ones(hold), mode="valid") / hold
    c2 = np.convolve(hh ** 2, np.ones(hold), mode="valid") / hold
    ok &= np.sqrt(np.maximum(c2 - c1 ** 2, 0.0)) < SETTLE_H
    hit = np.flatnonzero(ok)
    return float(t[stop_idx + int(hit[0])] - t[stop_idx]) if hit.size else np.nan


def _segment_latencies(sess: Session, win: MotionWindow) -> Dict[str, float]:
    """Lag from the ``skill_send`` that caused each segment to the first motion."""
    sends = [e for e in sess.skill_sends() if e.skill not in PARAM_SKILLS]
    t0 = sess.t0_monotonic
    if not sends or t0 is None or not win.ok:
        return {"send_to_motion_s": np.nan, "trigger_skill": ""}
    s_start = sess.t[win.segments[0][0]]
    send_times = np.array([e.t_mono - t0 for e in sends])
    prior = np.flatnonzero(send_times <= s_start)
    if prior.size == 0:
        return {"send_to_motion_s": np.nan, "trigger_skill": ""}
    k = int(prior[-1])
    return {"send_to_motion_s": float(s_start - send_times[k]), "trigger_skill": sends[k].skill or ""}


def profile_session(sess: Session) -> Dict[str, object]:
    t, fs = sess.t, sess.fs
    win = detect_motion(sess)
    mask = win.mask if win.ok else np.ones(sess.n, dtype=bool)
    segs = win.segments if win.ok else [(0, sess.n)]

    row: Dict[str, object] = {
        "group": sess.group,
        "session": sess.path.name,
        "name": sess.name,
    }
    row.update(_skill_names(sess))
    cmd = _cmd_params(sess)
    row.update(cmd)
    row["motion_type"] = _motion_type(cmd, str(row["primary_skill"]))
    row["samples"] = sess.n
    row["fs_hz"] = round(fs, 1)
    row["rec_duration_s"] = round(sess.duration_s, 3)

    # -- motion window --------------------------------------------------
    i0, i1 = (segs[0][0], segs[-1][1] - 1)
    row["win_start_s"] = float(t[i0])
    row["win_end_s"] = float(t[i1])
    row["win_span_s"] = float(t[i1] - t[i0])
    row["active_s"] = float(mask.sum() / fs)
    row["n_segments"] = len(segs)
    row["idle_dq_rad_s"] = float(win.idle_level)
    row["peak_dq_rad_s"] = float(np.nanmax(win.speed))
    row.update(_segment_latencies(sess, win))
    last_hot = np.flatnonzero(win.speed[: segs[-1][1]] > win.enter_level)
    row["settle_s"] = _settle_time(
        sess, int(last_hot[-1]) if last_hot.size else segs[-1][1] - 1, win.speed, win.exit_level
    )

    # -- velocity --------------------------------------------------------
    vx = sess.col("base_vx")[mask]
    vy = sess.col("base_vy")[mask]
    wz = sess.col("yaw_speed")[mask]
    fvx, fvy, fwz = vx[np.isfinite(vx)], vy[np.isfinite(vy)], wz[np.isfinite(wz)]
    row["vx_mean"] = float(fvx.mean()) if fvx.size else np.nan
    row["vx_std"] = float(fvx.std()) if fvx.size else np.nan
    row["vx_p5"] = float(np.percentile(fvx, 5)) if fvx.size else np.nan
    row["vx_p95"] = float(np.percentile(fvx, 95)) if fvx.size else np.nan
    row["vx_max"] = float(fvx.max()) if fvx.size else np.nan
    row["vx_min"] = float(fvx.min()) if fvx.size else np.nan
    row["vx_absmax"] = float(np.abs(fvx).max()) if fvx.size else np.nan
    row["vy_mean"] = float(fvy.mean()) if fvy.size else np.nan
    row["vy_absmax"] = float(np.abs(fvy).max()) if fvy.size else np.nan
    sp = np.hypot(vx, vy)
    sp = sp[np.isfinite(sp)]
    row["speed_mean"] = float(sp.mean()) if sp.size else np.nan
    # steady state = middle 60 % of the longest active segment (drops accel/decel)
    ls, le = win.longest() if win.ok else (0, sess.n)
    cut = int(0.2 * (le - ls))
    ss = slice(ls + cut, le - cut) if (le - ls) > 4 * max(cut, 1) else slice(ls, le)
    sv = sess.col("base_vx")[ss]
    row["vx_steady_mean"] = float(np.nanmean(sv)) if np.isfinite(sv).any() else np.nan
    sm = np.hypot(sess.col("base_vx")[ss], sess.col("base_vy")[ss])
    row["speed_steady_mean"] = float(np.nanmean(sm)) if np.isfinite(sm).any() else np.nan
    svy = sess.col("base_vy")[ss]
    row["vy_steady_mean"] = float(np.nanmean(svy)) if np.isfinite(svy).any() else np.nan
    sw = sess.col("yaw_speed")[ss]
    row["yaw_rate_steady_mean"] = float(np.nanmean(sw)) if np.isfinite(sw).any() else np.nan
    row["yaw_rate_mean"] = float(fwz.mean()) if fwz.size else np.nan
    row["yaw_rate_absmax"] = float(np.abs(fwz).max()) if fwz.size else np.nan

    px, py, yaw = sess.col("base_pos_x"), sess.col("base_pos_y"), sess.col("yaw")
    dxy = np.hypot(px[i1] - px[i0], py[i1] - py[i0])
    row["odom_dist_m"] = float(dxy)
    row["odom_speed_mean"] = float(dxy / (t[i1] - t[i0])) if t[i1] > t[i0] else np.nan
    dyaw = np.rad2deg(np.arctan2(np.sin(yaw[i1] - yaw[i0]), np.cos(yaw[i1] - yaw[i0])))
    row["odom_yaw_deg"] = float(dyaw)

    # -- gait -------------------------------------------------------------
    force = sess.foot_force()
    cr = detect_contact(force, mask, fs)
    duty = cr.contact[mask].mean(axis=0)
    freq, per_leg, stride_cv, n_iv = stride_frequency(cr.contact, t, segs)
    flight = (~cr.contact).all(axis=1) & mask
    fl_runs = [(e - s) / fs for s, e in runs_of(flight)]
    phases = relative_phases(cr.contact, t, segs, 1.0 / freq if np.isfinite(freq) and freq > 0 else np.nan)
    pattern, pat_scores = classify_pattern(cr.contact, mask)

    for i, leg in enumerate(LEGS):
        row[f"contact_thr_{leg}_N"] = float(cr.thresholds[i])
        row[f"duty_{leg}"] = float(duty[i])
    row["contact_thr_mean_N"] = float(np.mean(cr.thresholds))
    row["duty_mean"] = float(duty.mean())
    row["duty_min"] = float(duty.min())
    row["duty_max"] = float(duty.max())
    row["stride_hz"] = float(freq)
    row["stride_hz_leg_spread"] = float(np.nanmax(per_leg) - np.nanmin(per_leg)) if np.isfinite(per_leg).any() else np.nan
    row["stride_cv"] = float(stride_cv)
    row["n_stride_intervals"] = n_iv
    row["periodic"] = bool(n_iv >= 8 and np.isfinite(stride_cv) and stride_cv < 0.30)
    row["flight_frac"] = float(flight.sum() / max(mask.sum(), 1))
    row["flight_max_s"] = float(max(fl_runs)) if fl_runs else 0.0
    row["flight_mean_s"] = float(np.mean(fl_runs)) if fl_runs else 0.0
    row["n_flight_phases"] = len(fl_runs)
    # how synchronous the swings are: of the time at least one foot is off the
    # ground, how much of it has *all four* off (~flight fraction for a running
    # trot, close to 1 for a four-leg-sync jump)
    any_off = (~cr.contact).any(axis=1) & mask
    row["swing_sync"] = float(flight.sum() / any_off.sum()) if any_off.any() else np.nan
    for i, leg in enumerate(LEGS[1:], start=1):
        row[f"phase_{leg}"] = float(phases[i])
    row["gait_pattern"] = pattern
    for key in ("trot", "pace", "bound"):
        row[f"pair_score_{key}"] = float(pat_scores.get(key, np.nan))
    ftot = np.nansum(force, axis=1)[mask]
    row["foot_force_tot_mean_N"] = float(np.mean(ftot)) if ftot.size else np.nan
    row["foot_force_peak_N"] = float(np.nanmax(force[mask])) if mask.any() else np.nan
    row["foot_force_clipped_frac"] = float((force[mask] >= 209.0).mean()) if mask.any() else np.nan

    # -- body height ------------------------------------------------------
    h = sess.col("body_height")[mask]
    fh = h[np.isfinite(h)]
    row["height_mean_m"] = float(fh.mean()) if fh.size else np.nan
    row["height_std_m"] = float(fh.std()) if fh.size else np.nan
    row["height_min_m"] = float(fh.min()) if fh.size else np.nan
    row["height_max_m"] = float(fh.max()) if fh.size else np.nan
    row["height_range_m"] = float(fh.max() - fh.min()) if fh.size else np.nan
    hs = sess.col("body_height")
    fhs = hs[np.isfinite(hs)]
    row["height_session_min_m"] = float(fhs.min()) if fhs.size else np.nan
    row["height_session_max_m"] = float(fhs.max()) if fhs.size else np.nan

    # -- torque ------------------------------------------------------------
    tau = sess.joint_matrix("tau")[mask]
    with np.errstate(invalid="ignore"):
        row["tau_rms_Nm"] = float(np.sqrt(np.nanmean(tau ** 2)))
        row["tau_peak_Nm"] = float(np.nanmax(np.abs(tau)))
    for k, joint in enumerate(JOINTS):
        sub = tau[:, k::3]
        with np.errstate(invalid="ignore"):
            row[f"tau_rms_{joint}_Nm"] = float(np.sqrt(np.nanmean(sub ** 2)))
            row[f"tau_peak_{joint}_Nm"] = float(np.nanmax(np.abs(sub)))
    flat = np.abs(tau)
    if np.isfinite(flat).any():
        j = int(np.nanargmax(flat) % 12)
        row["tau_peak_joint"] = f"{LEGS[j // 3]}_{JOINTS[j % 3]}"
    else:
        row["tau_peak_joint"] = ""

    # -- q_des sentinel ------------------------------------------------------
    qd = sess.joint_matrix("q_des")
    sent = np.isclose(qd, POS_STOP_F, rtol=1e-3, atol=0.0)
    finite = np.isfinite(qd)
    row["posstop_frac"] = float(sent[mask].mean())
    row["posstop_frac_session"] = float(sent.mean())
    row["posstop_rows_frac"] = float((sent.sum(axis=1) == 12)[mask].mean())
    if sent.any():
        first = int(np.flatnonzero(sent.any(axis=1))[0])
        row["posstop_first_s"] = float(t[first])
        row["posstop_total_s"] = float(sent.any(axis=1).sum() / fs)
    else:
        row["posstop_first_s"] = np.nan
        row["posstop_total_s"] = 0.0
    kp = sess.joint_matrix("kp")
    kpf = kp[np.isfinite(kp) & ~np.isclose(kp, POS_STOP_F, rtol=1e-3)]
    row["kp_max"] = float(kpf.max()) if kpf.size else np.nan

    # -- data quality ---------------------------------------------------------
    used = ["base_vx", "base_vy", "yaw_speed", "body_height"]
    row["nan_frac_state"] = float(max(float(np.mean(~np.isfinite(sess.col(c)))) for c in used))
    row["nan_frac_qdes"] = float(np.mean(~finite))
    row["nan_frac_tau"] = float(np.mean(~np.isfinite(sess.joint_matrix("tau"))))
    row["has_events"] = bool(sess.events)

    if not row["motion_type"]:
        row["motion_type"] = _measured_motion_type(
            float(row["vx_steady_mean"]), float(row["vy_mean"]), float(row["yaw_rate_steady_mean"])
        )
    row["skill_key"] = _skill_key(row)
    return row


LEAD_COLUMNS = [
    "group", "session", "name", "skill_key", "primary_skill", "gait_mode", "motion_type",
    "skill_seq", "prereq_skills", "cmd_vx", "cmd_vy", "cmd_wz", "cmd_duration_s", "speed_level",
]


def profile_all(sessions: List[Session]) -> pd.DataFrame:
    df = pd.DataFrame([profile_session(s) for s in sessions])
    rest = [c for c in df.columns if c not in LEAD_COLUMNS]
    df = df[LEAD_COLUMNS + rest]
    return df.sort_values(["group", "session"]).reset_index(drop=True)
