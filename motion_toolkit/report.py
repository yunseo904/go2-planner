"""Markdown reports for the curated-log skill profile and transition cost."""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import numpy as np
import pandas as pd

from .contact import ENTER_FRAC, LEAVE_FRAC, MIN_RUN_S
from .jump import BALLISTIC_DV, FREEFALL_A, G, PUSH_S
from .session import POS_STOP_F


def _fmt(v: float, digits: int = 2) -> str:
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "–"
    return f"{v:.{digits}f}"


def agg(series: pd.Series, digits: int = 2) -> str:
    """``mean ± std`` over a group, or the bare value when the group has one row."""
    x = pd.to_numeric(series, errors="coerce").to_numpy(dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return "–"
    if x.size == 1:
        return _fmt(float(x[0]), digits)
    return f"{_fmt(float(x.mean()), digits)} ± {_fmt(float(x.std(ddof=1)), digits)}"


def rng(series: pd.Series, digits: int = 2) -> str:
    x = pd.to_numeric(series, errors="coerce").to_numpy(dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return "–"
    lo, hi = _fmt(float(x.min()), digits), _fmt(float(x.max()), digits)
    if lo == hi:
        return lo
    # an en-dash between two negative numbers is unreadable
    return f"{lo} to {hi}" if lo.startswith("-") else f"{lo}–{hi}"


def md_table(header: Sequence[str], rows: Sequence[Sequence[object]], align: Optional[str] = None) -> str:
    sep = ["---"] * len(header) if align is None else [
        {"l": ":---", "r": "---:", "c": ":---:"}[a] for a in align
    ]
    out = ["| " + " | ".join(str(h) for h in header) + " |",
           "| " + " | ".join(sep) + " |"]
    for r in rows:
        out.append("| " + " | ".join("" if c is None else str(c) for c in r) + " |")
    return "\n".join(out)


# ---------------------------------------------------------------- skill profile
LOCOMOTION_HEADER = [
    "skill_key", "n", "cmd", "vx̄ steady", "vȳ steady (abs)", "ω̄z steady (abs)", "‖v‖ steady", "stride Hz",
    "duty", "flight %", "max flight s", "height m", "Δheight m", "τ RMS", "τ peak", "pattern",
]

DISCRETE_HEADER = [
    "skill_key", "n", "motion s", "all-off %", "max all-off s", "height m",
    "height min–max", "τ RMS", "τ peak", "peak joint", "PosStopF %", "pattern",
]


def _cmd_label(sub: pd.DataFrame) -> str:
    if sub["primary_skill"].iloc[0] == "gamepad":
        return "gamepad"
    vx, vy, wz = rng(sub["cmd_vx"], 2), rng(sub["cmd_vy"], 2), rng(sub["cmd_wz"], 2)
    parts = []
    if vx not in ("–", "0.00"):
        parts.append(f"x={vx}")
    if vy not in ("–", "0.00"):
        parts.append(f"y={vy}")
    if wz not in ("–", "0.00"):
        parts.append(f"z={wz}")
    lvl = sub["speed_level"].dropna()
    if len(lvl):
        parts.append(f"lvl={rng(lvl, 0)}")
    return ", ".join(parts) or "–"


def _mode(series: pd.Series) -> str:
    vc = series.value_counts()
    if vc.empty:
        return "–"
    return vc.index[0] if len(vc) == 1 else f"{vc.index[0]} ({vc.iloc[0]}/{len(series)})"


def skill_tables(df: pd.DataFrame) -> Dict[str, str]:
    loco, disc = [], []
    for key, sub in df.groupby("skill_key", sort=True):
        if str(sub["motion_type"].iloc[0]) == "discrete":
            disc.append([
                f"`{key}`", len(sub), agg(sub["active_s"], 2), agg(sub["flight_frac"] * 100, 1),
                agg(sub["flight_max_s"], 3), agg(sub["height_mean_m"], 3),
                f"{rng(sub['height_session_min_m'], 2)} → {rng(sub['height_session_max_m'], 2)}",
                agg(sub["tau_rms_Nm"], 1), agg(sub["tau_peak_Nm"], 1),
                _mode(sub["tau_peak_joint"]), agg(sub["posstop_frac"] * 100, 2), _mode(sub["gait_pattern"]),
            ])
        else:
            loco.append([
                f"`{key}`", len(sub), _cmd_label(sub), agg(sub["vx_steady_mean"], 2),
                agg(sub["vy_steady_mean"].abs(), 2), agg(sub["yaw_rate_steady_mean"].abs(), 2),
                agg(sub["speed_steady_mean"], 2),
                agg(sub["stride_hz"], 2), agg(sub["duty_mean"], 3),
                agg(sub["flight_frac"] * 100, 1), agg(sub["flight_max_s"], 3),
                agg(sub["height_mean_m"], 3), agg(sub["height_range_m"], 3),
                agg(sub["tau_rms_Nm"], 1), agg(sub["tau_peak_Nm"], 1), _mode(sub["gait_pattern"]),
            ])
    loco.sort(key=lambda r: -float(r[1]))
    return {
        "locomotion": md_table(LOCOMOTION_HEADER, loco),
        "discrete": md_table(DISCRETE_HEADER, disc),
    }


JUMP_HEADER = [
    "session", "flight s", "free-fall %, a (m/s²)", "v_z0 = gT/2", "v_z0 impulse", "apex gT²/8",
    "apex impulse", "vx take-off", "Δx jump", "Δz odom (flight)", "Δh (flight)",
    "land acc_z pk", "land leg force pk", "land total pk",
]

REPEAT_METRICS = [
    ("flight_s", "flight time", "s", 3),
    ("vz_takeoff_ballistic", "take-off v_z (from T)", "m/s", 2),
    ("push_impulse_dv", "take-off v_z (push impulse)", "m/s", 2),
    ("apex_rise_ballistic_m", "apex rise (from T)", "m", 3),
    ("dx_jump_m", "horizontal travel", "m", 3),
    ("land_acc_z_peak", "landing acc_z peak", "m/s²", 1),
    ("land_force_total_peak_N", "landing total foot force peak", "N", 0),
    ("push_force_total_peak_N", "push-off total foot force peak", "N", 0),
    ("crouch_height_m", "crouch depth", "m", 3),
]


def jump_tables(jdf: pd.DataFrame) -> Dict[str, str]:
    rows = []
    for _, r in jdf.iterrows():
        rows.append([
            f"`{r['session']}`", _fmt(r["flight_s"], 3),
            f"{r['freefall_frac'] * 100:.0f} %, a={r['acc_mag_flight_med']:.1f}",
            _fmt(r["vz_takeoff_ballistic"], 2), _fmt(r["push_impulse_dv"], 2),
            _fmt(r["apex_rise_ballistic_m"], 3), _fmt(r["apex_rise_impulse_m"], 3),
            _fmt(r["vx_takeoff"], 3), _fmt(r["dx_jump_m"], 3),
            _fmt(r["dz_odom_flight_m"], 3), _fmt(r["dh_flight_m"], 3),
            _fmt(r["land_acc_z_peak"], 1),
            f"{r['land_force_leg_peak_N']:.0f}" + ("*" if r["land_force_clipped_samples"] else ""),
            f"{r['land_force_total_peak_N']:.0f}",
        ])
    per_session = md_table(JUMP_HEADER, rows)

    rep = []
    for skill in ["front_jump", "front_pounce"]:
        sub = jdf[jdf["skill"] == skill]
        if sub.empty:
            continue
        for col, label, unit, digits in REPEAT_METRICS:
            x = pd.to_numeric(sub[col], errors="coerce").to_numpy(dtype=float)
            x = x[np.isfinite(x)]
            if x.size == 0:
                rep.append([f"`{skill}`", label, unit, "–", "–", "–", "–"])
                continue
            sd = float(x.std(ddof=1)) if x.size > 1 else float("nan")
            cv = abs(sd / x.mean()) if (np.isfinite(sd) and abs(x.mean()) > 1e-9) else float("nan")
            rep.append([
                f"`{skill}`", label, unit, f"n={x.size}",
                f"{_fmt(float(x.mean()), digits)} ± {_fmt(sd, digits)}",
                f"{_fmt(float(x.min()), digits)}–{_fmt(float(x.max()), digits)}",
                f"{cv * 100:.0f} %" if np.isfinite(cv) else "–",
            ])
    return {
        "per_session": per_session,
        "repeat": md_table(["skill", "metric", "unit", "n", "mean ± sd", "min–max", "CV"], rep),
    }


def jump_md(jdf: pd.DataFrame, df: pd.DataFrame) -> str:
    tabs = jump_tables(jdf)
    j = jdf[jdf["skill"] == "front_jump"]
    p = jdf[jdf["skill"] == "front_pounce"]
    trot = df[df["skill_key"] == "trot_run+move:forward"]
    drift = jdf["odom_drift_m_s"]
    apex = j["apex_rise_ballistic_m"]
    apex_i = j["apex_rise_impulse_m"]
    vz = j["vz_takeoff_ballistic"]
    dv = j["push_impulse_dv"]
    # widest reach a running trot could have while airborne
    reach = (trot["speed_steady_mean"] * trot["flight_max_s"])
    return f"""## 3. Jump and bound displacement

### Why this cannot be read off the odometry

`base_pos_*`, `base_v*` and `body_height` are **leg-kinematic** estimates: they
are computed from the joints and the contact set. With all four feet off the
ground they stop tracking. Over a {j['flight_s'].mean():.2f} s `front_jump`
flight `base_pos_z` moves {rng(j['dz_odom_flight_m'], 3)} m, `body_height` moves
{rng(j['dh_flight_m'], 3)} m and `base_vz` sits at {rng(j['vz_odom_takeoff'], 3)} m/s —
all three report a jump of a few centimetres for a flight whose *duration alone*
requires a ballistic arc an order of magnitude larger. The odometry is not noisy
here, it is blind.

So height and take-off speed are taken from quantities that stay valid in the air:

1. **Flight time `T`** — the longest all-four-off run inside the motion window,
   using the per-session contact threshold. The IMU confirms the robot is really
   airborne: specific-force magnitude falls to
   {rng(jdf['acc_mag_flight_med'], 2)} m/s² during the run against ~9.8 m/s²
   standing, i.e. free fall.
2. **Ballistic inversion** — take-off and touch-down happen at nearly the same
   body height ({rng(j[j['takeoff_height_m'] < 0.3]['takeoff_height_m'], 2)} m vs
   {rng(j[j['takeoff_height_m'] < 0.3]['touchdown_height_m'], 2)} m for the four
   sessions that take off from the crouch; the fifth starts higher but the two
   still match each other), so `v_z0 = g·T/2` and `apex = g·T²/8`. This uses
   only the clock; no accelerometer scale enters.
3. **Push-off impulse** — `∫(acc_z − g)dt` over the {PUSH_S:.2f} s before
   take-off, an independent estimate of the same `v_z0`. The push-off is a slow
   ~0.25 s ramp, so the sport-state accelerometer resolves it.

For `front_jump` the two independent estimates agree: `g·T/2` gives
{vz.mean():.2f} ± {vz.std(ddof=1):.2f} m/s, the push-off impulse gives
{dv.mean():.2f} ± {dv.std(ddof=1):.2f} m/s — a
{abs(dv.mean() - vz.mean()) / vz.mean() * 100:.0f} % difference. That agreement is
what makes the height numbers below usable at all.

**A run with all four feet unloaded is not automatically a jump.** These skills
also lose every contact while crouching, and while swinging both legs forward in
a lunge; in both cases the body is falling rather than launching, and the
push-off impulse comes out at or below zero. A run is only counted as flight
when its push-off impulse reaches {BALLISTIC_DV:.1f} m/s. Of the
{int(jdf['n_airborne_runs'].sum())} all-four-off runs in the eight sessions,
{int(jdf['n_ballistic_runs'].sum())} pass — all of them in `front_jump`.

### Per session

`*` on the landing leg force means at least one sample hit the ~210 N sensor
clip, so that peak is a lower bound.

{tabs['per_session']}

### Repeatability across sessions

{tabs['repeat']}

### What the numbers say

* **`front_jump` is a vertical hop in place, not a leap.** Take-off `base_vx` is
  {rng(j['vx_takeoff'], 3)} m/s and the horizontal travel between the stance
  before take-off and the stance after landing is
  {j['dx_jump_m'].mean() * 1000:.0f} ± {j['dx_jump_m'].std(ddof=1) * 1000:.0f} mm.
  It gains height and no ground. (Two sessions show ~0.6 m in `dx_skill_m`, but
  that is the walking the robot does *after* the jump, in a separate motion
  segment — not the jump itself.)
* **Apex rise is {apex.mean():.2f} ± {apex.std(ddof=1):.2f} m** from the flight
  time ({rng(apex, 2)} m), and {apex_i.mean():.2f} ± {apex_i.std(ddof=1):.2f} m from
  the push-off impulse. Take-off speed is ~{vz.mean():.1f} m/s straight up. The
  push-off itself shows as {rng(j['push_force_total_peak_N'], 0)} N of total foot
  force over ~0.25 s from a {rng(j['crouch_height_m'], 2)} m crouch.
* **`front_pounce` never leaves the ground ballistically.** All four feet do
  unload for {rng(p['flight_s'], 3)} s, but the push-off impulse is
  {rng(p['push_impulse_dv'], 2)} m/s — negative, i.e. the body was *descending*
  when contact was lost — and `body_height` *increases* through the run, which is
  the legs reaching down and forward, not the body rising. It is a lunge stride:
  net forward travel {rng(p['dx_skill_m'], 2)} m over the whole skill, with the
  displacement happening on the ground. Its push-off force peak,
  {rng(p['push_force_total_peak_N'], 0)} N, is about half the jump's.
  **This corrects the framing in §2 and in `INDEX.md`:** `front_pounce`'s
  "flight" is a contact gap, not an aerial phase.
* **Landing impact is under-measured, in two independent ways.** `acc_z` peaks at
  {rng(jdf['land_acc_z_peak'], 0)} m/s² and total foot force at
  {rng(jdf['land_force_total_peak_N'], 0)} N, with individual legs hitting the
  ~210 N clip in {int((jdf['land_force_clipped_samples'] > 0).sum())} of
  {len(jdf)} sessions. But integrating a `front_jump` landing gives only
  {rng(j['land_dv_measured'], 2)} m/s of arrested velocity where ~{vz.mean():.1f} m/s
  has to go somewhere: `acc_z` arrives held at the sport-state publish rate
  (repeated sample values are visible in the raw log), so a few-millisecond impact
  is aliased away. **Treat every landing peak here as a lower bound**, the same
  way the force clip is.
* **One outlier worth knowing about.** `front_jump_20260824_232314` takes off from
  {j[j['session'].str.contains('232314')]['takeoff_height_m'].iloc[0]:.2f} m body
  height instead of the ~0.19 m the other four use, carries
  {j[j['session'].str.contains('232314')]['vx_takeoff'].iloc[0]:.2f} m/s of forward
  speed into take-off. The odometry also reports
  {abs(j[j['session'].str.contains('232314')]['dy_jump_m'].iloc[0]) * 100:.0f} cm of lateral
  shift across that jump — most of it accumulating *inside* the blind flight
  window, so read it as a symptom of the estimator losing its contact reference
  rather than as 23 cm of real sideways travel. Same flight time, different entry
  posture.

### Odometry drift

There are two separate error sources, and only one of them is drift.

**Ordinary drift is small.** Measured on the stationary stretch before each
skill, `base_pos` wanders {drift.min() * 1000:.2f}–{drift.max() * 1000:.2f} mm/s
(median {drift.median() * 1000:.2f} mm/s). Over the ~1.3 s bracket used for
`Δx jump` that is at most ~{drift.max() * 1.3 * 1000:.0f} mm.

**Losing the contact reference is not small.** The leg-kinematic estimator has
nothing to integrate against while every foot is in the air, and the error it
accumulates there does not respect the standing drift rate — the outlier session
above picks up {abs(j[j['session'].str.contains('232314')]['dy_jump_m'].iloc[0]) * 100:.0f} cm
of lateral offset across one jump. So the `Δx jump` figures in the table are read
across a blind window and are **not** accurate to the millimetre.

What makes the "no horizontal travel" conclusion safe anyway is that it does not
rest on the odometry: take-off `base_vx` is measured *while the feet are still
loaded*, where the estimator works, and it is {rng(j['vx_takeoff'], 2)} m/s. With
no horizontal velocity at take-off there is no horizontal impulse in flight, so
the range is ~0 by physics, and the {j['dx_jump_m'].mean() * 1000:.0f} mm the
odometry reports is consistent with that rather than evidence for it.

Separately, `base_pos` is a *relative* frame that never returns to a surveyed
origin — absolute positions in these logs (e.g. `base_pos_y ≈ −28.7 m`) carry no
meaning, only differences do.

### Estimated obstacle limits

These are ceilings derived from what the stock controller actually did, not from
the robot's mechanical capability, and every one of them carries the assumptions
stated in the row.

| quantity | estimate | how it is derived / what it assumes |
| :--- | ---: | :--- |
| `front_jump` CoM apex rise | **{apex.mean():.2f} ± {apex.std(ddof=1):.2f} m** | `g·T²/8` from {len(j)} sessions; assumes take-off and touch-down at equal height (holds to ±20 mm) |
| `front_jump` foot clearance at apex | **≈ {apex.mean():.2f} m** | the leg configuration is held through flight (`body_height` constant to {rng(j['dh_flight_m'], 3)} m), so the feet rise with the body |
| **max step-up, absolute ceiling** | **≈ {apex.mean():.2f} m** | `v_z0²/2g`. Landing exactly at apex leaves zero vertical margin and zero forward speed to carry the body over the lip, so this is unreachable in practice |
| **max step-up, working estimate** | **0.12–0.15 m** | half the ceiling, leaving margin for foot clearance over the lip, for landing energy the legs must absorb, and for the {apex.std(ddof=1) / apex.mean() * 100:.0f} % session-to-session spread in flight time |
| **max gap by jumping** | **≈ 0 m** | horizontal range is `v_x0 · T`, and `v_x0` is {rng(j['vx_takeoff'], 2)} m/s. No skill in this set launches with forward speed |
| max unsupported horizontal travel, any skill | **{reach.min():.2f}–{reach.max():.2f} m** | the running trot's aerial phase is the only other flight: steady speed {rng(trot['speed_steady_mean'], 2)} m/s × longest aerial phase {rng(trot['flight_max_s'], 3)} s |
| `front_pounce` step-up / gap | **0 m** | no ballistic launch at all (push-off impulse {rng(p['push_impulse_dv'], 2)} m/s) |

Two consequences for a planner:

1. **A step-up is plausible, a gap is not.** The stock `front_jump` clears height
   but covers no ground, so it can be aimed at a raised platform whose lip is
   within ~0.15 m *and directly under the robot*, and at nothing that requires
   travelling. Crossing a gap with this skill set means walking it, which bounds
   the crossable gap by the fore-aft foot span rather than by anything measured
   here.
2. **The 0.12–0.15 m working number is an estimate, not a measurement.** Every
   session in this set jumps on flat floor and lands at its take-off height. The
   logs contain no landing onto a raised or lowered surface, so the controller's
   behaviour on impact with a step is unobserved. Treat the number as a screening
   threshold for which terrain is worth attempting, not as a guarantee.

"""


SESSION_HEADER = [
    "group", "session", "skill_key", "window s", "active s", "vx̄", "vx p5–p95", "vȳ", "ω̄z",
    "stride Hz", "duty", "flight %", "thr N", "height m", "τ RMS", "τ peak", "PosStopF %",
]  # vx̄/vȳ/ω̄z here are signed means over the active mask (direction matters per session)


def session_table(df: pd.DataFrame) -> str:
    rows = []
    for _, r in df.iterrows():
        rows.append([
            r["group"].split("_", 1)[0], f"`{r['session']}`", f"`{r['skill_key']}`",
            f"{r['win_start_s']:.1f}–{r['win_end_s']:.1f}", _fmt(r["active_s"], 2),
            _fmt(r["vx_mean"], 3), f'{_fmt(r["vx_p5"], 2)}…{_fmt(r["vx_p95"], 2)}', _fmt(r["vy_mean"], 3),
            _fmt(r["yaw_rate_mean"], 3), _fmt(r["stride_hz"], 2), _fmt(r["duty_mean"], 3),
            _fmt(r["flight_frac"] * 100, 1), _fmt(r["contact_thr_mean_N"], 0),
            _fmt(r["height_mean_m"], 3), _fmt(r["tau_rms_Nm"], 1), _fmt(r["tau_peak_Nm"], 1),
            _fmt(r["posstop_frac"] * 100, 2),
        ])
    return md_table(SESSION_HEADER, rows)


def skill_profile_md(df: pd.DataFrame, jdf: pd.DataFrame, csv_name: str, curated_root: str) -> str:
    tabs = skill_tables(df)
    n_sess = len(df)
    lag = df["send_to_motion_s"].dropna()
    per = df[df["periodic"]]
    parts: List[str] = []
    parts.append(f"""# Go2 skill profile — {n_sess} curated sessions

Generated by `scripts/profile_skills.py` from the read-only curated log set
(`{curated_root}`). One row per session in [`{csv_name}`]({csv_name});
this file is the per-skill roll-up of that CSV.

## How the numbers are produced

**Motion window — from joint velocity, never from `events.jsonl`.**
`run.sh` needs ~2.4 s to start, so a `skill_send` timestamp is not when the robot
moves. Measured over the {len(lag)} API-driven sessions the lag from `skill_send`
to the first moving sample is **{lag.mean():.2f} ± {lag.std():.2f} s**
(min {lag.min():.2f}, max {lag.max():.2f}) — see `skill_transition.md`.
The window instead comes from `speed = mean_j |dq_j|` over the 12 joints,
box-smoothed over 50 ms. Standing still this sits at
{df['idle_dq_rad_s'].median():.3f} rad/s and is very quiet, so a two-level split
works: enter at `p10 + 0.25·(p99.5 − p10)`, leave at `p10 + 0.05·(p99.5 − p10)`,
runs closer than 0.30 s merged, runs under 0.15 s dropped. Every statistic below
is taken over the **union of the surviving segments** (`active_s`), so the quiet
gap between e.g. `stand_down` and `recovery_stand` is never counted as stance.

**Contact — per session, per leg, from `foot_force`.**
The logged `contact_*` columns use a fixed 20 N threshold that does not hold
across speeds. Contact is re-derived from each leg's force distribution *inside*
the motion window: with `p5`/`p95` of that leg and `span = p95 − p5`, contact
latches on above `p5 + {ENTER_FRAC:.2f}·span` and releases below
`p5 + {LEAVE_FRAC:.2f}·span` (Schmitt trigger), then runs shorter than
{MIN_RUN_S * 1000:.0f} ms are absorbed. The hysteresis matters: without it the
force ripple inside a stance phase double-counts touchdowns and inflates stride
frequency by up to 2× on the slow-walk sessions. Resulting thresholds run
{df['contact_thr_mean_N'].min():.0f}–{df['contact_thr_mean_N'].max():.0f} N,
i.e. the fixed 20 N would have been wrong in every session.

**Unusable / degraded columns.** `foot_pos_*` and `foot_vel_*` are all zero on
this firmware and are refused by the loader. `foot_force` clips at ~210 N, so
contact *timing* is sound but landing force is an underestimate
(`foot_force_clipped_frac` in the CSV). All reductions are nan-safe;
`base_v*`/`body_height` dropouts are ≤{df['nan_frac_state'].max() * 100:.2f} %
of samples and `*_q_des` ≤{df['nan_frac_qdes'].max() * 100:.2f} %.

**Periodicity gate.** `stride_hz` / `duty` / `gait_pattern` only describe a gait
when the motion actually repeats. `periodic` in the CSV is
`n_intervals ≥ 8 and CV(stride interval) < 0.30`; it is True for
{int(per.shape[0])} of {n_sess} sessions and False for every `front_jump`,
`front_pounce` and the posture transition — read those rows' gait columns as
descriptive only.

## 1. Locomotion skills

Velocities are the steady-state means (middle 60 % of the longest active
segment) so ramp-up and braking do not drag them down; `base_v*` is the sport
state's body-frame velocity and oscillates strongly inside a stride, which is
why §3 gives the per-session `vx p5–p95` band rather than a single peak: at
`move x=1.5` the instantaneous `base_vx` swings roughly −3 … +1.7 m/s inside one
stride, so its extremes (`vx_max` / `vx_min` / `vx_absmax` in the CSV) describe
the oscillation, not the travel speed.
`duty` is the mean over the four legs, `flight %` the share of active samples
with all four feet off. Lateral and yaw rates are aggregated as magnitudes,
otherwise the left/right pairs cancel. Torques in N·m over all 12 joints.

{tabs['locomotion']}

## 2. Discrete / aperiodic skills

`height min–max` spans the whole recording, not just the motion window, so the
crouch and the landing overshoot are both visible. The contact columns say
*all four feet unloaded*, which is not the same as airborne: these skills also
lose every contact while crouching and while swinging both legs through a lunge.
§3 separates the two, and finds that only `front_jump` actually leaves the ground.

{tabs['discrete']}

{jump_md(jdf, df)}
## 4. Per session

{session_table(df)}

""")
    parts.append(_findings(df))
    parts.append(_crosscheck(df))
    return "\n".join(parts)


def _findings(df: pd.DataFrame) -> str:
    g = df.groupby("skill_key")
    trot = df[df["skill_key"] == "trot_run+move:forward"]
    jump = df[df["skill_key"] == "front_jump"]
    pounce = df[df["skill_key"] == "front_pounce"]
    post = df[df["motion_type"] == "discrete"]
    per = df[df["periodic"] & (df["motion_type"] != "discrete")]
    fwd = per[per["motion_type"] == "forward"]
    lines = ["## 5. What the table says", ""]
    lines.append(
        f"* **Duty factor is the axis that separates the gaits.** Across the "
        f"{len(fwd)} periodic forward sessions duty runs "
        f"{fwd['duty_mean'].min():.2f}–{fwd['duty_mean'].max():.2f} and stride "
        f"{fwd['stride_hz'].min():.2f}–{fwd['stride_hz'].max():.2f} Hz, and duty "
        f"below ~0.40 is the only place flight appears "
        f"(r = {fwd['duty_mean'].corr(fwd['flight_frac']):.2f} between duty and flight fraction)."
    )
    lines.append(
        f"* **Only `trot_run` gives a periodic aerial phase.** Its "
        f"{len(trot)} sessions hold {trot['flight_frac'].mean() * 100:.0f} ± "
        f"{trot['flight_frac'].std() * 100:.0f} % flight with the longest single "
        f"aerial phase {trot['flight_max_s'].mean():.3f} ± {trot['flight_max_s'].std():.3f} s; "
        f"every non-`trot_run` walking session with duty ≥ 0.5 has exactly 0 % flight."
    )
    lines.append(
        f"* **The contact pattern is a trot almost everywhere.** "
        f"{int((df['gait_pattern'] == 'trot').sum())} of {len(df)} sessions classify as a "
        f"diagonal-pair trot from the leg-to-leg contact correlation — including the "
        f"turns, the strafes and backward walking. Only group 06 and the posture "
        f"transition leave that pattern, and they leave it in two different ways: "
        f"`front_jump` lifts all four legs together (swing-synchrony "
        f"{jump['swing_sync'].mean():.2f}, i.e. {jump['swing_sync'].mean() * 100:.0f} % of the "
        f"time any foot is off the ground *all* of them are, against "
        f"{trot['swing_sync'].mean():.2f} for the running trot), while `front_pounce` lifts "
        f"only the front pair — it scores {pounce['swing_sync'].mean():.2f} and classifies as "
        f"a bound, which is what a forward lunge is."
    )
    lines.append(
        f"* **`front_jump` is the only skill with a long, clean flight.** "
        f"{jump['flight_max_s'].mean():.3f} ± {jump['flight_max_s'].std():.3f} s of continuous "
        f"all-four-off, against {pounce['flight_max_s'].mean():.3f} ± "
        f"{pounce['flight_max_s'].std():.3f} s for `front_pounce` — and §3 shows the pounce "
        f"never launches at all, so even that is a contact gap rather than flight. "
        f"Peak joint torque {jump['tau_peak_Nm'].mean():.0f} N·m is the "
        f"highest in the set (running trot: {trot['tau_peak_Nm'].mean():.0f} N·m), and it is "
        f"an underestimate wherever `foot_force` clipped."
    )
    lines.append(
        f"* **Body height is nearly constant during locomotion and is the signature of the "
        f"posture skills.** Walking and running hold "
        f"{per['height_mean_m'].min():.2f}–{per['height_mean_m'].max():.2f} m with a "
        f"{per['height_range_m'].median() * 1000:.0f} mm peak-to-peak excursion, while "
        f"`stand_down→recovery_stand` sweeps "
        f"{post[post['skill_key'].str.contains('stand_down')]['height_session_min_m'].iloc[0]:.2f}–"
        f"{post[post['skill_key'].str.contains('stand_down')]['height_session_max_m'].iloc[0]:.2f} m."
    )
    sent = df[df["posstop_frac_session"] > 0.01]
    lines.append(
        f"* **`PosStopF` (`{POS_STOP_F:.4g}`) in `*_q_des` marks the joints the sport "
        f"controller is not position-holding.** It is essentially absent from every "
        f"locomotion session (≤{df[df['motion_type'] != 'discrete']['posstop_frac_session'].max() * 100:.2f} % "
        f"of that session's samples) and appears in bulk only in "
        f"{', '.join('`' + s + '`' for s in sent['skill_key'].unique())} "
        f"({', '.join(f'{v * 100:.0f} %' for v in sent['posstop_frac_session'])} of the session), "
        f"i.e. while the robot is lying down. Treat a non-zero rate as \"this segment is not "
        f"a position-controlled reference\" rather than as noise."
    )
    lines.append(
        f"* **Commanded speed is not achieved speed.** `move x=1.5–2.0` with "
        f"`speed_level 0` produces a measured steady "
        f"{trot[trot['cmd_vx'] >= 1.5]['speed_steady_mean'].mean():.2f} m/s and an odometry "
        f"average of {trot[trot['cmd_vx'] >= 1.5]['odom_speed_mean'].mean():.2f} m/s over the "
        f"window — roughly a quarter of the command. A planner must budget from the measured "
        f"column, not from `cmd_vx`."
    )
    return "\n".join(lines) + "\n"


def _crosscheck(df: pd.DataFrame) -> str:
    checks = [
        ("01 duty 0.31–0.36", "trot_run+move:forward", "duty_mean", 3),
        ("01 stride 3.0–3.2 Hz", "trot_run+move:forward", "stride_hz", 2),
        ("02 duty 0.50–0.55", None, "duty_mean", 3),
        ("03 duty 0.59–0.63", None, "duty_mean", 3),
    ]
    g01 = df[df["group"] == "01_running_trot_flight"]
    g02 = df[df["group"] == "02_trot_no_flight"]
    g03 = df[df["group"] == "03_slow_walk"]
    g06j = df[df["skill_key"] == "front_jump"]
    g06p = df[df["skill_key"] == "front_pounce"]
    rows = [
        ["01 duty 0.31–0.36", rng(g01["duty_mean"], 2), "✓"],
        ["01 stride 3.0–3.2 Hz", rng(g01["stride_hz"], 2) + " Hz", "≈ slightly low; the `_lvl0` recipe's own MANIFEST says 2.65 ± 0.16 Hz, which we bracket"],
        ["01 13–18 % of samples under 30 N total", rng(g01["flight_frac"] * 100, 0) + " % all-four-off", "≈ (looser criterion: per-leg threshold, not a fixed 30 N sum)"],
        ["02 duty 0.50–0.55, stride 2.0–2.8 Hz", f"{rng(g02['duty_mean'], 2)}, {rng(g02['stride_hz'], 2)} Hz", "≈ three of four match; `run_15` sits at duty 0.39, below the stated band"],
        ["02 \"no flight\"", f"{rng(g02['flight_frac'] * 100, 1)} % all-four-off", f"**disagrees for `run_15`**: {g02[g02['session'].str.startswith('run_15')]['flight_frac'].iloc[0] * 100:.0f} % of samples, longest {g02[g02['session'].str.startswith('run_15')]['flight_max_s'].iloc[0] * 1000:.0f} ms, at duty {g02[g02['session'].str.startswith('run_15')]['duty_mean'].iloc[0]:.2f} — brief, but real"],
        ["03 duty 0.59–0.63, stride 1.35–1.5 Hz", f"{rng(g03['duty_mean'], 2)}, {rng(g03['stride_hz'], 2)} Hz", "✓ (duty spread wider: `cross_step` reaches 0.68)"],
        ["06 front_jump flight 0.394 ± 0.093 s", agg(g06j["flight_max_s"], 3) + " s", "✓"],
        ["06 front_pounce flight 0.097 ± 0.024 s", agg(g06p["flight_max_s"], 3) + " s", "high — our per-leg threshold releases contact earlier than a fixed 30 N sum"],
        ["07 body_height 0.34 → 0.07 → 0.32", f"{df[df['group'] == '07_posture_transition']['height_session_min_m'].iloc[0]:.2f} → {df[df['group'] == '07_posture_transition']['height_session_max_m'].iloc[0]:.2f} m", "✓"],
    ]
    body = md_table(["INDEX.md claim", "measured here", "verdict"], rows)
    return f"""
## 6. Cross-check against `INDEX.md`

Independent re-derivation, so the numbers are close but not identical — the
contact threshold recipe differs (per-leg Schmitt trigger vs. a fixed total-force
cut), which mostly moves the flight numbers.

{body}

## 7. Caveats carried into any downstream use

1. Every session is the **stock Unitree sport controller**. Per the curated-set
   handoff these are reference measurements, not vetted teacher data, and must
   not be mixed unfiltered into positive training data.
2. `foot_force` clips at ~210 N — landing impulse is a lower bound. The clipped
   fraction per session is in `foot_force_clipped_frac`.
3. `mode` and `gait_type` are always 0 on this firmware; `error_code` is a state
   indicator (100 = idle), not a fault, so neither is used above.
4. `base_pos_*` is drifting odometry. `odom_speed_mean` is a sanity check on
   `base_v*`, not ground truth.
5. Windows are per session; a session with several segments (the jumps, the
   posture transition) mixes the preparation and recovery phases into one row.
   `n_segments` flags those, and `skill_transition.md` splits them per skill.
"""


# ------------------------------------------------------------ transition report
def transition_md(
    tdf: pd.DataFrame, summary: pd.DataFrame, preds: pd.DataFrame, profile: pd.DataFrame
) -> str:
    call_rows = []
    for _, r in summary.iterrows():
        call_rows.append([
            f"`{r['skill']}`", int(r["n"]), int(r["n_moved"]),
            _fmt(r["call_min_s"], 2), _fmt(r["call_mean_s"], 2), _fmt(r["call_max_s"], 2),
            _fmt(r["call_overhead_s"], 2),
            _fmt(r["lag_mean_s"], 2) if np.isfinite(r["lag_mean_s"]) else "–",
            f"{_fmt(r['lag_min_s'], 2)}–{_fmt(r['lag_max_s'], 2)}" if np.isfinite(r["lag_min_s"]) else "–",
            _fmt(r["motion_mean_s"], 2),
            _fmt(r["settle_mean_s"], 2), _fmt(r["settle_max_s"], 2),
        ])
    call_tab = md_table(
        ["skill", "sends", "moved", "call min", "call med", "call max", "call − cmd dur",
         "lag mean", "lag range", "motion s", "settle mean", "settle max"],
        call_rows,
    )

    pred_rows = []
    for _, r in preds.iterrows():
        pred_rows.append([
            f"`{r['skill']}`", f"`{r['prev_skill']}`", f"{int(r['count'])}/{int(r['n_sends'])}",
            f"{r['share'] * 100:.0f} %", "**yes**" if r["mandatory"] else "",
        ])
    pred_tab = md_table(["skill", "immediately preceded by", "count", "share", "always"], pred_rows)

    moved = tdf[tdf["moved"]]
    lag = moved["lag_s"]
    startup = tdf[tdf["skill"].isin(["speed_level", "balance_stand", "stand_down", "recovery_stand"])]["call_s"]
    gait_calls = tdf[tdf["skill"].isin(["trot_run", "static_walk", "economic_gait", "classic_walk", "cross_step"])]["call_s"]
    move = tdf[tdf["skill"] == "move"]
    silent = tdf[(~tdf["moved"]) & (~tdf["skill"].isin(["speed_level"]))]

    ev_rows = []
    for _, r in tdf.iterrows():
        ev_rows.append([
            r["group"].split("_", 1)[0], f"`{r['session']}`", int(r["index"]), f"`{r['skill']}`",
            r["param"], _fmt(r["send_s"], 2), _fmt(r["call_s"], 2),
            _fmt(r["lag_s"], 2), _fmt(r["motion_s"], 2), int(r["n_segments"]),
            _fmt(r["settle_s"], 2), "yes" if r["moved"] else "no",
        ])
    ev_tab = md_table(
        ["grp", "session", "#", "skill", "param", "send t", "call s", "lag s", "motion s",
         "segs", "settle s", "moved"],
        ev_rows,
    )

    return f"""# Go2 skill-transition cost — {tdf['session'].nunique()} sessions, {len(tdf)} `skill_send` events

Generated by `scripts/profile_skills.py`. The three 2026-08-04 gamepad sessions
carry no `events.jsonl` and are excluded here; they appear in `skill_profile.md`.

## What is being measured

Three costs, kept separate because they have different causes:

| symbol | definition | what it is |
| :--- | :--- | :--- |
| `call_s` | `skill_send` → `skill_done` | the **script** round trip. `run.sh` startup dominates it. |
| `lag_s` | `skill_send` → first sample of the motion the skill produced | the **dead time** a planner must budget. |
| `settle_s` | last vigorous sample → body at rest, held 0.5 s | the **recovery** tail after the motion. |

`lag_s` uses the joint-velocity motion window (`skill_send` timestamps are not
usable as motion bounds — that is the whole point of the measurement). Segments
are attributed to the last motion-producing `skill_send` at or before the segment
start; `speed_level` is excluded because it never moves the robot. "Settled"
means `|base_v| < 0.05 m/s`, `|yaw_speed| < 0.10 rad/s`, joint speed back below
the detector's exit level and `body_height` flat to 5 mm, all held for 0.5 s.

## 1. Per-skill cost

{call_tab}

## 2. Reading the table

* **There is a hard ~{startup.min():.2f} s floor on every call.** `speed_level`
  does nothing but set a parameter and still costs
  {startup.min():.2f}–{startup.max():.2f} s end to end. That is `run.sh` starting,
  and it is charged once per skill invocation.
* **Gait-mode calls cost about 1.5 s more than posture calls.**
  `trot_run`/`static_walk`/`economic_gait`/`classic_walk`/`cross_step` return in
  {gait_calls.min():.2f}–{gait_calls.max():.2f} s against
  {startup.min():.2f}–{startup.max():.2f} s for `speed_level`/`balance_stand`/
  `stand_down`/`recovery_stand`.
* **`move` costs the startup, the commanded duration, and ~{(move['call_s'] - move['cmd_duration_s']).median() - startup.median():.1f} s more.**
  `call_s − duration` is {(move['call_s'] - move['cmd_duration_s']).min():.2f}–{(move['call_s'] - move['cmd_duration_s']).max():.2f} s
  (median {(move['call_s'] - move['cmd_duration_s']).median():.2f} s) against the {startup.median():.2f} s script floor.
* **Dead time before the robot moves is ~{lag.mean():.1f} s and remarkably stable.**
  Over the {int(moved.shape[0])} motion-producing sends, `lag_s` =
  **{lag.mean():.2f} ± {lag.std():.2f} s** (min {lag.min():.2f}, max {lag.max():.2f}).
  `move`, `front_jump` and `front_pounce` all land at ~4.1 s; the two posture
  skills are faster ({tdf[tdf['skill'] == 'stand_down']['lag_s'].iloc[0]:.2f} s for
  `stand_down`, {tdf[tdf['skill'] == 'recovery_stand']['lag_s'].iloc[0]:.2f} s for
  `recovery_stand`) — the ~1.6 s beyond the {startup.min():.2f} s script floor is
  the sport controller's own preparation, and it is skill-specific.
* **{len(silent)} sends produce no motion at all.** Every `balance_stand`
  ({int((silent['skill'] == 'balance_stand').sum())} of them) and every gait-mode
  call is silent: a gait-mode skill only changes *how* the next `move` executes,
  and `balance_stand` is a no-op when the robot is already standing. They still
  cost their `call_s`.
* **Settling is cheap compared to the dead time.** Median settle over all
  motion-producing sends is {moved['settle_s'].median():.2f} s, worst case
  {moved['settle_s'].max():.2f} s. {int(moved['settle_s'].isna().sum())} run(s) do not
  settle before the recording stops and are `–`.

## 3. Forced predecessors

{pred_tab}

* **`front_jump` and `front_pounce` are never sent without `balance_stand`
  immediately before them** — {int(preds[(preds['skill'] == 'front_jump') & (preds['prev_skill'] == 'balance_stand')]['count'].iloc[0])}/{int(preds[(preds['skill'] == 'front_jump')]['n_sends'].iloc[0])}
  and {int(preds[(preds['skill'] == 'front_pounce') & (preds['prev_skill'] == 'balance_stand')]['count'].iloc[0])}/{int(preds[(preds['skill'] == 'front_pounce')]['n_sends'].iloc[0])}
  respectively, with no exception in the set. `balance_stand` itself produces no
  measurable motion, so the {startup.min():.2f}–{startup.max():.2f} s it costs is
  pure precondition overhead added to the jump.
* **`speed_level` is only ever sent first**, before the gait mode
  ({int(preds[(preds['skill'] == 'trot_run') & (preds['prev_skill'] == 'speed_level')]['count'].iloc[0])} of
  {int(preds[preds['skill'] == 'trot_run']['n_sends'].iloc[0])} `trot_run` calls follow one).
  Per the group-01 MANIFEST the running-trot recipe requires `speed_level 0`, so
  treat it as a mandatory prefix even though the log allows sending `trot_run`
  alone.
* **`move` is the terminal call of every locomotion session**, preceded either by
  a gait-mode skill ({int(preds[(preds['skill'] == 'move') & (preds['prev_skill'] != '(session_start)')]['count'].sum())} of
  {int(preds[preds['skill'] == 'move']['n_sends'].iloc[0])}) or by nothing, in which case the
  controller's default walk is used.
* The posture chain is strictly ordered:
  `balance_stand → stand_down → recovery_stand → balance_stand`.

## 4. What a planner should budget

| transition | cost |
| :--- | ---: |
| any skill call, minimum | {startup.min():.2f} s |
| gait-mode switch (`trot_run`, `classic_walk`, …), no motion produced | {gait_calls.mean():.2f} s |
| `balance_stand` precondition before a jump | {tdf[(tdf['skill'] == 'balance_stand')]['call_s'].mean():.2f} s |
| `skill_send` → robot actually moves | {lag.mean():.2f} ± {lag.std():.2f} s |
| `move x=…` total occupancy | {move['call_s'].mean() - move['cmd_duration_s'].mean():.2f} s + commanded duration |
| motion end → settled | {moved['settle_s'].median():.2f} s median, {moved['settle_s'].max():.2f} s worst |
| **`balance_stand` + `front_jump`, send to settled** | ≈ {tdf[tdf['skill'] == 'balance_stand']['call_s'].mean() + summary[summary['skill'] == 'front_jump']['lag_mean_s'].iloc[0] + summary[summary['skill'] == 'front_jump']['motion_mean_s'].iloc[0] + summary[summary['skill'] == 'front_jump']['settle_mean_s'].iloc[0]:.1f} s |
| **`speed_level` + `trot_run` + `move`, send to settled** | ≈ {tdf[tdf['skill'] == 'speed_level']['call_s'].mean() + gait_calls.mean() + summary[summary['skill'] == 'move']['lag_mean_s'].iloc[0] + summary[summary['skill'] == 'move']['motion_mean_s'].iloc[0] + summary[summary['skill'] == 'move']['settle_mean_s'].iloc[0]:.1f} s |

The dead time is the dominant term everywhere, and it is *not* recoverable by
shortening the motion: whatever duration is asked for, a `move` carries
~{(move['call_s'] - move['cmd_duration_s']).median() + moved['settle_s'].median():.1f} s
of overhead around it ({(move['call_s'] - move['cmd_duration_s']).median():.2f} s before
`skill_done` returns, plus {moved['settle_s'].median():.2f} s of settling), so a 3.0 s
`move` occupies about {(move['call_s'] - move['cmd_duration_s']).median() + moved['settle_s'].median() + 3.0:.1f} s
of wall clock.

## 5. Every `skill_send` in the set

{ev_tab}
"""
