"""Replayable skill clips cut from the curated logs.

What a clip is
--------------
A clip is the *commanded* low-level trajectory for one skill, in a form Isaac Lab
can play back on a flat floor:

    q_des, dq_des, kp, kd, tau_ff   -- (n, 12), the four channels of the Unitree
                                       low-level command, plus the desired velocity
    q, dq                           -- (n, 12), what the real joints actually did
    contact                         -- (n, 4),  per-foot stance, re-derived per leg
                                       (never the logged fixed-20 N ``contact_*``)

Two kinds
---------
``cyclic``   WALK / TROT / RUN.  One gait cycle, cut on *contact events* (the
             reference foot's touchdown), phase-averaged over every clean cycle in
             the session and stored on a phase grid so the last sample wraps onto
             the first.  Playing it on repeat is a valid gait.
``oneshot``  JUMP.  The whole event, crouch -> take-off -> flight -> touchdown ->
             settle, with no cutting or averaging.  Playing it on repeat is not.

Sample rates
------------
Both rates are written for every clip:

``hi``  the session's own rate (nominally ~419 Hz; the real per-session rate is
        407-443 Hz and is stored per clip, never assumed).
``lo``  50 Hz, for a sim stepping at 50 Hz.

The 50 Hz version is **anti-aliased before decimation**, never plain-decimated:

* cyclic clips are periodic, so the exact anti-alias filter is truncation of the
  Fourier series at the target Nyquist.  That is zero-phase, has no edge
  transient, and leaves the result exactly periodic -- a windowed FIR would ring
  at the loop seam.
* one-shot clips get a 4th-order zero-phase Butterworth (``filtfilt``) at
  ``LOWPASS_FRAC`` x the target rate, then linear resampling.
* ``kp``/``kd`` are **not** filtered.  They are piecewise-constant gain *levels*
  commanded by the sport controller (see the gain note below); low-passing them
  would invent gains that were never sent.  They are resampled nearest-neighbour.
  ``contact`` likewise.

Each clip records ``alias_energy_frac``: the fraction of ``q_des`` variance that
sat above the 50 Hz Nyquist and was removed.  If that number is small the filter
did not matter; if it is not, plain decimation would have folded it into the
gait band.

Leg order
---------
Every log column is in the firmware's native ``FR, FL, RR, RL``.  Clips are
written in ``FL, FR, RL, RR`` -- the order ``meta.json`` records as
``user_requested_leg_order`` and the order Isaac Lab's Go2 asset uses.  The
permutation is applied once, here, and the clip carries the target order as a
string so a consumer cannot silently assume the wrong one.

PosStopF
--------
``q_des`` carries the 2.146e9 ``PosStopF`` sentinel whenever the joint is not
position-controlled.  It is <=0.05% of samples on these sessions, but a single
2e9 sample would destroy any filter it touched.  Sentinel and NaN samples are
masked out, filled by linear interpolation along time (nearest at the ends), and
the boolean ``q_des_valid`` mask is stored alongside.  **Filled samples are not
commanded values.**

Gains are not constant -- read this before replaying
-----------------------------------------------------
The sport controller schedules ``kp``/``kd`` per skill *and within a skill*, and
the schedule is not a detail:

    slow walk, turn, lateral, default trot   kp = 40 / 40 / 40   kd = 1   tau_ff = 0
    running trot (``trot_run``)              kp = 13 /  3 /  2   kd ~ 2   tau_ff RMS ~11 Nm (calf)
    economic / static walk                   kp = 3.5/  3 /  2   kd ~ 1   tau_ff RMS ~7 Nm
    front_jump                               kp = 40-70          kd = 1-7 tau_ff RMS ~10-14 Nm

(hip / thigh / calf, medians inside the motion window.)

So for the walking skills ``q_des`` really is the command and a position replay
is faithful.  For the running trot it is not: with a calf gain of 2 Nm/rad the
leg is nearly free and the gait is carried by ``tau_ff``.  Replaying RUN as
position targets under the fork's Go2 gains (kp 40 / kd 1) will not reproduce it.
That is why ``kp``, ``kd`` and ``tau_ff`` are stored as full time series rather
than as per-clip constants, and why they must be applied.

Sign and zero conventions are UNVERIFIED
----------------------------------------
Nothing here has been checked against the Isaac Lab Go2 asset.  The joint sign
convention, the zero offsets and the hip abduction direction are taken from the
log as-is.  ``scripts/verify_skill_replay.py`` exists precisely because this is
an open question; until it has been run on a machine with Isaac Lab, treat every
clip as "the right numbers, possibly in the wrong frame".
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from scipy.signal import butter, filtfilt

from .session import JOINTS, LEGS, POS_STOP_F, Session
from .window import detect_motion
from .contact import detect_contact, touchdown_times

# --------------------------------------------------------------------------- #
# Conventions
# --------------------------------------------------------------------------- #

#: Leg order the clips are written in (``meta.json: user_requested_leg_order``).
TARGET_LEGS: List[str] = ["FL", "FR", "RL", "RR"]

#: ``TARGET_LEGS[i] == LEGS[LEG_PERM[i]]``.
LEG_PERM: np.ndarray = np.asarray([LEGS.index(l) for l in TARGET_LEGS], dtype=int)

#: Joint permutation induced by ``LEG_PERM`` on a (n, 12) leg-major matrix.
JOINT_PERM: np.ndarray = np.concatenate([np.arange(3) + 3 * l for l in LEG_PERM])

#: Target rate of the decimated copy, Hz.
TARGET_FS = 50.0

#: Anti-alias cutoff as a fraction of the *target* rate (0.4 x 50 Hz = 20 Hz,
#: i.e. 0.8 x the target Nyquist).
LOWPASS_FRAC = 0.4

#: Anything above this in ``q_des`` is the PosStopF sentinel.
SENTINEL_LEVEL = 1e9

#: Channels resampled with linear interpolation / filtered.
SMOOTH_CHANNELS = ("q_des", "dq_des", "tau_ff", "q", "dq", "tau")
#: Channels held piecewise-constant (gain levels, booleans).
STEP_CHANNELS = ("kp", "kd", "contact", "q_des_valid")

#: A cycle is kept if its duration is within this fraction of the median.
CYCLE_TOL = 0.20
#: Cycles at the very start / end of a segment carry the accel and decel
#: transient and are dropped.
CYCLE_EDGE_DROP = 1
#: Below this many clean cycles the phase average is not trustworthy.
MIN_CYCLES = 3

#: Seconds of quiet held either side of a one-shot event.
ONESHOT_PAD_S = 0.25


# --------------------------------------------------------------------------- #
# Clip selection: which session represents which skill
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class ClipSpec:
    """How to find the session that represents one skill.

    Sessions are *selected by measurement*, never named: the duty band and the
    flight requirement come from ``outputs/skill_profile.md`` (duty is the
    separating axis -- 0.31 running trot / 0.52 trot / 0.64 slow walk, and flight
    only exists below duty 0.40), and among the sessions that qualify the one
    with the steadiest cycle wins.  Hard-coding a session name would hide the
    fact that the choice is a judgement.
    """

    name: str
    kind: str                      # "cyclic" | "oneshot"
    duty_lo: float = 0.0
    duty_hi: float = 1.0
    motion_type: Optional[str] = "forward"
    primary_skill: Optional[str] = None
    require_flight: bool = False
    forbid_flight: bool = False
    note: str = ""


#: WALK / TROT / RUN mirror ``planner.skills.SkillId``; JUMP is ``front_jump``.
CLIP_SPECS: List[ClipSpec] = [
    ClipSpec("WALK", "cyclic", duty_lo=0.60, duty_hi=1.00, forbid_flight=True,
             note="slow walk, duty ~0.64, no flight phase"),
    ClipSpec("TROT", "cyclic", duty_lo=0.45, duty_hi=0.60, forbid_flight=True,
             note="trot, duty ~0.52, no flight phase"),
    ClipSpec("RUN", "cyclic", duty_lo=0.00, duty_hi=0.40, require_flight=True,
             note="running trot, duty ~0.31, flight phase present"),
    ClipSpec("JUMP", "oneshot", motion_type=None, primary_skill="front_jump",
             note="front_jump: vertical hop, ~0.45 s flight, ~26 mm horizontal"),
]


def select_sessions(df, specs: Sequence[ClipSpec] = CLIP_SPECS) -> Dict[str, dict]:
    """``{clip name: {session, why, candidates}}`` from a ``profile_all`` frame.

    Cyclic skills are ranked by ``stride_cv`` (lowest first): the clip is going
    to be looped, so cycle-to-cycle regularity is the property that matters, not
    speed.  ``front_jump`` is ranked by distance from the cohort's median flight
    time, i.e. the most typical jump rather than the best one.
    """
    out: Dict[str, dict] = {}
    for spec in specs:
        m = np.ones(len(df), dtype=bool)
        if spec.motion_type is not None:
            m &= (df["motion_type"] == spec.motion_type).to_numpy()
        if spec.primary_skill is not None:
            m &= (df["primary_skill"] == spec.primary_skill).to_numpy()
        duty = df["duty_mean"].to_numpy(dtype=float)
        m &= np.isfinite(duty) & (duty >= spec.duty_lo) & (duty < spec.duty_hi)
        flight = np.nan_to_num(df["flight_frac"].to_numpy(dtype=float))
        if spec.require_flight:
            m &= flight > 0.10
        if spec.forbid_flight:
            m &= flight < 0.02
        cand = df[m]
        if len(cand) == 0:
            out[spec.name] = {"session": None, "why": "no session matched", "candidates": []}
            continue
        if spec.kind == "cyclic":
            cand = cand[np.isfinite(cand["stride_cv"].to_numpy(dtype=float))]
            if len(cand) == 0:
                out[spec.name] = {"session": None, "why": "no periodic candidate", "candidates": []}
                continue
            order = np.argsort(cand["stride_cv"].to_numpy(dtype=float))
            why = "lowest stride_cv (steadiest cycle) among %d candidates" % len(cand)
        else:
            fl = cand["flight_frac"].to_numpy(dtype=float)
            order = np.argsort(np.abs(fl - np.nanmedian(fl)))
            why = "flight fraction closest to the cohort median of %d" % len(cand)
        picked = cand.iloc[int(order[0])]
        out[spec.name] = {
            "session": str(picked["session"]),
            "group": str(picked["group"]),
            "why": why,
            "duty_mean": float(picked["duty_mean"]),
            "stride_hz": float(picked["stride_hz"]),
            "vx_steady_mean": float(picked["vx_steady_mean"]),
            "flight_frac": float(picked["flight_frac"]),
            "candidates": [str(s) for s in cand["session"]],
        }
    return out


# --------------------------------------------------------------------------- #
# Channel preparation
# --------------------------------------------------------------------------- #

def _fill_invalid(x: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Column-wise linear fill of ``~valid`` samples; nearest at the ends."""
    out = np.array(x, dtype=float, copy=True)
    idx = np.arange(x.shape[0], dtype=float)
    for j in range(x.shape[1]):
        v = valid[:, j]
        if v.all():
            continue
        if not v.any():
            out[:, j] = 0.0
            continue
        out[:, j] = np.interp(idx, idx[v], x[v, j])
    return out


def raw_channels(sess: Session) -> Tuple[Dict[str, np.ndarray], np.ndarray]:
    """Every clip channel for a whole session, native leg order.

    Returns ``(channels, q_des_valid)``.  ``q_des`` comes back sentinel-free and
    NaN-free; the mask says which samples were real.
    """
    q_des = sess.joint_matrix("q_des")
    valid = np.isfinite(q_des) & (np.abs(q_des) < SENTINEL_LEVEL)
    ch: Dict[str, np.ndarray] = {"q_des": _fill_invalid(q_des, valid)}
    for name in ("dq_des", "tau_ff", "q", "dq", "tau", "kp", "kd"):
        raw = sess.joint_matrix(name)
        ok = np.isfinite(raw) & (np.abs(raw) < SENTINEL_LEVEL)
        ch[name] = _fill_invalid(raw, ok)
    return ch, valid


def _alias_energy_frac(x: np.ndarray, fs: float, nyq: float) -> float:
    """Share of the signal's variance sitting above ``nyq``, pooled over joints.

    This is what plain decimation would have folded back into the gait band.
    """
    y = x - x.mean(axis=0, keepdims=True)
    spec = np.abs(np.fft.rfft(y, axis=0)) ** 2
    f = np.fft.rfftfreq(y.shape[0], d=1.0 / fs)
    tot = spec.sum()
    return float(spec[f > nyq].sum() / tot) if tot > 0 else 0.0


# --------------------------------------------------------------------------- #
# Cyclic clips: cut on contact, phase-average, keep it loopable
# --------------------------------------------------------------------------- #

def _cycle_bounds(sess: Session, ref_leg: int = 0) -> Tuple[List[Tuple[float, float]], dict]:
    """Clean single-cycle time intervals from the reference foot's touchdowns."""
    win = detect_motion(sess)
    if not win.ok:
        raise ValueError("no motion window")
    cr = detect_contact(sess.foot_force(), win.mask, sess.fs)
    seg = win.longest()
    td = touchdown_times(cr.contact[:, ref_leg], sess.t, [seg])
    if td.size < MIN_CYCLES + 1:
        raise ValueError("only %d touchdowns on the reference leg" % td.size)
    pairs = list(zip(td[:-1], td[1:]))
    if len(pairs) > 2 * CYCLE_EDGE_DROP + MIN_CYCLES:
        pairs = pairs[CYCLE_EDGE_DROP: len(pairs) - CYCLE_EDGE_DROP]
    dur = np.asarray([b - a for a, b in pairs])
    med = float(np.median(dur))
    keep = np.abs(dur - med) <= CYCLE_TOL * med
    kept = [p for p, k in zip(pairs, keep) if k]
    if len(kept) < MIN_CYCLES:
        raise ValueError("only %d cycles within +/-%d%% of the median" % (len(kept), int(100 * CYCLE_TOL)))
    info = {
        "ref_leg": LEGS[ref_leg],
        "n_touchdowns": int(td.size),
        "n_cycles_seen": int(len(dur)),
        "n_cycles_kept": int(len(kept)),
        "cycle_s": float(np.median([b - a for a, b in kept])),
        "cycle_s_spread": float(np.std([b - a for a, b in kept])),
        "contact": cr.contact,
        "window": win,
    }
    return kept, info


def _phase_average(
    sess: Session,
    channels: Dict[str, np.ndarray],
    valid: np.ndarray,
    contact: np.ndarray,
    cycles: List[Tuple[float, float]],
    n_phase: int,
) -> Tuple[Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    """Median over cycles on a shared phase grid.

    Median, not mean: one mis-detected touchdown shifts a whole cycle and a mean
    would smear the whole clip.  The median also preserves the discrete
    ``kp``/``kd`` levels instead of inventing gains between them.
    """
    t = sess.t
    phase = np.arange(n_phase, dtype=float) / n_phase        # [0, 1), no duplicate endpoint
    stacks: Dict[str, List[np.ndarray]] = {k: [] for k in list(channels) + ["contact", "q_des_valid"]}
    for a, b in cycles:
        grid = a + phase * (b - a)
        for name, arr in channels.items():
            kind = "step" if name in STEP_CHANNELS else "smooth"
            stacks[name].append(_resample_at(t, arr, grid, kind))
        stacks["contact"].append(_resample_at(t, contact.astype(float), grid, "step"))
        stacks["q_des_valid"].append(_resample_at(t, valid.astype(float), grid, "step"))
    out, spread = {}, {}
    for name, lst in stacks.items():
        cube = np.stack(lst, axis=0)
        out[name] = np.median(cube, axis=0)
        spread[name] = cube.std(axis=0)
    out["contact"] = out["contact"] >= 0.5
    out["q_des_valid"] = out["q_des_valid"] >= 0.5
    return out, spread


def _resample_at(t: np.ndarray, arr: np.ndarray, grid: np.ndarray, kind: str) -> np.ndarray:
    """``arr`` sampled at ``grid``: linear for smooth channels, nearest for steps."""
    if kind == "step":
        idx = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 1)
        return arr[idx]
    return np.stack([np.interp(grid, t, arr[:, j]) for j in range(arr.shape[1])], axis=1)


# --------------------------------------------------------------------------- #
# Anti-aliased decimation
# --------------------------------------------------------------------------- #

def periodic_resample(x: np.ndarray, n_out: int, cutoff_bin: int) -> np.ndarray:
    """Fourier-truncate a periodic signal and evaluate it on ``n_out`` points.

    The signal is one gait cycle, so its exact band-limit is a harmonic
    truncation.  Harmonics above ``cutoff_bin`` are zeroed (that IS the
    anti-alias filter, zero-phase and with no seam transient), then the series is
    evaluated on the shorter grid.
    """
    n_in = x.shape[0]
    F = np.fft.rfft(x, axis=0)
    keep = min(cutoff_bin, n_out // 2)
    F[keep + 1:] = 0.0
    G = np.zeros((n_out // 2 + 1,) + x.shape[1:], dtype=complex)
    G[: keep + 1] = F[: keep + 1]
    return np.fft.irfft(G, n=n_out, axis=0) * (n_out / n_in)


def lowpass_resample(t: np.ndarray, x: np.ndarray, fs_in: float, grid: np.ndarray, cutoff_hz: float) -> np.ndarray:
    """Zero-phase Butterworth then linear resampling, for non-periodic clips."""
    wn = min(cutoff_hz / (fs_in / 2.0), 0.99)
    b, a = butter(4, wn)
    pad = min(3 * max(len(a), len(b)), x.shape[0] - 1)
    y = filtfilt(b, a, x, axis=0, padlen=pad)
    return np.stack([np.interp(grid, t, y[:, j]) for j in range(y.shape[1])], axis=1)


# --------------------------------------------------------------------------- #
# Clip assembly
# --------------------------------------------------------------------------- #

@dataclass
class Clip:
    name: str
    kind: str
    session: str
    group: str
    fs_hi: float
    fs_lo: float
    hi: Dict[str, np.ndarray]
    lo: Dict[str, np.ndarray]
    meta: dict = field(default_factory=dict)


def _permute(d: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    """Native FR,FL,RR,RL -> FL,FR,RL,RR on every stored channel."""
    out = {}
    for k, v in d.items():
        if v.ndim == 2 and v.shape[1] == 12:
            out[k] = v[:, JOINT_PERM]
        elif v.ndim == 2 and v.shape[1] == 4:
            out[k] = v[:, LEG_PERM]
        else:
            out[k] = v
    return out


def _loop_seam(q: np.ndarray) -> Dict[str, float]:
    """How visible the wrap-around jump is.

    Normalised against the *largest* in-clip sample-to-sample step, not the
    median: ``q_des`` is a zero-order-held command stream, so most consecutive
    samples are identical and the median step is 0.  A ratio at or below 1 means
    the seam is no faster than a transition the clip already contains.
    """
    step_max = float(np.abs(np.diff(q, axis=0)).max())
    seam = float(np.abs(q[0] - q[-1]).max())
    p2p = float((q.max(axis=0) - q.min(axis=0)).max())
    return {
        "loop_seam_rad": seam,
        "loop_seam_over_max_step": seam / step_max if step_max > 0 else np.nan,
        "loop_seam_over_p2p": seam / p2p if p2p > 0 else np.nan,
    }


def _update_rate_hz(x: np.ndarray, fs: float) -> float:
    """Rate at which a zero-order-held command stream actually changes.

    The log samples at ~419 Hz, but the sport controller does not necessarily
    write a new ``q_des`` that often -- on the slow-walk sessions it writes at
    ~44 Hz and the log holds the value in between.  A clip whose command rate is
    already near the 50 Hz target has almost nothing left to decimate; one at
    ~277 Hz does.
    """
    changed = (np.abs(np.diff(x, axis=0)) > 1e-9).any(axis=1)
    return float(changed.mean() * fs)


def build_cyclic_clip(sess: Session, spec: ClipSpec, *, cycle_subset=None) -> Clip:
    """One cycle of ``sess``, cut on contact and phase-averaged.

    ``cycle_subset`` selects which of the clean cycles feed the median, by index
    into the kept list.  ``None`` (the default) uses all of them and is what the
    frozen archive is built from -- this argument must never change that path's
    output.  A single-element subset makes the median a no-op, which is how a raw
    unaveraged cycle is obtained: same contact detection, same phase grid, same
    low-pass, same seam and alias bookkeeping, so an A/B against the default
    differs in the averaging and in nothing else.
    """
    cycles, info = _cycle_bounds(sess)
    sel = cycles if cycle_subset is None else [cycles[i] for i in cycle_subset]
    if not sel:
        raise ValueError("cycle_subset selected no cycles")
    channels, valid = raw_channels(sess)
    fs = sess.fs
    # The cycle's OWN duration, not the session median: a raw cycle that is 6%
    # long is 6% long, and normalising it to the median would be a second kind of
    # averaging.  With sel == cycles this is exactly info["cycle_s"].
    cycle_s = float(np.median([b - a for a, b in sel]))
    n_hi = max(int(round(cycle_s * fs)), 8)
    hi, spread = _phase_average(sess, channels, valid, info["contact"], sel, n_hi)

    nyq_lo = TARGET_FS / 2.0
    alias = _alias_energy_frac(hi["q_des"], n_hi / cycle_s, nyq_lo)

    n_lo = max(int(round(cycle_s * TARGET_FS)), 4)
    cutoff_bin = int(np.floor(LOWPASS_FRAC * TARGET_FS * cycle_s))
    lo: Dict[str, np.ndarray] = {}
    for name, arr in hi.items():
        if name in STEP_CHANNELS:
            idx = np.clip(np.round(np.arange(n_lo) * (n_hi / n_lo)).astype(int), 0, n_hi - 1)
            lo[name] = arr[idx]
        else:
            lo[name] = periodic_resample(arr, n_lo, cutoff_bin)

    t_hi = np.arange(n_hi) * (cycle_s / n_hi)
    t_lo = np.arange(n_lo) * (cycle_s / n_lo)
    hi["t"], lo["t"] = t_hi, t_lo

    duty = float(np.mean(hi["contact"]))
    meta = {
        "cycle_s": cycle_s,
        "cycle_hz": 1.0 / cycle_s,
        "cycle_s_spread": float(np.std([b - a for a, b in sel])),
        "n_cycles_kept": info["n_cycles_kept"],
        "n_cycles_seen": info["n_cycles_seen"],
        "n_cycles_used": int(len(sel)),
        "cycle_indices": list(range(len(cycles))) if cycle_subset is None else list(cycle_subset),
        "averaged": cycle_subset is None or len(sel) > 1,
        "ref_leg": info["ref_leg"],
        "duty_clip": duty,
        **_loop_seam(hi["q_des"]),
        "q_des_update_hz": _update_rate_hz(channels["q_des"], fs),
        "alias_energy_frac_above_25hz": alias,
        "lowpass_cutoff_hz": LOWPASS_FRAC * TARGET_FS,
        "lowpass_method": "Fourier harmonic truncation (periodic, zero-phase)",
        "q_des_spread_rad_max": float(spread["q_des"].max()),
        "q_des_invalid_frac": float(1.0 - hi["q_des_valid"].mean()),
        "note": spec.note,
    }
    return Clip(spec.name, spec.kind, sess.path.name, sess.group, n_hi / cycle_s, n_lo / cycle_s,
                _permute(hi), _permute(lo), meta)


def build_oneshot_clip(sess: Session, spec: ClipSpec) -> Clip:
    """Whole event, uncut: crouch -> take-off -> flight -> landing -> settle."""
    win = detect_motion(sess)
    if not win.ok:
        raise ValueError("no motion window")
    cr = detect_contact(sess.foot_force(), win.mask, sess.fs)
    t, fs = sess.t, sess.fs
    s0, s1 = win.start_stop()
    a = int(max(0, s0 - round(ONESHOT_PAD_S * fs)))
    b = int(min(sess.n, s1 + round(ONESHOT_PAD_S * fs)))

    channels, valid = raw_channels(sess)
    hi = {k: v[a:b] for k, v in channels.items()}
    hi["contact"] = cr.contact[a:b]
    hi["q_des_valid"] = valid[a:b]
    t_seg = t[a:b] - t[a]
    hi["t"] = t_seg

    nyq_lo = TARGET_FS / 2.0
    alias = _alias_energy_frac(hi["q_des"], fs, nyq_lo)

    dur = float(t_seg[-1])
    n_lo = max(int(round(dur * TARGET_FS)) + 1, 4)
    grid = np.linspace(0.0, dur, n_lo)
    cutoff = LOWPASS_FRAC * TARGET_FS
    lo: Dict[str, np.ndarray] = {}
    for name, arr in hi.items():
        if name == "t":
            continue
        if name in STEP_CHANNELS:
            idx = np.clip(np.searchsorted(t_seg, grid, side="right") - 1, 0, len(t_seg) - 1)
            lo[name] = arr[idx]
        else:
            lo[name] = lowpass_resample(t_seg, arr, fs, grid, cutoff)
    lo["t"] = grid

    airborne = (~hi["contact"]).all(axis=1)
    runs = np.flatnonzero(np.diff(airborne.astype(int)) != 0)
    meta = {
        "duration_s": dur,
        "pad_s": ONESHOT_PAD_S,
        "q_des_update_hz": _update_rate_hz(hi["q_des"], fs),
        "flight_frac": float(airborne.mean()),
        "flight_s": float(airborne.sum() / fs),
        "n_airborne_runs": int(len(runs) // 2 + (1 if airborne[0] else 0)),
        "alias_energy_frac_above_25hz": alias,
        "lowpass_cutoff_hz": cutoff,
        "lowpass_method": "4th-order zero-phase Butterworth (filtfilt) then linear resample",
        "q_des_invalid_frac": float(1.0 - hi["q_des_valid"].mean()),
        "loopable": False,
        "note": spec.note,
    }
    return Clip(spec.name, spec.kind, sess.path.name, sess.group, fs, TARGET_FS,
                _permute(hi), _permute(lo), meta)


def build_full_session_clip(sess: Session, spec: ClipSpec) -> Clip:
    """The recording end to end: every sample, from the first to the last.

    The other two builders both remove something.  ``build_cyclic_clip`` cuts on
    touchdown and takes a median over cycles, so the seam, the phase grid and the
    averaging are all in play; ``build_oneshot_clip`` trims to the detected motion
    window (+``ONESHOT_PAD_S``), so the start-up is gone and the first frame is a
    moving pose.  This one cuts nothing, averages nothing and loops nothing: the
    start-up the session begins with (``balance_stand`` -> the skill spinning up
    -> steady gait) is played as it was recorded, and the replay's first frame is
    the standing pose the robot was actually in.

    That is the point of it.  A replay of a cut clip can always be blamed on the
    cut -- the seam, the median, the chosen start phase.  None of those exist
    here, so a collapse cannot be attributed to any of them.

    The only thing done to the samples is the downsample, and only in the ``lo``
    copy: ``hi`` is the recording at the session's own rate.  The ``lo`` copy uses
    the same anti-aliasing as ``build_oneshot_clip`` (zero-phase Butterworth, then
    linear resampling) because plain decimation folds the out-of-band energy back
    into the gait band.  ``kp``/``kd``/``contact``/``q_des_valid`` are held, never
    filtered, for the reason in the module docstring: a filtered gain is a gain
    that was never commanded.

    Contact thresholds are still derived inside the motion window (they are
    per-session, per-leg, and calibrating them on the standing start-up would put
    the p5-p95 span on the wrong range), but the channel is kept for the whole
    session.  Nothing about the window is used to cut.
    """
    win = detect_motion(sess)
    if not win.ok:
        raise ValueError("no motion window")
    cr = detect_contact(sess.foot_force(), win.mask, sess.fs)
    t, fs = sess.t, sess.fs

    channels, valid = raw_channels(sess)
    hi = dict(channels)
    hi["contact"] = cr.contact
    hi["q_des_valid"] = valid
    t_seg = t - t[0]
    hi["t"] = t_seg
    for k, v in hi.items():
        if len(v) != sess.n:
            raise ValueError(f"channel {k} is {len(v)} samples, session is {sess.n}")

    nyq_lo = TARGET_FS / 2.0
    alias = _alias_energy_frac(hi["q_des"], fs, nyq_lo)

    dur = float(t_seg[-1])
    n_lo = max(int(round(dur * TARGET_FS)) + 1, 4)
    grid = np.linspace(0.0, dur, n_lo)
    cutoff = LOWPASS_FRAC * TARGET_FS
    lo: Dict[str, np.ndarray] = {}
    for name, arr in hi.items():
        if name == "t":
            continue
        if name in STEP_CHANNELS:
            idx = np.clip(np.searchsorted(t_seg, grid, side="right") - 1, 0, len(t_seg) - 1)
            lo[name] = arr[idx]
        else:
            lo[name] = lowpass_resample(t_seg, arr, fs, grid, cutoff)
    lo["t"] = grid

    s0, s1 = win.start_stop()
    airborne = (~hi["contact"]).all(axis=1)
    meta = {
        "duration_s": dur,
        "n_samples_session": int(sess.n),
        "cut": "none -- whole session",
        "startup_s": float(t_seg[s0]),
        "motion_window_s": [float(t_seg[s0]), float(t_seg[min(s1, sess.n - 1)])],
        "q_des_update_hz": _update_rate_hz(hi["q_des"], fs),
        "flight_frac": float(airborne.mean()),
        "flight_s": float(airborne.sum() / fs),
        "alias_energy_frac_above_25hz": alias,
        "lowpass_cutoff_hz": cutoff,
        "lowpass_method": "4th-order zero-phase Butterworth (filtfilt) then linear resample",
        "q_des_invalid_frac": float(1.0 - hi["q_des_valid"].mean()),
        "loopable": False,
        "skill_sequence": sess.skill_sequence(),
        "note": spec.note,
    }
    return Clip(spec.name, "oneshot", sess.path.name, sess.group, fs, TARGET_FS,
                _permute(hi), _permute(lo), meta)


def build_clip(sess: Session, spec: ClipSpec, *, cycle_subset=None) -> Clip:
    if spec.kind != "cyclic":
        if cycle_subset is not None:
            raise ValueError("cycle_subset is meaningless for a one-shot clip")
        return build_oneshot_clip(sess, spec)
    return build_cyclic_clip(sess, spec, cycle_subset=cycle_subset)


def gain_summary(sess: Session) -> dict:
    """Median kp/kd and tau_ff RMS per joint type, inside the motion window.

    Reported because the sport controller's gain schedule is skill-dependent and
    a replay that ignores it will not reproduce the gait (module docstring).
    """
    win = detect_motion(sess)
    m = win.mask if win.ok else np.ones(sess.n, dtype=bool)
    ch, _ = raw_channels(sess)
    out = {}
    for name in ("kp", "kd"):
        v = ch[name][m].reshape(-1, 4, 3)
        out[name] = [float(x) for x in np.median(v, axis=(0, 1))]
    tf = ch["tau_ff"][m].reshape(-1, 4, 3)
    out["tau_ff_rms"] = [float(x) for x in np.sqrt(np.mean(tf ** 2, axis=(0, 1)))]
    out["joint_order"] = list(JOINTS)
    out["position_controlled"] = bool(min(out["kp"]) >= 20.0)
    return out
