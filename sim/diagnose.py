"""Diagnosing a bad replay: which mapping, which sign, which gain.

The clips in ``data/skill_clips.npz`` carry joint trajectories in the log's own
convention.  Nothing has checked that convention against the Isaac Lab Go2 asset
(``motion_toolkit/clips.py``, "Sign and zero conventions are UNVERIFIED"), and
the two most likely ways to get it wrong are silent:

* **Leg order.**  The log is ``FR, FL, RR, RL``; clips are written ``FL, FR, RL,
  RR``; Isaac Lab's articulation DOF order comes from the USD and on the Go2 it
  is *joint-type major* (all four hips, then all four thighs, then all four
  calves), not leg-major.  Indexing a 12-vector positionally is wrong three
  different ways.
* **Sign / zero.**  A flipped thigh or calf sign folds the knee backwards; a
  flipped hip sign splays the legs.

Guessing from a video is slow.  ``best_mapping`` instead searches the small space
of plausible mistakes and reports which one the data supports, with a margin, so
"the legs are swapped" is a measurement rather than an impression.  It runs on
recorded arrays and needs no simulator, which is why it can be tested here.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

#: Order the clips are stored in.
CLIP_LEGS: List[str] = ["FL", "FR", "RL", "RR"]
JOINTS: List[str] = ["hip", "thigh", "calf"]

#: The leg permutations worth testing, as ``name -> new order of CLIP_LEGS``.
LEG_PERMUTATIONS: Dict[str, Tuple[int, ...]] = {
    "identity": (0, 1, 2, 3),
    "left_right_swapped": (1, 0, 3, 2),      # FL<->FR and RL<->RR
    "front_rear_swapped": (2, 3, 0, 1),      # FL<->RL and FR<->RR
    "diagonal_swapped": (3, 2, 1, 0),
    "native_log_order": (1, 0, 3, 2),        # FR,FL,RR,RL read as FL,FR,RL,RR
    "joint_type_major": (0, 1, 2, 3),        # handled separately, see below
}

#: Nominal Go2 standing height, m. Only used to call a collapse a collapse.
NOMINAL_HEIGHT_M = 0.32


def joint_type_major_index(leg_order: Sequence[str] = CLIP_LEGS) -> np.ndarray:
    """Index that turns a leg-major 12-vector into joint-type-major, and back.

    Isaac Lab reports ``robot.joint_names`` in the USD's own order, which for the
    Go2 is ``FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, ...``.  A clip is
    ``FL_hip, FL_thigh, FL_calf, FR_hip, ...``.  This is the reindex between
    them -- but it is only a fallback.  **Resolve by joint name at runtime**
    (``robot.find_joints``); this function exists to show what the mistake looks
    like, not to be relied on.
    """
    return np.asarray([3 * leg + j for j in range(3) for leg in range(len(leg_order))], dtype=int)


def apply_mapping(q: np.ndarray, legs: Tuple[int, ...], signs: Tuple[int, int, int]) -> np.ndarray:
    """Re-order legs and flip joint signs on an ``(n, 12)`` leg-major matrix."""
    idx = np.concatenate([np.arange(3) + 3 * l for l in legs])
    out = q[:, idx].copy()
    out *= np.tile(np.asarray(signs, dtype=float), 4)
    return out


@dataclass
class MappingResult:
    name: str
    legs: Tuple[int, ...]
    signs: Tuple[int, int, int]
    score: float
    per_joint: np.ndarray = field(repr=False)


def _score(a: np.ndarray, b: np.ndarray) -> Tuple[float, np.ndarray]:
    """Mean per-joint correlation between two trajectories, offsets removed."""
    x = a - a.mean(axis=0, keepdims=True)
    y = b - b.mean(axis=0, keepdims=True)
    sx, sy = x.std(axis=0), y.std(axis=0)
    live = (sx > 1e-6) & (sy > 1e-6)
    per = np.full(a.shape[1], np.nan)
    per[live] = (x[:, live] * y[:, live]).mean(axis=0) / (sx[live] * sy[live])
    return (float(np.nanmean(per)) if live.any() else np.nan), per


def best_mapping(q_measured: np.ndarray, q_commanded: np.ndarray, top: int = 3) -> List[MappingResult]:
    """Rank leg permutations x per-joint-type sign flips by trajectory match.

    Both inputs are ``(n, 12)`` leg-major in ``CLIP_LEGS`` order and must be on
    the same time base.  Correlation, not RMS: it is insensitive to the zero
    offset, which is a *separate* question answered by ``offset_error``.
    """
    if q_measured.shape != q_commanded.shape:
        raise ValueError(f"shape mismatch {q_measured.shape} vs {q_commanded.shape}")
    seen: Dict[Tuple, MappingResult] = {}
    for perm_name, legs in LEG_PERMUTATIONS.items():
        if perm_name == "joint_type_major":
            continue
        for signs in itertools.product((1, -1), repeat=3):
            key = (legs, signs)
            if key in seen:
                continue
            s, per = _score(q_measured, apply_mapping(q_commanded, legs, signs))
            label = perm_name
            flips = [JOINTS[i] for i, v in enumerate(signs) if v < 0]
            if flips:
                label += " + " + "/".join(flips) + " sign flipped"
            seen[key] = MappingResult(label, legs, signs, s, per)
    ranked = sorted(seen.values(), key=lambda r: (-r.score if np.isfinite(r.score) else 1e9))
    return ranked[:top]


def offset_error(q_measured: np.ndarray, q_commanded: np.ndarray) -> np.ndarray:
    """Per-joint mean offset, rad.  A constant here is a zero-point error."""
    return np.asarray(q_measured.mean(axis=0) - q_commanded.mean(axis=0), dtype=float)


# --------------------------------------------------------------------------- #
# Gait measurement (same definitions the clips were cut with)
# --------------------------------------------------------------------------- #

def gait_from_force(force: np.ndarray, dt: float) -> dict:
    """Gait numbers from per-foot FORCE, using the rule the logs are read with.

    ``gait_from_contact`` below takes an already-thresholded boolean.  Feeding it
    ``force > contact_threshold_n`` -- a bare fixed threshold, no hysteresis, no
    minimum run -- is the mistake ``motion_toolkit/contact.py`` exists to avoid,
    and it fails in the sim for the same reason it failed on the logs: a foot that
    hovers near the threshold crosses it repeatedly inside one stance, the stance
    is chopped into pieces, and since the stride comes from the MEDIAN touchdown
    interval the median becomes the chatter interval rather than the gait period.

    Measured, on the in-place turn where a foot carries only ~45 N against the 30 N
    threshold: bare threshold 2.80 Hz against the clip's 1.12, with the FL foot
    credited with 13 touchdowns where the clip has 5.  Same run, same forces, this
    rule: 1.12 Hz.  WALK and TROT are unaffected (1.37 and 1.56 either way) because
    they load a foot far above the threshold -- which is why this went unseen.

    The per-leg thresholds are derived from the run's own force distribution, so
    nothing here is tuned per clip.
    """
    from motion_toolkit.contact import detect_contact
    f = np.asarray(force, dtype=float)
    cr = detect_contact(f, np.ones(len(f), dtype=bool), 1.0 / dt)
    out = gait_from_contact(cr.contact, dt)
    out["contact_rule"] = "schmitt+despeckle on force"
    return out


def gait_from_contact(contact: np.ndarray, dt: float) -> dict:
    """Stride Hz / duty / phase from an ``(n, 4)`` boolean stance signal.

    Deliberately the same definitions ``motion_toolkit.contact`` uses on the real
    logs -- stride frequency from the *median* touchdown interval of the
    reference foot, duty from the stance fraction -- so the sim number and the
    log number are the same quantity.
    """
    c = np.asarray(contact, dtype=bool)
    out = {"duty": float(c.mean()), "duty_per_leg": c.mean(axis=0).tolist()}
    td = np.flatnonzero(np.diff(c[:, 0].astype(np.int8)) == 1) + 1
    if td.size >= 2:
        iv = np.diff(td) * dt
        out["stride_hz"] = float(1.0 / np.median(iv))
        out["stride_cv"] = float(iv.std() / iv.mean()) if iv.mean() > 0 else np.nan
        out["n_cycles"] = int(iv.size)
    else:
        out.update(stride_hz=np.nan, stride_cv=np.nan, n_cycles=int(max(td.size - 1, 0)))
    airborne = (~c).all(axis=1)
    out["flight_frac"] = float(airborne.mean())
    return out


# --------------------------------------------------------------------------- #
# Symptom -> cause
# --------------------------------------------------------------------------- #

@dataclass
class Finding:
    severity: str        # "fail" | "warn" | "ok"
    symptom: str
    cause: str
    check: str

    def __str__(self) -> str:
        tag = {"fail": "FAIL", "warn": "WARN", "ok": "  ok"}[self.severity]
        return f"[{tag}] {self.symptom}\n        likely: {self.cause}\n        check : {self.check}"


def diagnose(measured: dict, expected: dict, mapping: Optional[List[MappingResult]] = None,
             offsets: Optional[np.ndarray] = None, tol: dict = None) -> List[Finding]:
    """Turn a replay's numbers into named, checkable causes.

    ``measured`` comes from the sim run, ``expected`` from the clip metadata.
    Every threshold is an argument; none is buried in the body.
    """
    t = {"stride_rel": 0.15, "duty_abs": 0.10, "speed_rel": 0.40, "roll_deg": 15.0,
         "pitch_deg": 20.0, "height_frac": 0.6, "lateral_m_per_s": 0.10,
         "yaw_deg_per_s": 15.0, "mapping_margin": 0.05, "offset_rad": 0.15,
         "repro_stride_rel": 0.10, "repro_speed_ratio": 3.0, "inplace_speed_mps": 0.05}
    t.update(tol or {})
    F: List[Finding] = []

    # -- did it stay up at all -------------------------------------------
    h = measured.get("base_height_mean_m", np.nan)
    if np.isfinite(h) and h < t["height_frac"] * NOMINAL_HEIGHT_M:
        F.append(Finding(
            "fail",
            f"base sits at {h:.3f} m, below {t['height_frac']:.0%} of the {NOMINAL_HEIGHT_M:.2f} m nominal — the robot is on its belly",
            "thigh or calf sign flipped (the knee folds the wrong way), or a zero offset error",
            "run --self-test, then look at the mapping ranking below; a sign-flipped mapping "
            "scoring above identity confirms it",
        ))
    if measured.get("terminated_s") is not None and np.isfinite(measured["terminated_s"]):
        F.append(Finding(
            "fail",
            f"episode ended early at {measured['terminated_s']:.2f} s",
            "the base hit the ground; almost always sign or leg-order, not gains",
            "same as above — the mapping ranking is decisive, a video is not",
        ))

    roll = abs(measured.get("roll_abs_max_deg", 0.0))
    if roll > t["roll_deg"]:
        F.append(Finding(
            "fail" if roll > 2 * t["roll_deg"] else "warn",
            f"roll reaches {roll:.1f}deg (limit {t['roll_deg']:.0f})",
            "hip (abduction) sign flipped — the legs splay outward symmetrically",
            "check the hip row of the sign search; a hip-only flip that scores best is the answer",
        ))

    # -- the mapping search ----------------------------------------------
    if mapping:
        best = mapping[0]
        margin = best.score - (mapping[1].score if len(mapping) > 1 else -1.0)
        if best.name != "identity":
            F.append(Finding(
                "fail",
                f"the commanded trajectory matches the measured one best under '{best.name}' "
                f"(r={best.score:.3f}), not under identity",
                "the clip is being written into the wrong DOF indices, or with the wrong sign",
                "resolve joint indices by NAME (robot.find_joints), never positionally: Isaac Lab's "
                "Go2 DOF order is joint-type major (all hips, all thighs, all calves), the clip is "
                "leg major (FL hip/thigh/calf, FR ...)",
            ))
        elif margin < t["mapping_margin"]:
            F.append(Finding(
                "warn",
                f"identity wins by only {margin:.3f} over '{mapping[1].name}'",
                "the motion may be too symmetric to tell the mappings apart",
                "re-run on a clip with an asymmetric phase (TROT or RUN, not a stand)",
            ))
        else:
            F.append(Finding("ok", f"leg order and signs match identity (r={best.score:.3f}, "
                                   f"margin {margin:.3f})", "—", "—"))

    if offsets is not None and np.isfinite(offsets).any():
        worst = int(np.nanargmax(np.abs(offsets)))
        if abs(offsets[worst]) > t["offset_rad"]:
            leg, jn = CLIP_LEGS[worst // 3], JOINTS[worst % 3]
            F.append(Finding(
                "warn",
                f"{leg}_{jn} sits {offsets[worst]:+.3f} rad away from its command on average",
                "joint zero-point differs between the firmware and the USD, or the gain is too "
                "low to track (see the gain comparison)",
                "a uniform offset across all four legs of one joint type is a zero-point "
                "convention difference; a single leg is a mapping error",
            ))

    # -- gait numbers ------------------------------------------------------
    ms, es = measured.get("stride_hz", np.nan), expected.get("stride_hz", np.nan)
    if np.isfinite(ms) and np.isfinite(es) and es > 0:
        rel = abs(ms - es) / es
        if rel > t["stride_rel"]:
            F.append(Finding(
                "fail" if rel > 2 * t["stride_rel"] else "warn",
                f"stride {ms:.2f} Hz vs the clip's {es:.2f} Hz ({rel:+.0%})",
                "the clip is being played at the wrong rate, or the sim step does not divide the "
                "clip rate evenly",
                "the clip's own rate is stored per clip (fs_hi/fs_lo); do not assume 419 or 50",
            ))
        else:
            F.append(Finding("ok", f"stride {ms:.2f} Hz vs {es:.2f} Hz expected", "—", "—"))

    md, ed = measured.get("duty", np.nan), expected.get("duty", np.nan)
    if np.isfinite(md) and np.isfinite(ed) and abs(md - ed) > t["duty_abs"]:
        F.append(Finding(
            "warn",
            f"duty {md:.2f} vs the clip's {ed:.2f}",
            "contact threshold differs, or the legs are not reaching the ground (gains) / are "
            "never leaving it (gains too stiff)",
            "duty is measured in sim from contact-sensor force; the log used a per-leg Schmitt "
            "trigger. A 0.05 disagreement is threshold, a 0.2 disagreement is not",
        ))

    mv, ev = measured.get("vx_mean", np.nan), expected.get("vx_mean", np.nan)
    if np.isfinite(mv) and np.isfinite(ev):
        if abs(ev) > 0.05 and abs(mv - ev) / abs(ev) > t["speed_rel"]:
            hint = ("gains: this clip was recorded with low position gains and large tau_ff "
                    "(see the gain table) — a position-only replay slips"
                    if expected.get("position_controlled") is False
                    else "ground friction, or the gait is running but not gripping")
            F.append(Finding(
                "warn",
                f"forward speed {mv:.3f} m/s vs the log's {ev:.3f} m/s",
                hint,
                "re-run with --apply-tau-ff and --gains log before blaming the trajectory",
            ))
        else:
            F.append(Finding("ok", f"forward speed {mv:.3f} m/s vs {ev:.3f} m/s", "—", "—"))

    vy = abs(measured.get("vy_mean", 0.0))
    if vy > t["lateral_m_per_s"]:
        F.append(Finding(
            "warn",
            f"drifts sideways at {vy:.3f} m/s while commanded straight",
            "left/right leg pair swapped — the gait is mirrored",
            "look for 'left_right_swapped' in the mapping ranking; if identity still wins, this "
            "is asymmetric friction or an uncentred spawn, not a mapping bug",
        ))
    # Against the LOG's own yaw rate, not against zero.  TURN is a clip whose whole
    # content is a yaw rate (-22.7 deg/s measured), and judging it against straight-line
    # intent reported the skill working correctly as a leg-indexing bug -- on the one run
    # that finally reproduced its stride.  A clip that carries no expected yaw rate keeps
    # the old test, because for the straight clips zero is the right reference.
    yaw = measured.get("yaw_rate_deg_s", 0.0)
    eyaw = expected.get("yaw_rate_deg_s", 0.0)
    eyaw = eyaw if np.isfinite(eyaw) else 0.0
    if abs(yaw - eyaw) > t["yaw_deg_per_s"]:
        turning = abs(eyaw) > t["yaw_deg_per_s"]
        F.append(Finding(
            "warn",
            (f"yaws at {yaw:+.1f} deg/s against the log's {eyaw:+.1f}"
             if turning else f"yaws at {abs(yaw):.1f} deg/s while commanded straight"),
            ("the turn is not being reproduced at the rate it was recorded at"
             if turning else
             "one diagonal pair leads the other — front/rear swap, or one leg mis-indexed"),
            "check 'front_rear_swapped' in the ranking and the per-joint correlations",
        ))
    elif abs(eyaw) > t["yaw_deg_per_s"]:
        F.append(Finding("ok", f"turn rate {yaw:+.1f} deg/s vs the log's {eyaw:+.1f}", "—", "—"))
    if np.isfinite(measured.get("vx_mean", np.nan)) and np.isfinite(ev) and ev > 0.05 and measured["vx_mean"] < -0.05:
        F.append(Finding(
            "fail",
            f"walks backward ({measured['vx_mean']:.3f} m/s) on a forward clip",
            "front/rear legs swapped, or the whole trajectory time-reversed",
            "front_rear_swapped in the ranking distinguishes the two",
        ))
    F.extend(gait_reproduced(measured, expected, t))
    return F


def gait_reproduced(measured: dict, expected: dict, tol: dict = None) -> List[Finding]:
    """Did this stretch of the run produce the gait it was commanded?

    Separated from :func:`diagnose` so that a run which plays more than one clip
    can be judged SEGMENT BY SEGMENT.  Scoring a whole sequence against the first
    clip's expected stride is how every TROT->WALK handover came out FAIL while the
    robot was walking correctly for 39 of its 40 cycles.
    """
    t = {"repro_stride_rel": 0.10, "repro_speed_ratio": 3.0, "inplace_speed_mps": 0.05}
    t.update(tol or {})
    F: List[Finding] = []
    # -- did it reproduce the GAIT, or merely stay upright? ----------------
    # A replay that fails to produce the gait cannot fall over from the gait, so
    # "terminated_s is empty" scores it as a success.  That is how --hip-sign flip
    # came to look like the answer for TROT: it survived 60 cycles at 2.62 Hz
    # against the clip's 1.56 and 0.059 m/s against the log's 0.444 -- standing,
    # not trotting.  Staying up is necessary and is not sufficient, so it is
    # asserted here as its own finding rather than left implied by the absence of
    # others.
    ms, es = measured.get("stride_hz", np.nan), expected.get("stride_hz", np.nan)
    mv, ev = measured.get("vx_mean", np.nan), expected.get("vx_mean", np.nan)
    if not (np.isfinite(es) and es > 0) or not np.isfinite(ev):
        F.append(Finding(
            "warn",
            "cannot check whether the gait was reproduced: the clip carries no expected "
            f"stride ({es}) or speed ({ev})",
            "a one-shot or whole-session clip has no cycle_hz, so the gait gate has nothing "
            "to compare against",
            "judge it on the trace, not on this verdict; a PASS here would mean 'not checked', "
            "which is exactly the reading that has to be impossible",
        ))
    else:
        s_ok = np.isfinite(ms) and abs(ms - es) / es <= t["repro_stride_rel"]
        if abs(ev) >= t["inplace_speed_mps"]:
            r = abs(mv) / abs(ev) if np.isfinite(mv) and abs(mv) > 0 else 0.0
            v_ok = bool(np.isfinite(mv) and np.sign(mv) == np.sign(ev)
                        and 1.0 / t["repro_speed_ratio"] <= r <= t["repro_speed_ratio"])
            vtxt = f"speed {mv:.3f} vs {ev:.3f} m/s"
        else:
            # An in-place clip (a turn) has no forward speed to match; requiring one
            # would pass a robot that walked away from the spot it should have
            # turned on.  The test becomes "it also did not travel".
            v_ok = bool(np.isfinite(mv) and abs(mv) <= t["inplace_speed_mps"])
            vtxt = f"speed {mv:.3f} m/s, in-place clip (expected |v| < {t['inplace_speed_mps']})"
        stxt = f"stride {ms:.2f} vs {es:.2f} Hz ({abs(ms-es)/es:+.0%})"
        if s_ok and v_ok:
            F.append(Finding("ok", f"gait reproduced: {stxt}, {vtxt}", "-", "-"))
        else:
            bad = " and ".join(x for x, ok in ((stxt, s_ok), (vtxt, v_ok)) if not ok)
            F.append(Finding(
                "fail",
                f"gait NOT reproduced: {bad}",
                "the robot is doing something other than the recorded gait -- staying upright "
                "while doing it is not evidence for the replay",
                f"stride must be within {t['repro_stride_rel']:.0%} and the speed within a "
                f"factor of {t['repro_speed_ratio']:.0f} of the log, both, before any survival "
                "time means anything",
            ))
    return F


def format_findings(findings: Sequence[Finding]) -> str:
    order = {"fail": 0, "warn": 1, "ok": 2}
    return "\n".join(str(f) for f in sorted(findings, key=lambda f: order[f.severity]))


def verdict(findings: Sequence[Finding]) -> str:
    if any(f.severity == "fail" for f in findings):
        return "FAIL"
    if any(f.severity == "warn" for f in findings):
        return "WARN"
    return "PASS"
