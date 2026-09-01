#!/usr/bin/env python3
"""Replay a frozen skill clip on flat ground in Isaac Lab and check it is the same gait.

    # on the sim machine
    python scripts/verify_skill_replay.py --clip TROT --headless
    python scripts/verify_skill_replay.py --clip RUN --mode torque --headless
    python scripts/verify_skill_replay.py --all --headless

    # anywhere, no Isaac Lab needed
    python scripts/verify_skill_replay.py --self-test     # exercise the diagnostics
    python scripts/verify_skill_replay.py --convention    # is the joint frame the same? (offline)
    python scripts/verify_skill_replay.py --headroom      # will the effort clip bind? (offline)
    python scripts/verify_skill_replay.py --explain       # gains + expectations, no sim

What it answers
---------------
The clips in ``data/skill_clips.npz`` are the *real robot's* commanded joint
trajectories. Whether they mean the same thing to Isaac Lab's Go2 is unverified:
the DOF order, the joint sign convention and the joint zeros all have to agree,
and none of them has been checked. This script plays a clip on a flat floor,
measures the gait it produces, and compares it with the gait the log recorded --
period, duty factor, forward speed -- then names the likely cause of any
mismatch instead of leaving a video to be squinted at.

Status: the Isaac Lab path in this file has **not been executed** -- this machine
has no Isaac Lab. The measurement and diagnosis logic in ``sim/diagnose.py`` has
been, via ``--self-test``, which injects known leg-order and sign errors and
requires the diagnosis to name them. Expect to fix import paths and asset names
on first run; the physics-free half is the part that was testable.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim import diagnose as D
from sim import isaac_cfg as IC
from sim.replay import (Capabilities, ReplayMode, apply_gains, assert_not_aliased,
                        default_mode_for, foot_body_ids, ground_material_cfg,
                        probe_capabilities, quat_to_rpy_deg, set_robot_friction, snap,
                        torque_headroom)
from terrain_toolkit.paths import (
    REPLAY_REPORT_MD,
    REPLAY_RESULTS_CSV,
    SKILL_CLIPS_META_JSON,
    SKILL_CLIPS_NPZ,
    SKILL_PROFILE_CSV,
)

# --------------------------------------------------------------------------- #
# Gains
# --------------------------------------------------------------------------- #

def upstream_go2() -> IC.Go2Config:
    """The Go2 config the *training code* uses, parsed from the upstream source.

    Not Isaac Lab's stock ``UNITREE_GO2_CFG``. The eurekaverse fork replaces it
    with an explicit-PD config whose gains were taken from the real Go2 parkour
    deployment, and those are the numbers a replay has to be compared against.
    """
    return IC.load()


def gain_comparison(meta: dict, clip: str, cfg: IC.Go2Config | None = None) -> str:
    """The log's commanded gains next to the ones the sim will actually use."""
    g = meta["clips"][clip]["gains"]
    cfg = cfg or upstream_go2()
    L = [f"Commanded gains \u2014 log vs eurekaverse Go2 config, clip {clip}", ""]
    L.append(f"  log source   : {meta['clips'][clip]['session']}")
    L.append(f"  sim source   : {cfg.source}")
    L.append(f"  control rate : {cfg.control_hz:.0f} Hz (sim dt {cfg.sim_dt} x decimation {cfg.decimation})")
    L.append("")
    L.append(f"  {'joint':8s} {'log kp':>8s} {'sim kp':>8s} {'log kd':>8s} {'sim kd':>8s} {'tau_ff RMS':>11s} {'effort lim':>11s}")
    names, _, limits = cfg.ordered(["FL", "FR", "RL", "RR"], g["joint_order"])
    for i, jn in enumerate(g["joint_order"]):
        grp = cfg.group_of(f"FL_{jn}_joint")
        L.append(f"  {jn:8s} {g['kp'][i]:8.1f} {(grp.stiffness if grp else float('nan')):8.1f} "
                 f"{g['kd'][i]:8.2f} {(grp.damping if grp else float('nan')):8.2f} "
                 f"{g['tau_ff_rms'][i]:10.2f}  {(grp.effort_limit if grp else float('nan')):10.2f}")
    L.append("")
    for a in cfg.actuators:
        L.append(f"  actuator[{a.name}] = {a.cls}  explicit={a.explicit}  joints={a.joint_names_expr}")
    L.append("")
    if not g["position_controlled"]:
        L.append("  ** This clip is NOT position-controlled. The sport controller held the position")
        L.append("     gains near zero and drove the gait with tau_ff, so replaying q_des at the")
        L.append("     config's kp commands a different controller. Use --mode torque (or the best")
        L.append("     mode the capability probe allows) and read a POSITION-mode mismatch as")
        L.append("     expected, not as a bug.")
    else:
        L.append("  This clip was position-controlled and its gains match the sim config, so a")
        L.append("  position replay is faithful in kind and in magnitude.")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
# Convention check \u2014 no simulator required
# --------------------------------------------------------------------------- #

def convention_report(meta: dict, names: list) -> str:
    """Compare each clip's posture with the articulation's zero-action pose."""
    import numpy as np
    cfg = upstream_go2()
    z = np.load(SKILL_CLIPS_NPZ, allow_pickle=False)
    legs = [str(x) for x in z["leg_order"]]
    joints = [str(x) for x in z["joint_order"]]
    jn, iso, _ = cfg.ordered(legs, joints)

    L = ["Joint convention: clip posture vs the zero-action pose", "",
         f"  articulation : {cfg.source}",
         f"  zero-action pose is init_state.joint_pos, i.e. what the env commands at action = 0",
         f"  clip posture is the mean commanded q_des over the clip (a one-shot clip uses its",
         f"  first 0.12 s, which is the stand before the crouch)", ""]
    L.append("  " + f"{'joint':12s}" + "".join(f"{n:>10s}" for n in ["zero-act"] + names))
    poses = {}
    for name in names:
        q = z[f"{name}__hi__q_des"]
        kind = meta["clips"][name]["kind"]
        poses[name] = q.mean(axis=0) if kind == "cyclic" else q[: max(int(0.12 * meta["clips"][name]["fs_hi_hz"]), 1)].mean(axis=0)
    for i, n in enumerate(jn):
        L.append("  " + f"{n:12s}" + f"{iso[i]:10.3f}" + "".join(f"{poses[c][i]:10.3f}" for c in names))
    L.append("")
    checks = {}
    for name in names:
        c = IC.compare_pose(poses[name], cfg, legs, joints)
        checks[name] = c
        flips = [joints[k] for k, v in enumerate(c.best_signs) if v < 0]
        L.append(f"  {name:5s} mean|delta| = {c.err_identity:.3f} rad; best sign combo "
                 f"{c.best_signs} -> {c.err_best:.3f} rad"
                 + (f"   [{'/'.join(flips)} flipped]" if flips else "   [no flip helps]"))
    L.append("")
    for k, jt in enumerate(joints):
        cross = [checks[n].per_type[jt]["sign_crossings"] for n in names]
        tot = checks[names[0]].per_type[jt]["n"]
        L.append(f"  {jt:6s} sign crossings vs the zero-action pose: "
                 + ", ".join(f"{n}={c}/{tot}" for n, c in zip(names, cross)))
    L.append("")
    suspect = [n for n, c in checks.items() if c.hip_flip_suspected]
    if len(suspect) == len(names) and names:
        L += ["  VERDICT: every clip agrees that the HIP sign is flipped relative to the",
              "  articulation. A magnitude offset would be posture; a sign CROSSING on both left",
              "  legs and both right legs, in the same direction, on all clips, is a frame",
              "  difference. Thigh and calf agree with the articulation (flipping them is worse).",
              "",
              "  This is evidence, not proof: a gait-cycle mean is not a nominal stand. Confirm it",
              "  with the mapping search on RUN or JUMP (--clip RUN), which separates the",
              "  candidates by ~0.18 where WALK and TROT separate by ~0.04.",
              "",
              "  If confirmed, negate the hip columns when writing clips into the articulation;",
              "  do NOT edit the frozen archive, which records the log's own convention."]
    else:
        L += ["  VERDICT: no consistent sign flip across the clips. Treat the convention as",
              "  matching until the mapping search says otherwise."]
    return "\n".join(L)


def headroom_report(meta: dict, name: str) -> str:
    """Did the real robot apply more torque than the sim will allow?"""
    import numpy as np
    from motion_toolkit.clips import JOINT_PERM, raw_channels
    from motion_toolkit.session import load_session
    from motion_toolkit.window import detect_motion
    from terrain_toolkit.paths import CURATED_ROOT

    cfg = upstream_go2()
    z = np.load(SKILL_CLIPS_NPZ, allow_pickle=False)
    legs = [str(x) for x in z["leg_order"]]
    joints = [str(x) for x in z["joint_order"]]
    _, _, limits = cfg.ordered(legs, joints)

    sess_dir = next(p for p in CURATED_ROOT.glob("*/" + meta["clips"][name]["session"]))
    sess = load_session(sess_dir)
    win = detect_motion(sess)
    ch, _ = raw_channels(sess)
    tau = ch["tau"][win.mask][:, JOINT_PERM]

    hs = torque_headroom(tau, limits, joints)
    L = [f"Torque headroom for {name} (logged applied torque vs the sim's static effort clip)", "",
         f"  {'joint':8s} {'peak':>9s} {'p99.9':>9s} {'limit':>9s} {'peak/limit':>11s}"]
    for h in hs:
        L.append(f"  {h.joint_type:8s} {h.peak_logged_nm:9.2f} {h.p999_logged_nm:9.2f} "
                 f"{h.limit_nm:9.2f} {h.ratio:10.2f}x" + ("   OVER" if h.over else ""))
    over = [h for h in hs if h.over]
    if over:
        L += ["", "  The real robot applied more torque than IdealPDActuator will allow. The sim",
              "  robot is weaker than the one that produced this clip, so the replay will fall",
              "  short and any limit calibrated from it is a property of the SIM robot.",
              "  That is the correct thing to calibrate \u2014 the planner runs in sim \u2014 but do not",
              "  raise effort_limit for the calibration run only: that would measure a robot the",
              "  evaluation never uses. Record the gap instead."]
    else:
        L += ["", "  Within the clip; the effort clip will not bind."]
    return "\n".join(L)


# --------------------------------------------------------------------------- #
# Clip loading
# --------------------------------------------------------------------------- #

def load_clip(name: str, rate: str = "lo", archive: Path | None = None) -> dict:
    z = np.load(archive or SKILL_CLIPS_NPZ, allow_pickle=False)
    names = [str(x) for x in z["clip_names"]]
    if name not in names:
        raise SystemExit(f"clip {name!r} not in archive; have {names}")
    d = {ch: np.asarray(z[f"{name}__{rate}__{ch}"]) for ch in
         ("t", "q_des", "dq_des", "tau_ff", "q", "dq", "kp", "kd", "contact", "q_des_valid")}
    d["contact"] = d["contact"].astype(bool)
    d["fs"] = float(z[f"{name}__fs_{rate}"])
    d["leg_order"] = [str(x) for x in z["leg_order"]]
    d["joint_order"] = [str(x) for x in z["joint_order"]]
    d["name"], d["rate"] = name, rate
    d["kind"] = str(z["clip_kinds"][names.index(name)])
    return d


# Frame-indexed channels: a phase rotation has to move all of them together, or the
# gains and the feed-forward torque stop lining up with the pose they belong to.
_PER_FRAME = ("t", "q_des", "dq_des", "tau_ff", "q", "dq", "kp", "kd", "contact", "q_des_valid")


def stance_time_s(clip: dict) -> float:
    """Mean stance duration per cycle, from the clip's own contact channel.

    A cyclic clip spans exactly one gait cycle by construction, so the fraction of
    frames a leg is loaded for, times the cycle's length, is that leg's stance
    time.  This is the ``T_stance`` the Raibert neutral point is defined against
    and it is MEASURED -- duty x period, both from the recording -- so the foot
    placement law's leading term carries no free parameter.  Cross-check for TROT:
    duty_clip 0.577 x cycle 0.643 s = 0.371 s, against 0.369 s from the meta's
    duty_mean / stride_hz.
    """
    c = np.asarray(clip["contact"], dtype=float)
    cycle_s = c.shape[0] / float(clip["fs"])
    return float(c.mean() * cycle_s)


def quiescent_start(clip: dict) -> tuple[int, dict]:
    """Pick the phase of a cyclic clip to begin the replay at.

    A cyclic clip is a loop: starting at frame 0 rather than frame k is an arbitrary
    choice made by whoever cut the recording, not a property of the motion.  The
    replay's settle drops the robot into whichever pose it starts on, so a start
    frame with a foot in swing lands the robot on three legs and it tips.  This
    picks the frame that is easiest to be dropped into -- most feet on the ground,
    and among those the slowest-moving -- which is a choice about WHERE IN THE LOOP
    to begin, not an edit to the recording.

    Returns ``(index, info)``.  ``info["all_stance"]`` is False when the clip has no
    four-foot frame at all: a running gait with a flight phase has none, and for
    those this criterion cannot deliver a quiet initial condition at any phase.
    """
    c = clip["contact"].astype(int)
    down = c.sum(1)
    speed = np.linalg.norm(clip["dq"], axis=1)
    best_down = int(down.max())
    cand = np.flatnonzero(down == best_down)
    idx = int(cand[np.argmin(speed[cand])])
    return idx, {
        "start_frame": idx,
        "feet_down": best_down,
        "all_stance": bool(best_down == 4),
        "speed_at_start": float(speed[idx]),
        "speed_at_frame0": float(speed[0]),
        "feet_down_at_frame0": int(down[0]),
        "n_all_stance_frames": int((down == 4).sum()),
    }


def level_start(clip: dict, robot, sim, idx_t, phys_dt) -> tuple[int, dict]:
    """Pick the phase whose pose the robot can actually be stood on.

    The clip's contact channel says which feet the REAL robot had on the ground.
    That does not transfer: a clip stores joint angles and no base trajectory, so
    where the feet end up in sim is decided by the joint angles alone.  Measured at
    handover, the clip's labels and the sim's geometry disagree outright -- WALK's
    frame 0 is labelled 3/4 down yet puts all four feet within 0.0 mm of a plane
    (and is the one configuration that walks), while TROT's frame 3 is labelled 4/4
    down and leaves a foot 44 mm in the air.

    So the criterion is kinematic, not labelled: hold each candidate pose and keep
    the one whose four feet are closest to coplanar.  Still a choice of WHERE IN THE
    LOOP to start -- the recording is untouched.
    """
    import torch
    foot_ids, _ = foot_body_ids(robot)
    n = clip["q_des"].shape[0]
    root = robot.data.default_root_state.clone()
    speed = np.linalg.norm(clip["dq"], axis=1)
    spreads = np.empty(n)
    for k in range(n):
        robot.write_root_state_to_sim(root)
        q = robot.data.default_joint_pos.clone()
        q[:, idx_t] = torch.as_tensor(clip["q_des"][k], device=q.device, dtype=q.dtype)
        robot.write_joint_state_to_sim(q, torch.zeros_like(q))
        robot.write_data_to_sim()
        sim.step()
        robot.update(phys_dt)
        fz = robot.data.body_pos_w[0, foot_ids, 2]
        spreads[k] = float((fz.max() - fz.min()).item())
    # Ties on a flat clip are broken by joint speed: of two equally level poses,
    # prefer the one the robot is moving through more slowly.
    best = float(spreads.min())
    cand = np.flatnonzero(spreads <= best + 1e-4)
    idx = int(cand[np.argmin(speed[cand])])
    return idx, {
        "start_frame": idx,
        "spread_at_start": float(spreads[idx]),
        "spread_at_frame0": float(spreads[0]),
        "spread_worst": float(spreads.max()),
        "feet_down": int(clip["contact"][idx].sum()),
        "all_stance": bool(clip["contact"][idx].sum() == 4),
        "n_level_frames": int((spreads <= 0.01).sum()),
    }


# --------------------------------------------------------------------------- #
# Diagnostic ablations.  These CHANGE THE CLIP and are for attribution only --
# never for a reported skill, and the frozen archive is never touched.  Their
# purpose is to answer "is the drift caused by the recording's own left-right
# asymmetry?", which cannot be answered by looking at the recording alone.
#
#   mirror     : play the gait mirrored about the sagittal plane.  If the robot
#                then falls the other way, the bias lives in the clip; if it
#                falls the same way, the bias lives in the robot or the sim.
#   symmetrize : average the clip with its own mirror half a cycle later, which
#                removes the asymmetric component and keeps the symmetric one.
#                If the collapse survives that, asymmetry was not the cause.
#
# Neither is a stabiliser: both are open-loop joint trajectories with no feedback
# on base state, exactly like the recording.
# --------------------------------------------------------------------------- #

_MIRROR_LEG = {"FL": "FR", "FR": "FL", "RL": "RR", "RR": "RL"}


# Channels that carry a signed quantity about the abduction axis, and so change
# sign under the mirror.  Gains, validity flags and contact booleans do not: a
# negated kp is not a mirrored robot, it is an unstable one.
_SIGNED = ("q_des", "dq_des", "tau_ff", "q", "dq")


def _mirror_rows(a, leg_order, per_leg: bool, signed: bool):
    """Swap left for right in a (n, 4) or (n, 12) per-leg array."""
    out = np.empty_like(a)
    w = 1 if per_leg else 3
    for i, leg in enumerate(leg_order):
        j = leg_order.index(_MIRROR_LEG[leg])
        out[:, j * w:(j + 1) * w] = a[:, i * w:(i + 1) * w]
    if not per_leg and signed:
        out[:, 0::3] *= -1.0          # abduction axis reverses under the mirror
    return out


def mirror_clip(clip: dict, mode: str) -> dict:
    """``mode`` in {none, mirror, symmetrize}.  Returns a new clip dict."""
    if mode == "none":
        return clip
    legs = list(clip["leg_order"])
    n = len(clip["q_des"])
    out = dict(clip)
    for ch in _PER_FRAME:
        if ch == "t":
            continue
        a = np.asarray(clip[ch], dtype=np.float64)
        m = _mirror_rows(a, legs, per_leg=(a.shape[1] == 4), signed=(ch in _SIGNED))
        if mode == "mirror":
            out[ch] = m.astype(clip[ch].dtype)
        else:                          # symmetrize: mirror is half a cycle away
            half = np.roll(m, -(n // 2), axis=0)
            out[ch] = (0.5 * (a + half)).astype(clip[ch].dtype)
    return out

def rotate_clip(clip: dict, k: int) -> dict:
    """Start the loop at frame k. Same samples, same order, different entry point."""
    if k % len(clip["q_des"]) == 0:
        return clip
    out = dict(clip)
    for ch in _PER_FRAME:
        out[ch] = np.roll(clip[ch], -k, axis=0)
    # t is a timebase, not a signal: re-derive it so it stays monotone after the roll.
    out["t"] = clip["t"][0] + np.arange(len(clip["t"])) / clip["fs"]
    return out


def expected_from_meta(meta: dict, clip: str) -> dict:
    c = meta["clips"][clip]
    sel = c.get("selection", {})
    return {
        "stride_hz": c.get("cycle_hz", np.nan),
        "duty": c.get("duty_clip", np.nan),
        "vx_mean": sel.get("vx_steady_mean", np.nan),
        # The yaw rate the recording actually had, in deg/s, so a turn is judged against
        # its own turn and not against zero.  nan when the profile table is absent, which
        # diagnose() reads as "no expectation" and falls back to the straight-line test.
        "yaw_rate_deg_s": float(np.degrees(_log_motion_for(meta, clip)[1])),
        "flight_frac": c.get("flight_frac", sel.get("flight_frac", np.nan)),
        "position_controlled": c["gains"]["position_controlled"],
        "kind": c["kind"],
    }


def deviation_report(foot_u: np.ndarray, q_meas: np.ndarray, q_orig: np.ndarray,
                     swing: np.ndarray, legs: list, joints: list) -> tuple:
    """How far this run departed from the recording, per joint and per gait phase.

    Two different departures, kept apart because they answer different questions:

    ``cmd``   what the compensator OVERWROTE -- the correction added to the
              commanded angle.  This is the quantity the supervisor's release is
              spent on: "how far from the original do you have to go before it
              stands up".  By construction it is nonzero only on the hip columns
              and only in swing, so the zeros in the table are a claim being
              checked, not padding.
    ``meas``  what the JOINT ACTUALLY DID against what the recording commanded.
              Nonzero everywhere even with the compensator off -- a PD tracking a
              trajectory under load always lags -- so it is reported next to the
              off run rather than on its own.

    Split by the recording's own swing/stance labels for the leg the joint belongs
    to, because a deviation while the foot is in the air and the same deviation
    while it is carrying the robot are not the same event.
    """
    n = min(len(foot_u), len(q_meas), len(q_orig), len(swing))
    u, qm, qo, sw = foot_u[:n], q_meas[:n], q_orig[:n], swing[:n].astype(bool)
    dm = qm - qo
    rows, summ = [], {}
    rms = lambda x: float(np.sqrt(np.mean(np.square(x)))) if x.size else float("nan")
    for k, leg in enumerate(legs):
        for j, jn in enumerate(joints):
            col = 3 * k + j
            for phase, mask in (("swing", sw[:, k]), ("stance", ~sw[:, k])):
                rows.append({
                    "leg": leg, "joint": jn, "phase": phase,
                    "n_steps": int(mask.sum()),
                    "cmd_rms_rad": rms(u[mask, col]),
                    "cmd_max_rad": float(np.abs(u[mask, col]).max()) if mask.any() else float("nan"),
                    "meas_rms_rad": rms(dm[mask, col]),
                    "meas_max_rad": float(np.abs(dm[mask, col]).max()) if mask.any() else float("nan"),
                })
    # Roll-ups: by joint type x phase, which is the shape the result is read in.
    for j, jn in enumerate(joints):
        cols = np.arange(3 * len(legs))[j::3]
        for phase in ("swing", "stance"):
            mask = np.stack([sw[:, k] if phase == "swing" else ~sw[:, k]
                             for k in range(len(legs))], axis=1)
            sel_u = np.concatenate([u[mask[:, k], cols[k]] for k in range(len(legs))])
            sel_m = np.concatenate([dm[mask[:, k], cols[k]] for k in range(len(legs))])
            summ[f"dev_cmd_rms_{jn}_{phase}"] = rms(sel_u)
            summ[f"dev_meas_rms_{jn}_{phase}"] = rms(sel_m)
    touched = np.abs(u) > 1e-6
    summ["dev_cmd_max_rad"] = float(np.abs(u).max())
    summ["overwrite_frac_time"] = float(touched.any(axis=1).mean())
    summ["overwrite_frac_legsteps"] = float(touched[:, 0::3].mean())
    summ["swing_frac"] = float(sw.mean())
    return rows, summ




def swing_lift_offsets(robot, sim, idx_t, clip, target_m, phys_dt, eps=0.02, sign=None,
                       symmetric=True, air_z=1.5, spread=False):
    """Per-frame thigh/calf offsets that raise each swing foot to ``target_m`` at its apex.

    The real robot's ``classic_walk`` lifts its rear feet about 5 mm and its front ones
    12-29 mm (outputs/achieved_clearance.md); Unitree's own ``footRaiseHeight`` default is
    0.08 m.  This raises the clip's swing arc to a chosen apex rather than waiting for a
    re-recording, which is the supervisor's decision, and it is applied at replay time
    only -- the archive is not touched.

    Shape.  Height is added over each swing bout as ``A * sin^2(pi * phase)``: zero at
    liftoff and at touchdown, zero SLOPE at both, peak at mid-swing.  The endpoints are
    where stride length and forward speed live, so leaving them untouched is what keeps
    this from being a speed edit.  ``A`` is per leg per bout, ``target - the apex that leg
    already has``, so a leg that already clears the target is left alone.

    Geometry from ``q``, not ``q_des``.  The commanded stream is not a pose the robot can
    hold (outputs/commanded_angles.md), so the existing apex is measured from the achieved
    angles, against the CHORD joining that bout's liftoff and touchdown feet -- which is
    the height a step cares about and is independent of trunk attitude.

    Joints.  The vertical displacement is turned into thigh and calf offsets by a
    numerical Jacobian solved per frame for (dz = lift, dx = 0), so the foot goes straight
    up and its fore-aft position is unchanged.  The HIP is not used: that is foot
    placement's and heading hold's joint, and the two must not compete for it.

    Left-right symmetry (``symmetric``, the default).  The amount ADDED is what the robot
    feels as a change; the recording walks straight with the asymmetry it already has.
    Choosing the amplitude per leg from that leg's own apex therefore adds a DIFFERENT
    amount left and right -- measured at 60 mm: FL +39.7 vs FR +28.4, RL +60.0 vs
    RR +51.4 -- and an asymmetric edit is a roll input.  It showed up as +2 -> +55 deg of
    roll and 1.5 m of sideways crab over a 3 m flat approach, at under 5 deg of yaw error
    the whole way, so heading hold never saw it (outputs/swing_lift_symmetry.md).

    So the amplitude is chosen PER MIRROR PAIR, from the pair's worst bout, and added
    equally to both legs: the added Delta z is identical left and right and the clip's own
    left-right relationship survives untouched.  Front and rear are free to differ -- that
    is a pitch-symmetric difference, not a steering one.  ``symmetric=False`` restores the
    per-leg choice, for the A/B only.
    """
    import torch
    from sim.replay import quat_rotate_inv, snap

    foot_ids, foot_names = robot.find_bodies(".*_foot")
    fcol = {n.split("_")[0]: i for i, n in enumerate(foot_names)}
    legs = clip["leg_order"]
    q_ach = np.asarray(clip["q"], dtype=np.float32)
    if sign is not None:
        q_ach = q_ach * sign
    contact = np.asarray(clip["contact"], dtype=bool)
    n = len(q_ach)
    air = robot.data.default_root_state.clone()
    # FREE-FLOATING, and it has to actually be free.  CLAUDE.md 6.5: the way to measure a
    # clip's own geometry is to hold the robot in the air and pass q through it, so neither
    # contact nor ground can intervene.  1.5 m does that on the flat rig and does NOT on the
    # benchmark grid, where staircase_climbing reaches 3.96 m -- there the robot is measured
    # INSIDE the terrain, PhysX floods "Reached maximum number of allocated blocks" (271 MB
    # of it in one run), and the apex being read is a collision response.  Callers with
    # terrain pass a height above it; the default is unchanged so the flat harness is
    # bit-identical.
    air[:, 2] = float(air_z)
    # KEEP EACH ROBOT WHERE IT IS IN X-Y (spread=True).  `default_root_state` carries the
    # articulation's configured origin, which is the SAME point for every instance -- so on
    # a fleet this stacks all of them at one place and they measure each other.  With 200
    # robots that is 200 interpenetrating bodies: PhysX runs out of contact blocks and the
    # measurement stops making progress (one run sat at 121% CPU emitting nothing for
    # twenty minutes).  Invisible on a one-robot rig, which is every caller that predates
    # this, so the default is off and they stay bit-identical.
    if spread:
        air[:, :2] = robot.data.root_pos_w[:, :2]

    def feet_at(qv):
        qj = robot.data.default_joint_pos.clone()
        qj[:, idx_t] = torch.as_tensor(np.asarray(qv, np.float32), device=robot.device,
                                       dtype=torch.float32)
        robot.write_root_state_to_sim(air)
        robot.write_joint_state_to_sim(qj, torch.zeros_like(qj))
        robot.write_data_to_sim(); sim.step(); robot.update(phys_dt)
        bq = snap(robot.data.root_quat_w[0]); bpos = snap(robot.data.root_pos_w[0])
        bp = snap(robot.data.body_pos_w[0])
        return np.array([quat_rotate_inv(bq[None, :], (bp[foot_ids[fcol[l]]] - bpos)[None, :])[0]
                         for l in legs])

    fpos = np.stack([feet_at(q_ach[i]) for i in range(n)])       # (n, 4, 3)

    def bouts(mask):
        d = np.diff(np.concatenate([[False], mask, [False]]).astype(np.int8))
        return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))

    lift = np.zeros((n, 4))
    report = {}

    # --- pass 1: every leg's swing bouts and the apex each one already has
    leg_bouts, leg_apex = {}, {}
    for j, leg in enumerate(legs):
        bl, ap = [], []
        for a, b in bouts(~contact[:, j]):
            if b - a < 3:
                continue
            p0, p1 = fpos[a, j], fpos[b - 1, j]
            t = np.linspace(0.0, 1.0, b - a)
            chord_z = p0[2] + (p1[2] - p0[2]) * t
            h = fpos[a:b, j, 2] - chord_z        # height above the liftoff/touchdown chord
            bl.append((a, b, t))
            ap.append(float(h.max()))
        leg_bouts[leg], leg_apex[leg] = bl, ap

    # --- pass 2: how much to add.  Symmetric picks one amplitude per mirror pair, from
    # the WORST bout in the pair, so the added dz is identical left and right and the
    # lower leg still reaches the target.  Per-leg is the old, asymmetric choice.
    added_of = {}
    for leg in legs:
        ap = leg_apex[leg]
        if not ap:
            added_of[leg] = None
            continue
        if symmetric:
            mate = _MIRROR_LEG[leg]
            pool = ap + (leg_apex.get(mate) or [])
            added_of[leg] = max(0.0, target_m - min(pool))
        else:
            added_of[leg] = [max(0.0, target_m - a) for a in ap]

    for j, leg in enumerate(legs):
        need = added_of[leg]
        if need is None:
            report[leg] = {"apex_existing_mm": [], "added_mm": []}
            continue
        adds = []
        for k, (a, b, t) in enumerate(leg_bouts[leg]):
            v = float(need) if symmetric else need[k]
            adds.append(v)
            lift[a:b, j] = v * np.sin(np.pi * t) ** 2
        report[leg] = {"apex_existing_mm": [round(x * 1000, 1) for x in leg_apex[leg]],
                       "added_mm": [round(x * 1000, 1) for x in adds]}
    report["_symmetric"] = bool(symmetric)

    # --- vertical displacement -> (thigh, calf), per frame, foot straight up
    off = np.zeros((n, 12))
    jac = {l: [] for l in legs}
    leg_of = {j: l for j, l in enumerate(legs)}
    for i in range(n):
        if not lift[i].any():
            continue
        f0 = fpos[i]
        d = np.zeros(12, dtype=np.float32); d[1::3] = eps
        f_th = feet_at(q_ach[i] + d)
        d = np.zeros(12, dtype=np.float32); d[2::3] = eps
        f_ca = feet_at(q_ach[i] + d)
        for j in range(4):
            if lift[i, j] <= 0:
                continue
            A = np.array([[(f_th[j, 2] - f0[j, 2]) / eps, (f_ca[j, 2] - f0[j, 2]) / eps],
                          [(f_th[j, 0] - f0[j, 0]) / eps, (f_ca[j, 0] - f0[j, 0]) / eps]])
            try:
                sol = np.linalg.solve(A, np.array([lift[i, j], 0.0]))
            except np.linalg.LinAlgError:
                continue
            off[i, 3 * j + 1] = sol[0]
            off[i, 3 * j + 2] = sol[1]
            jac[leg_of[j]].append((A, sol / max(lift[i, j], 1e-12)))

    # The Jacobian is a property of the POSE, and the four legs are at different poses at
    # their own mid-swing, so the rad-per-mm each one needs is not expected to match.
    # Reported so that difference is a measured number rather than a suspicion.
    for leg, rows in jac.items():
        if not rows:
            continue
        A = np.mean([r[0] for r in rows], axis=0)
        g = np.mean([r[1] for r in rows], axis=0)
        report[leg]["jac_dz_dthigh_dcalf"] = [round(float(x), 4) for x in A[0]]
        report[leg]["jac_dx_dthigh_dcalf"] = [round(float(x), 4) for x in A[1]]
        report[leg]["cond"] = round(float(np.linalg.cond(A)), 1)
        report[leg]["rad_per_mm_thigh_calf"] = [round(float(x) / 1000.0, 6) for x in g]
    return off, report, lift


def _log_motion_for(meta: dict, clip: str) -> tuple:
    """The lateral speed and YAW RATE the log measured for this clip's session.

    Returned as ``(v_y, omega_z)`` in m/s and rad/s, or ``(nan, nan)`` if the
    profile table is not on disk.  Read from ``outputs/skill_profile.csv`` by the
    session name the archive meta records, so nothing is re-extracted and nothing
    is chosen: TURN's target comes out as omega_z = -0.3954 rad/s (-22.7 deg/s),
    which is the recording's own steady turn rate and NOT the -0.6 rad/s that was
    commanded to produce it (CLAUDE.md 3: command is not measurement).

    This is what makes an in-place turn's foot placement well posed. For a body
    rotating at omega about its own centre, the hip at body-frame offset
    (x_i, y_i) is itself moving: v_i = v_base + omega x r_i, so the lateral
    velocity that hip SHOULD have is v_y + omega * x_i -- forward hips one way,
    rear hips the other. Driving every foot to v_y = 0 fights the turn instead of
    supporting it, which is what the -37% stride on TURN was.
    """
    import csv
    sess = meta["clips"][clip].get("session")
    if not sess or not SKILL_PROFILE_CSV.is_file():
        return float("nan"), float("nan")
    try:
        for r in csv.DictReader(open(SKILL_PROFILE_CSV)):
            if r.get("session") == sess:
                return (float(r["vy_steady_mean"]), float(r["yaw_rate_steady_mean"]))
    except (KeyError, ValueError, OSError):
        pass
    return float("nan"), float("nan")


def _vx_des_for(meta: dict, clip: str) -> float:
    """The forward speed the LOG measured for this clip, or nan.

    The foot placement law needs a velocity target and CLAUDE.md 3 is explicit
    that the commanded speed and the achieved speed are different numbers
    (``move x=1.5-2.0`` produced 0.48 m/s).  So the target is the log's own
    steady-state measurement, never the command that produced it, and never a
    number chosen to make the sim look better.
    """
    try:
        return float(meta["clips"][clip]["selection"]["vx_steady_mean"])
    except (KeyError, TypeError, ValueError):
        return float("nan")


# --------------------------------------------------------------------------- #
# Isaac Lab replay  (UNVERIFIED — see the module docstring)
# --------------------------------------------------------------------------- #

# SimulationApp.close() tears the process down, so anything after it never runs.
# Both scripts originally closed inside the Isaac helper, before the metrics were
# computed and returned -- the run exited 0 having produced no verdict and no CSV.
# The app is held here and closed once, in main(), after the results are written.
_SIM_APP = None

#: One timestamp per process, so every row of a run carries the same one.
_RUN_UTC = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def run_isaac(args, clip: dict, meta: dict, then_clip: dict | None = None,
              via_clip: dict | None = None) -> dict:
    """Play ``clip`` on a flat floor and return measured gait numbers.

    The replay mode is *requested*, then resolved against what the installed
    Isaac Lab actually supports, and the resolved mode is returned with the
    result. A run can never be reported under a mode it did not use.
    """
    from isaaclab.app import AppLauncher                      # noqa: F401  (import order matters)

    global _SIM_APP
    app_launcher = AppLauncher(args)
    simulation_app = _SIM_APP = app_launcher.app

    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sensors import ContactSensor, ContactSensorCfg
    from isaaclab.sim import SimulationCfg, SimulationContext

    ucfg = upstream_go2()
    sys.path.insert(0, str(Path(ucfg.source).parents[3]))
    from legged_gym.envs.base.legged_robot_config import UNITREE_GO2_CFG

    # The clip sets the CONTROL period; the repo config sets the PHYSICS step.
    #
    # Stepping PhysX once per clip sample integrates contact at ~50 Hz, four times
    # coarser than the legged_robot_config.py this whole comparison is measured
    # against (sim_dt 0.005 x decimation 4 = 200 Hz physics, 50 Hz control). The
    # robot then goes down on its belly regardless of clip, mode or joint sign.
    # Measured on WALK, position mode, everything else held fixed:
    #     1 step  per sample: base 0.169 m mean, falls at 1.40 s, 72/296 steps
    #     4 steps per sample: base 0.316 m mean, no fall,        296/296 steps
    # 0.32 m is nominal. Holding the command across the substeps is exactly what
    # decimation means in the env.
    #
    # Both numbers are read from the config; neither is hard-coded here. The
    # control period stays the clip's own, so a cyclic clip -- whose n_lo samples
    # span exactly one gait cycle by construction -- loops with no phase drift.
    ctrl_dt = 1.0 / clip["fs"]
    decim = int(ucfg.decimation or 1)
    phys_dt = ctrl_dt / decim
    dt = ctrl_dt                       # everything measured per control step uses this
    if ucfg.control_hz and abs(clip["fs"] - ucfg.control_hz) > 1.0:
        print(f"[replay] NOTE clip rate {clip['fs']:.1f} Hz != the env's control rate "
              f"{ucfg.control_hz:.0f} Hz; stepping at the clip rate")
    print(f"[replay] control {1.0/ctrl_dt:.2f} Hz (clip) x decimation {decim} "
          f"-> physics {1.0/phys_dt:.1f} Hz   [config: {ucfg.control_hz:.0f} Hz x "
          f"{ucfg.decimation} -> {1.0/ucfg.sim_dt:.0f} Hz]")
    if ucfg.sim_dt and abs(phys_dt - ucfg.sim_dt) / ucfg.sim_dt > 0.05:
        print(f"[replay] WARNING physics dt {phys_dt:.5f} s is more than 5% off the "
              f"config's {ucfg.sim_dt} s; the clip rate is far from the env's control rate")
    sim = SimulationContext(SimulationCfg(dt=phys_dt, device=args.device))
    sim.set_camera_view([2.0, 2.0, 1.0], [0.0, 0.0, 0.3])

    # Isaac's default ground is static/dynamic friction 0.5, which is BELOW the
    # whole range the env randomises over.  Stand on the range's midpoint instead,
    # read from the env config -- see Go2Config.ground_friction.
    mu = ucfg.ground_friction
    if mu is None:
        print("[replay] WARNING no friction_range in the env config; falling back to "
              "Isaac's default ground (0.5), which the env never uses")
        ground = sim_utils.GroundPlaneCfg()
    else:
        ground = sim_utils.GroundPlaneCfg(physics_material=ground_material_cfg(sim_utils))
        print(f"[replay] ground 1.00/1.00 multiply (the env's terrain material); "
              f"robot shapes -> {mu:.2f}, the midpoint of friction_range "
              f"{ucfg.friction_range}; effective mu {mu:.2f}")
    ground.func("/World/ground", ground)
    sim_utils.DomeLightCfg(intensity=2000.0).func("/World/light", sim_utils.DomeLightCfg(intensity=2000.0))

    robot_cfg = UNITREE_GO2_CFG.replace(prim_path="/World/Robot")

    requested = ReplayMode(args.mode) if args.mode else default_mode_for(
        meta["clips"][clip["name"]]["gains"]["position_controlled"])

    # FIXED_GAIN has to be decided before the sim starts: it edits the config.
    fixed_kp = float(np.median(clip["kp"]))
    fixed_kd = float(np.median(clip["kd"]))
    if requested is ReplayMode.FIXED_GAIN:
        for act in robot_cfg.actuators.values():
            act.stiffness, act.damping = fixed_kp, fixed_kd
        print(f"[replay] FIXED_GAIN: actuator gains set to kp={fixed_kp:g} kd={fixed_kd:g} "
              f"(clip medians) before sim start")

    robot = Articulation(robot_cfg)
    contact_cfg = ContactSensorCfg(prim_path="/World/Robot/.*_foot", history_length=1, track_air_time=True)
    contacts = ContactSensor(contact_cfg)

    # Optional side-view recording.  The failure mode is a roll, so the camera sits
    # abeam the robot and tracks its x only: lateral drift then reads as a change in
    # apparent size rather than being cancelled out by the camera follow, and roll is
    # seen square on.
    #
    # This must not change what is being measured.  The camera is a passive sensor;
    # frames are pulled at CONTROL-step boundaries with an explicit sim.render(), and
    # the physics substep loop below is not touched -- same phys_dt, same decimation,
    # same order of writes.  The check that this held is the run's own termination
    # time against the un-recorded run (see --video in the docstring).
    video = None
    if args.video:
        from isaaclab.sensors import Camera, CameraCfg
        cam_cfg = CameraCfg(
            prim_path="/World/side_cam",
            update_period=0.0,                     # on demand only; never self-schedules
            height=args.video_height, width=args.video_width,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, clipping_range=(0.05, 60.0)),
        )
        video = {"cam": Camera(cam_cfg), "frames": 0}
        print(f"[replay] video: side view {args.video_width}x{args.video_height}, "
              f"every {args.video_stride} control step(s) -> {args.video}")

    sim.reset()

    if mu is not None:
        ba = set_robot_friction(robot, mu)
        if ba:
            print(f"[replay] robot shape friction {ba[0]:.2f} (from go2.usd) -> {ba[1]:.2f}")
    cap = probe_capabilities(robot, ucfg.all_explicit)
    mode = cap.resolve(requested)
    if mode is not requested:
        print(f"[replay] requested mode {requested.value} -> using {mode.value}")
    for n in cap.notes:
        print(f"[replay]   capability: {n}")
    print(f"[replay] mode={mode.value}  explicit_actuators={cap.explicit_actuators}  "
          f"runtime_gains={cap.runtime_gain_write}  effort_target={cap.effort_target}")

    # Resolve DOF indices BY NAME. Isaac Lab's Go2 DOF order is joint-type major
    # and the clip is leg major; positional indexing is wrong and silently so.
    want = [f"{leg}_{j}_joint" for leg in clip["leg_order"] for j in clip["joint_order"]]
    idx, missing = [], []
    for jn in want:
        hit = [i for i, n in enumerate(robot.joint_names) if n == jn]
        (idx.append(hit[0]) if hit else missing.append(jn))
    if missing:
        raise SystemExit(f"joint names not found in the articulation: {missing}\n"
                         f"articulation has: {robot.joint_names}")
    idx_t = torch.as_tensor(idx, device=sim.device, dtype=torch.long)
    print(f"[replay] clip joint -> DOF index: {dict(zip(want, idx))}")

    # Hip-sign hypothesis from the offline convention check (see --convention).
    sign = np.ones(12, dtype=np.float32)
    if args.hip_sign == "flip":
        sign[0::3] = -1.0
        print("[replay] hip columns negated (--hip-sign flip)")

    # Choose where in the loop to start, before anything is tiled.  Cyclic clips only:
    # a one-shot clip has a beginning that means something.
    if args.start_phase == "level" and clip["kind"] == "cyclic":
        k, ph = level_start(clip, robot, sim, idx_t, phys_dt)
        clip = rotate_clip(clip, k)
        print(f"[replay] start phase: frame {ph['start_frame']}/{len(clip['q_des'])}, "
              f"foot-height spread {ph['spread_at_start']*1000:.1f} mm "
              f"(frame 0: {ph['spread_at_frame0']*1000:.1f} mm; "
              f"worst in clip: {ph['spread_worst']*1000:.1f} mm)")
        if ph["spread_at_start"] > 0.01:
            print(f"[replay] WARNING no phase of this clip puts the four feet within "
                  f"10 mm of a plane (best {ph['spread_at_start']*1000:.1f} mm). The clip "
                  f"stores joint angles only -- it carries no base trajectory -- so a "
                  f"gait with a flight phase has no pose that can be stood on.")
    elif args.start_phase == "stance" and clip["kind"] == "cyclic":
        k, ph = quiescent_start(clip)
        clip = rotate_clip(clip, k)
        print(f"[replay] start phase: frame {ph['start_frame']}/{len(clip['q_des'])} "
              f"({ph['feet_down']}/4 feet down, |dq| {ph['speed_at_start']:.2f}) instead of "
              f"frame 0 ({ph['feet_down_at_frame0']}/4 down, |dq| {ph['speed_at_frame0']:.2f})")
        if not ph["all_stance"]:
            print(f"[replay] WARNING this clip has NO frame with all four feet down "
                  f"(max {ph['feet_down']}), so no phase gives a four-foot initial "
                  f"condition. A flight-phase gait cannot be started from rest.")
    elif args.start_phase == "measured" and clip["kind"] == "cyclic":
        # The entry phase as a MEASUREMENT rather than a criterion.  For TURN the two
        # kinematic rules above both pick badly -- `level` picks frame 24, which is
        # inside a ten-frame band where the flat control never completes its 90 deg --
        # so the frame is carried in planner/config.py, derived from a sweep of every
        # phase in both foot-comp arms (outputs/turn_entry_phase.md).  A clip with no
        # measurement falls back to `level` and says so.
        from planner.config import DEFAULT as _CFG
        k = int(getattr(_CFG.skill, f"ENTRY_FRAME_{clip['name']}", -1))
        if k < 0:
            print(f"[replay] start phase: no measured entry frame for {clip['name']} in "
                  f"planner/config.py -- falling back to the coplanarity rule")
            k, ph = level_start(clip, robot, sim, idx_t, phys_dt)
        else:
            _, ph = quiescent_start(clip)
            ph["start_frame"] = k
            print(f"[replay] *** MEASURED ENTRY PHASE: frame {k}/{len(clip['q_des'])} for "
                  f"{clip['name']}, from planner.config.skill.ENTRY_FRAME_{clip['name']} "
                  f"(outputs/turn_entry_phase.md, 810 flat runs). This is a choice of WHERE "
                  f"IN THE LOOP to start; the recording is unchanged. Default is "
                  f"--start-phase first, which is what every earlier run used. ***")
        clip = rotate_clip(clip, k)
    elif clip["kind"] == "cyclic":
        _, ph = quiescent_start(clip)
    else:
        ph = {"start_frame": 0, "feet_down": None, "all_stance": None}
    ph.setdefault("spread_at_start", float("nan"))

    n = clip["q_des"].shape[0]
    reps = args.cycles if clip["kind"] == "cyclic" else 1
    total = n * reps

    # ------------------------------------------------------- plant compensation
    # NOT a balance controller and not a clip edit: the archive on disk is never
    # touched, and this adds no feedback -- it is a constant, precomputed offset,
    # so the run stays an open-loop joint trajectory.
    #
    # What it corrects.  The sport controller's standing stance is about 0.17 rad
    # narrower at the hip than this fork's init_state.joint_pos, so replayed as
    # recorded the robot walks with its feet 25-30 mm closer to the midline than
    # the env's own default, and its geometric tipping angle drops from ~27 deg to
    # ~23 deg. That is a posture mismatch between two nominal stances, of the same
    # kind as the ground friction being left at Isaac's 0.5 instead of the env's
    # 1.3 -- see outputs/... the hip-sign pairs. It is settled on FLAT GROUND from
    # the two stances alone; no terrain, no depth, no benchmark score enters it.
    #
    # The offset moves only the hip DC level. The hip's motion within the cycle,
    # and thigh and calf entirely, are the recording's.  alpha 0 is the null
    # control and must reproduce --plant-comp off exactly.
    q_des_signed = clip["q_des"] * sign
    q_des_untouched = q_des_signed.copy()      # deviation baseline: before ANY edit
    comp_offset = np.zeros(12, dtype=np.float64)
    lift_off = np.zeros_like(q_des_signed, dtype=np.float64)
    lift_report = {}
    if args.swing_lift > 0:
        lift_off, lift_report, lift_z = swing_lift_offsets(
            robot, sim, idx_t, clip, args.swing_lift / 1000.0, phys_dt, sign=sign,
            symmetric=not args.swing_lift_asym)
        q_des_signed = q_des_signed + lift_off
        print(f"[replay] *** SWING LIFT --swing-lift {args.swing_lift:g} mm "
              f"({'PER-LEG (asymmetric)' if args.swing_lift_asym else 'LEFT-RIGHT SYMMETRIC'}): each swing "
              f"foot's arc is raised to that apex above its own liftoff/touchdown chord, "
              f"as A*sin^2(pi*phase) so both ends and both end SLOPES are unchanged. "
              f"Stance is untouched and so is the hip. This EDITS THE RECORDING and every "
              f"number below is stamped with it. The archive on disk is unchanged. ***")
        for leg, r in lift_report.items():
            if leg.startswith("_"):
                continue
            print(f"[replay]   {leg}: apex already {r['apex_existing_mm']} mm, "
                  f"adding {r['added_mm']} mm, dz/dthigh,dcalf "
                  f"{r.get('jac_dz_dthigh_dcalf')} cond {r.get('cond')}")
        print(f"[replay]   joint offsets: thigh max {np.abs(lift_off[:, 1::3]).max():.4f} rad, "
              f"calf max {np.abs(lift_off[:, 2::3]).max():.4f} rad")
    if args.plant_comp == "height":
        # The body sits BELOW the clip's own stance geometry from the first step -- not a
        # drift, measured flat from 1 s onward -- because kp 40 sags under load: the
        # commanded stance hip-to-foot drop is 338 mm (WALK) and the achieved is 297.
        # Deficits: WALK 39.5 mm, TROT 45.5, TURN 28.2 (outputs/stance_height.json).
        #
        # The correction lengthens the leg with THIGH and CALF, in the ratio that moves
        # the foot straight DOWN and leaves its fore-aft position alone -- solved as a
        # 2x2 from a numerical Jacobian at each clip's own mean stance pose. Widening the
        # stance is what the earlier plant compensation did and it cost a third of the
        # forward speed; this deliberately does not touch the hip, which is also the
        # joint foot placement and heading hold use, so the three do not fight for the
        # same actuator.
        #
        # alpha is in units of the SOLVED deficit: 1.0 asks for the whole of it. The
        # Jacobian is kinematic and the sag is a load effect, so what alpha actually buys
        # in base height is not 1:1 and is measured per run rather than assumed.
        import json as _json
        rec = _json.loads(Path(args.stance_height_json).read_text())[clip["name"]]
        for k, leg in enumerate(clip["leg_order"]):
            dth, dca = rec["offset_rad"][leg]
            comp_offset[3 * k + 1] = args.plant_comp_alpha * dth
            comp_offset[3 * k + 2] = args.plant_comp_alpha * dca
        q_des_signed = q_des_signed + comp_offset
        print(f"[replay] *** PLANT COMPENSATION --plant-comp height "
              f"--plant-comp-alpha {args.plant_comp_alpha:g}: thigh and calf are offset to "
              f"stand the body at the clip's own stance geometry, {np.mean(rec['deficit_mm']):.1f} mm "
              f"higher. Constant, no feedback, hips untouched. This is NOT the recording as "
              f"played elsewhere and every number below is stamped with it. The archive on "
              f"disk is unchanged. ***")
        for k, leg in enumerate(clip["leg_order"]):
            print(f"[replay]   {leg}: thigh {comp_offset[3*k+1]:+.4f}, calf "
                  f"{comp_offset[3*k+2]:+.4f} rad  (deficit {rec['deficit_mm'][k]:.1f} mm)")
    if args.plant_comp == "stance":
        _, env_pose, _ = ucfg.ordered(clip["leg_order"], clip["joint_order"])
        hip_default = np.asarray(env_pose, dtype=float)[0::3]
        hip_mean = q_des_signed[:, 0::3].mean(axis=0)
        comp_offset[0::3] = args.plant_comp_alpha * (hip_default - hip_mean)
        q_des_signed = q_des_signed + comp_offset
        print("[replay] *** PLANT COMPENSATION --plant-comp stance "
              f"--plant-comp-alpha {args.plant_comp_alpha:g}: the hip DC level is shifted "
              f"toward the env's default stance. This is NOT the recording as played "
              f"elsewhere and every number below is stamped with it. The archive on disk "
              f"is unchanged. ***")
        for k, leg in enumerate(clip["leg_order"]):
            print(f"[replay]   {leg}_hip: clip mean {hip_mean[k]:+.4f} -> "
                  f"{hip_mean[k] + comp_offset[3*k]:+.4f} rad "
                  f"(env default {hip_default[k]:+.4f}, offset {comp_offset[3*k]:+.4f})")

    # ------------------------------------------------------------ skill change
    # A sequence of clips played back to back, with the seam recorded.  Nothing is
    # blended and nothing is inserted: at the switch step the commanded pose jumps
    # from one recording to the other, exactly as it would when a planner issues a
    # new skill.  The size of that jump is measured and reported rather than
    # smoothed away, because a discontinuity the PD cannot follow is a real cost of
    # switching and hiding it would flatter the result.
    # Each entry carries, beyond the command channels, the two things the foot
    # placement law needs per step and which are properties of the CLIP rather than
    # of the run: the recording's own swing/stance schedule, and its measured
    # stance time and forward speed.  They travel in the sequence so a run that
    # plays two clips uses each clip's own numbers rather than the first one's --
    # the same defect the per-segment gait gate was fixed for.
    def _par(e, m_rows):
        vy_log, wz_log = _log_motion_for(meta, e["name"])
        # Column 4 is the clip's cycle length IN CONTROL STEPS.  A cyclic clip spans
        # exactly one gait cycle, so it is the averaging window that removes the
        # within-stride oscillation from a rate signal without introducing a cutoff
        # frequency to tune.
        return np.tile(np.array([[stance_time_s(e), _vx_des_for(meta, e["name"]),
                                  vy_log, wz_log, float(len(e["q_des"]))]]), (m_rows, 1))

    seq = [("A", clip["name"], np.tile(q_des_signed, (reps, 1)),
            np.tile(clip["tau_ff"] * sign, (reps, 1)),
            np.tile(clip["kp"], (reps, 1)), np.tile(clip["kd"], (reps, 1)),
            np.tile(clip["contact"], (reps, 1)), _par(clip, reps * n))]
    seams = []
    if args.then_clip:
        seq[0] = ("A", clip["name"],
                  np.tile(q_des_signed, (args.switch_after_cycles, 1)),
                  np.tile(clip["tau_ff"] * sign, (args.switch_after_cycles, 1)),
                  np.tile(clip["kp"], (args.switch_after_cycles, 1)),
                  np.tile(clip["kd"], (args.switch_after_cycles, 1)),
                  np.tile(clip["contact"], (args.switch_after_cycles, 1)),
                  _par(clip, args.switch_after_cycles * n))
        for role, extra, ncyc, hold_s in (("VIA", via_clip, None, args.via_s),
                                          ("B", then_clip, args.then_cycles, None)):
            if extra is None:
                continue
            e = extra
            if e["kind"] == "cyclic" and args.start_phase == "level":
                k, ph = level_start(e, robot, sim, idx_t, phys_dt)
                e = rotate_clip(e, k)
                print(f"[replay] {role} clip {e['name']}: start phase frame {ph['start_frame']}, "
                      f"foot spread {ph['spread_at_start']*1000:.1f} mm")
            m = (int(np.ceil(hold_s * e["fs"] / len(e["q_des"]))) if hold_s is not None
                 else ncyc) if e["kind"] == "cyclic" or hold_s is not None else 1
            m = max(int(m), 1)
            qs = np.tile(e["q_des"] * sign, (m, 1))
            if hold_s is not None:
                qs = qs[: max(int(round(hold_s * e["fs"])), 1)]
            n_keep = len(qs)
            seq.append((role, e["name"], qs,
                        np.tile(e["tau_ff"] * sign, (m, 1))[:n_keep],
                        np.tile(e["kp"], (m, 1))[:n_keep],
                        np.tile(e["kd"], (m, 1))[:n_keep],
                        np.tile(e["contact"], (m, 1))[:n_keep],
                        _par(e, n_keep)))
        off = 0
        for i, (role, nm, qs, *_rest) in enumerate(seq):
            if i:
                jump = float(np.abs(seq[i][2][0] - seq[i - 1][2][-1]).max())
                seams.append({"step": off, "t_s": off * ctrl_dt, "from": seq[i - 1][1],
                              "to": nm, "cmd_jump_rad": jump})
                print(f"[replay] switch at step {off} ({off*ctrl_dt:.2f} s): "
                      f"{seq[i-1][1]} -> {nm}, commanded pose jumps {jump:.3f} rad "
                      f"on its largest joint")
            off += len(qs)

    q_des_signed = np.concatenate([x[2] for x in seq])
    total = len(q_des_signed)
    q_cmd = torch.as_tensor(q_des_signed, device=sim.device, dtype=torch.float32)
    tau_ff = torch.as_tensor(np.concatenate([x[3] for x in seq]), device=sim.device, dtype=torch.float32)
    kp_seq = np.concatenate([x[4] for x in seq])
    kd_seq = np.concatenate([x[5] for x in seq])
    clip_contact_seq = np.concatenate([x[6] for x in seq]).astype(bool)   # (total, 4)
    clip_par_seq = np.concatenate([x[7] for x in seq])                    # (total, 2)

    # ------------------------------------------------------------------ settle
    # The initial condition is not a formality here: it decides the verdict.  The
    # env spawns the base at 0.42 m, a height chosen for its DEFAULT STANDING pose
    # (all four feet down).  `drop` overrides the joints to the clip's first frame
    # -- a mid-gait pose with one or two feet in swing -- and lets the robot fall
    # into it from that same 0.42 m.  It lands on three legs and starts tipping, so
    # the replay begins with the base already moving.  Measured at handover:
    #   WALK 0.12 m/s /  6 deg/s -> survives
    #   TROT 0.37 m/s / 52 deg/s -> collapses at 0.82 s
    #   RUN  0.19 m/s / 13 deg/s, front-left unloaded -> collapses at 1.11 s
    # `stand` instead settles on the default pose the spawn height is designed for,
    # then drives the joints to the clip's first frame with the PD while the robot
    # is already standing.  Same settle_s for each half, so no new tuned constant.
    root = robot.data.default_root_state.clone()
    robot.write_root_state_to_sim(root)
    q_clip0 = robot.data.default_joint_pos.clone()
    q_clip0[:, idx_t] = q_cmd[0]

    def _hold(target, n_ctrl):
        for _ in range(n_ctrl):
            robot.set_joint_position_target(target)
            for _ in range(decim):
                robot.write_data_to_sim()
                sim.step()
                robot.update(phys_dt)
                # the sensor has to be stepped too, or the handover contact reading
                # below is whatever was in the buffer before the settle ran
                contacts.update(phys_dt)

    n_settle = int(args.settle_s / ctrl_dt)
    if args.settle_mode == "drop":
        robot.write_joint_state_to_sim(q_clip0, torch.zeros_like(q_clip0))
        _hold(q_clip0, n_settle)
    else:
        q_stand = robot.data.default_joint_pos.clone()
        robot.write_joint_state_to_sim(q_stand, torch.zeros_like(q_stand))
        _hold(q_stand, n_settle)
        for k in range(1, n_settle + 1):
            a = k / n_settle
            _hold(q_stand * (1.0 - a) + q_clip0 * a, 1)

    # The handover state is reported whatever the mode: a replay that begins with the
    # base already moving is measuring the settle, not the clip.
    hand_v = float(torch.linalg.norm(robot.data.root_lin_vel_b[0]).item())
    hand_w = float(np.degrees(torch.linalg.norm(robot.data.root_ang_vel_b[0]).item()))
    hand_f = contacts.data.net_forces_w[0].norm(dim=-1)
    hand_loaded = int((hand_f > args.contact_threshold_n).sum().item())
    print(f"[replay] handover after --settle-mode {args.settle_mode}: "
          f"|v| {hand_v:.3f} m/s, |w| {hand_w:.1f} deg/s, {hand_loaded}/4 feet loaded, "
          f"base {robot.data.root_pos_w[0, 2].item():.3f} m")

    # ------------------------------------------------------- watching it live
    # A run is 6-20 s long and a livestream client needs longer than that just to
    # connect, so there has to be a way to stop the interesting second happening
    # before anyone is looking at it.
    #
    # Neither of these may perturb the run.  `_hold_open` calls sim.render() and
    # nothing else: no write_data_to_sim, no sim.step, no robot.update, so the
    # physics clock does not advance while it holds.  `--slowdown` sleeps between
    # control steps; sleeping changes when a step is integrated, never how.  The
    # check that this held is the same one --video is held to: the run's own
    # termination time against the un-watched run.
    def _hold_open(seconds: float, why: str) -> None:
        if seconds <= 0:
            return
        print(f"[replay] holding {seconds:.0f} s -- {why}. Physics is NOT advancing; "
              f"the scene is frozen at this frame.", flush=True)
        t_end = time.time() + seconds
        while time.time() < t_end:
            sim.render()
        print("[replay] hold over", flush=True)

    _hold_open(args.hold_s, "connect the viewer now; playback starts when this ends")

    if video is not None:
        import imageio.v2 as imageio
        fps = args.video_fps if args.video_fps else max(1.0, (1.0 / dt) / args.video_stride)
        Path(args.video).parent.mkdir(parents=True, exist_ok=True)
        video["writer"] = imageio.get_writer(args.video, fps=fps, macro_block_size=None,
                                             codec="libx264", quality=8)
        video["fps"] = fps
        print(f"[replay] video: writing {fps:.1f} fps ({'real time' if not args.video_fps else 'forced'})")

    def grab_frame():
        """Pull one RGB frame. Renders only; never steps physics."""
        cam = video["cam"]
        bx = float(robot.data.root_pos_w[0, 0].item())
        bz = float(robot.data.root_pos_w[0, 2].item())
        eye = torch.tensor([[bx, args.video_side_m, max(0.35, bz + 0.12)]],
                           device=sim.device, dtype=torch.float32)
        tgt = torch.tensor([[bx, 0.0, 0.28]], device=sim.device, dtype=torch.float32)
        cam.set_world_poses_from_view(eye, tgt)
        sim.render()
        cam.update(dt, force_recompute=True)
        rgb = cam.data.output["rgb"][0]
        rgb = rgb.detach().cpu().numpy()
        video["writer"].append_data(np.ascontiguousarray(rgb[..., :3]).astype(np.uint8))
        video["frames"] += 1

    rec = {k: [] for k in ("q", "tau", "root_lin_vel_b", "root_ang_vel_b", "root_pos_w", "contact",
                           "root_quat_w", "contact_f", "foot_pos_w", "foot_u", "swing")}
    foot_ids, foot_names = foot_body_ids(robot)   # articulation indices, not sensor indices
    terminated_s, term_reason, gain_writes, clipped = None, "", 0, 0
    # ------------------------------------------------------- balance compensation
    # This one DOES close a loop.  Everything else in this file is an open-loop
    # replay; --balance-comp reads the base attitude the clip never carried and
    # feeds it back, so a run made with it is not "the recording played back" and
    # cannot be compared with one that is except as a pair.
    #
    # Stage 1, deliberately the cheap version: a PD on base roll added to the hip
    # (abduction) targets, and on base pitch added to the thigh targets -- the two
    # axes that actually have authority over those rotations.  Roll goes to ALL
    # FOUR hips with the same sign, which slides the whole stance laterally: the
    # zero-action pose is +0.1/-0.1/+0.1/-0.1, so outboard is +hip on the left legs
    # and -hip on the right, and a uniform offset therefore moves both sides the
    # same way in y rather than widening the stance. Widening is what --plant-comp
    # does and it kills forward speed; this has to be the other thing.
    #
    # No terrain, no depth, no benchmark score: the only inputs are the base's own
    # roll and pitch and their rates.
    bc_kp_roll, bc_kd_roll = args.balance_kp_roll, args.balance_kd_roll
    bc_kp_pitch, bc_kd_pitch = args.balance_kp_pitch, args.balance_kd_pitch
    if args.balance_comp != "off":
        print(f"[replay] *** BALANCE COMPENSATION --balance-comp {args.balance_comp}: "
              f"roll PD ({bc_kp_roll:g}, {bc_kd_roll:g}) -> hip, "
              f"pitch PD ({bc_kp_pitch:g}, {bc_kd_pitch:g}) -> thigh. THIS CLOSES A LOOP "
              f"on base attitude. The run below is NOT an open-loop replay and no result "
              f"from it may be reported as one. The archive on disk is unchanged. ***")
    bc_hip = np.zeros(12, dtype=bool); bc_hip[0::3] = True
    bc_thigh = np.zeros(12, dtype=bool); bc_thigh[1::3] = True
    bc_hip_t = torch.as_tensor(bc_hip.astype(np.float32), device=sim.device)
    bc_thigh_t = torch.as_tensor(bc_thigh.astype(np.float32), device=sim.device)
    bc_max = 0.0

    # ------------------------------------------- foot placement, Raibert (stage 2)
    # Stage 1 (--balance-comp pd) moved WHERE THE BODY IS and did not hold any of
    # these gaits; the reading it left was that for a trot what holds the robot up
    # is where the next foot LANDS.  This is that, and it is the first thing here
    # that is allowed to overwrite the recording: the supervisor has released the
    # constraint that the played trajectory stay the recorded one, on condition
    # that the skill stays the same skill.  So the recording is no longer the
    # output -- it is the BASELINE, and how far a run has to depart from it before
    # it stands up is the measurement (see the deviation table below).
    #
    # The law, lateral first, because roll is what diverges and left-right foot
    # position is what has authority over roll:
    #
    #     foot_y = (T_stance / 2) * v_y  +  k * (v_y - v_y_target),  v_y_target = 0
    #
    # The leading term is the neutral point -- the place a foot has to land for the
    # stance to be symmetric about the body at the speed it is actually going -- and
    # it has NO free parameter: T_stance is duty x period measured from the clip's
    # own contact channel (stance_time_s).  ``--foot-k`` adds velocity damping on
    # top and defaults to 0, so the default law is the parameter-free one.
    #
    # Joint mapping.  Every Go2 hip rotates about +x, so a positive hip angle moves
    # that foot toward +y whichever side it is on (which is why the zero-action pose
    # is +0.1/-0.1/+0.1/-0.1 for outboard).  The lever is then exactly the hip-to-foot
    # VERTICAL drop: y_foot = y_hip + L sin(q), so dy/dq = L cos(q) = z_hip - z_foot.
    # It is measured once, here, from the robot's own body positions after the
    # settle, and printed -- proprioception and geometry, no terrain and no depth.
    #
    # Swing only.  A correction is added to a leg's hip target for the frames the
    # RECORDING has that leg in the air; a loaded leg keeps the recording exactly.
    # That means the correction is released the moment the clip's schedule says the
    # foot is down, which is a discontinuity of at most --foot-clip-rad and is
    # deliberately not smoothed: the cap is the quantity being swept, so it must
    # also bound every edit the run makes.
    #
    # This CLOSES A LOOP on the base's lateral velocity.  A run with it on is not an
    # open-loop replay, exactly as --balance-comp is not, and is banner-printed and
    # stamped for the same reason.
    fc_on = args.foot_comp != "off"
    # The two OPEN-LOOP steering probes.  They add a constant differential offset to the
    # swing legs and read nothing back, so a run with one of them on is still an
    # open-loop replay plus a constant edit -- the same category as --plant-comp, not the
    # same category as --foot-comp.  They exist because "can differential foot placement
    # steer this robot, and how hard?" is a question about the ACTUATOR, and a closed
    # loop that fails answers it only ambiguously.  Measure the gain, then decide whether
    # a controller is worth building on it.
    fc_bias = abs(args.foot_yaw_bias) > 0 or abs(args.foot_len_bias) > 0
    fc_geo = fc_on or fc_bias
    fc_lever = np.full(4, np.nan)
    fc_hip_xy = np.zeros((4, 2))
    fc_sensor_col = None
    fc_vy = fc_vx = fc_wz = 0.0
    fc_wz_buf, fc_wz_sum, fc_wz_i, fc_wz_n = None, 0.0, 0, 0
    fc_alpha = 1.0
    fc_cap_hits = fc_applied = 0
    u_foot = np.zeros(12, dtype=np.float32)
    if fc_geo:
        hip_ids, hip_names = robot.find_bodies(".*_hip")
        h_by = {n.split("_")[0]: i for i, n in zip(hip_ids, hip_names)}
        f_by = {n.split("_")[0]: i for i, n in zip(foot_ids, foot_names)}
        missing = [l for l in clip["leg_order"] if l not in h_by or l not in f_by]
        if missing:
            raise SystemExit(f"[replay] cannot build the foot placement lever: no hip/foot "
                             f"body for {missing} (hips {hip_names}, feet {foot_names})")
        bp = snap(robot.data.body_pos_w[0])
        for k, leg in enumerate(clip["leg_order"]):
            fc_lever[k] = float(bp[h_by[leg], 2] - bp[f_by[leg], 2])
        # Where each hip sits on the body, in the BODY frame.  This is the lever the
        # yaw-rate term acts through: a body turning at omega carries the hip at
        # (x_i, y_i) sideways at omega * x_i, so front and rear hips need opposite
        # lateral placement to support the same turn.  Taken from the articulation's
        # own body positions, rotated out of world by the base quaternion -- measured
        # geometry, no constant typed in.
        from sim.replay import quat_rotate_inv
        base_p = snap(robot.data.root_pos_w[0])
        base_q = snap(robot.data.root_quat_w[0])
        for k, leg in enumerate(clip["leg_order"]):
            fc_hip_xy[k] = quat_rotate_inv(base_q[None, :], (bp[h_by[leg]] - base_p)[None, :])[0, :2]
        if not np.all(fc_lever > 0.05):
            raise SystemExit(f"[replay] hip-to-foot drop {fc_lever} is not a usable lever; "
                             f"the robot is not standing on the pose the correction is "
                             f"linearised about. Refusing to report numbers.")
        if args.foot_swing_source == "sim":
            cn = [str(x) for x in getattr(contacts, "body_names", [])]
            c_by = {n.split("_")[0]: i for i, n in enumerate(cn)}
            if any(l not in c_by for l in clip["leg_order"]):
                raise SystemExit(f"--foot-swing-source sim needs the contact sensor's body "
                                 f"names to identify legs; it reports {cn}")
            # The sensor's body order is NOT the articulation's and NOT the clip's
            # (harness_findings.md 5). Resolve it by name or not at all.
            fc_sensor_col = np.array([c_by[l] for l in clip["leg_order"]], dtype=int)
        if args.foot_vel_filter_hz > 0:
            fc_alpha = float(1.0 - np.exp(-2 * np.pi * args.foot_vel_filter_hz * dt))
        if fc_bias:
            print(f"[replay] *** STEERING PROBE --foot-yaw-bias {args.foot_yaw_bias:g} "
                  f"--foot-len-bias {args.foot_len_bias:g}: a CONSTANT differential offset "
                  f"is added to the swing legs (yaw-bias: hips, +front/-rear; len-bias: "
                  f"thighs, +left/-right). No feedback -- this measures how much yaw rate "
                  f"the placement is worth, open loop. The archive on disk is unchanged. ***")
        if not fc_on:
            print("[replay]   (--foot-comp is off: the only edit is the constant bias above)")
        if fc_on:
            print(f"[replay] *** FOOT PLACEMENT --foot-comp {args.foot_comp} "
                  f"axis={args.foot_axis} k={args.foot_k:g}s cap={args.foot_clip_rad:g} rad "
                  f"sign={args.foot_sign:+g}: a Raibert lateral foot-placement term is added "
                  f"to the SWING legs' hip targets. THIS CLOSES A LOOP on base lateral "
                  f"velocity and it OVERWRITES the recording. The run below is not an "
                  f"open-loop replay and its departure from the clip is measured, not "
                  f"assumed. The archive on disk is unchanged. ***")
        print(f"[replay]   T_stance (clip, measured) {clip_par_seq[0, 0]:.3f} s -> neutral-point "
              f"gain {0.5 * clip_par_seq[0, 0]:.3f} s; vx target (log) {clip_par_seq[0, 1]:.3f} m/s")
        print("[replay]   hip->foot lever: " + ", ".join(
            f"{leg} {fc_lever[k]:.3f} m" for k, leg in enumerate(clip["leg_order"]))
            + f"  (cap {args.foot_clip_rad:g} rad = "
              f"{args.foot_clip_rad * float(np.mean(fc_lever)) * 1000:.0f} mm of foot travel)")
        if args.foot_yaw != "off":
            vyl, wzl = float(clip_par_seq[0, 2]), float(clip_par_seq[0, 3])
            if not (np.isfinite(vyl) and np.isfinite(wzl)):
                raise SystemExit(
                    f"--foot-yaw {args.foot_yaw} needs the log's own v_y and yaw rate for "
                    f"{clip['name']}, "
                    f"and {SKILL_PROFILE_CSV} did not supply them for session "
                    f"{meta['clips'][clip['name']].get('session')!r}. Refusing to fall back "
                    f"to a target of zero silently -- that is the assumption being tested.")
            print(f"[replay]   TARGET from the log, not from zero: v_y {vyl:+.4f} m/s, "
                  f"yaw {wzl:+.4f} rad/s ({np.degrees(wzl):+.2f} deg/s)")
            print("[replay]   hip offsets in body frame (the yaw lever): " + ", ".join(
                f"{leg} x{fc_hip_xy[k, 0]:+.3f} y{fc_hip_xy[k, 1]:+.3f}"
                for k, leg in enumerate(clip["leg_order"])))
            print("[replay]   per-leg v_y target = v_y_log + wz_log * x_i: " + ", ".join(
                f"{leg} {vyl + wzl * fc_hip_xy[k, 0]:+.4f}"
                for k, leg in enumerate(clip["leg_order"])) + " m/s")
        print(f"[replay]   swing source: {args.foot_swing_source}"
              + (f", velocity low-pass {args.foot_vel_filter_hz:g} Hz (alpha {fc_alpha:.3f})"
                 if args.foot_vel_filter_hz > 0 else ", velocity unfiltered"))

    wall_per_step = dt * args.slowdown if args.slowdown > 1.0 else 0.0
    if wall_per_step:
        print(f"[replay] pacing playback at 1/{args.slowdown:g} real time "
              f"({wall_per_step*1000:.0f} ms of wall clock per control step). This "
              f"sleeps between steps; it does not change the integration.")
    t_wall = time.time()
    for i in range(total):
        if wall_per_step:
            lag = t_wall + i * wall_per_step - time.time()
            if lag > 0:
                time.sleep(lag)
        if mode is ReplayMode.TORQUE:
            gain_writes += int(apply_gains(robot, idx_t, kp_seq[i], kd_seq[i], cap))
        tgt = robot.data.default_joint_pos.clone()
        cmd_i = q_cmd[i]
        if args.balance_comp != "off":
            r_now, p_now, _ = quat_to_rpy_deg(snap(robot.data.root_quat_w[0])[None, :])
            w_b = robot.data.root_ang_vel_b[0]
            roll_e = float(np.radians(r_now[0]))
            pitch_e = float(np.radians(p_now[0]))
            droll = float(w_b[0].item())
            dpitch = float(w_b[1].item())
            # Sign settled by measurement, not by reasoning about the joint frame:
            # on TROT at |kp| 0.5 the two directions gave 5.52 s and 14.04 s against
            # 1.95 s uncompensated, so the stabilising direction is the one written
            # here and positive gains are the useful ones.
            u_hip = bc_kp_roll * roll_e + bc_kd_roll * droll
            u_thigh = bc_kp_pitch * pitch_e + bc_kd_pitch * dpitch
            u_hip = float(np.clip(u_hip, -args.balance_clip_rad, args.balance_clip_rad))
            u_thigh = float(np.clip(u_thigh, -args.balance_clip_rad, args.balance_clip_rad))
            bc_max = max(bc_max, abs(u_hip), abs(u_thigh))
            cmd_i = cmd_i + bc_hip_t * u_hip + bc_thigh_t * u_thigh
        u_foot[:] = 0.0
        # The recording's own swing/stance schedule, phase-locked to the frame being
        # played.  Recorded whether or not the compensator is on, so an off run
        # carries the same phase labels the deviation table is split by.
        swing_now = ~clip_contact_seq[i]
        if fc_on:
            vb = robot.data.root_lin_vel_b[0]
            fc_vy += fc_alpha * (float(vb[1].item()) - fc_vy)
            fc_vx += fc_alpha * (float(vb[0].item()) - fc_vx)
            t_st, vx_des = float(clip_par_seq[i, 0]), float(clip_par_seq[i, 1])
            gain = 0.5 * t_st + args.foot_k
            if args.foot_yaw != "off":
                # Rotation kinematics, not a chosen target.  The hip at body-frame
                # (x_i, y_i) on a base with velocity v and yaw rate w is itself moving
                # at v + w x r_i, so the error this foot has to answer for is
                #   (v_y - v_y_log) + (w - w_log) * x_i
                # per leg.  Front and rear hips get opposite signs from the same yaw
                # error, which is what makes an in-place turn representable at all --
                # and, on a straight clip where w_log is ~0, is a heading corrector.
                wz_raw = float(robot.data.root_ang_vel_b[0][2].item())
                if args.foot_yaw == "log-cycle":
                    # Mean over exactly one cycle of the clip being played.  Measured on
                    # the working TROT: the instantaneous yaw rate is +5.32 deg/s with a
                    # standard deviation of 20.70 -- 3.9x larger than the bias it is
                    # supposed to correct -- while the one-cycle mean keeps the same
                    # +5.32 at sd 3.64.  Feeding back the raw signal is feeding back the
                    # stride, and that is what --foot-yaw log did when it turned a 60-cycle
                    # TROT into a 2.71 s one.  The window is the clip's own period, so
                    # this adds no constant to tune.
                    L = int(clip_par_seq[i, 4])
                    if fc_wz_buf is None or fc_wz_buf.size != L:
                        fc_wz_buf, fc_wz_sum, fc_wz_i, fc_wz_n = np.zeros(L), 0.0, 0, 0
                    fc_wz_sum += wz_raw - fc_wz_buf[fc_wz_i]
                    fc_wz_buf[fc_wz_i] = wz_raw
                    fc_wz_i = (fc_wz_i + 1) % L
                    fc_wz_n = min(fc_wz_n + 1, L)
                    fc_wz = fc_wz_sum / fc_wz_n          # warms up over the first cycle
                else:
                    fc_wz += fc_alpha * (wz_raw - fc_wz)
                vy_log, wz_log = float(clip_par_seq[i, 2]), float(clip_par_seq[i, 3])
                dvy = (fc_vy - vy_log) + (fc_wz - wz_log) * fc_hip_xy[:, 0]
                dvx = ((fc_vx - vx_des) - (fc_wz - wz_log) * fc_hip_xy[:, 1]
                       if args.foot_axis == "xy" and np.isfinite(vx_des) else np.zeros(4))
            else:
                dvy = np.full(4, fc_vy - args.foot_vy_target)
                dvx = (np.full(4, fc_vx - vx_des)
                       if args.foot_axis == "xy" and np.isfinite(vx_des) else np.zeros(4))
            dy = gain * dvy
            dx = gain * dvx
            if args.foot_swing_source == "sim":
                fnow = snap(contacts.data.net_forces_w[0].norm(dim=-1))
                swing_now = fnow[fc_sensor_col] <= args.contact_threshold_n
            swing = swing_now
            # +hip moves the foot toward +y on every leg, so the same sign serves
            # all four; the thigh is the other way round (+thigh swings the foot
            # backward), hence the minus on dx.
            raw_hip = args.foot_sign * dy / fc_lever
            raw_thigh = -args.foot_sign * dx / fc_lever
            # NOT `cap`: that name is the Capabilities record this run's mode was
            # resolved against, and shadowing it made the whole result block raise
            # after every number in it had already been computed.
            fc_cap = args.foot_clip_rad
            u_foot[0::3] = np.where(swing, np.clip(raw_hip, -fc_cap, fc_cap), 0.0)
            if args.foot_axis == "xy":
                u_foot[1::3] = np.where(swing, np.clip(raw_thigh, -fc_cap, fc_cap), 0.0)
            fc_applied += int(swing.sum())
            fc_cap_hits += int((swing & (np.abs(raw_hip) > fc_cap)).sum())
        if fc_bias:
            # Constant, open loop, and applied AFTER the feedback term so the two ADD.
            # The first version put it before, where --foot-comp's assignment to
            # u_foot[0::3] silently erased it -- caught because the probe's own null
            # control reproduced the uncompensated run to the digit instead of steering.
            # Not clipped by --foot-clip-rad either: the bias IS the amplitude being
            # set, so clipping it would measure the clip.
            #
            # +front/-rear on the hips is a differential LATERAL placement, the yaw
            # moment a quadruped steers with; +left/-right on the thighs is a
            # differential STEP LENGTH, the other candidate. The signs follow the
            # measured hip geometry: sign(x_i) is +1 for the front pair, sign(y_i) is
            # +1 for the left pair.
            u_foot[0::3] += np.where(swing_now,
                                     args.foot_yaw_bias * np.sign(fc_hip_xy[:, 0]), 0.0)
            u_foot[1::3] += np.where(swing_now,
                                     args.foot_len_bias * np.sign(fc_hip_xy[:, 1]), 0.0)
        if fc_on or fc_bias:
            cmd_i = cmd_i + torch.as_tensor(u_foot, device=sim.device, dtype=torch.float32)
        tgt[:, idx_t] = cmd_i
        robot.set_joint_position_target(tgt)
        if mode.needs_effort:
            eff = torch.zeros_like(tgt)
            eff[:, idx_t] = tau_ff[i]
            robot.set_joint_effort_target(eff)
        for _ in range(decim):
            robot.write_data_to_sim()
            sim.step()
            robot.update(phys_dt)
            contacts.update(phys_dt)

        # snap() copies out of the sim -- see sim/replay.py for why that is not
        # optional, and outputs/harness_findings.md 6 for what it cost.
        tau_now = robot.data.applied_torque[0, idx_t]
        rec["tau"].append(snap(tau_now))
        rec["q"].append(snap(robot.data.joint_pos[0, idx_t]))
        rec["root_lin_vel_b"].append(snap(robot.data.root_lin_vel_b[0]))
        rec["root_ang_vel_b"].append(snap(robot.data.root_ang_vel_b[0]))
        rec["root_pos_w"].append(snap(robot.data.root_pos_w[0]))
        f = contacts.data.net_forces_w[0].norm(dim=-1)
        rec["contact"].append(snap(f > args.contact_threshold_n))
        # Kept alongside the boolean so a collapse can be read back without a rerun:
        # which foot lost load first, and what the base was doing when it did.
        rec["contact_f"].append(snap(f))
        rec["root_quat_w"].append(snap(robot.data.root_quat_w[0]))
        rec["foot_pos_w"].append(snap(robot.data.body_pos_w[0, foot_ids]))
        # What this step overwrote, and which legs the gate called swing.  Recorded
        # per step rather than reconstructed afterwards: the swing gate can come
        # from the sim's own contacts, which no replay of the clip can recover.
        rec["foot_u"].append(u_foot.copy())
        rec["swing"].append(np.asarray(swing_now, dtype=bool).copy())
        if video is not None and i % args.video_stride == 0:
            grab_frame()
        # Height alone does not detect a fall.  A Go2 lying on its side keeps its
        # base above 0.15 m, so TROT handed over after two cycles ended at +90 deg
        # of roll and was recorded as "no fall".  Attitude is checked too.
        #
        # 60 deg is a stated bound, not a fitted one: it is roughly twice the
        # largest geometric tipping angle any of these runs has (26.9-30.9 deg,
        # atan(half stance width / base height)), so it cannot fire on a robot that
        # is merely leaning, and a base past it is not coming back.
        _r, _p, _ = quat_to_rpy_deg(snap(robot.data.root_quat_w[0])[None, :])
        if terminated_s is None:
            why = ("height" if robot.data.root_pos_w[0, 2].item() < args.fall_height_m
                   else "roll" if abs(float(_r[0])) > args.fall_attitude_deg
                   else "pitch" if abs(float(_p[0])) > args.fall_attitude_deg else None)
        else:
            why = None
        if why is not None:
            terminated_s, term_reason = i * dt, why
            print(f"[replay] terminated at {terminated_s:.2f} s on {why}: "
                  f"base {robot.data.root_pos_w[0, 2].item():.3f} m, "
                  f"roll {float(_r[0]):+.1f} deg, pitch {float(_p[0]):+.1f} deg")
            if video is not None:
                # a beat of the collapse, so the last frame is not the trigger frame
                for _ in range(int(round(0.4 / dt / args.video_stride))):
                    for _ in range(decim * args.video_stride):
                        robot.write_data_to_sim(); sim.step()
                        robot.update(phys_dt); contacts.update(phys_dt)
                    grab_frame()
            break

    if video is not None:
        video["writer"].close()
        print(f"[replay] video: wrote {video['frames']} frames "
              f"({video['frames'] / video['fps']:.2f} s at {video['fps']:.1f} fps) -> {args.video}")

    _hold_open(args.hold_s, "the run is over -- last frame held so it can be looked at")

    a = {k: np.asarray(v) for k, v in rec.items()}
    try:
        assert_not_aliased({k: a[k] for k in ("root_pos_w", "root_quat_w", "root_lin_vel_b")})
    except AssertionError as exc:
        raise SystemExit(f"[replay] BUG {exc} Refusing to report numbers.")
    _, _, limits = ucfg.ordered(clip["leg_order"], clip["joint_order"])
    lim = np.asarray([l if l is not None else np.inf for l in limits], dtype=float)
    saturated = float((np.abs(a["tau"]) >= lim[None, :] - 1e-3).mean())

    # How far the run departed from the recording it is a replay of.  This is the
    # deliverable of stage 2, not a diagnostic: the constraint that was released is
    # "keep the recorded trajectory", and what replaces it is a measured budget.
    # The baseline is the clip as stored, before plant compensation and before the swing
    # lift -- otherwise an edit that is applied to q_des before the run would report a
    # deviation of zero, which is the opposite of what "how far from the original" means.
    n_meas = a["q"].shape[0]
    q_orig_np = np.tile(q_des_untouched, (reps, 1))[:n_meas]
    q_cmd_np = q_cmd.detach().cpu().numpy()[:n_meas]
    edit_total = (q_cmd_np - q_orig_np) + a["foot_u"]      # every edit, per step per joint
    dev_rows, dev_summ = deviation_report(edit_total, a["q"], q_orig_np, a["swing"],
                                          clip["leg_order"], clip["joint_order"])
    for r in dev_rows:
        r.update({"clip": clip["name"], "foot_comp": args.foot_comp,
                  "foot_k": args.foot_k, "foot_clip_rad": args.foot_clip_rad,
                  "foot_axis": args.foot_axis, "foot_sign": args.foot_sign,
                  "run_utc": _RUN_UTC, "tag": args.tag})
    print(f"-- departure from the recording ({clip['name']}, "
          f"--foot-comp {args.foot_comp}) --")
    print(f"   {'joint':6s} {'cmd RMS swing':>14s} {'cmd RMS stance':>15s} "
          f"{'meas RMS swing':>15s} {'meas RMS stance':>16s}   (rad)")
    for jn in clip["joint_order"]:
        print(f"   {jn:6s} {dev_summ[f'dev_cmd_rms_{jn}_swing']:14.4f} "
              f"{dev_summ[f'dev_cmd_rms_{jn}_stance']:15.4f} "
              f"{dev_summ[f'dev_meas_rms_{jn}_swing']:15.4f} "
              f"{dev_summ[f'dev_meas_rms_{jn}_stance']:16.4f}")
    print(f"   overwritten: {dev_summ['overwrite_frac_time']:.1%} of control steps, "
          f"{dev_summ['overwrite_frac_legsteps']:.1%} of leg-steps "
          f"(swing is {dev_summ['swing_frac']:.1%} of leg-steps); "
          f"largest edit {dev_summ['dev_cmd_max_rad']:.4f} rad")

    # Gait numbers come from the FORCE, read the way the logs are read.  The
    # boolean at args.contact_threshold_n is still recorded in the trace and still
    # answers "how many feet are loaded right now"; it is just not what a stride is
    # measured from.  See sim/diagnose.gait_from_force.
    m = D.gait_from_force(a["contact_f"], dt)
    m["duty_bare_threshold"] = float(a["contact"].mean())
    m["stride_hz_bare_threshold"] = D.gait_from_contact(a["contact"], dt)["stride_hz"]
    m.update({
        "vx_mean": float(a["root_lin_vel_b"][:, 0].mean()),
        "vy_mean": float(a["root_lin_vel_b"][:, 1].mean()),
        "yaw_rate_deg_s": float(np.degrees(a["root_ang_vel_b"][:, 2].mean())),
        "base_height_mean_m": float(a["root_pos_w"][:, 2].mean()),
        "terminated_s": terminated_s,
        "term_reason": term_reason,
        "n_steps": int(a["q"].shape[0]),
        "dt": dt,
        "phys_dt": phys_dt,
        "decimation": decim,
        "mode_requested": requested.value,
        "mode_used": mode.value,
        "gain_writes": gain_writes,
        "torque_saturated_frac": saturated,
        "cap_runtime_gains": cap.runtime_gain_write,
        "cap_effort_target": cap.effort_target,
        "hip_sign": args.hip_sign,
        "then_clip": args.then_clip or "",
        "via_clip": args.via_clip or "",
        "switch_after_cycles": args.switch_after_cycles if args.then_clip else "",
        "switch_step": seams[0]["step"] if seams else "",
        "switch_t_s": seams[0]["t_s"] if seams else "",
        "cmd_jump_rad": max((x["cmd_jump_rad"] for x in seams), default=""),
        # Every setting that decides the outcome, so a row can be reproduced from
        # the file alone.  These were the ones missing.
        "contact_threshold_n": args.contact_threshold_n,
        "fall_height_m": args.fall_height_m,
        "ground_friction_mu": mu,
        "plant_comp": args.plant_comp,
        "balance_comp": args.balance_comp,
        "balance_kp_roll": bc_kp_roll if args.balance_comp != "off" else 0.0,
        "balance_kd_roll": bc_kd_roll if args.balance_comp != "off" else 0.0,
        "balance_kp_pitch": bc_kp_pitch if args.balance_comp != "off" else 0.0,
        "balance_kd_pitch": bc_kd_pitch if args.balance_comp != "off" else 0.0,
        "balance_max_rad": bc_max,
        "plant_comp_alpha": args.plant_comp_alpha if args.plant_comp != "off" else 0.0,
        # Foot placement (stage 2).  Every knob is a column: a cap sweep is only
        # readable if the row says which cap produced it.
        "foot_comp": args.foot_comp,
        "foot_axis": args.foot_axis if fc_on else "",
        "foot_k": args.foot_k if fc_on else 0.0,
        "foot_clip_rad": args.foot_clip_rad if fc_on else 0.0,
        "foot_sign": args.foot_sign if fc_on else 0,
        "foot_vy_target": args.foot_vy_target if fc_on and args.foot_yaw == "off" else "",
        "foot_yaw": args.foot_yaw if fc_on else "",
        "foot_yaw_bias": args.foot_yaw_bias,
        "foot_len_bias": args.foot_len_bias,
        "foot_vy_log": float(clip_par_seq[0, 2]),
        "foot_wz_log": float(clip_par_seq[0, 3]),
        "foot_hip_x_front_m": float(np.mean(fc_hip_xy[fc_hip_xy[:, 0] > 0, 0])) if fc_on else 0.0,
        "foot_swing_source": args.foot_swing_source if fc_on else "",
        "foot_vel_filter_hz": args.foot_vel_filter_hz if fc_on else 0.0,
        "foot_lever_m": float(np.mean(fc_lever)) if fc_on else 0.0,
        "foot_t_stance_s": float(clip_par_seq[0, 0]),
        "foot_vx_des": float(clip_par_seq[0, 1]),
        "foot_cap_hit_frac": (fc_cap_hits / fc_applied) if fc_applied else 0.0,
        "tag": args.tag,
        "swing_lift_mm": args.swing_lift,
        "swing_lift_sym": int(not args.swing_lift_asym),
        "swing_lift_added_mm": json.dumps(
            {k: v.get("added_mm") for k, v in lift_report.items() if not k.startswith("_")}),
        "swing_lift_thigh_max_rad": float(np.abs(lift_off[:, 1::3]).max()),
        "swing_lift_calf_max_rad": float(np.abs(lift_off[:, 2::3]).max()),
        **dev_summ,
        "hip_offset_rad_max": float(np.abs(comp_offset).max()),
        "run_utc": _RUN_UTC,
        "argv": " ".join(sys.argv[1:]),
        "settle_mode": args.settle_mode,
        "start_phase": args.start_phase,
        "ablation": args.ablate,
        "clip_archive": (Path(args.clip_archive).name if args.clip_archive else "skill_clips.npz"),
        "start_frame": ph["start_frame"],
        "start_foot_spread_m": ph["spread_at_start"],
        "start_feet_down": ph["feet_down"],
        "handover_speed_mps": hand_v,
        "handover_ang_dps": hand_w,
        "handover_feet_loaded": hand_loaded,
    })
    if hand_loaded < 4 or hand_v > 0.15:
        print(f"[replay] WARNING the replay started from a base already moving at "
              f"{hand_v:.3f} m/s with {hand_loaded}/4 feet loaded. Anything measured "
              f"below is partly the settle, not the clip -- compare --settle-mode stand.")
    if saturated > 0.02:
        print(f"[replay] WARNING {saturated:.1%} of joint-samples hit the effort clip; "
              f"the sim robot is torque-limited on this clip (see --headroom)")
    if args.trace_npz:
        # The two foot orderings are NOT the same list and must travel with the
        # trace: foot_pos_w is in the articulation's body order, contact/contact_f
        # in the contact sensor's own.  Attributing a force to a leg by position
        # is the same class of mistake as harness_findings.md 5.
        np.savez_compressed(args.trace_npz, dt=dt, phys_dt=phys_dt,
                            terminated_s=(np.nan if terminated_s is None else terminated_s),
                            clip_name=clip["name"], start_frame=int(ph["start_frame"]),
                            clip_n=int(n), cycles=int(reps),
                            seam_steps=np.array([x["step"] for x in seams], dtype=int),
                            seam_names=np.array([f"{x['from']}->{x['to']}" for x in seams]),
                            leg_order=np.array(clip["leg_order"]),
                            joint_order=np.array(clip["joint_order"]),
                            foot_names=np.array(foot_names),
                            contact_names=np.array(getattr(contacts, "body_names", [""] * 4)),
                            q_cmd=q_cmd.detach().cpu().numpy()[: a["q"].shape[0]].copy(),
                            **{k: v for k, v in a.items()})
        print(f"[replay] trace -> {args.trace_npz}  ({a['q'].shape[0]} control steps)")
    # Per-segment gait numbers, so a sequence is judged against the clip that was
    # actually playing rather than against whichever one started the run.
    segs, off = [], 0
    for role, nm, qs, *_rest in seq:
        lo, hi_ = off, min(off + len(qs), a["q"].shape[0])
        off += len(qs)
        if hi_ - lo < 4:
            continue
        g = D.gait_from_force(a["contact_f"][lo:hi_], dt)
        segs.append({"role": role, "clip": nm, "t0_s": lo * dt, "t1_s": hi_ * dt,
                     "stride_hz": g.get("stride_hz", np.nan), "duty": g.get("duty", np.nan),
                     "vx_mean": float(a["root_lin_vel_b"][lo:hi_, 0].mean()),
                     "vy_mean": float(a["root_lin_vel_b"][lo:hi_, 1].mean()),
                     "yaw_rate_deg_s": float(np.degrees(a["root_ang_vel_b"][lo:hi_, 2].mean()))})
    m["_segments"] = segs

    m["_dev_rows"] = dev_rows
    m["_q_measured"] = a["q"]
    m["_q_commanded"] = np.tile(q_des_signed, (reps, 1))[: a["q"].shape[0]]
    return m


# --------------------------------------------------------------------------- #
# Self-test: prove the diagnosis names known faults
# --------------------------------------------------------------------------- #

def self_test() -> int:
    """Inject known faults into a synthetic replay and require them to be named."""
    rng = np.random.default_rng(0)
    n = 400
    ph = np.linspace(0, 4 * 2 * np.pi, n)
    base = np.stack([
        0.05 * np.sin(ph + 0.3 * leg) if j == 0 else
        0.40 * np.sin(ph + np.pi * (leg in (1, 2))) if j == 1 else
        0.30 * np.cos(ph + np.pi * (leg in (1, 2)))
        for leg in range(4) for j in range(3)], axis=1)
    noise = lambda x: x + 0.002 * rng.standard_normal(x.shape)

    cases = [
        ("clean", (0, 1, 2, 3), (1, 1, 1), "identity"),
        ("legs left/right swapped", (1, 0, 3, 2), (1, 1, 1), "left_right_swapped"),
        ("legs front/rear swapped", (2, 3, 0, 1), (1, 1, 1), "front_rear_swapped"),
        ("hip sign flipped", (0, 1, 2, 3), (-1, 1, 1), "hip sign flipped"),
        ("thigh+calf sign flipped", (0, 1, 2, 3), (1, -1, -1), "thigh/calf sign flipped"),
    ]
    fails = 0
    print("== mapping search ==")
    for label, legs, signs, want in cases:
        measured = noise(D.apply_mapping(base, legs, signs))
        ranked = D.best_mapping(measured, base)
        got = ranked[0].name
        ok = want in got
        fails += 0 if ok else 1
        print(f"  {'ok  ' if ok else 'FAIL'} {label:26s} -> {got:42s} r={ranked[0].score:+.3f} "
              f"(runner-up {ranked[1].name}, r={ranked[1].score:+.3f})")

    print("\n== gait measurement ==")
    dt = 1 / 200.0
    duty, period = 0.6, 0.5
    tt = np.arange(1000) * dt
    contact = np.stack([((tt / period + 0.25 * k) % 1.0) < duty for k in range(4)], axis=1)
    g = D.gait_from_contact(contact, dt)
    ok = abs(g["stride_hz"] - 1 / period) < 0.05 and abs(g["duty"] - duty) < 0.02
    fails += 0 if ok else 1
    print(f"  {'ok  ' if ok else 'FAIL'} stride {g['stride_hz']:.3f} Hz (want {1/period:.3f}), "
          f"duty {g['duty']:.3f} (want {duty:.3f})")

    print("\n== symptom -> cause ==")
    scenarios = [
        ("collapsed", {"base_height_mean_m": 0.10, "terminated_s": 0.4, "stride_hz": np.nan,
                       "duty": np.nan, "vx_mean": 0.0, "roll_abs_max_deg": 3.0},
         "on its belly"),
        ("splayed", {"base_height_mean_m": 0.30, "terminated_s": None, "roll_abs_max_deg": 40.0,
                     "stride_hz": 1.5, "duty": 0.6, "vx_mean": 0.1}, "hip (abduction) sign flipped"),
        ("mirrored", {"base_height_mean_m": 0.31, "terminated_s": None, "roll_abs_max_deg": 4.0,
                      "stride_hz": 1.5, "duty": 0.6, "vx_mean": 0.15, "vy_mean": 0.35},
         "left/right leg pair swapped"),
        ("backward", {"base_height_mean_m": 0.31, "terminated_s": None, "roll_abs_max_deg": 3.0,
                      "stride_hz": 1.5, "duty": 0.6, "vx_mean": -0.20}, "walks backward"),
        ("healthy", {"base_height_mean_m": 0.31, "terminated_s": None, "roll_abs_max_deg": 3.0,
                     "stride_hz": 1.52, "duty": 0.61, "vx_mean": 0.19, "vy_mean": 0.0,
                     "yaw_rate_deg_s": 1.0}, None),
    ]
    exp = {"stride_hz": 1.5, "duty": 0.6, "vx_mean": 0.19, "position_controlled": True}
    for label, m, want in scenarios:
        F = D.diagnose(m, exp)
        text = D.format_findings(F)
        v = D.verdict(F)
        if want is None:
            ok = v == "PASS"
        else:
            ok = want in text
        fails += 0 if ok else 1
        print(f"  {'ok  ' if ok else 'FAIL'} {label:10s} verdict={v:4s} "
              f"{'(no finding expected)' if want is None else 'names: ' + want}")

    # Real clips, if they are on disk: q (what the joints did) against q_des (what
    # was commanded). Same robot, same convention, so identity must win -- and the
    # margin it wins by is the honest one, not the one a symmetric toy gives.
    if SKILL_CLIPS_NPZ.is_file():
        print("\n== mapping search on the real clips (q vs q_des) ==")
        z = np.load(SKILL_CLIPS_NPZ, allow_pickle=False)
        for name in [str(x) for x in z["clip_names"]]:
            c = load_clip(name, "hi")
            ranked = D.best_mapping(c["q"], c["q_des"])
            margin = ranked[0].score - ranked[1].score
            ok = ranked[0].name == "identity"
            fails += 0 if ok else 1
            print(f"  {'ok  ' if ok else 'FAIL'} {name:5s} -> {ranked[0].name:20s} "
                  f"r={ranked[0].score:+.3f}  margin {margin:+.3f} over {ranked[1].name}")
        print("  (this validates the search, not the Isaac convention: both sides are the log's own)")

    print(f"\nself-test: {'PASS' if fails == 0 else f'{fails} FAILURES'}")
    return 0 if fails == 0 else 1


# --------------------------------------------------------------------------- #

def report(rows: list, out=None) -> None:
    """APPEND the rows, under an exclusive lock, merging the header if it grew.

    This used to open the file in "w".  Two people replaying at once then deleted
    each other's results, and a row could not be reproduced from the file anyway
    because the settings that decide the outcome -- contact threshold, fall
    height, ground friction -- were never columns.  Both are fixed here.

    The lock matters as much as the append: without it two appends racing on the
    same file interleave inside a line.  flock is advisory, so it only works
    because every writer goes through this function.
    """
    import csv
    import fcntl
    out = Path(out) if out else REPLAY_RESULTS_CSV
    out.parent.mkdir(parents=True, exist_ok=True)
    new_keys = {k for r in rows for k in r if not k.startswith("_")}

    with open(out, "a+", newline="") as fh:
        fcntl.flock(fh.fileno(), fcntl.LOCK_EX)
        try:
            fh.seek(0)
            existing = list(csv.DictReader(fh))
            old_keys = set(existing[0]) if existing else set()
            keys = sorted(old_keys | new_keys)
            if existing and old_keys != set(keys):
                # The schema grew.  Rewrite in place so the old rows keep their
                # values and gain empty cells, rather than being orphaned under a
                # header that no longer describes them.
                fh.seek(0)
                fh.truncate()
                w = csv.DictWriter(fh, fieldnames=keys)
                w.writeheader()
                for r in existing:
                    w.writerow({k: r.get(k, "") for k in keys})
            elif not existing:
                w = csv.DictWriter(fh, fieldnames=keys)
                w.writeheader()
            w = csv.DictWriter(fh, fieldnames=keys)
            for r in rows:
                w.writerow({k: r.get(k, "") for k in keys})
            fh.flush()
        finally:
            fcntl.flock(fh.fileno(), fcntl.LOCK_UN)
    print(f"[replay] appended {len(rows)} row(s) to {out}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clip", default="TROT", help="clip name (WALK/TROT/RUN/JUMP)")
    ap.add_argument("--all", action="store_true", help="every clip in the archive")
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo", help="which stored rate to play")
    ap.add_argument("--cycles", type=int, default=8, help="repeats for a cyclic clip")
    ap.add_argument("--mode", choices=[m.value for m in ReplayMode], default=None,
                    help="replay mode; default is position for a position-controlled clip and "
                         "torque otherwise. Falls back automatically if unsupported.")
    ap.add_argument("--hip-sign", choices=("keep", "flip"), default="keep",
                    help="negate the hip columns; see --convention for why this is a live question")
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--start-phase", choices=("first", "stance", "level", "measured"),
                    default="first",
                    help="Where in the loop to begin. first: frame 0, as recorded. "
                         "stance: the frame with the most feet down per the clip's contact "
                         "channel. level: the frame whose joint angles put the four feet "
                         "closest to a plane, measured from the robot's own kinematics. "
                         "measured: the frame planner/config.py records for this clip, "
                         "swept rather than inferred -- neither kinematic rule predicts "
                         "which phases of TURN complete a turn (outputs/turn_entry_phase.md). "
                         "A cyclic clip is a loop, so this selects an entry point; it does "
                         "not alter the recording. Ignored for one-shot clips.")
    ap.add_argument("--settle-mode", choices=("drop", "stand"), default="drop",
                    help="drop: fall into the clip's first pose from the spawn height "
                         "(original). stand: settle on the default standing pose first, "
                         "then drive to the clip pose with the PD.")
    ap.add_argument("--contact-threshold-n", type=float, default=1.0)
    ap.add_argument("--fall-height-m", type=float, default=0.15)
    ap.add_argument("--fall-attitude-deg", type=float, default=60.0,
                    help="also terminate when |roll| or |pitch| exceeds this. Height alone "
                         "misses a robot lying on its side. 60 deg is about twice the largest "
                         "geometric tipping angle in these runs, so it cannot fire on a lean.")
    ap.add_argument("--clip-archive", default=None,
                    help="play clips from another archive built the same way, e.g. the "
                         "per-cycle one from scripts/extract_raw_cycles.py. Its "
                         "<name>.meta.json must sit beside it. Default: the frozen "
                         "data/skill_clips.npz.")
    ap.add_argument("--results-csv", default=None,
                    help="where to write the result row (default outputs/replay_verify.csv); "
                         "give a separate file when sweeping so runs do not overwrite each other")
    ap.add_argument("--ablate", choices=("none", "mirror", "symmetrize"), default="none",
                    help="DIAGNOSTIC ONLY, never a reported skill. mirror: play the gait "
                         "left-right mirrored, to find out whether a lateral bias lives in "
                         "the clip or in the robot. symmetrize: average the clip with its own "
                         "mirror half a cycle later, removing the asymmetric component. "
                         "Neither adds feedback; both stay open loop.")
    ap.add_argument("--video", default=None,
                    help="record a side view to this .mp4 (needs --enable_cameras and a GPU). "
                         "Physics and control rates are unchanged; the recorded run's "
                         "termination time must match the un-recorded one.")
    ap.add_argument("--video-width", type=int, default=960)
    ap.add_argument("--video-height", type=int, default=540)
    ap.add_argument("--video-stride", type=int, default=1,
                    help="capture every Nth control step (1 = every step, real time)")
    ap.add_argument("--video-fps", type=float, default=None,
                    help="force a frame rate; default is real time for the stride")
    ap.add_argument("--video-side-m", type=float, default=2.4,
                    help="camera distance abeam the robot, metres (+y side)")
    ap.add_argument("--then-clip", default=None,
                    help="after --switch-after-cycles cycles of --clip, switch to this clip and "
                         "keep playing. Tests whether a skill that diverges can be handed over "
                         "to one that does not, which is a planner action rather than a fix.")
    ap.add_argument("--switch-after-cycles", type=int, default=1)
    ap.add_argument("--then-cycles", type=int, default=40)
    ap.add_argument("--via-clip", default=None,
                    help="hold this clip between the two (e.g. BALANCE). The real robot put "
                         "balance_stand in front of front_jump in 8 of 8 recordings, so routing "
                         "a change through it is the machine's own procedure.")
    ap.add_argument("--via-s", type=float, default=0.21,
                    help="how long to hold --via-clip; default is the measured median settle "
                         "time after a skill change (CLAUDE.md 3)")
    ap.add_argument("--balance-comp", choices=("off", "pd"), default="off",
                    help="off (default): open-loop replay. pd: close a loop on base attitude "
                         "-- a PD on roll added to the hip targets and on pitch added to the "
                         "thigh targets. A run with this on is NOT an open-loop replay; it is "
                         "banner-printed and stamped, and must be reported paired with off.")
    ap.add_argument("--balance-kp-roll", type=float, default=0.0)
    ap.add_argument("--balance-kd-roll", type=float, default=0.0)
    ap.add_argument("--balance-kp-pitch", type=float, default=0.0)
    ap.add_argument("--balance-kd-pitch", type=float, default=0.0)
    ap.add_argument("--balance-clip-rad", type=float, default=0.30,
                    help="hard limit on the correction, so a diverging loop cannot be reported "
                         "as a gait; the hip range is +-1.047 rad")
    ap.add_argument("--swing-lift", type=float, default=0.0,
                    help="raise every swing foot's arc to this apex in MILLIMETRES above "
                         "its own liftoff/touchdown chord. 0 (default) plays the recording. "
                         "80 is Unitree's own footRaiseHeight default, so it restores a "
                         "documented value rather than tuning one. Endpoints and their "
                         "slopes are untouched, so stride and speed are not being edited; "
                         "stance and the hip are untouched too.")
    ap.add_argument("--swing-lift-asym", action="store_true",
                    help="choose the lift amplitude PER LEG from that leg's own apex, the "
                         "original behaviour. The default is per MIRROR PAIR, because an "
                         "unequal left-right addition is a roll input the heading "
                         "controller cannot see. For the A/B only.")
    ap.add_argument("--stance-height-json", default="outputs/stance_height.json",
                    help="per-clip stance-height deficit and the solved thigh/calf offset, "
                         "from scripts/check_stance_height.py")
    ap.add_argument("--plant-comp", choices=("off", "stance", "height"), default="off",
                    help="off (default): play the recording as stored. stance: shift the hip "
                         "DC level toward the env's init_state.joint_pos, correcting the "
                         "~0.17 rad stance-width difference between the sport controller's "
                         "stand and this fork's default. Constant offset, no feedback, archive "
                         "untouched; banner printed and stamped into the results row. "
                         "height: offset thigh and calf so the body stands at the clip's own "
                         "stance geometry, correcting the 28-46 mm the PD sags under load. "
                         "Does not touch the hip, so it does not fight foot placement or "
                         "heading hold for the same joint.")
    ap.add_argument("--plant-comp-alpha", type=float, default=1.0,
                    help="how much of the stance difference to take out: 0 = none (the null "
                         "control, must equal --plant-comp off), 0.5 = half, 1 = all of it")
    ap.add_argument("--foot-comp", choices=("off", "raibert"), default="off",
                    help="off (default): the swing legs play the recording. raibert: add a "
                         "lateral foot-placement term to the SWING legs' hip targets, "
                         "foot_y = (T_stance/2) v_y + k (v_y - v_y_target), with T_stance "
                         "measured from the clip's own contact channel. Stance legs keep the "
                         "recording. This CLOSES A LOOP on base lateral velocity and it "
                         "OVERWRITES the recording; both are banner-printed and stamped, and "
                         "how far it departed is measured into the results row.")
    ap.add_argument("--foot-axis", choices=("y", "xy"), default="y",
                    help="y (default): lateral only -- roll is what diverges and left/right "
                         "foot position is what has authority over it. xy: also correct "
                         "fore-aft on the thigh, toward the LOG's measured forward speed "
                         "(not the command that produced it).")
    ap.add_argument("--foot-k", type=float, default=0.0,
                    help="velocity-error gain, in seconds, on top of the neutral point. "
                         "0 (default) leaves the parameter-free law: the (T_stance/2) v term "
                         "alone, whose only input is measured from the clip.")
    ap.add_argument("--foot-clip-rad", type=float, default=0.10,
                    help="hard cap on the correction, per joint, in radians. This is the "
                         "sweep knob: it bounds how far a run is allowed to depart from the "
                         "recording, so raising it until the gait holds is the measurement "
                         "'how much of the trajectory has to be overwritten'.")
    ap.add_argument("--foot-sign", type=float, choices=(1.0, -1.0), default=1.0,
                    help="direction of the correction. +1 is the one the geometry says "
                         "(+hip moves the foot toward +y on every leg); -1 exists because "
                         "stage 1's stabilising sign turned out not to be the one reasoning "
                         "about the joint frame predicted, so it is settled by measurement.")
    ap.add_argument("--foot-yaw", choices=("off", "log", "log-cycle"), default="off",
                    help="off (default, and what the stage-2 result was measured with): "
                         "every leg is driven toward --foot-vy-target. log: the target is "
                         "the recording's own motion and the ROTATION KINEMATICS that "
                         "follow from it -- leg i answers for "
                         "(v_y - v_y_log) + (wz - wz_log) * x_i, with v_y_log and wz_log "
                         "read from outputs/skill_profile.csv for this clip's session and "
                         "x_i measured from the articulation's own hip positions. No free "
                         "parameter is introduced. This is what an in-place turn needs "
                         "(v_y = 0 is wrong for it), and on a straight clip it is a "
                         "yaw-rate corrector. log-cycle: the same, but the yaw rate is "
                         "averaged over exactly one cycle of the clip being played -- the "
                         "instantaneous rate is 3.9x noisier than the heading bias it is "
                         "meant to correct, and feeding it back raw destabilises a trot. "
                         "The window is the clip's own period, so it is not a tuned filter.")
    ap.add_argument("--foot-vy-target", type=float, default=0.0,
                    help="lateral velocity the placement aims for. 0 by design: the clips are "
                         "straight-line gaits and the log's own vy is ~0.")
    ap.add_argument("--foot-swing-source", choices=("clip", "sim"), default="clip",
                    help="which leg counts as swinging. clip (default): the recording's own "
                         "contact channel, phase-locked and identical run to run. sim: the "
                         "contact sensor, resolved to legs BY NAME.")
    ap.add_argument("--foot-vel-filter-hz", type=float, default=0.0,
                    help="first-order low-pass on the base velocity the law reads. 0 (default) "
                         "is unfiltered.")
    ap.add_argument("--foot-yaw-bias", type=float, default=0.0,
                    help="OPEN-LOOP STEERING PROBE, radians. Adds a constant +front/-rear "
                         "differential to the swing legs' hip targets, which is a "
                         "differential lateral foot placement and therefore a yaw moment. "
                         "No feedback: this measures the actuator's gain (deg/s of yaw per "
                         "rad of bias) so a heading controller can be costed before it is "
                         "built. Works with --foot-comp off, and then the run is still an "
                         "open-loop replay plus a constant edit.")
    ap.add_argument("--foot-len-bias", type=float, default=0.0,
                    help="OPEN-LOOP STEERING PROBE, radians. The other candidate: a constant "
                         "+left/-right differential on the swing legs' THIGH targets, i.e. a "
                         "step-length difference between the two sides.")
    ap.add_argument("--dev-csv", default=None,
                    help="write the per-leg/per-joint/per-phase departure-from-the-recording "
                         "table here (one file, appended, same lock as --results-csv)")
    ap.add_argument("--tag", default="",
                    help="free-text label carried into every row, so a sweep point can be "
                         "found again without parsing argv")
    ap.add_argument("--hold-s", type=float, default=0.0,
                    help="render the scene, without advancing physics, for this many "
                         "seconds before playback starts and again after it ends. For "
                         "livestreaming: it takes longer to connect a client than the run "
                         "lasts. Does not perturb the run -- verify by the termination time.")
    ap.add_argument("--slowdown", type=float, default=1.0,
                    help="play back at 1/N real time by sleeping between control steps "
                         "(4 = quarter speed). Sleeping does not change the integration.")
    ap.add_argument("--trace-npz", default=None,
                    help="dump the per-step record (base pose, per-foot force, foot "
                         "positions) so a collapse can be read back without a rerun")
    ap.add_argument("--self-test", action="store_true", help="no Isaac Lab needed")
    ap.add_argument("--explain", action="store_true", help="print gains + expectations, no sim")
    ap.add_argument("--convention", action="store_true",
                    help="compare clip posture with the articulation's zero-action pose, no sim")
    ap.add_argument("--headroom", action="store_true",
                    help="compare logged torque with the sim's effort clip, no sim")
    try:
        from isaaclab.app import AppLauncher
        AppLauncher.add_app_launcher_args(ap)
        # AppLauncher owns --device on Isaac Lab 6.x and rejects a duplicate.
        # Its default is cuda:0; day 1 runs the physics on CPU, so re-assert that.
        ap.set_defaults(device="cpu")
    except Exception:
        ap.add_argument("--headless", action="store_true")
        ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    archive = Path(args.clip_archive) if args.clip_archive else SKILL_CLIPS_NPZ
    meta_json = (archive.with_suffix("").with_suffix(".meta.json")
                 if args.clip_archive else SKILL_CLIPS_META_JSON)
    if not archive.is_file():
        raise SystemExit(f"no clip archive at {archive}; run scripts/extract_skill_clips.py")
    if not meta_json.is_file():
        raise SystemExit(f"no meta beside the archive at {meta_json}")
    if args.clip_archive:
        print(f"[replay] clip archive {archive} (not the frozen one)")
    meta = json.loads(meta_json.read_text())
    names = list(meta["clips"]) if args.all else [args.clip]

    if args.convention:
        print(convention_report(meta, list(meta["clips"])))
        return 0

    if args.headroom:
        for name in names:
            print("=" * 78)
            print(headroom_report(meta, name))
        return 0

    if args.explain:
        cfg = upstream_go2()
        for name in names:
            e = expected_from_meta(meta, name)
            print("=" * 78)
            print(gain_comparison(meta, name, cfg))
            print(f"\nExpected from the log for {name}: stride {e['stride_hz']:.3f} Hz, "
                  f"duty {e['duty']:.3f}, vx {e['vx_mean']:.3f} m/s"
                  if np.isfinite(e["stride_hz"]) else
                  f"\n{name} is a one-shot clip: duration {meta['clips'][name]['duration_s']:.2f} s, "
                  f"flight {meta['clips'][name]['flight_s']:.3f} s")
        print("\n" + meta["convention_note"])
        return 0

    rows = []
    try:
        # --all reuses one SimulationApp across clips; only the single-clip path day 1
        # prescribes has been exercised on this machine.
        for name in names:
            clip = load_clip(name, args.rate, archive)
            if args.ablate != "none":
                if clip["kind"] != "cyclic":
                    raise SystemExit(f"--ablate {args.ablate} is defined for cyclic clips only")
                clip = mirror_clip(clip, args.ablate)
                print(f"[replay] *** DIAGNOSTIC ABLATION --ablate {args.ablate}: this run does "
                      f"NOT play the recording. It is an attribution probe; the archive on "
                      f"disk is untouched and no skill may be reported from it. ***")
            exp = expected_from_meta(meta, name)
            print("=" * 78)
            print(gain_comparison(meta, name))
            print()
            tc = load_clip(args.then_clip, args.rate, archive) if args.then_clip else None
            vc = load_clip(args.via_clip, args.rate, archive) if args.via_clip else None
            m = run_isaac(args, clip, meta, tc, vc)
            ranked = D.best_mapping(m.pop("_q_measured"), m.pop("_q_commanded"))
            offs = None
            F = D.diagnose(m, exp, mapping=ranked, offsets=offs)
            segs = m.pop("_segments", [])
            dev = m.pop("_dev_rows", [])
            if args.dev_csv and dev:
                report(dev, args.dev_csv)
            if len(segs) > 1:
                print(f"-- {name} sequence, per segment --")
                for sg in segs:
                    e = expected_from_meta(meta, sg["clip"]) if sg["clip"] in meta["clips"] else {}
                    for f in D.gait_reproduced(sg, e):
                        print(f"  [{sg['role']}] {sg['clip']:8s} "
                              f"{sg['t0_s']:6.2f}-{sg['t1_s']:6.2f}s  "
                              f"{f.severity.upper():4s} {f.symptom}")
            print(f"-- {name} [{args.rate}, mode={m['mode_used']}, hip={args.hip_sign}] --")
            print(D.format_findings(F))
            print(f"verdict: {D.verdict(F)}")
            rows.append({"clip": name, "rate": args.rate, "verdict": D.verdict(F),
                         "best_mapping": ranked[0].name, "best_mapping_r": ranked[0].score,
                         **{k: v for k, v in m.items() if not k.startswith("_")}})
        report(rows, args.results_csv)
    finally:
        if _SIM_APP is not None:
            # SimulationApp.close() tears the process down, so an exception on its
            # way out of this try never reaches stderr: the interpreter is gone
            # before the traceback is printed, and the run looks like a clean exit 0
            # that simply stopped talking half way through the report.  Two runs
            # were spent finding that out.  Print it here, first, then close.
            exc = sys.exc_info()[1]
            if exc is not None:
                import traceback
                print("[replay] EXCEPTION -- printed here because closing the app below "
                      "would otherwise take the traceback with it:", file=sys.stderr)
                traceback.print_exc()
                sys.stderr.flush()
                sys.stdout.flush()
            _SIM_APP.close()
    return 0 if all(r["verdict"] != "FAIL" for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
