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
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim import diagnose as D
from sim import isaac_cfg as IC
from sim.replay import (Capabilities, ReplayMode, apply_gains, assert_not_aliased,
                        default_mode_for, foot_body_ids, ground_material_cfg,
                        probe_capabilities, set_robot_friction, snap, torque_headroom)
from terrain_toolkit.paths import (
    REPLAY_REPORT_MD,
    REPLAY_RESULTS_CSV,
    SKILL_CLIPS_META_JSON,
    SKILL_CLIPS_NPZ,
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
        "flight_frac": c.get("flight_frac", sel.get("flight_frac", np.nan)),
        "position_controlled": c["gains"]["position_controlled"],
        "kind": c["kind"],
    }


# --------------------------------------------------------------------------- #
# Isaac Lab replay  (UNVERIFIED — see the module docstring)
# --------------------------------------------------------------------------- #

# SimulationApp.close() tears the process down, so anything after it never runs.
# Both scripts originally closed inside the Isaac helper, before the metrics were
# computed and returned -- the run exited 0 having produced no verdict and no CSV.
# The app is held here and closed once, in main(), after the results are written.
_SIM_APP = None


def run_isaac(args, clip: dict, meta: dict) -> dict:
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
    elif clip["kind"] == "cyclic":
        _, ph = quiescent_start(clip)
    else:
        ph = {"start_frame": 0, "feet_down": None, "all_stance": None}
    ph.setdefault("spread_at_start", float("nan"))

    n = clip["q_des"].shape[0]
    reps = args.cycles if clip["kind"] == "cyclic" else 1
    total = n * reps
    q_cmd = torch.as_tensor(np.tile(clip["q_des"] * sign, (reps, 1)), device=sim.device, dtype=torch.float32)
    tau_ff = torch.as_tensor(np.tile(clip["tau_ff"] * sign, (reps, 1)), device=sim.device, dtype=torch.float32)
    kp_seq, kd_seq = np.tile(clip["kp"], (reps, 1)), np.tile(clip["kd"], (reps, 1))

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
                           "root_quat_w", "contact_f", "foot_pos_w")}
    foot_ids, foot_names = foot_body_ids(robot)   # articulation indices, not sensor indices
    terminated_s, gain_writes, clipped = None, 0, 0
    for i in range(total):
        if mode is ReplayMode.TORQUE:
            gain_writes += int(apply_gains(robot, idx_t, kp_seq[i], kd_seq[i], cap))
        tgt = robot.data.default_joint_pos.clone()
        tgt[:, idx_t] = q_cmd[i]
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
        if video is not None and i % args.video_stride == 0:
            grab_frame()
        if robot.data.root_pos_w[0, 2].item() < args.fall_height_m and terminated_s is None:
            terminated_s = i * dt
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

    a = {k: np.asarray(v) for k, v in rec.items()}
    try:
        assert_not_aliased({k: a[k] for k in ("root_pos_w", "root_quat_w", "root_lin_vel_b")})
    except AssertionError as exc:
        raise SystemExit(f"[replay] BUG {exc} Refusing to report numbers.")
    _, _, limits = ucfg.ordered(clip["leg_order"], clip["joint_order"])
    lim = np.asarray([l if l is not None else np.inf for l in limits], dtype=float)
    saturated = float((np.abs(a["tau"]) >= lim[None, :] - 1e-3).mean())

    m = D.gait_from_contact(a["contact"], dt)
    m.update({
        "vx_mean": float(a["root_lin_vel_b"][:, 0].mean()),
        "vy_mean": float(a["root_lin_vel_b"][:, 1].mean()),
        "yaw_rate_deg_s": float(np.degrees(a["root_ang_vel_b"][:, 2].mean())),
        "base_height_mean_m": float(a["root_pos_w"][:, 2].mean()),
        "terminated_s": terminated_s,
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
                            leg_order=np.array(clip["leg_order"]),
                            joint_order=np.array(clip["joint_order"]),
                            foot_names=np.array(foot_names),
                            contact_names=np.array(getattr(contacts, "body_names", [""] * 4)),
                            q_cmd=q_cmd.detach().cpu().numpy()[: a["q"].shape[0]].copy(),
                            **{k: v for k, v in a.items()})
        print(f"[replay] trace -> {args.trace_npz}  ({a['q'].shape[0]} control steps)")
    m["_q_measured"] = a["q"]
    m["_q_commanded"] = np.tile(clip["q_des"] * sign, (reps, 1))[: a["q"].shape[0]]
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
    import csv
    out = Path(out) if out else REPLAY_RESULTS_CSV
    out.parent.mkdir(parents=True, exist_ok=True)
    keys = sorted({k for r in rows for k in r if not k.startswith("_")})
    with open(out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})
    print(f"[replay] wrote {out}")


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
    ap.add_argument("--start-phase", choices=("first", "stance", "level"), default="first",
                    help="Where in the loop to begin. first: frame 0, as recorded. "
                         "stance: the frame with the most feet down per the clip's contact "
                         "channel. level: the frame whose joint angles put the four feet "
                         "closest to a plane, measured from the robot's own kinematics. "
                         "A cyclic clip is a loop, so this selects an entry point; it does "
                         "not alter the recording. Ignored for one-shot clips.")
    ap.add_argument("--settle-mode", choices=("drop", "stand"), default="drop",
                    help="drop: fall into the clip's first pose from the spawn height "
                         "(original). stand: settle on the default standing pose first, "
                         "then drive to the clip pose with the PD.")
    ap.add_argument("--contact-threshold-n", type=float, default=1.0)
    ap.add_argument("--fall-height-m", type=float, default=0.15)
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
            m = run_isaac(args, clip, meta)
            ranked = D.best_mapping(m.pop("_q_measured"), m.pop("_q_commanded"))
            offs = None
            F = D.diagnose(m, exp, mapping=ranked, offsets=offs)
            print(f"-- {name} [{args.rate}, mode={m['mode_used']}, hip={args.hip_sign}] --")
            print(D.format_findings(F))
            print(f"verdict: {D.verdict(F)}")
            rows.append({"clip": name, "rate": args.rate, "verdict": D.verdict(F),
                         "best_mapping": ranked[0].name, "best_mapping_r": ranked[0].score,
                         **{k: v for k, v in m.items() if not k.startswith("_")}})
        report(rows, args.results_csv)
    finally:
        if _SIM_APP is not None:
            _SIM_APP.close()
    return 0 if all(r["verdict"] != "FAIL" for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
