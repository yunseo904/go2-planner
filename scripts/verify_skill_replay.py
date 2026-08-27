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
from sim.replay import Capabilities, ReplayMode, apply_gains, default_mode_for, probe_capabilities, torque_headroom
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

def load_clip(name: str, rate: str = "lo") -> dict:
    z = np.load(SKILL_CLIPS_NPZ, allow_pickle=False)
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

    dt = 1.0 / clip["fs"]
    if ucfg.control_hz and abs(clip["fs"] - ucfg.control_hz) > 1.0:
        print(f"[replay] NOTE clip rate {clip['fs']:.1f} Hz != the env's control rate "
              f"{ucfg.control_hz:.0f} Hz; stepping at the clip rate")
    sim = SimulationContext(SimulationCfg(dt=dt, device=args.device))
    sim.set_camera_view([2.0, 2.0, 1.0], [0.0, 0.0, 0.3])

    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
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

    sim.reset()

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

    n = clip["q_des"].shape[0]
    reps = args.cycles if clip["kind"] == "cyclic" else 1
    total = n * reps
    q_cmd = torch.as_tensor(np.tile(clip["q_des"] * sign, (reps, 1)), device=sim.device, dtype=torch.float32)
    tau_ff = torch.as_tensor(np.tile(clip["tau_ff"] * sign, (reps, 1)), device=sim.device, dtype=torch.float32)
    kp_seq, kd_seq = np.tile(clip["kp"], (reps, 1)), np.tile(clip["kd"], (reps, 1))

    # Settle on the clip's first pose before playing, so step 0 is not a jump.
    root = robot.data.default_root_state.clone()
    robot.write_root_state_to_sim(root)
    q0 = robot.data.default_joint_pos.clone()
    q0[:, idx_t] = q_cmd[0]
    robot.write_joint_state_to_sim(q0, torch.zeros_like(q0))
    for _ in range(int(args.settle_s / dt)):
        robot.set_joint_position_target(q0)
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)

    rec = {k: [] for k in ("q", "tau", "root_lin_vel_b", "root_ang_vel_b", "root_pos_w", "contact")}
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
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)
        contacts.update(dt)

        tau_now = robot.data.applied_torque[0, idx_t]
        rec["tau"].append(tau_now.cpu().numpy())
        rec["q"].append(robot.data.joint_pos[0, idx_t].cpu().numpy())
        rec["root_lin_vel_b"].append(robot.data.root_lin_vel_b[0].cpu().numpy())
        rec["root_ang_vel_b"].append(robot.data.root_ang_vel_b[0].cpu().numpy())
        rec["root_pos_w"].append(robot.data.root_pos_w[0].cpu().numpy())
        f = contacts.data.net_forces_w[0].norm(dim=-1)
        rec["contact"].append((f > args.contact_threshold_n).cpu().numpy())
        if robot.data.root_pos_w[0, 2].item() < args.fall_height_m and terminated_s is None:
            terminated_s = i * dt
            break

    a = {k: np.asarray(v) for k, v in rec.items()}
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
        "mode_requested": requested.value,
        "mode_used": mode.value,
        "gain_writes": gain_writes,
        "torque_saturated_frac": saturated,
        "cap_runtime_gains": cap.runtime_gain_write,
        "cap_effort_target": cap.effort_target,
        "hip_sign": args.hip_sign,
    })
    if saturated > 0.02:
        print(f"[replay] WARNING {saturated:.1%} of joint-samples hit the effort clip; "
              f"the sim robot is torque-limited on this clip (see --headroom)")
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

def report(rows: list) -> None:
    import csv
    REPLAY_RESULTS_CSV.parent.mkdir(parents=True, exist_ok=True)
    keys = sorted({k for r in rows for k in r if not k.startswith("_")})
    with open(REPLAY_RESULTS_CSV, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})
    print(f"[replay] wrote {REPLAY_RESULTS_CSV}")


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
    ap.add_argument("--contact-threshold-n", type=float, default=1.0)
    ap.add_argument("--fall-height-m", type=float, default=0.15)
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

    if not SKILL_CLIPS_NPZ.is_file():
        raise SystemExit(f"no clip archive at {SKILL_CLIPS_NPZ}; run scripts/extract_skill_clips.py")
    meta = json.loads(SKILL_CLIPS_META_JSON.read_text())
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
            clip = load_clip(name, args.rate)
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
        report(rows)
    finally:
        if _SIM_APP is not None:
            _SIM_APP.close()
    return 0 if all(r["verdict"] != "FAIL" for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
